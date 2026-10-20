"""串口读取、控制命令和自动重连。"""

from __future__ import annotations

import errno
import glob
import os
import queue
import re
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any, Protocol

import serial

from .protocol import ParsedPacket, PTSProtocolReader

BIAS_COMMAND = b"z\n"
START_SLIP_COMMAND = b"S\n"
STOP_SLIP_COMMAND = b"s\n"
SUPPORTED_SAMPLING_RATES = frozenset({100, 250, 500, 1000})
# packet_counter 为无符号 32 位；半模规则区分前进（含自然回绕）与后退。
_UINT32_MASK = 0xFFFFFFFF
_UINT32_HALF = 0x80000000
# 单次缺包达到该值才告警；少量缺包只计入统计，避免 1000 Hz 下日志刷屏。
_LARGE_GAP_WARNING_PACKETS = 8
# 串口打开后设备端积压的旧帧要几 ms 才涌入内核接收缓冲，立即 flush 清不到，
# 静默 0.1 s 覆盖 USB 首个读 URB 往返与设备缓冲倾泻后再清空才有效（2026-10 实测）。
BACKLOG_DRAIN_GRACE_SEC = 0.1
# USB 插入后控制器固件启动 + 枚举约 1~2 s 才出现设备节点，5 s 默认值给足裕量。
DEFAULT_DEVICE_WAIT_SEC = 5.0
# 轮询间隔 50 ms：对枚举延迟的感知误差远小于固件启动时间本身。
DEVICE_POLL_SEC = 0.05


class SerialPort(Protocol):
    """描述驱动使用的最小串口接口。"""

    def read(self, size: int) -> bytes:
        """读取串口字节。"""
        ...

    def write(self, data: bytes) -> int:
        """写入串口字节。"""
        ...

    def flush(self) -> None:
        """等待已写数据发送完毕。"""
        ...

    def reset_input_buffer(self) -> None:
        """丢弃操作系统接收缓冲区中的积压字节。"""
        ...

    def close(self) -> None:
        """关闭串口。"""
        ...


@dataclass(frozen=True)
class SerialWorkerConfig:
    """串口工作线程配置。

    Args:
        port: 串口设备路径。
        baud_rate: 串口波特率，单位 baud。
        sampling_rate: 控制器采样频率，单位 Hz。
        timeout_sec: 单次串口读取超时，单位 s。
        max_packet_bytes: 单个协议帧允许的最大字节数。
        reconnect_initial_delay_sec: 首次重连等待时间，单位 s。
        reconnect_max_delay_sec: 最大重连等待时间，单位 s。
        backlog_drain_grace_sec: 打开串口后等待设备端积压涌入内核再清空的静默时间，
            单位 s；覆盖 USB 首个读 URB 往返与设备缓冲倾泻，0 表示跳过该防御。
    """

    port: str
    baud_rate: int
    sampling_rate: int
    timeout_sec: float = 1.0
    max_packet_bytes: int = 8192
    reconnect_initial_delay_sec: float = 1.0
    reconnect_max_delay_sec: float = 10.0
    backlog_drain_grace_sec: float = BACKLOG_DRAIN_GRACE_SEC

    def validate(self) -> None:
        """校验配置。

        Raises:
            ValueError: 配置值不满足串口驱动约束。
        """
        if not self.port:
            raise ValueError("com_port 不能为空")
        if self.baud_rate <= 0:
            raise ValueError("baud_rate 必须大于 0")
        if self.sampling_rate not in SUPPORTED_SAMPLING_RATES:
            rates = ", ".join(str(rate) for rate in sorted(SUPPORTED_SAMPLING_RATES))
            raise ValueError(f"sampling_rate 必须是 {rates} Hz 之一")
        if self.timeout_sec <= 0:
            raise ValueError("serial_timeout_sec 必须大于 0")
        if self.max_packet_bytes <= 0:
            raise ValueError("max_packet_bytes 必须大于 0")
        if self.reconnect_initial_delay_sec <= 0:
            raise ValueError("reconnect_initial_delay_sec 必须大于 0")
        if self.reconnect_max_delay_sec < self.reconnect_initial_delay_sec:
            raise ValueError("reconnect_max_delay_sec 不能小于初始重连间隔")
        if self.backlog_drain_grace_sec < 0:
            raise ValueError("backlog_drain_grace_sec 不能为负")


@dataclass
class _PendingCommand:
    payload: bytes
    completed: threading.Event
    succeeded: bool = False
    started: bool = False
    cancelled: bool = False
    lock: threading.Lock = field(default_factory=threading.Lock)


def _default_serial_factory(**kwargs: Any) -> SerialPort:
    """打开 pyserial 串口。

    Args:
        **kwargs: 传递给 ``serial.Serial`` 的参数。

    Returns:
        已打开的串口对象。
    """
    return serial.Serial(**kwargs)


_OPEN_FAILURE_HINTS = {
    errno.EACCES: "无权访问, 请确认当前用户在 dialout 组",
    errno.EBUSY: "端口被其他进程占用, 可用 lsof 查看占用进程",
}

_KERNEL_TTY_PATTERN = re.compile(r"/dev/tty(ACM|USB|S)\d+$")


def wait_for_serial_device(port: str, wait_sec: float = DEFAULT_DEVICE_WAIT_SEC) -> bool:
    """等待串口设备节点出现，覆盖 USB 热插拔后的枚举延迟。

    USB 插入后控制器固件启动 + 枚举约 1~2 s 才出现 ``/dev/ttyACM*`` 节点；
    入口程序在节点出现前启动会立即失败，统一在此提供有界等待。

    Args:
        port: 串口设备路径，如 /dev/ttyACM0 或 udev 软链接 /dev/papillarray。
        wait_sec: 最长等待时间，单位 s；小于等于 0 时只检查一次不等待。

    Returns:
        True 表示节点已出现；False 表示等待超时仍不存在。
    """
    deadline = time.monotonic() + max(wait_sec, 0.0)
    while True:
        if os.path.exists(port):
            return True
        if time.monotonic() >= deadline:
            return False
        time.sleep(DEVICE_POLL_SEC)


def _format_available_ports() -> str:
    """汇总系统当前可用的串口及其 udev 别名。

    扫描 /dev/ttyACM* 与 /dev/ttyUSB*, 再遍历 /dev 顶层符号链接找出指向这些
    设备的别名 (如 /dev/papillarray), 让"端口不存在"的报错自带设备发现能力。

    Returns:
        形如 "/dev/ttyACM0 (别名 /dev/papillarray)" 的逗号分隔列表; 无设备时
        为 "无"。
    """
    devices = sorted(set(glob.glob("/dev/ttyACM*")) | set(glob.glob("/dev/ttyUSB*")))
    if not devices:
        return "无"
    real_paths = {os.path.realpath(device): device for device in devices}
    aliases: dict[str, list[str]] = {}
    try:
        entries = list(os.scandir("/dev"))
    except OSError:
        entries = []
    for entry in entries:
        try:
            if not entry.is_symlink():
                continue
            target = os.path.realpath(entry.path)
        except OSError:
            continue
        if target in real_paths and target != entry.path:
            aliases.setdefault(target, []).append(f"/dev/{entry.name}")
    parts = []
    for device in devices:
        names = sorted(aliases.get(os.path.realpath(device), []))
        suffix = f" (别名 {'、'.join(names)})" if names else ""
        parts.append(device + suffix)
    return ", ".join(parts)


def _describe_open_failure(port: str, exc: Exception) -> str:
    """把串口打开失败翻译成带排查建议的错误消息。

    按 errno 与端口路径形态区分 "ttyACM 编号写错"、"udev 别名未安装" 与
    "by-id 设备未连接" 等情形; 打开成功后的读写中断不经过此分类, 仍按原文上报。

    Args:
        port: 打开失败的串口路径。
        exc: ``serial_factory`` 抛出的异常。

    Returns:
        分类后的错误消息。
    """
    code = getattr(exc, "errno", None)
    if code == errno.ENOENT:
        available = _format_available_ports()
        if port.startswith("/dev/serial/"):
            return f"串口设备 {port} 不存在: 设备未连接; 系统当前可用: {available}"
        if _KERNEL_TTY_PATTERN.match(port):
            return f"串口设备 {port} 不存在, 系统当前可用: {available}; 请确认 com_port 参数"
        return (
            f"串口别名 {port} 不存在: 设备未连接, 或 udev 规则未安装"
            f" (规则文件见本仓库 udev/ 目录); 系统当前可用: {available}"
        )
    if code in _OPEN_FAILURE_HINTS:
        return f"串口 {port} 打开失败: {_OPEN_FAILURE_HINTS[code]}"
    return f"串口连接中断: {exc}"


class SerialWorker:
    """在独立线程中读取 PTS 帧并管理控制命令。

    Args:
        config: 串口和重连配置。
        on_packet: 收到有效协议包时的回调。
        serial_factory: 串口工厂，测试时可替换为 fake serial。
        on_info: 普通状态日志回调。
        on_warning: 警告日志回调。
        on_error: 错误日志回调。
    """

    def __init__(
        self,
        config: SerialWorkerConfig,
        on_packet: Callable[[ParsedPacket], None],
        serial_factory: Callable[..., SerialPort] = _default_serial_factory,
        on_info: Callable[[str], None] | None = None,
        on_warning: Callable[[str], None] | None = None,
        on_error: Callable[[str], None] | None = None,
    ) -> None:
        config.validate()
        self._config = config
        self._on_packet = on_packet
        self._serial_factory = serial_factory
        self._on_info = on_info or (lambda _message: None)
        self._on_warning = on_warning or (lambda _message: None)
        self._on_error = on_error or (lambda _message: None)
        self._commands: queue.Queue[_PendingCommand] = queue.Queue()
        self._stop_event = threading.Event()
        self._connected_event = threading.Event()
        self._thread: threading.Thread | None = None
        # 包时间戳仅供调用方判断数据流是否稳定；I/O 线程单写，float 赋值原子。
        self._first_packet_monotonic: float | None = None
        self._last_packet_monotonic: float | None = None
        # 连续性统计同样只有 I/O 线程写入，int 赋值原子；基线在连接与 bias 后重置。
        self._last_packet_counter: int | None = None
        self._last_timestamp_us: int | None = None
        self._dropped_packets = 0
        self._disordered_packets = 0
        self._non_monotonic_timestamps = 0

    @property
    def first_packet_monotonic(self) -> float | None:
        """返回首个有效数据包的 ``time.monotonic()`` 时间戳，未收到数据时为 ``None``。"""
        return self._first_packet_monotonic

    @property
    def last_packet_monotonic(self) -> float | None:
        """返回最近一个有效数据包的 ``time.monotonic()`` 时间戳，未收到数据时为 ``None``。"""
        return self._last_packet_monotonic

    @property
    def dropped_packets(self) -> int:
        """累计缺包数（按 packet_counter 正向差值推断，不含会话间隔）。"""
        return self._dropped_packets

    @property
    def disordered_packets(self) -> int:
        """累计的重复、乱序或计数器回退事件数。"""
        return self._disordered_packets

    @property
    def non_monotonic_timestamps(self) -> int:
        """累计的设备时间戳非递增事件数。"""
        return self._non_monotonic_timestamps

    @property
    def is_connected(self) -> bool:
        """返回串口当前是否已连接。"""
        return self._connected_event.is_set()

    def start(self) -> None:
        """启动串口工作线程。

        Raises:
            RuntimeError: 工作线程已启动。
        """
        if self._thread is not None and self._thread.is_alive():
            raise RuntimeError("串口工作线程已经启动")
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="papillarray-serial-worker",
            daemon=True,
        )
        self._thread.start()

    def stop(self, join_timeout_sec: float | None = None) -> None:
        """停止线程并等待串口关闭。

        Args:
            join_timeout_sec: 等待线程退出的最长时间，单位 s；默认依据串口超时计算。
        """
        self._stop_event.set()
        self._fail_pending_commands()
        if self._thread is not None:
            timeout = join_timeout_sec or self._config.timeout_sec + 1.0
            self._thread.join(timeout=timeout)
            if self._thread.is_alive():
                self._on_error("串口工作线程未在预期时间内退出")
        self._thread = None

    def send_command(self, payload: bytes, timeout_sec: float | None = None) -> bool:
        """请求 I/O 线程发送命令。

        Args:
            payload: 完整 ASCII 控制命令字节。
            timeout_sec: 等待命令写入的最长时间，单位 s。

        Returns:
            命令是否已完整写入并 flush。断线时立即返回 ``False``。
        """
        if not self.is_connected or self._stop_event.is_set():
            return False
        command = _PendingCommand(payload=payload, completed=threading.Event())
        self._commands.put(command)
        wait_timeout = timeout_sec or self._config.timeout_sec + 0.5
        if not command.completed.wait(wait_timeout):
            with command.lock:
                if not command.started:
                    command.cancelled = True
            return False
        return command.succeeded

    def _run(self) -> None:
        reconnect_delay = self._config.reconnect_initial_delay_sec
        while not self._stop_event.is_set():
            serial_port: SerialPort | None = None
            try:
                serial_port = self._serial_factory(
                    port=self._config.port,
                    baudrate=self._config.baud_rate,
                    timeout=self._config.timeout_sec,
                )
                self._connected_event.set()
                self._on_info(f"已连接 {self._config.port}，波特率 {self._config.baud_rate} baud")
                self._reset_continuity()
                self._write_payload(
                    serial_port,
                    f"f{self._config.sampling_rate}\n".encode("ascii"),
                )
                # 旧会话的帧滞留在设备端发送缓冲（实测可达 40 s 前），打开后几 ms
                # 才涌入内核，因此立即 flush 清不到。采样率命令实测会让控制器自行
                # 丢弃积压，但不能依赖该隐式固件行为，静默后统一清空做显式保证，
                # 避免旧帧被盖上本次接收时间流入回调。
                time.sleep(self._config.backlog_drain_grace_sec)
                serial_port.reset_input_buffer()
                reader = PTSProtocolReader(serial_port, self._config.max_packet_bytes)
                received_valid_packet = False

                while not self._stop_event.is_set():
                    self._drain_commands(serial_port, reader)
                    try:
                        packet = reader.read_packet()
                    except ValueError as exc:
                        # 单帧结构异常不代表串口失效，继续同步后续帧可避免无谓重连。
                        self._on_warning(f"已丢弃无法解析的数据包: {exc}")
                        continue
                    try:
                        packet_monotonic_s = time.monotonic()
                        if self._first_packet_monotonic is None:
                            self._first_packet_monotonic = packet_monotonic_s
                        self._last_packet_monotonic = packet_monotonic_s
                        self._track_continuity(packet)
                        self._on_packet(packet)
                    except Exception as exc:  # noqa: BLE001
                        # ROS 发布回调异常不应终止串口线程，否则设备仍连接却永久停止采集。
                        self._on_error(f"数据包消费回调失败: {exc}")
                    if not received_valid_packet:
                        reconnect_delay = self._config.reconnect_initial_delay_sec
                        received_valid_packet = True
            except (OSError, TimeoutError, serial.SerialException) as exc:
                if not self._stop_event.is_set():
                    if serial_port is None:
                        # 打开阶段失败按 errno 分类提示, 区分参数写错与线缆脱落等情形。
                        self._on_error(_describe_open_failure(self._config.port, exc))
                    else:
                        self._on_error(f"串口连接中断: {exc}")
            finally:
                self._connected_event.clear()
                self._fail_pending_commands()
                if serial_port is not None:
                    try:
                        # 所有退出路径都关闭字符设备，避免下一次启动时串口仍被占用。
                        serial_port.close()
                    except (OSError, serial.SerialException) as exc:
                        self._on_warning(f"关闭串口时发生异常: {exc}")

            if not self._stop_event.is_set():
                self._on_info(f"将在 {reconnect_delay:.1f} s 后重连")
                self._stop_event.wait(reconnect_delay)
                reconnect_delay = min(reconnect_delay * 2.0, self._config.reconnect_max_delay_sec)

    def _drain_commands(self, serial_port: SerialPort, reader: PTSProtocolReader) -> None:
        while True:
            try:
                command = self._commands.get_nowait()
            except queue.Empty:
                return
            with command.lock:
                if command.cancelled:
                    command.completed.set()
                    continue
                command.started = True
            is_bias = command.payload == BIAS_COMMAND
            try:
                if is_bias:
                    # bias 前串口里的残留字节仍是旧零偏数据，清掉避免混入归零后的窗口。
                    serial_port.reset_input_buffer()
                    reader.reset_buffer()
                self._write_payload(serial_port, command.payload)
                command.succeeded = True
                if is_bias:
                    self._reset_continuity()
            except (OSError, serial.SerialException) as exc:
                self._on_error(f"控制命令写入失败: {exc}")
                command.succeeded = False
                raise
            finally:
                command.completed.set()

    def _track_continuity(self, packet: ParsedPacket) -> None:
        """在线监督 packet_counter 与设备时间戳的连续性。

        计数器按无符号 32 位半模规则判定方向：正向差值代表缺包（自然回绕
        亦是正向），差值为 0 代表重复，反向代表乱序或设备重启。少量缺包只
        计入 ``dropped_packets``，达到 ``_LARGE_GAP_WARNING_PACKETS`` 才告警。

        Args:
            packet: 刚收到的有效协议包。
        """
        counter = packet.packet_counter & _UINT32_MASK
        if self._last_packet_counter is None:
            self._last_packet_counter = counter
            self._last_timestamp_us = packet.timestamp_us
            return
        delta = (counter - self._last_packet_counter) & _UINT32_MASK
        if delta == 0:
            self._disordered_packets += 1
            self._on_warning(f"包计数器重复: {counter}")
        elif delta < _UINT32_HALF:
            missing = delta - 1
            if missing:
                self._dropped_packets += missing
                if missing >= _LARGE_GAP_WARNING_PACKETS:
                    self._on_warning(
                        f"包计数器 {self._last_packet_counter} -> {counter}，缺 {missing} 包"
                    )
        else:
            self._disordered_packets += 1
            self._on_warning(
                f"包计数器回退: {self._last_packet_counter} -> {counter}，按设备重启重置基线"
            )
        self._last_packet_counter = counter
        if packet.timestamp_us <= self._last_timestamp_us:
            self._non_monotonic_timestamps += 1
            self._on_warning(
                f"设备时间戳未递增: {self._last_timestamp_us} -> {packet.timestamp_us} us"
            )
        self._last_timestamp_us = packet.timestamp_us

    def _reset_continuity(self) -> None:
        """重置连续性基线。

        连接建立与 bias 写入后调用，缺包数因此不含会话间隔和归零瞬态；
        累计统计保留，反映整次节点生命周期的数据质量。
        """
        self._last_packet_counter = None
        self._last_timestamp_us = None

    @staticmethod
    def _write_payload(serial_port: SerialPort, payload: bytes) -> None:
        written = serial_port.write(payload)
        if written != len(payload):
            raise OSError(f"串口仅写入 {written}/{len(payload)} 字节")
        serial_port.flush()

    def _fail_pending_commands(self) -> None:
        while True:
            try:
                command = self._commands.get_nowait()
            except queue.Empty:
                return
            with command.lock:
                command.cancelled = True
                command.succeeded = False
                command.completed.set()
