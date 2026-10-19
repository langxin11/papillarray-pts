#!/usr/bin/env python3
"""串口命令、释放和自动重连的离线测试。"""

from __future__ import annotations

import errno
import threading
import time
from typing import Any

import numpy as np

import papillarray_pts.serial_worker as worker_module
from papillarray_pts.protocol import ParsedPacket
from papillarray_pts.serial_worker import (
    BIAS_COMMAND,
    START_SLIP_COMMAND,
    STOP_SLIP_COMMAND,
    SerialWorker,
    SerialWorkerConfig,
)


def _packet(packet_counter: int = 1, timestamp_us: int = 2) -> ParsedPacket:
    return ParsedPacket(
        packet_counter=packet_counter,
        timestamp_us=timestamp_us,
        pillar_forces=[np.empty((0, 3))],
        pillar_displacements=[np.empty((0, 3))],
        global_forces=[np.zeros(3)],
        global_torques=[np.zeros(3)],
    )


class _FakeReader:
    def __init__(self, _serial_port: object, _max_packet_bytes: int) -> None:
        pass

    def reset_buffer(self) -> None:
        pass

    def read_packet(self) -> ParsedPacket:
        time.sleep(0.005)
        return _packet()


class _BlockingReader:
    entered = threading.Event()
    release = threading.Event()

    def __init__(self, _serial_port: object, _max_packet_bytes: int) -> None:
        pass

    def read_packet(self) -> ParsedPacket:
        self.entered.set()
        self.release.wait(1.0)
        return _packet()


class _FakeSerial:
    def __init__(self) -> None:
        self.writes: list[bytes] = []
        self.events: list[tuple[str, bytes | None]] = []
        self.closed = False

    def read(self, _size: int) -> bytes:
        return b""

    def write(self, data: bytes) -> int:
        self.writes.append(data)
        self.events.append(("write", data))
        return len(data)

    def flush(self) -> None:
        pass

    def reset_input_buffer(self) -> None:
        self.events.append(("flush", None))

    def close(self) -> None:
        self.closed = True


class _StubReader:
    """记录 reset_buffer 调用次数的最小协议读取器。"""

    def __init__(self) -> None:
        self.reset_buffer_calls = 0

    def reset_buffer(self) -> None:
        self.reset_buffer_calls += 1


def _wait_for(predicate: Any, timeout_sec: float = 1.0) -> None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("等待条件超时")


def _config() -> SerialWorkerConfig:
    return SerialWorkerConfig(
        port="/dev/fake",
        baud_rate=115200,
        sampling_rate=500,
        timeout_sec=0.05,
        reconnect_initial_delay_sec=0.01,
        reconnect_max_delay_sec=0.02,
    )


def test_writes_sampling_rate_and_service_commands_in_io_thread(
    monkeypatch: Any,
) -> None:
    monkeypatch.setattr(worker_module, "PTSProtocolReader", _FakeReader)
    fake_serial = _FakeSerial()
    callback_threads: list[int] = []
    worker = SerialWorker(
        _config(),
        lambda _packet_value: callback_threads.append(threading.get_ident()),
        serial_factory=lambda **_kwargs: fake_serial,
    )

    worker.start()
    _wait_for(lambda: worker.is_connected and bool(callback_threads))
    assert worker.send_command(BIAS_COMMAND)
    assert worker.send_command(START_SLIP_COMMAND)
    assert worker.send_command(STOP_SLIP_COMMAND)
    worker.stop()

    assert fake_serial.writes[:4] == [b"f500\n", b"z\n", b"S\n", b"s\n"]
    assert callback_threads[0] != threading.get_ident()
    assert fake_serial.closed


def test_returns_false_instead_of_queueing_command_while_disconnected() -> None:
    worker = SerialWorker(_config(), lambda _packet_value: None)

    assert worker.send_command(BIAS_COMMAND) is False


def test_does_not_send_command_after_wait_timeout(monkeypatch: Any) -> None:
    _BlockingReader.entered.clear()
    _BlockingReader.release.clear()
    monkeypatch.setattr(worker_module, "PTSProtocolReader", _BlockingReader)
    fake_serial = _FakeSerial()
    worker = SerialWorker(
        _config(),
        lambda _packet_value: None,
        serial_factory=lambda **_kwargs: fake_serial,
    )

    worker.start()
    assert _BlockingReader.entered.wait(1.0)
    assert worker.send_command(BIAS_COMMAND, timeout_sec=0.01) is False
    _BlockingReader.release.set()
    _wait_for(lambda: len(fake_serial.writes) >= 1)
    time.sleep(0.02)
    worker.stop()

    assert fake_serial.writes == [b"f500\n"]


def test_reconnects_after_open_failure(monkeypatch: Any) -> None:
    monkeypatch.setattr(worker_module, "PTSProtocolReader", _FakeReader)
    fake_serial = _FakeSerial()
    attempts = 0

    def serial_factory(**_kwargs: object) -> _FakeSerial:
        nonlocal attempts
        attempts += 1
        if attempts == 1:
            raise OSError("设备暂时不存在")
        return fake_serial

    received = threading.Event()
    worker = SerialWorker(_config(), lambda _packet_value: received.set(), serial_factory)

    worker.start()
    assert received.wait(1.0)
    worker.stop()

    assert attempts >= 2
    assert fake_serial.writes[0] == b"f500\n"
    assert fake_serial.closed


def _capture_first_open_error(
    serial_factory: Any, port: str = "/dev/fake"
) -> tuple[SerialWorker, list[str]]:
    """启动一个打开必失败的工作线程并等待首条错误日志。

    Args:
        serial_factory: 抛出打开异常的串口工厂。
        port: 待打开的串口路径, 决定错误消息的分类形态。

    Returns:
        (工作线程对象, 已捕获的错误消息列表); 调用方负责 ``stop()``。
    """
    errors: list[str] = []
    config = SerialWorkerConfig(
        port=port,
        baud_rate=115200,
        sampling_rate=500,
        timeout_sec=0.05,
        reconnect_initial_delay_sec=0.01,
        reconnect_max_delay_sec=0.02,
    )
    worker = SerialWorker(
        config,
        lambda _packet_value: None,
        serial_factory,
        on_error=errors.append,
    )
    worker.start()
    _wait_for(lambda: bool(errors))
    return worker, errors


def test_open_failure_with_missing_kernel_tty_lists_available_ports() -> None:
    """ttyACM 编号不存在时应列出系统可用串口, 提示 com_port 参数可能写错。"""

    def serial_factory(**_kwargs: object) -> _FakeSerial:
        raise OSError(errno.ENOENT, "No such file or directory: '/dev/ttyACM9'")

    worker, errors = _capture_first_open_error(serial_factory, port="/dev/ttyACM9")
    worker.stop()

    assert "串口设备 /dev/ttyACM9 不存在" in errors[0]
    assert "com_port" in errors[0]
    assert "系统当前可用: " in errors[0]


def test_open_failure_with_missing_alias_hints_udev_install() -> None:
    """udev 别名不存在时应提示设备未连接或规则未安装两种可能。"""

    def serial_factory(**_kwargs: object) -> _FakeSerial:
        raise OSError(errno.ENOENT, "No such file or directory: '/dev/papillarray'")

    worker, errors = _capture_first_open_error(serial_factory, port="/dev/papillarray")
    worker.stop()

    assert "串口别名 /dev/papillarray 不存在" in errors[0]
    assert "udev 规则未安装" in errors[0]
    assert "系统当前可用: " in errors[0]


def test_open_failure_with_missing_by_id_reports_disconnected() -> None:
    """by-id 路径不存在时应提示设备未连接, 而非参数写错。"""
    by_id_port = "/dev/serial/by-id/usb-Contactile_Controller_v2.0-if00"

    def serial_factory(**_kwargs: object) -> _FakeSerial:
        raise OSError(errno.ENOENT, "No such file or directory")

    worker, errors = _capture_first_open_error(serial_factory, port=by_id_port)
    worker.stop()

    assert "设备未连接" in errors[0]
    assert "com_port" not in errors[0]


def test_open_failure_with_permission_error_suggests_dialout_group() -> None:
    """无权访问串口时应提示检查 dialout 组。"""

    def serial_factory(**_kwargs: object) -> _FakeSerial:
        raise OSError(errno.EACCES, "Permission denied")

    worker, errors = _capture_first_open_error(serial_factory)
    worker.stop()

    assert "串口 /dev/fake 打开失败" in errors[0]
    assert "dialout" in errors[0]


def test_open_failure_with_busy_error_suggests_checking_occupancy() -> None:
    """端口被占用时应提示用 lsof 查看占用进程。"""

    def serial_factory(**_kwargs: object) -> _FakeSerial:
        raise OSError(errno.EBUSY, "Device or resource busy")

    worker, errors = _capture_first_open_error(serial_factory)
    worker.stop()

    assert "占用" in errors[0]
    assert "lsof" in errors[0]


def test_open_failure_with_unknown_error_keeps_original_message() -> None:
    """未分类的打开失败应保持原有错误文案, 不丢原始信息。"""

    def serial_factory(**_kwargs: object) -> _FakeSerial:
        raise OSError("设备暂时不存在")

    worker, errors = _capture_first_open_error(serial_factory)
    worker.stop()

    assert errors[0] == "串口连接中断: 设备暂时不存在"


def test_stream_failure_keeps_plain_disconnect_message(monkeypatch: Any) -> None:
    """打开成功后的读写中断应保持原有断开文案, 不套用打开失败分类。"""

    class _FailingReader:
        def __init__(self, _serial_port: object, _max_packet_bytes: int) -> None:
            pass

        def read_packet(self) -> ParsedPacket:
            raise OSError("读流中断")

    monkeypatch.setattr(worker_module, "PTSProtocolReader", _FailingReader)
    fake_serial = _FakeSerial()
    errors: list[str] = []
    worker = SerialWorker(
        _config(),
        lambda _packet_value: None,
        serial_factory=lambda **_kwargs: fake_serial,
        on_error=errors.append,
    )
    worker.start()
    _wait_for(lambda: bool(errors))
    worker.stop()

    assert errors[0] == "串口连接中断: 读流中断"
    assert fake_serial.closed


def test_connect_flushes_stale_input_before_sampling_rate_command(
    monkeypatch: Any,
) -> None:
    """连接后应先清空接收缓冲再配置采样率，旧会话字节不进入本次数据流。"""
    monkeypatch.setattr(worker_module, "PTSProtocolReader", _FakeReader)
    fake_serial = _FakeSerial()
    worker = SerialWorker(
        _config(), lambda _packet_value: None, serial_factory=lambda **_kwargs: fake_serial
    )

    worker.start()
    _wait_for(lambda: worker.is_connected)
    worker.stop()

    assert fake_serial.events[:2] == [("flush", None), ("write", b"f500\n")]


def test_bias_command_clears_input_and_protocol_buffers() -> None:
    """bias 写入前应清空串口接收缓冲与协议半包，并重置连续性基线。"""
    worker = SerialWorker(_config(), lambda _packet_value: None)
    worker._track_continuity(_packet(packet_counter=10, timestamp_us=100))
    fake_serial = _FakeSerial()
    reader = _StubReader()
    worker._commands.put(worker_module._PendingCommand(BIAS_COMMAND, threading.Event()))
    worker._drain_commands(fake_serial, reader)

    assert fake_serial.events == [("flush", None), ("write", b"z\n")]
    assert reader.reset_buffer_calls == 1
    assert worker._last_packet_counter is None
    # bias 后首包重新建立基线，不与归零前的计数和时间戳比较。
    worker._track_continuity(_packet(packet_counter=12, timestamp_us=1))
    assert worker.dropped_packets == 0
    assert worker.non_monotonic_timestamps == 0


def _continuity_worker() -> tuple[SerialWorker, list[str]]:
    """构造未启动的工作线程并捕获告警，供连续性监督的纯逻辑测试使用。"""
    warnings: list[str] = []
    worker = SerialWorker(_config(), lambda _packet_value: None, on_warning=warnings.append)
    return worker, warnings


def test_first_packet_only_establishes_baseline() -> None:
    worker, warnings = _continuity_worker()

    worker._track_continuity(_packet(packet_counter=7, timestamp_us=42))

    assert worker.dropped_packets == 0
    assert worker.disordered_packets == 0
    assert worker.non_monotonic_timestamps == 0
    assert not warnings


def test_forward_gap_counts_missing_packets_without_warning() -> None:
    worker, warnings = _continuity_worker()

    worker._track_continuity(_packet(packet_counter=5, timestamp_us=100))
    worker._track_continuity(_packet(packet_counter=8, timestamp_us=104))

    assert worker.dropped_packets == 2
    assert not warnings


def test_counter_wraparound_is_forward_not_massive_loss() -> None:
    """uint32 自然回绕应视为连续前进，不误报巨量丢包。"""
    worker, _warnings = _continuity_worker()

    worker._track_continuity(_packet(packet_counter=0xFFFFFFFF, timestamp_us=1))
    worker._track_continuity(_packet(packet_counter=0, timestamp_us=2))

    assert worker.dropped_packets == 0
    assert worker.disordered_packets == 0


def test_duplicate_counter_is_disorder_event() -> None:
    worker, warnings = _continuity_worker()

    worker._track_continuity(_packet(packet_counter=6, timestamp_us=10))
    worker._track_continuity(_packet(packet_counter=6, timestamp_us=11))

    assert worker.disordered_packets == 1
    assert any("重复" in message for message in warnings)


def test_counter_backwards_jump_resets_baseline_like_restart() -> None:
    """计数器回退按设备重启处理：告警一次并把基线移到当前值。"""
    worker, warnings = _continuity_worker()

    worker._track_continuity(_packet(packet_counter=100, timestamp_us=10))
    worker._track_continuity(_packet(packet_counter=90, timestamp_us=11))
    worker._track_continuity(_packet(packet_counter=91, timestamp_us=12))

    assert worker.disordered_packets == 1
    assert worker.dropped_packets == 0
    assert any("回退" in message for message in warnings)


def test_large_gap_logs_warning() -> None:
    worker, warnings = _continuity_worker()

    worker._track_continuity(_packet(packet_counter=0, timestamp_us=10))
    worker._track_continuity(_packet(packet_counter=12, timestamp_us=30))

    assert worker.dropped_packets == 11
    assert any("缺 11 包" in message for message in warnings)


def test_non_increasing_device_timestamp_is_reported() -> None:
    """设备时间戳回退或停滞都应告警，与实机验证使用的判据一致。"""
    worker, warnings = _continuity_worker()

    worker._track_continuity(_packet(packet_counter=1, timestamp_us=500))
    worker._track_continuity(_packet(packet_counter=2, timestamp_us=499))
    worker._track_continuity(_packet(packet_counter=3, timestamp_us=499))

    assert worker.non_monotonic_timestamps == 2
    assert any("未递增" in message for message in warnings)
