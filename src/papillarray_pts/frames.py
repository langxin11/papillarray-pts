"""PTS 帧数据模型与采集会话生成器。

``ParsedPacket`` 是协议层的原始形状， ``PTSFrame`` 在其上提供逐传感器的
物理量视图（位移 mm / 力 N / 力矩 N·mm，sensor frame），供可视化、记录
与算法消费；``iter_sensor_frames`` 把"设备等待 → 开串口 → 清积压 → 逐包
解析"的采集会话收敛为单一实现，避免各入口复制粘贴时漏改（历史上积压
清理就因此漏改过两个可视化入口）。本模块只依赖 numpy 与 pyserial，
与库其余部分一致，不引入渲染或 ROS 概念。
"""

from __future__ import annotations

import time
import warnings
from collections.abc import Callable, Iterator
from dataclasses import dataclass

import numpy as np

from .protocol import PTSProtocolReader
from .serial_worker import (
    BACKLOG_DRAIN_GRACE_SEC,
    DEFAULT_DEVICE_WAIT_SEC,
    SerialPort,
    _default_serial_factory,
    wait_for_serial_device,
)

# PTS v2.0 阵列为固定 3x3 pillar，异常个数在帧转换层占位告警
PTS_N_PILLARS = 9
DEFAULT_BAUD_RATE = 115200
DEFAULT_TIMEOUT_SEC = 1.0


@dataclass(frozen=True)
class PTSFrame:
    """单个传感器单帧数据（与渲染无关的输入）。

    Attributes:
        timestamp_us: 控制器时间戳，单位 us。
        displacements: pillar 位移，单位 mm，sensor frame，shape=(9, 3)。
        forces: pillar 力，单位 N，sensor frame，shape=(9, 3)。
        global_force: 全局力，单位 N，sensor frame，shape=(3,)。
        global_torque: 全局力矩，单位 N·mm，sensor frame，shape=(3,)。
    """

    timestamp_us: int
    displacements: np.ndarray
    forces: np.ndarray
    global_force: np.ndarray
    global_torque: np.ndarray

    def __post_init__(self) -> None:
        """统一 dtype 并校验 shape，尽早暴露协议解析错位。"""
        for name, shape in (
            ("displacements", (PTS_N_PILLARS, 3)),
            ("forces", (PTS_N_PILLARS, 3)),
            ("global_force", (3,)),
            ("global_torque", (3,)),
        ):
            value = np.asarray(getattr(self, name), dtype=np.float64)
            if value.shape != shape:
                raise ValueError(f"{name} shape 应为 {shape}，实际为 {value.shape}")
            object.__setattr__(self, name, value)


def frames_from_packet(packet: object) -> list[PTSFrame | None]:
    """把 ``ParsedPacket`` 拆成每个传感器一帧。

    Args:
        packet: ``ParsedPacket``，用鸭子类型以免本模块反向依赖协议包定义。

    Returns:
        每个传感器一个条目，与传感器索引位置对齐；pillar 数不为 9 的
        传感器对应位置为 ``None`` 占位。

    Raises:
        RuntimeWarning: 某个传感器 pillar 数异常时通过 ``warnings`` 告警。
    """
    frames: list[PTSFrame | None] = []
    for disp, force, gforce, gtorque in zip(
        packet.pillar_displacements,
        packet.pillar_forces,
        packet.global_forces,
        packet.global_torques,
        strict=True,
    ):
        if np.shape(disp)[0] != PTS_N_PILLARS:
            # 协议错位时占位 None 而不是删除元素：调用方按位置 zip 配对
            # entity_path/topic，删除元素会把后续传感器数据错配到前一个通道
            warnings.warn(
                f"跳过 pillar 数异常的传感器（期望 {PTS_N_PILLARS}，"
                f"实际 {np.shape(disp)[0]}），该通道本轮不发布",
                RuntimeWarning,
                stacklevel=2,
            )
            frames.append(None)
            continue
        frames.append(PTSFrame(packet.timestamp_us, disp, force, gforce, gtorque))
    return frames


def iter_sensor_frames(
    port: str,
    baud_rate: int = DEFAULT_BAUD_RATE,
    timeout_sec: float = DEFAULT_TIMEOUT_SEC,
    *,
    max_packet_bytes: int = 8192,
    wait_sec: float = DEFAULT_DEVICE_WAIT_SEC,
    backlog_drain_grace_sec: float = BACKLOG_DRAIN_GRACE_SEC,
    serial_factory: Callable[..., SerialPort] = _default_serial_factory,
) -> Iterator[list[PTSFrame | None]]:
    """打开串口并逐包产出各传感器的帧列表，覆盖完整采集会话生命周期。

    统一封装设备节点等待（USB 枚举延迟）、积压旧帧清理与串口释放；
    生成器被关闭或耗尽时在 ``finally`` 中关闭串口。本生成器只读不发
    控制命令，采样率沿用设备当前设置；需要下发采样率/bias 命令或自动
    重连时使用 ``SerialWorker``。

    Args:
        port: 串口设备路径，如 /dev/ttyACM0。
        baud_rate: 串口波特率，单位 baud。
        timeout_sec: 单次串口读取超时，单位 s，超时以 ``TimeoutError`` 上抛。
        max_packet_bytes: 单个协议帧允许的最大字节数。
        wait_sec: 等待设备节点出现的总时长，单位 s。
        backlog_drain_grace_sec: 打开串口后等待设备端积压涌入内核再清空的
            静默时间，单位 s；0 表示跳过该防御。
        serial_factory: 串口工厂，测试时可替换为 fake serial。

    Yields:
        每个数据包对应的帧列表，与传感器索引位置对齐（异常传感器为
        ``None`` 占位），语义同 ``frames_from_packet``。

    Raises:
        FileNotFoundError: 设备节点在 ``wait_sec`` 内未出现。
        TimeoutError: 串口读取超时（检查供电/接线）。
        serial.SerialException: 串口打开或读写失败原样上抛，由调用方分类处理。
    """
    if not wait_for_serial_device(port, wait_sec):
        raise FileNotFoundError(f"串口设备 {port} 在 {wait_sec:.1f} s 内未出现，请检查连接和权限")
    serial_port = serial_factory(port=port, baudrate=baud_rate, timeout=timeout_sec)
    try:
        if backlog_drain_grace_sec > 0:
            # 旧会话的帧滞留在设备端发送缓冲（实测可达 40 s 前），打开后几 ms
            # 才涌入内核，立即 flush 清不到；静默后再清空才有效（2026-10 实测）
            time.sleep(backlog_drain_grace_sec)
            serial_port.reset_input_buffer()
        reader = PTSProtocolReader(serial_port, max_packet_bytes)
        while True:
            yield frames_from_packet(reader.read_packet())
    finally:
        serial_port.close()
