#!/usr/bin/env python3
"""Contactile PapillArray (PTS v2.0) 串口协议的纯 Python 实现。

为什么自己写: 原厂 cp310 wheel (ptsdk 1.0.2) 受 Python 3.10 约束，且 pybind/native
层不便调试。本模块按官方协议 PTSCOM_2.0_SPEC 直接解析 USB 串口字节流，可绕开 wheel
版本约束并做协议级回归测试。此前关于 Controller v2.0 上 SDK 崩溃的判断已修正，真实
原因是 hub 输出 SEN0/SEN1 两路时调用方只注册了一个 PTSDKSensor；注册两路后官方 SDK 可读。

数据包结构 (小端):
    [起始 55 66 77 88] [Index Size:1] [数据索引表] [各数据块...] [校验和:2] [结束 AA BB CC DD]

数据索引表: 若干 (Type:2, Offset:IndexSize) 项，Offset 为相对"起始字节之后第 0 字节"的索引。
    Type 1 = Hub (PacketCounter:4 + Timestamp_us:8)
    Type 3 = Resolved Pillar Data
    Type 4 = Resolved Global Data
    Type 5 = Pillar 滑动状态与摩擦估计
    Type 6 = Sensor 滑动检测状态、摩擦估计与目标抓握力
    Type 7 = 此版本厂商 SDK 未定义；以原始字节保留，避免猜测其布局

Resolved Pillar Data 块: Ns(2) + [sensor 偏移表 Ns*ISZ] + 每 sensor{ Np(2) +
    [pillar 偏移表 Np*ISZ] + 每 pillar( Type 0x02:2 + Fx,Fy,Fz,Dx,Dy,Dz:6*float32 ) }
Resolved Global Data 块: Ns(2) + [sensor 偏移表 Ns*ISZ] + 每 sensor( Type 0x02:2 +
    Fx,Fy,Fz,Tx,Ty,Tz:6*float32 )
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field

import numpy as np

_START = bytes([0x55, 0x66, 0x77, 0x88])
_END = bytes([0xAA, 0xBB, 0xCC, 0xDD])
_TYPE_HUB = 1
_TYPE_PILLAR = 3
_TYPE_GLOBAL = 4
_TYPE_PILLAR_SLIP = 5
_TYPE_SENSOR_SLIP = 6
_TYPE_RESERVED = 7
_NDIM = 3


@dataclass(frozen=True)
class ParsedPacket:
    """从一个数据包解析出的内容 (可能含 1~2 个 sensor)。

    Args:
        packet_counter: 控制器递增包计数。
        timestamp_us: 控制器时间戳，单位 us。
        pillar_forces: 每 sensor 的 pillar 力，list[np.ndarray shape=(Np,3)]，单位 N。
        pillar_displacements: 每 sensor 的 pillar 位移，list[np.ndarray shape=(Np,3)]，单位 mm。
        global_forces: 每 sensor 全局力，list[np.ndarray shape=(3,)]，单位 N。
        global_torques: 每 sensor 全局力矩，list[np.ndarray shape=(3,)]，单位 N·mm。
        pillar_slip_states: 每 sensor 的 pillar 滑动状态，list[np.ndarray shape=(Np,)]。
        pillar_friction_estimates: 每 sensor 的 pillar 摩擦估计，list[np.ndarray shape=(Np,)]。
        slip_detection_active: 每 sensor 的滑动检测是否激活，list[bool]。
        reference_pillar_loaded: 每 sensor 的参考 pillar 是否已加载，list[bool]。
        sensor_friction_estimates: 每 sensor 的摩擦估计，list[float]，无估计时通常为 -1。
        target_grip_forces: 每 sensor 防滑目标抓握力，list[float]，单位 N，无估计时通常为 -1。
        type_7_data: Type 7 块的原始字节；厂商 v2.0 SDK 未定义其布局时为 ``bytes``，
            包中没有该块时为 ``None``。
    """

    packet_counter: int
    timestamp_us: int
    pillar_forces: list[np.ndarray]
    pillar_displacements: list[np.ndarray]
    global_forces: list[np.ndarray]
    global_torques: list[np.ndarray]
    pillar_slip_states: list[np.ndarray] = field(default_factory=list)
    pillar_friction_estimates: list[np.ndarray] = field(default_factory=list)
    slip_detection_active: list[bool] = field(default_factory=list)
    reference_pillar_loaded: list[bool] = field(default_factory=list)
    sensor_friction_estimates: list[float] = field(default_factory=list)
    target_grip_forces: list[float] = field(default_factory=list)
    type_7_data: bytes | None = None

    @property
    def n_sensors(self) -> int:
        return len(self.pillar_forces)


def _u16(buf: bytes, p: int) -> int:
    return int.from_bytes(buf[p : p + 2], "little")


def verify_checksum(data: bytes) -> bool:
    """校验和: data 除最后 2 字节外按字节求和，截断为 2 字节无符号，与末 2 字节比较。

    Args:
        data: 起始与结束标记之间的全部字节 (含末尾 2 字节校验和)。
    """
    return (sum(data[:-2]) & 0xFFFF) == _u16(data, len(data) - 2)


def parse_packet(data: bytes) -> ParsedPacket:
    """解析单个数据包 (起始/结束标记之间、含校验和的字节)。

    Args:
        data: frame[4:-4]，即去掉 4 字节起始与 4 字节结束标记后的内容。

    Returns:
        ParsedPacket。

    Raises:
        ValueError: 索引表缺少 pillar/global 块或结构异常。
    """
    isz = data[0]
    # 解析数据索引表，得到各块在 data 中的起始偏移
    offsets: dict[int, int] = {}
    pos = 1
    min_off = len(data)
    while pos + 2 + isz <= len(data) and pos < min_off:
        typ = _u16(data, pos)
        off = int.from_bytes(data[pos + 2 : pos + 2 + isz], "little")
        if typ == 0 or typ > 0x20:
            break
        offsets[typ] = off
        min_off = min(min_off, off)
        pos += 2 + isz

    counter, timestamp = 0, 0
    if _TYPE_HUB in offsets:
        h = offsets[_TYPE_HUB]
        counter = int.from_bytes(data[h : h + 4], "little")
        timestamp = int.from_bytes(data[h + 4 : h + 12], "little")

    if _TYPE_PILLAR not in offsets or _TYPE_GLOBAL not in offsets:
        raise ValueError("数据包缺少 pillar 或 global 块")

    forces, disps = _parse_pillar_block(data, offsets[_TYPE_PILLAR], isz)
    gforces, gtorques = _parse_global_block(data, offsets[_TYPE_GLOBAL], isz)
    slip_states, pillar_friction = (
        _parse_pillar_slip_block(data, offsets[_TYPE_PILLAR_SLIP], isz)
        if _TYPE_PILLAR_SLIP in offsets
        else ([], [])
    )
    slip_active, ref_pillar_loaded, sensor_friction, target_grip = (
        _parse_sensor_slip_block(data, offsets[_TYPE_SENSOR_SLIP], isz)
        if _TYPE_SENSOR_SLIP in offsets
        else ([], [], [], [])
    )
    type_7_data = _extract_block(data, offsets, _TYPE_RESERVED)

    return ParsedPacket(
        counter,
        timestamp,
        forces,
        disps,
        gforces,
        gtorques,
        slip_states,
        pillar_friction,
        slip_active,
        ref_pillar_loaded,
        sensor_friction,
        target_grip,
        type_7_data,
    )


def _parse_pillar_block(
    data: bytes, off: int, isz: int
) -> tuple[list[np.ndarray], list[np.ndarray]]:
    """顺序解析 Resolved Pillar Data 块 (布局连续，按 Ns/Np 顺序读即可)。"""
    p = off
    n_sensors = _u16(data, p)
    p += 2 + n_sensors * isz  # 跳过 sensor 偏移表
    all_forces, all_disps = [], []
    for _ in range(n_sensors):
        n_pillars = _u16(data, p)
        p += 2 + n_pillars * isz  # 跳过 pillar 偏移表
        f = np.empty((n_pillars, _NDIM), dtype=np.float64)
        d = np.empty((n_pillars, _NDIM), dtype=np.float64)
        for k in range(n_pillars):
            p += 2  # 跳过 pillar Type (0x0002)
            fx, fy, fz, dx, dy, dz = struct.unpack_from("<6f", data, p)
            p += 24
            d[k] = (dx, dy, dz)
            f[k] = (fx, fy, fz)
        all_forces.append(f)
        all_disps.append(d)
    return all_forces, all_disps


def _parse_global_block(
    data: bytes, off: int, isz: int
) -> tuple[list[np.ndarray], list[np.ndarray]]:
    """顺序解析 Resolved Global Data 块。"""
    p = off
    n_sensors = _u16(data, p)
    p += 2 + n_sensors * isz  # 跳过 sensor 偏移表
    gforces, gtorques = [], []
    for _ in range(n_sensors):
        p += 2  # 跳过 Type (0x0002)
        fx, fy, fz, tx, ty, tz = struct.unpack_from("<6f", data, p)
        p += 24
        gforces.append(np.array([fx, fy, fz], dtype=np.float64))
        gtorques.append(np.array([tx, ty, tz], dtype=np.float64))
    return gforces, gtorques


def _parse_pillar_slip_block(
    data: bytes, off: int, isz: int
) -> tuple[list[np.ndarray], list[np.ndarray]]:
    """解析 Type 5 的每 pillar 滑动状态与摩擦估计。"""
    p = off
    n_sensors = _u16(data, p)
    p += 2 + n_sensors * isz  # 偏移表仅用于随机访问，连续读取可避免重复计算基址。
    all_states, all_friction = [], []
    for _ in range(n_sensors):
        n_pillars = _u16(data, p)
        p += 2 + n_pillars * isz
        states = np.empty(n_pillars, dtype=np.int8)
        friction = np.empty(n_pillars, dtype=np.float64)
        for pillar_index in range(n_pillars):
            entry_type = _u16(data, p)
            p += 2
            if entry_type != 1:
                raise ValueError(f"不支持的 Type 5 pillar 条目类型: {entry_type}")
            states[pillar_index] = struct.unpack_from("<b", data, p)[0]
            friction[pillar_index] = struct.unpack_from("<f", data, p + 1)[0]
            p += 5
        all_states.append(states)
        all_friction.append(friction)
    return all_states, all_friction


def _parse_sensor_slip_block(
    data: bytes, off: int, isz: int
) -> tuple[list[bool], list[bool], list[float], list[float]]:
    """解析 Type 6 的传感器滑动检测状态、摩擦估计与目标抓握力。"""
    p = off
    n_sensors = _u16(data, p)
    p += 2 + n_sensors * isz  # 与厂商 SDK 一致，按传感器块的连续布局读取。
    active, reference_loaded, friction, target_grip = [], [], [], []
    for _ in range(n_sensors):
        entry_type = _u16(data, p)
        p += 2
        if entry_type != 1:
            raise ValueError(f"不支持的 Type 6 sensor 条目类型: {entry_type}")
        state = data[p]
        p += 1
        friction_estimate, target_grip_force = struct.unpack_from("<2f", data, p)
        p += 8
        active.append(state != 0)
        reference_loaded.append(state == 2)
        friction.append(float(friction_estimate))
        target_grip.append(float(target_grip_force))
    return active, reference_loaded, friction, target_grip


def _extract_block(data: bytes, offsets: dict[int, int], block_type: int) -> bytes | None:
    """按索引表边界提取未定义块，供调用方记录或后续实现解码。"""
    if block_type not in offsets:
        return None
    start = offsets[block_type]
    following_offsets = [offset for offset in offsets.values() if offset > start]
    end = min(following_offsets, default=len(data) - 2)  # 不把帧校验和误交给未知块。
    return data[start:end]


class PTSProtocolReader:
    """从已打开的串口对象按帧读取并校验 PTS 数据包。

    Args:
        ser: 已打开的 pyserial Serial 对象。
        max_packet: 单包字节上限，防异常流导致缓冲无限增长。
    """

    def __init__(self, ser, max_packet: int = 8192) -> None:
        self._ser = ser
        self._buf = bytearray()
        self._max_packet = max_packet

    def reset_buffer(self) -> None:
        """清空协议层残留字节，常用于 bias 后丢弃旧半包。"""
        self._buf.clear()

    def read_packet(self) -> ParsedPacket:
        """阻塞读取下一个校验通过的数据包并解析。

        Raises:
            TimeoutError: 串口读不到足够数据 (检查供电/接线)。
        """
        while True:
            # 定位起始标记
            si = self._buf.find(_START)
            if si >= 0:
                ei = self._buf.find(_END, si + 4)
                if ei >= 0:
                    frame_data = bytes(self._buf[si + 4 : ei])  # 含校验和, 不含标记
                    del self._buf[: ei + 4]
                    if verify_checksum(frame_data):
                        return parse_packet(frame_data)
                    continue  # 校验失败，丢弃继续找下一帧
                if len(self._buf) - si > self._max_packet:
                    del self._buf[: si + 4]  # 半包过长，丢弃残片防膨胀
            chunk = self._ser.read(4096)
            if not chunk:
                raise TimeoutError("串口超时无数据，请检查传感器供电/接线")
            self._buf += chunk
