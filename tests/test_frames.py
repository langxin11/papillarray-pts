#!/usr/bin/env python3
"""帧模型与采集会话生成器的离线测试。

字节流全部由 test_protocol 的构造器生成，串口用 fake 替身注入，
不依赖真实硬件即可回归"等待 → 清积压 → 逐包产帧 → 释放"的会话语义。
"""

from __future__ import annotations

from typing import Any

import numpy as np
import pytest
from test_protocol import _build_frame, _build_packet_data

from papillarray_pts.frames import (
    PTS_N_PILLARS,
    PTSFrame,
    frames_from_packet,
    iter_sensor_frames,
)
from papillarray_pts.protocol import parse_packet


def _sensor_spec(pillar_count: int = 9, fill: float = 0.0) -> dict[str, Any]:
    """构造单个 sensor 的协议块内容；pillar_count 用于模拟异常传感器。"""
    return {
        "pillars": [[fill + 0.01 * i] * 6 for i in range(pillar_count)],
        "global": [fill, fill, fill, 0.0, 0.0, 0.0],
    }


class _FakeSerial:
    """按块吐出预置字节流并记录积压清理/关闭调用的串口替身。"""

    def __init__(self, chunks: list[bytes]) -> None:
        self._chunks = list(chunks)
        self.reset_input_buffer_calls = 0
        self.closed = False


class _FakeSerial:
    """按块吐出预置字节流并记录积压清理/关闭调用的串口替身。

    Args:
        chunks: 打开时内核缓冲中已积压的字节（旧会话数据）。
        after_reset_chunks: reset_input_buffer 之后设备继续流入的字节，
            用于模拟"清空积压后才到达的新会话数据"。
    """

    def __init__(self, chunks: list[bytes], after_reset_chunks: list[bytes] | None = None) -> None:
        self._chunks = list(chunks)
        self._after_reset_chunks = list(after_reset_chunks or [])
        self.reset_input_buffer_calls = 0
        self.closed = False

    def read(self, _size: int) -> bytes:
        if self._chunks:
            return self._chunks.pop(0)
        return b""

    def write(self, data: bytes) -> int:
        return len(data)

    def flush(self) -> None:
        pass

    def reset_input_buffer(self) -> None:
        # 对齐真串口语义：清空丢弃内核接收缓冲中的待读字节，之后才轮到新数据
        self._chunks.clear()
        self._chunks.extend(self._after_reset_chunks)
        self._after_reset_chunks.clear()
        self.reset_input_buffer_calls += 1

    def close(self) -> None:
        self.closed = True


def _fake_factory(
    chunks: list[bytes],
    after_reset_chunks: list[bytes] | None = None,
) -> tuple[list[_FakeSerial], Any]:
    """返回记录实例的 serial_factory 替身，便于断言生成器对串口的操作。"""
    created: list[_FakeSerial] = []

    def factory(**_kwargs: Any) -> _FakeSerial:
        port = _FakeSerial(chunks, after_reset_chunks)
        created.append(port)
        return port

    return created, factory


def test_pts_frame_validates_and_normalizes_shape() -> None:
    """shape 不匹配应在构造时报错，合法输入统一为 float64。"""
    good = PTSFrame(
        timestamp_us=1,
        displacements=np.zeros((9, 3), dtype=np.int32),
        forces=np.zeros((9, 3)),
        global_force=np.zeros(3),
        global_torque=np.zeros(3),
    )
    assert good.displacements.dtype == np.float64
    with pytest.raises(ValueError, match="shape"):
        PTSFrame(
            timestamp_us=1,
            displacements=np.zeros((8, 3)),
            forces=np.zeros((9, 3)),
            global_force=np.zeros(3),
            global_torque=np.zeros(3),
        )


def test_frames_from_packet_keeps_sensor_alignment() -> None:
    """异常传感器占位 None，后续传感器不前移，保证调用方按位置配对安全。"""
    data = _build_packet_data(
        7,
        123_456,
        [_sensor_spec(pillar_count=8), _sensor_spec(fill=0.5)],
    )
    packet = parse_packet(data)
    with pytest.warns(RuntimeWarning, match="pillar 数异常"):
        frames = frames_from_packet(packet)
    assert len(frames) == 2
    assert frames[0] is None
    assert frames[1] is not None
    assert frames[1].timestamp_us == 123_456
    np.testing.assert_allclose(frames[1].global_force[:3], 0.5)
    assert frames[1].forces.shape == (PTS_N_PILLARS, 3)


def test_iter_sensor_frames_yields_packets_then_closes_port() -> None:
    """生成器应先清积压再逐包产帧，关闭生成器时释放串口。"""
    chunks = [
        _build_frame(_build_packet_data(1, 1_000, [_sensor_spec(fill=0.25)])),
        _build_frame(_build_packet_data(2, 3_000, [_sensor_spec(fill=0.5)])),
    ]
    created, factory = _fake_factory(chunks)
    gen = iter_sensor_frames(
        "/dev/papillarray",
        backlog_drain_grace_sec=0.0,
        serial_factory=factory,
    )
    first = next(gen)
    second = next(gen)
    assert first[0] is not None and first[0].timestamp_us == 1_000
    np.testing.assert_allclose(first[0].global_force[:3], 0.25)
    assert second[0] is not None and second[0].timestamp_us == 3_000
    gen.close()
    assert created and created[0].closed


def test_iter_sensor_frames_drains_backlog_before_parsing() -> None:
    """grace > 0 时应先 reset_input_buffer 再读数据，丢弃旧会话积压字节。"""
    stale = _build_frame(_build_packet_data(99, 900_000, [_sensor_spec(fill=9.0)]))
    fresh = _build_frame(_build_packet_data(1, 1_000, [_sensor_spec(fill=0.1)]))
    created, factory = _fake_factory([stale], after_reset_chunks=[fresh])
    gen = iter_sensor_frames(
        "/dev/papillarray",
        backlog_drain_grace_sec=0.001,
        serial_factory=factory,
    )
    frame_list = next(gen)
    gen.close()
    # 积压清理发生在读取之前，stale 帧被整体丢弃，首帧即本次会话数据
    assert created[0].reset_input_buffer_calls == 1
    assert frame_list[0] is not None
    assert frame_list[0].timestamp_us == 1_000


def test_iter_sensor_frames_raises_when_device_missing() -> None:
    """设备节点未出现时上抛 FileNotFoundError，由调用方决定提示方式。"""
    _, factory = _fake_factory([])
    with pytest.raises(FileNotFoundError, match="未出现"):
        next(
            iter_sensor_frames(
                "/dev/nonexistent-pts-test-port",
                wait_sec=0.0,
                serial_factory=factory,
            )
        )
