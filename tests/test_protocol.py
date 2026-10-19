#!/usr/bin/env python3
"""PTS 协议层离线测试。

为什么放在协议层测: 这里最容易因为偏移、半包或校验和细节出错，而这些问题用真机肉眼看输
出很难稳定复现。构造确定性的字节流后，才能把解析正确性固定成可回归的断言。
"""

from __future__ import annotations

import struct
import unittest

import numpy as np
import numpy.testing as npt

from papillarray_pts.protocol import (
    PTSProtocolReader,
    parse_packet,
    verify_checksum,
)

_START = bytes([0x55, 0x66, 0x77, 0x88])
_END = bytes([0xAA, 0xBB, 0xCC, 0xDD])
_TYPE_HUB = 1
_TYPE_PILLAR = 3
_TYPE_GLOBAL = 4
_PILLAR_ENTRY_TYPE = 0x0002
_GLOBAL_ENTRY_TYPE = 0x0002
_INDEX_SIZE = 2


def _with_checksum(payload: bytes) -> bytes:
    checksum = sum(payload) & 0xFFFF
    return payload + checksum.to_bytes(2, "little")


def _build_hub_block(packet_counter: int, timestamp_us: int) -> bytes:
    return packet_counter.to_bytes(4, "little") + timestamp_us.to_bytes(8, "little")


def _build_pillar_block(sensor_specs: list[dict[str, object]]) -> bytes:
    sensor_blocks = []
    sensor_offsets = []
    running_offset = 2 + len(sensor_specs) * _INDEX_SIZE

    for sensor_spec in sensor_specs:
        pillars = sensor_spec["pillars"]
        pillar_offsets = []
        sensor_payload = bytearray()
        sensor_payload.extend(len(pillars).to_bytes(2, "little"))
        sensor_payload.extend(b"\x00" * (len(pillars) * _INDEX_SIZE))

        pillar_running_offset = 2 + len(pillars) * _INDEX_SIZE
        for pillar in pillars:
            pillar_offsets.append(pillar_running_offset)
            sensor_payload.extend(_PILLAR_ENTRY_TYPE.to_bytes(2, "little"))
            sensor_payload.extend(struct.pack("<6f", *pillar))
            pillar_running_offset += 2 + 24

        for pillar_index, pillar_offset in enumerate(pillar_offsets):
            start = 2 + pillar_index * _INDEX_SIZE
            sensor_payload[start : start + _INDEX_SIZE] = pillar_offset.to_bytes(
                _INDEX_SIZE, "little"
            )

        sensor_offsets.append(running_offset)
        sensor_blocks.append(bytes(sensor_payload))
        running_offset += len(sensor_payload)

    payload = bytearray()
    payload.extend(len(sensor_specs).to_bytes(2, "little"))
    for sensor_offset in sensor_offsets:
        payload.extend(sensor_offset.to_bytes(_INDEX_SIZE, "little"))
    for sensor_block in sensor_blocks:
        payload.extend(sensor_block)
    return bytes(payload)


def _build_global_block(sensor_specs: list[dict[str, object]]) -> bytes:
    sensor_offsets = []
    sensor_blocks = []
    running_offset = 2 + len(sensor_specs) * _INDEX_SIZE

    for sensor_spec in sensor_specs:
        sensor_offsets.append(running_offset)
        sensor_payload = bytearray()
        sensor_payload.extend(_GLOBAL_ENTRY_TYPE.to_bytes(2, "little"))
        sensor_payload.extend(struct.pack("<6f", *sensor_spec["global"]))
        sensor_blocks.append(bytes(sensor_payload))
        running_offset += len(sensor_payload)

    payload = bytearray()
    payload.extend(len(sensor_specs).to_bytes(2, "little"))
    for sensor_offset in sensor_offsets:
        payload.extend(sensor_offset.to_bytes(_INDEX_SIZE, "little"))
    for sensor_block in sensor_blocks:
        payload.extend(sensor_block)
    return bytes(payload)


def _build_packet_data(
    packet_counter: int, timestamp_us: int, sensor_specs: list[dict[str, object]]
) -> bytes:
    hub_block = _build_hub_block(packet_counter, timestamp_us)
    pillar_block = _build_pillar_block(sensor_specs)
    global_block = _build_global_block(sensor_specs)

    index_table_size = 1 + 3 * (2 + _INDEX_SIZE)
    hub_offset = index_table_size
    pillar_offset = hub_offset + len(hub_block)
    global_offset = pillar_offset + len(pillar_block)

    payload = bytearray()
    payload.append(_INDEX_SIZE)
    payload.extend(_TYPE_HUB.to_bytes(2, "little"))
    payload.extend(hub_offset.to_bytes(_INDEX_SIZE, "little"))
    payload.extend(_TYPE_PILLAR.to_bytes(2, "little"))
    payload.extend(pillar_offset.to_bytes(_INDEX_SIZE, "little"))
    payload.extend(_TYPE_GLOBAL.to_bytes(2, "little"))
    payload.extend(global_offset.to_bytes(_INDEX_SIZE, "little"))
    payload.extend(hub_block)
    payload.extend(pillar_block)
    payload.extend(global_block)
    return _with_checksum(bytes(payload))


def _build_frame(packet_data: bytes) -> bytes:
    return _START + packet_data + _END


class _FakeSerial:
    """用固定字节块模拟串口 read()，避免协议测试依赖真实硬件。"""

    def __init__(self, chunks: list[bytes]) -> None:
        self._chunks = list(chunks)

    def read(self, _size: int) -> bytes:
        if self._chunks:
            return self._chunks.pop(0)
        return b""


class VerifyChecksumTest(unittest.TestCase):
    def test_accepts_valid_checksum_and_rejects_corrupted_payload(self) -> None:
        packet_data = _build_packet_data(
            packet_counter=7,
            timestamp_us=123456,
            sensor_specs=[
                {
                    "pillars": [
                        (1.0, 2.0, 3.0, 0.1, 0.2, 0.3),
                    ],
                    "global": (4.0, 5.0, 6.0, 0.4, 0.5, 0.6),
                }
            ],
        )

        self.assertTrue(verify_checksum(packet_data))

        corrupted = bytearray(packet_data)
        corrupted[5] ^= 0x01
        self.assertFalse(verify_checksum(bytes(corrupted)))


class ParsePacketTest(unittest.TestCase):
    def test_parses_single_sensor_packet(self) -> None:
        packet = parse_packet(
            _build_packet_data(
                packet_counter=42,
                timestamp_us=987654321,
                sensor_specs=[
                    {
                        "pillars": [
                            (1.0, 2.0, 3.0, 0.1, 0.2, 0.3),
                            (4.0, 5.0, 6.0, 0.4, 0.5, 0.6),
                        ],
                        "global": (7.0, 8.0, 9.0, 0.7, 0.8, 0.9),
                    }
                ],
            )
        )

        self.assertEqual(packet.packet_counter, 42)
        self.assertEqual(packet.timestamp_us, 987654321)
        self.assertEqual(packet.n_sensors, 1)
        self.assertEqual(packet.pillar_forces[0].shape, (2, 3))
        self.assertEqual(packet.pillar_displacements[0].shape, (2, 3))
        npt.assert_allclose(packet.pillar_forces[0][0], np.array([1.0, 2.0, 3.0]))
        npt.assert_allclose(packet.pillar_displacements[0][1], np.array([0.4, 0.5, 0.6]))
        npt.assert_allclose(packet.global_forces[0], np.array([7.0, 8.0, 9.0]))
        npt.assert_allclose(packet.global_torques[0], np.array([0.7, 0.8, 0.9]))

    def test_parses_two_sensor_packet(self) -> None:
        packet = parse_packet(
            _build_packet_data(
                packet_counter=99,
                timestamp_us=555000,
                sensor_specs=[
                    {
                        "pillars": [
                            (1.0, 1.1, 1.2, 0.01, 0.02, 0.03),
                        ],
                        "global": (2.0, 2.1, 2.2, 0.21, 0.22, 0.23),
                    },
                    {
                        "pillars": [
                            (3.0, 3.1, 3.2, 0.31, 0.32, 0.33),
                            (4.0, 4.1, 4.2, 0.41, 0.42, 0.43),
                        ],
                        "global": (5.0, 5.1, 5.2, 0.51, 0.52, 0.53),
                    },
                ],
            )
        )

        self.assertEqual(packet.n_sensors, 2)
        self.assertEqual(packet.pillar_forces[0].shape, (1, 3))
        self.assertEqual(packet.pillar_forces[1].shape, (2, 3))
        npt.assert_allclose(packet.pillar_forces[1][1], np.array([4.0, 4.1, 4.2]))
        npt.assert_allclose(packet.global_forces[0], np.array([2.0, 2.1, 2.2]))
        npt.assert_allclose(packet.global_torques[1], np.array([0.51, 0.52, 0.53]))

    def test_parses_slip_blocks_and_preserves_type_7_data(self) -> None:
        """厂商 SDK 已定义 Type 5/6；未知 Type 7 必须保留而不能臆测解码。"""
        hub_block = _build_hub_block(12, 3456)
        sensor_specs = [
            {
                "pillars": [(1.0, 2.0, 3.0, 0.1, 0.2, 0.3)],
                "global": (4.0, 5.0, 6.0, 0.4, 0.5, 0.6),
            }
        ]
        pillar_block = _build_pillar_block(sensor_specs)
        global_block = _build_global_block(sensor_specs)
        pillar_slip_block = (
            b"\x01\x00\x04\x00"  # Ns=1，sensor 偏移为 4。
            b"\x01\x00\x04\x00"  # Np=1，pillar 偏移为 4。
            b"\x01\x00" + struct.pack("<bf", 3, 0.72)
        )
        sensor_slip_block = (
            b"\x01\x00\x04\x00"  # Ns=1，sensor 偏移为 4。
            b"\x01\x00\x02" + struct.pack("<2f", 0.68, 12.5)
        )
        type_7_data = b"\xde\xad\xbe\xef"
        blocks = [
            (_TYPE_HUB, hub_block),
            (_TYPE_PILLAR, pillar_block),
            (_TYPE_GLOBAL, global_block),
            (5, pillar_slip_block),
            (6, sensor_slip_block),
            (7, type_7_data),
        ]
        offset = 1 + len(blocks) * (2 + _INDEX_SIZE)
        payload = bytearray([_INDEX_SIZE])
        for block_type, block in blocks:
            payload.extend(block_type.to_bytes(2, "little"))
            payload.extend(offset.to_bytes(_INDEX_SIZE, "little"))
            offset += len(block)
        for _, block in blocks:
            payload.extend(block)

        packet = parse_packet(_with_checksum(bytes(payload)))

        npt.assert_array_equal(packet.pillar_slip_states[0], np.array([3], dtype=np.int8))
        npt.assert_allclose(packet.pillar_friction_estimates[0], np.array([0.72]))
        self.assertEqual(packet.slip_detection_active, [True])
        self.assertEqual(packet.reference_pillar_loaded, [True])
        self.assertAlmostEqual(packet.sensor_friction_estimates[0], 0.68)
        self.assertAlmostEqual(packet.target_grip_forces[0], 12.5)
        self.assertEqual(packet.type_7_data, type_7_data)

    def test_raises_when_required_blocks_are_missing(self) -> None:
        payload = bytearray()
        payload.append(_INDEX_SIZE)
        payload.extend(_TYPE_HUB.to_bytes(2, "little"))
        payload.extend((5).to_bytes(_INDEX_SIZE, "little"))
        payload.extend(b"\x00" * 12)
        packet_data = _with_checksum(bytes(payload))

        with self.assertRaisesRegex(ValueError, "pillar 或 global"):
            parse_packet(packet_data)


class ProtocolReaderTest(unittest.TestCase):
    def test_skips_garbage_and_bad_checksum_before_good_frame(self) -> None:
        good_packet = _build_packet_data(
            packet_counter=123,
            timestamp_us=777888999,
            sensor_specs=[
                {
                    "pillars": [
                        (9.0, 8.0, 7.0, 0.9, 0.8, 0.7),
                    ],
                    "global": (6.0, 5.0, 4.0, 0.6, 0.5, 0.4),
                }
            ],
        )
        bad_packet = bytearray(good_packet)
        bad_packet[8] ^= 0x10

        reader = PTSProtocolReader(
            _FakeSerial(
                [
                    b"\x99\x88noise",
                    _build_frame(bytes(bad_packet))[:17],
                    _build_frame(bytes(bad_packet))[17:],
                    _build_frame(good_packet)[:9],
                    _build_frame(good_packet)[9:25],
                    _build_frame(good_packet)[25:],
                ]
            )
        )

        packet = reader.read_packet()

        self.assertEqual(packet.packet_counter, 123)
        self.assertEqual(packet.timestamp_us, 777888999)
        npt.assert_allclose(packet.global_forces[0], np.array([6.0, 5.0, 4.0]))

    def test_raises_timeout_when_serial_returns_empty(self) -> None:
        reader = PTSProtocolReader(_FakeSerial([b"", b""]))

        with self.assertRaisesRegex(TimeoutError, "串口超时无数据"):
            reader.read_packet()


if __name__ == "__main__":
    unittest.main()
