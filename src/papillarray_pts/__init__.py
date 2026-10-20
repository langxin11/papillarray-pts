"""PapillArray PTS v2.0 串口协议、线程化串口工作器与帧采集会话。

只依赖 pyserial 与 numpy，不包含 ROS、MuJoCo 或任何夹爪控制逻辑；
ROS 封装由各消费仓库自行维护。"""

from .frames import PTSFrame, frames_from_packet, iter_sensor_frames
from .protocol import ParsedPacket, PTSProtocolReader, parse_packet, verify_checksum
from .serial_worker import (
    SerialWorker,
    SerialWorkerConfig,
    wait_for_serial_device,
)

__all__ = [
    "PTSProtocolReader",
    "PTSFrame",
    "ParsedPacket",
    "SerialWorker",
    "SerialWorkerConfig",
    "frames_from_packet",
    "iter_sensor_frames",
    "parse_packet",
    "verify_checksum",
    "wait_for_serial_device",
]
