"""PapillArray PTS v2.0 串口协议与线程化串口工作器。

只依赖 pyserial 与 numpy，不包含 ROS、MuJoCo 或任何夹爪控制逻辑；
ROS 封装由各消费仓库自行维护。"""

from .protocol import ParsedPacket, PTSProtocolReader, parse_packet, verify_checksum
from .serial_worker import SerialWorker, SerialWorkerConfig

__all__ = [
    "PTSProtocolReader",
    "ParsedPacket",
    "SerialWorker",
    "SerialWorkerConfig",
    "parse_packet",
    "verify_checksum",
]
