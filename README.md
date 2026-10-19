# papillarray-pts

Contactile PapillArray（PTS v2.0）串口协议解析与线程化串口工作器的**共享核心包**。
只依赖 `pyserial` 与 `numpy`，不包含 ROS、MuJoCo 或任何夹爪控制逻辑。

## 为什么存在

`tactile_grasp` 研究仓库与 [`contactile-papillarray-ros2`](https://github.com/langxin11/contactile-papillarray-ros2)
的 ROS 驱动曾各自维护一份同源的 `protocol.py` / `serial_worker.py`，每次改进都要改两遍。
本包是这两份实现的单一上游：ROS 封装（节点、消息映射）留在各消费仓库，核心在此统一维护。

## 提供什么

- `papillarray_pts.protocol`：按官方 `PTSCOM_2.0_SPEC` 直接解析 USB 串口字节流，
  解码 Type 1/3/4/5/6 块；厂商 SDK 未定义布局的 Type 7 以原始字节保留，不猜测含义。
  帧同步容忍前导噪声、半包与坏校验和。
- `papillarray_pts.serial_worker`：独立 I/O 线程读取 PTS 帧，管理控制命令
  （bias / 滑移检测启停）与指数退避自动重连。行为要点：
  - 连接后与 bias 前清空串口接收缓冲和协议半包缓存，旧会话字节不会污染
    本次接收时间戳或归零后的窗口；
  - `packet_counter` 按无符号 32 位半模规则在线监督：缺包累计、自然回绕不误报、
    重复/回退/乱序与非递增设备时间戳即时告警；
  - 统计通过 `dropped_packets`、`disordered_packets`、`non_monotonic_timestamps`
    只读属性暴露，基线在每次连接和 bias 后重置。

## 安装

以固定提交（标签）的 git 依赖引入：

```toml
"papillarray-pts @ git+https://github.com/langxin11/papillarray-pts.git@v0.1.0",
```

## 最小使用

```python
from papillarray_pts import SerialWorker, SerialWorkerConfig

config = SerialWorkerConfig(port="/dev/papillarray", baud_rate=115200, sampling_rate=1000)
worker = SerialWorker(config, on_packet=lambda packet: print(packet.packet_counter))
worker.start()
worker.send_command(b"z\n")  # bias：发送前必须确认传感器无负载
worker.stop()
```

## 来源与验证

`protocol.py` 与 `serial_worker.py` 提取自 `tactile_grasp`（其迁入来源为
contactile-papillarray-ros2 提交 `2f72ef9`，见该仓库 `docs/migration-sources.json`），
并吸收了 `parallel_gripper_tactile/packages/papillarray_hardware` 的采集边界经验。
协议与工作器的全部回归测试随包维护。

```bash
uv run pytest
uv run ruff check src tests
uv run ruff format --check src tests
```

## 许可

Proprietary，见 [LICENSE](LICENSE)。分发前需确认 Contactile 协议与参考实现的授权范围。
