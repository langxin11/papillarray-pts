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
  - 连接建立时先发采样率命令，静默 `backlog_drain_grace_sec`（默认 0.1 s）待设备端
    积压涌入内核后清空接收缓冲，再开始解析，旧会话字节不会污染本次接收时间戳；
    bias 前同样清空缓冲与协议半包缓存；
  - 提供 `wait_for_serial_device()`：USB 热插拔后等待设备节点出现的有界轮询
    （默认 5 s），入口程序不必因枚举未完成而立即失败；
  - `packet_counter` 按无符号 32 位半模规则在线监督：缺包累计、自然回绕不误报、
    重复/回退/乱序与非递增设备时间戳即时告警；
  - 统计通过 `dropped_packets`、`disordered_packets`、`non_monotonic_timestamps`
    只读属性暴露，基线在每次连接和 bias 后重置。

## USB 连接行为（2026-10 实测，Ubuntu 24.04）

- 控制器枚举为 Teensy CDC ACM（`16c0:0483`，自报 `Contactile_Controller_v2.0`）；
  插入后约 1~2 s 出现 `/dev/ttyACM0`，大头是固件上电启动，系统枚举本身 < 10 ms。
- 控制器**上电后即持续流式输出**，打开串口不会使其复位；纯协议链路打开后
  毫秒级即可读到数据。
- 串口关闭期间控制器仍在写入，旧帧滞留在**设备端发送缓冲**（实测可达 40 s 前），
  重新打开后几 ms 才涌入内核，因此"打开后立即 flush"清不到，必须**先静默再清空**。
  实测采样率命令会让控制器自行丢弃积压，但工作器不依赖该隐式固件行为。
- 原厂 SDK（`PTSDK_CXX_Pybind`）首次连接内部握手约 3 s 才出首帧，重开仅毫秒级；
  走 SDK 的消费方首帧超时应留 ≥ 5 s。
- Ubuntu 的 ModemManager 会探测新 ttyACM 端口，插入后头几秒可能抢占串口；
  安装 [udev/99-contactile-pts.rules](udev/99-contactile-pts.rules)（含
  `ID_MM_DEVICE_IGNORE` 与固定软链接 `/dev/papillarray`），步骤见
  [udev/README.md](udev/README.md)。

## 安装

以固定提交（标签）的 git 依赖引入：

```toml
"papillarray-pts @ git+https://github.com/langxin11/papillarray-pts.git@v0.2.0",
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
