# rm_serial_driver
机械臂仿真与电控系统的串口通讯模块

## Overview

本模块基于 [transport_drivers](https://github.com/ros-drivers/transport_drivers) 实现 ROS2 与单片机之间的串口收发，当前已去除原有自瞄链路，面向机械臂抓取场景。

## 使用指南

安装依赖 `sudo apt install ros-humble-serial-driver`

更改 [serial_driver.yaml](config/serial_driver.yaml) 中的参数以匹配与电控通讯的串口

启动串口模块 `ros2 launch rm_serial_driver serial_driver.launch.py`

## 协议结构

详情请参考 [packet.hpp](include/rm_serial_driver/packet.hpp)。

- 下行（电控 -> ROS）：`CmdPacket`（`0xA5`），接收端执行 `header/length/CRC/枚举` 校验。
- 上行（ROS -> 电控）：`FeedbackPacket`（`0xA6`），用于回传关节与夹爪反馈。
- 枚举定义：`Mode / CmdAction / Status / ErrorCode / Online`

当前两类数据帧均为固定 `32 bytes`，并统一包含：

- `length`：帧长度固定为 `32`，不可变
- `seq`：`uint8`，按模 `256` 回绕（`255 -> 0`）
- `checksum`：CRC16（覆盖除 `checksum` 字段外的全部字节）

下行 `CmdPacket` 实际校验链路（固定顺序）：

1. `header`
2. `length`
3. `CRC16`
4. `mode/cmd_action` 枚举合法性

任一步失败即丢弃该帧，不执行轨迹发布，并通过 `status/error_code` 回传错误状态。

## 字段语义冻结（P0）

以下字段语义冻结，后续版本不再变更：

- `j1` ~ `j4`：单位 `rad`
- `gripper`：单位 `m`
- `mode` / `cmd_action` / `status` / `error_code` / `online`：类型均为 `uint8` 枚举值
- `online`：`0 = offline`，`1 = online`
- `length`：固定 `32`，不可变
- `seq`：`uint8` 递增计数，按模 `256` 回绕（`255 -> 0`）
- 下行 `CmdPacket` 在 `expected_seq != 0` 时收到 `seq=0`，视为发送端重启并接受，不记
  `kCmdSeqJump`

## 超时规则冻结（P0）

- 参数：`cmd_timeout_ms`（默认 `300`）
- 仅在收到首条有效 `Execute` 后开始超时计时
- 若超过 `cmd_timeout_ms` 未收到有效命令：`status = kPaused`，`error_code = kCmdTimeout`
- 超时生效后不再执行新轨迹，直到收到下一条有效 `Execute`

## CmdAction 行为冻结（P0）

以下行为按当前实现冻结，电控与上位机按此联调：

| 命令 | 允许条件 | 执行动作 | 状态转移 | 错误码 |
| --- | --- | --- | --- | --- |
| `Pause` | 未被 `Estop` 锁存（`estop_latched_ == false`） | 置 `paused_by_cmd_=true`；发布保持轨迹（目标=最近一次 `joint_1~joint_4 + gripper`，`time_from_start=0.05s`） | `status -> kPaused` | 正常为 `kOk`；若该包 `seq` 跳变则为 `kCmdSeqJump` |
| `Resume` | 未被 `Estop` 锁存 | 置 `paused_by_cmd_=false`；若缓存了 `last_execute_cmd_`，重发一次恢复轨迹 | 有缓存执行命令时 `status -> kRunning`，否则 `status -> kReady` | 正常为 `kOk`；若该包 `seq` 跳变则为 `kCmdSeqJump`；若超时锁激活则 `kCmdTimeout`（并保持 `kPaused`） |
| `ClearFault` | 始终允许（也是 `Estop` 锁存下唯一允许命令） | 清 `estop_latched_` 与 `paused_by_cmd_`；不清 `cmd_timeout_active_`（超时冻结保留） | 若无超时锁：`status -> kReady`；若超时锁仍在：`status -> kPaused` | 无超时锁时 `kOk`；有超时锁时 `kCmdTimeout` |
| `Estop` | 始终允许 | 置 `estop_latched_=true`、`paused_by_cmd_=true`；发布保持轨迹（`0.05s`） | `status -> kEstop` | `kEstopTriggered` |

补充冻结规则：

- `Estop` 锁存后，除 `ClearFault` 外，其他 `cmd_action` 一律拒绝执行。
- 超时冻结（`cmd_timeout_active_`）下，仅“下一条有效 `Execute`”可解锁并恢复轨迹发布。
- `Pause/Resume/ClearFault/Estop` 在协议校验失败（`length/CRC/枚举`）时均不执行，状态按错误码上报。

下行命令通过轨迹消息发布到：

- `/arm_controller/joint_trajectory`
- `/gripper_controller/joint_trajectory`

## 机械臂上行反馈帧（ROS仿真 -> 电控）

已在驱动中新增 `FeedbackPacket`（见 `include/rm_serial_driver/packet.hpp`），从 `/joint_states`
读取并通过串口发送：

- `j1` ~ `j4`：对应 `joint_1` ~ `joint_4` 关节角（rad）
- `gripper`：对应 `left_finger_joint` 开合量（m）
- `seq`：反馈帧序号（发送时自增）
- `online`：串口在线状态（1/0）
- `status`：状态码（默认 `1`，可通过参数 `feedback_status` 配置）
- `error_code`：错误码（默认 `0`；当运行态无故障且关节数据缺失时置 `kJointStateMissing`）
- `checksum`：CRC16

帧头为 `0xA6`，CRC16 算法与原协议保持一致。

默认发送频率参数：

- `feedback_rate_hz: 50.0`
- `feedback_status: 1`
- `feedback_error_code: 0`
- `cmd_timeout_ms: 300`

## 单元测试（P1）

已新增以下测试：

- `test_packet_crc`：CRC追加/校验、篡改后校验失败、固定长度32字节校验

运行命令：

```bash
cd /home/robo/simulation_of_construction_robo
source /opt/ros/humble/setup.bash
colcon test --packages-select rm_serial_driver
colcon test-result --verbose
```

## 联调脚本（P1）

新增脚本：`tools/serial_link_tool.py`

能力：

- 发送 `CmdPacket`（`0xA5`，自动CRC）
- 接收解析 `FeedbackPacket`（`0xA6`，校验 `length/CRC`）
- 负向测试注入：`--inject-bad-length`、`--inject-bad-crc`

示例：发送一条执行命令并接收5帧反馈

```bash
python3 /home/robo/simulation_of_construction_robo/rm_serial_driver/tools/serial_link_tool.py \
  --port /dev/ttyACM0 \
  --baud 115200 \
  --mode joint \
  --cmd-action execute \
  --j1 0.2 --j2 -0.3 --j3 0.4 --j4 0.1 --gripper 0.01 \
  --tx --rx --rx-count 5
```

示例：发送坏CRC（验证驱动拒绝执行并上报错误）

```bash
python3 /home/robo/simulation_of_construction_robo/rm_serial_driver/tools/serial_link_tool.py \
  --port /dev/ttyACM0 \
  --mode joint \
  --cmd-action execute \
  --inject-bad-crc \
  --tx --rx --rx-count 3
```

## 可复现联调流程（虚拟串口）

1. 启动虚拟串口对（终端A）：

```bash
socat -d -d pty,raw,echo=0,link=/tmp/ttyV0 pty,raw,echo=0,link=/tmp/ttyV1
```

2. 将驱动串口改为 `/tmp/ttyV0`（`config/serial_driver.yaml`）。
3. 启动驱动（终端B）：

```bash
cd /home/robo/simulation_of_construction_robo
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rm_serial_driver serial_driver.launch.py
```

4. 运行联调脚本（终端C）：

```bash
python3 /home/robo/simulation_of_construction_robo/rm_serial_driver/tools/serial_link_tool.py \
  --port /tmp/ttyV1 \
  --mode joint \
  --cmd-action execute \
  --j1 0.1 --j2 0.0 --j3 -0.2 --j4 0.3 --gripper 0.005 \
  --tx --rx --rx-count 10 --hex
```

5. 观察脚本输出的 `status/error_code` 与驱动日志是否一致，完成联调闭环验证。

## 电控端的处理

TBD
