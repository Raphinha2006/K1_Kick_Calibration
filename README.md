# K1 原生踢球力度标定

在 Booster K1 上，通过终端指定 `power`，调用原生踢球技能，人工测量球自然停止后的距离，用于标定 **力度输入与实际距离的关系**。

## 功能

- `kick <power>` 单次踢球，支持 **1–10 的小数**，原值传给接口。
- 默认动作窗口 **2.5 秒**，结束后退出技能并停车，不自动追球或开始下一脚。
- 每脚保持触发时的参考方向，使用里程计补偿；目标点固定在球前方 6 米，与 power 分开设置。
- 保存原始 ROS2 消息、力度、接口响应、试验 ID 和人工距离，生成 CSV。
- 独立运行，不需要裁判机或接球人。

## 调用的原生接口

| 接口 / 组件 | 用途 |
|---|---|
| AAA `brain/msg/Kick` → `/kick_ball` | 发送球位置、方向、目标点和 `power` |
| SDK `VisualKick`（2038，V2 的接口值为 1） | 启动 / 退出原生踢球技能 |
| SDK `ChangeMode`、`GetUp` | `prepare` 时准备所需模式和起身 |
| SDK `RotateHead`、`Move` | 看球，以及退出技能后发送零速度 |
| 原 demo Vision、系统 `booster_interface` | 球检测及关节、IMU、里程计等反馈 |

通过 `/LocoApiTopicReq`、`/LocoApiTopicResp` 使用原生 RPC 协议；踢球走位与踢腿由原生技能执行。`power` 是接口参数，不保证其数值等于实际距离（米）。

## 在 K1 上运行

需要已安装 ROS2 Humble、Booster SDK / ROS2 Interface，以及本机可用的 Vision 模型、运行库和相机标定。模型与厂商二进制不随 Git 上传，清单见 [runtime_artifacts.json](vendor_reference/runtime_artifacts.json)。

首次编译：

```bash
cd ~/Workspace/K1_Kick_Calibration
./scripts/build.sh
```

停止其他比赛或传球程序，再启动：

```bash
./scripts/start.sh --real
```

启动后待命，在终端输入：

```text
calib> prepare
calib> status
calib> kick 3.0
calib> distance 2.8
calib> kick 4.5
calib> distance 4.1
calib> quit
```

等待 `prepared` 后才能踢球；每脚等待 `stopped`，让球自然停止，再测量并输入 `distance`。距离指 **实际踢出点到自然停止点的直线距离（米）**。下一脚前人工复位机器人朝向、位置及球位，球可放在正前方约 0.5–0.8 米。

| 命令 | 作用 |
|---|---|
| `prepare` | 准备机器人，反馈就绪后允许踢球 |
| `kick 1.5` | 用指定 power 执行一次技能尝试 |
| `distance 4.2` | 给当前试验记录人工距离 |
| `invalid 多次触球` | 将当前试验标为无效，也可写未触球、撞墙等原因 |
| `stop` / `status` / `quit` | 停止动作 / 查看状态 / 停止并退出 |

每脚填入 `distance` 或 `invalid` 后，才能开始下一脚。原生技能可能多次触球；撞墙、碰人或人为停球的样本用 `invalid` 排除。

日志位于 `logs/calib_时间_标识/`，包括 `raw/`、`trials.csv` 和 `training.csv`；日志不上传 Git。触球时刻、脚端速度、视觉位置及力矩估计不能直接作为物理真值。

不加 `--real` 为干运行；隔离测试运行 `./scripts/test.sh`。参数见 [calibration.json](config/calibration.json)，详细记录与补录说明见 [技术说明](docs/technical_details.md)。
