# K1 原生踢球标定

在同一场地、相同初始朝向与球位下，逐脚输入 **power**，人工测量球自然停止后的距离。用于建立“原生踢球输入 → 实测距离”的数据集。

独立目录：`/home/booster/Workspace/K1_Kick_Calibration`。不修改 AAA 或真人传球 demo；不需要裁判机、接球人、场地定位。

## 运行

首次在 K1 编译：

```bash
cd ~/Workspace/K1_Kick_Calibration
./scripts/build.sh
./scripts/test.sh
```

停止其他比赛/传球程序，然后启动：

```bash
./scripts/start.sh --real
```

启动只录制、待命。终端逐条输入，等 `prepared` 后再踢：

```text
calib> prepare
calib> status
calib> kick 3.0
# 等待 stopped，球自然停止后，人工量距
calib> distance 2.8
# 人工复位机器人位置、朝向、球位
calib> kick 4.5
calib> distance 4.1
calib> quit
```

- `power` 接受 **1–10 小数**，原值发送，不做比例换算或自动裁剪。它是原生接口的数值输入，不能先验当作牛顿或实际米数。
- `prepare` 才调用原生准备/起身及所需模式接口；等待有效反馈确认，失败不会无限重试。已处于可用原生模式时直接复用。
- 每次 `kick` **确认出球后再等 1 秒停车**；未确认时最多运行 **7 秒**。结束先退出 `VisualKick`，再发零速度，等待标签。不会自动开始下一脚。
- `stop` 中断动作；`invalid 未触球`、`invalid 多次触球` 等将当前试验作废。中断/接口异常试验不能直接标为有效距离。
- `status` 查看状态和反馈年龄。`quit` 或 Ctrl+C 先停止动作，再关闭录制。
- 不加 `--real` 默认干运行：仅发布 `/kick_calibration/preview_*`，不会向真实动作话题发指令，也不会模拟起身成功。

用地面标记复位机器人与球，球建议位于正前方 **0.5–0.8 米**。每脚触发时的机器人航向为参考，使用里程计补偿相对方向；目标点固定为球沿该方向前方 **6 米**，与 power 独立。里程计和原生动作有误差，不能保证完全同向。

**距离标签：实际踢出点 → 自然停止点的直线距离，单位米。** 不是初始摆球点，也不是弯曲路径长度。原生技能会自行调整走位，一次调用不保证只触球一次；未触球、多次触球、碰人/障碍、人为停球时用 `invalid` 排除。

## 复用了什么

| 原生接口/组件 | 用途 |
|---|---|
| AAA `brain/msg/Kick`、`/kick_ball` | 球位置、方向、目标点、power；消息定义保持原样 |
| AAA `RobotClient` 的 RPC 消息与 SDK 协议 | `VisualKick` 2038，V2 的接口值为 **1**；停止使用同接口 `start=false` |
| 原生 Prepare / GetUp / ChangeMode / RotateHead / Move | 仅显式准备、看球、停止；不自写踢腿或行走控制 |
| 已验证的 Vision 与真机标定 | 检测真实球；保留已有“不保存图像时启动”修复；没有传球决策节点 |
| 系统 `booster_interface` | 关节、IMU、电池、里程计、模式、恢复反馈 |

新增功能：终端逐脚操作、固定参考方向、录制/反馈检查、试验 ID、人工标签和 CSV 导出。没有加载 AAA 比赛行为树。

## 数据

每次启动生成 `logs/calib_时间_标识/`，`logs/latest.json` 指向最近会话。

| 文件 | 内容 |
|---|---|
| `raw/` | rosbag2 原始消息，独立进程接收后直接写入，不进行 50 Hz 降采样 |
| `events.jsonl`、`trials/*.json` | 指令、UUID、响应、时刻、power、人工标签；及时持久化 |
| `trials.csv` | 全部试验；退出后补充动作前状态、动作中统计 |
| `training.csv` | 有人工有效标签、接口成功、通过录制检查且非干运行的样本 |
| `topic_statistics.json` | 每话题数量、接收间隔、DDS 已报告丢失、时钟跳变 |
| `metadata.json`、`metadata_files/` | 版本、配置/标定/模型指纹、URDF、SDK 头、关节映射与消息解释 |

**直接记录**：22 个串联关节的 q、dq、ddq、温度、总线 lost/标志；IMU；电池电压、电流、SOC；模式/恢复状态；原生请求/响应和实际发布的 power。

**估计量**：`tau_est`、里程计、Vision 球位置/置信度保留原值。关节最大速度不等于脚端触球速度；没有把视觉距离或球速作为 ground-truth。`LowState` 无源时间戳，bag 使用接收时间，不能据此声称精确触球时刻或完全无丢包。

CSV 的 `pre_*` 只用动作请求之前的数据，并附样本年龄；`during_*` 是动作过程统计，训练踢前预测模型时不要混作输入。完整传感器波形仍在 bag 中。动作窗口覆盖原生准备走位，不能直接当作触球窗口。

`data_complete` 表示 bag 已正常关闭、必需反馈与开始/停止指令均有记录、力度一致且未报告写入错误/动作中时钟跳变；不表示每一帧都已收到。后台只读查询不阻止准备；其他程序的实际动作请求会中断本次试验。

出球采用已有视觉/里程计估计：连续看到球进入脚边，再连续看到球沿传球方向离开；不会把机器人后退或单帧跳变直接判为出球。日志保留 `ball_departure_detected` 和 `completion_trigger`。这是停车依据，不是精确触球测量；停车后继续录制球自然滚动。

补录/修改标签（会保留修改事件），以及重新生成摘要：

```bash
./scripts/data.sh label logs/<会话> trial_<ID> 2.8
./scripts/data.sh invalid logs/<会话> trial_<ID> "多次触球"
./scripts/data.sh summarize logs/<会话>
./scripts/data.sh export logs/<会话>
```

若进程异常中断，`./scripts/data.sh recover logs/<会话>` 将未结束试验标为无效，不猜测结果。

## 配置与测试

`config/calibration.json` 中 `post_kick_wait_s: 1.0` 为出球确认后的等待，`kick_duration_s: 7.0` 为最长动作时长；另含 6 米参考点和反馈超时。相机标定位于 `src/vision/config/vision.yaml`。两份 `no_*_override.yaml` 是 Vision 必需的空覆盖层，不覆盖标定。

```bash
./scripts/test.sh       # 纯逻辑/标签隔离测试，Mac 也可运行
./scripts/test_ros.sh   # K1 上使用独立 DDS domain 171 和模拟反馈，无真实动作
./scripts/test_ros.sh --departure  # 确认出球后约 1 秒退出技能并发零速度
```

源码独立 Git 管理；`logs/`、编译产物和厂商模型/库不入 Git，模型与库另存清单指纹。此目录已包含所需的现有 Vision 模型，复制到新设备时也应复制这些二进制依赖。
