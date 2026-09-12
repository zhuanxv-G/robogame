# RoboGame 当前状态

> 状态快照：2026-09-12
> 分析基准：本地 `main` 分支及当前工作区。
> 本文区分“代码可确认”“项目记录为已实机验证”和“尚待确认”，避免把历史脚本或规划接口误认为现役功能。

## 1. 项目简介

RoboGame 是一套参加 RoboGame 2026 竞技组比赛的移动机器人系统。机器人需要在固定场地中巡线和定位，识别并获取橙色、紫色建筑材料，最终在搭建区完成机械臂搭建任务。

当前采用上下位机分工：Raspberry Pi 5 负责视觉结果处理、比赛状态和任务决策；STM32F407ZGT6 负责底盘、巡线、Servo 和 Stepper 的实时执行。K230D / CanMV 作为独立视觉协处理器，把目标检测结果发送给 Pi；Logitech C270 USB Camera 直接连接 Pi，用于 AprilTag 检测。

项目已从单项硬件调试进入整车联调阶段，但还没有形成一条覆盖“启动—巡线—取橙色—取紫色—搭建”的全自动比赛主流程。

## 2. 当前目录结构

```text
robogame/
├── main_controller_align.py       # 当前 K230D 对准、接近并触发搭建的主控入口
├── arm_sequence.py                # 可复用的已验证机械臂动作与 STM32 请求/响应封装
├── build_sequence.py              # 独立机械臂搭建测试入口
├── config.py                      # 正式串口路径和波特率，现有入口尚未统一导入
├── hardware/
│   └── stm32_link.py              # 显式连接的 STM32 通信抽象层
├── vision/
│   ├── k230_link.py               # K230D 串口生命周期与 TARGET/NO_TARGET 解析
│   ├── apriltag_service.py        # 延迟连接的 AprilTag 检测服务接口
│   └── apriltag_detector.py       # USB Camera + tag36h11 原检测算法
├── competition/
│   ├── mission_state.py           # 比赛顶层状态定义
│   ├── mission_controller.py      # 仅管理状态的基础控制器
│   ├── competition_runner.py      # 事件驱动的第一版比赛流程调度
│   ├── task_controller.py         # 复用已验证机械臂动作的任务接口
│   ├── visual_approach_controller.py # K230 对准、接近和 READY 判断
│   ├── navigation_controller.py   # 巡线启动与 Tag 观测导航抽象层
│   └── main.py                    # 不连接硬件的未来比赛入口骨架
├── navigation/
│   ├── localization.py            # AprilTag 轻量定位结果
│   ├── topological_navigation.py  # 无硬件拓扑定位与 BFS 路线规划
│   ├── field_model.py             # 规则手册结构化世界模型
│   ├── field_map.py               # get_zone_by_tag() 兼容查询接口
│   ├── tag_map.py                 # AprilTag 旧接口兼容视图
│   └── navigation_state.py        # 导航状态枚举接口
├── strategy/
│   └── mission.py                 # 比赛任务枚举、顺序和任务元数据
├── tests/                         # 人工执行的相机/串口/运动诊断脚本
├── legacy/                        # 旧主控、旧协议和早期原型
├── docs/                          # 环境、场地、协议、架构和状态文档
├── RoboGame2026 竞技组规则手册3_0.pdf
├── README.md
└── AGENTS.md
```

Pi 仓库中没有 STM32 的 `.c`、`.h`、`.ioc`、链接脚本或 IDE 工程文件；Git 历史中也没有这些文件。当前 STM32 源码位于外部目录 `D:\Code\V1.3_vision\V1.3`，Keil 工程为 `MDK-ARM\f407_ustc_2022_robogame.uvprojx`。该外部目录没有可用 Git 历史，因此源码、编译产物与当前实机已烧录固件之间仍缺少版本标识。

## 3. 硬件架构

| 组件 | 当前角色 | 已知连接/信息 | 信息状态 |
| --- | --- | --- | --- |
| Raspberry Pi 5 | 上位机、视觉、状态与决策 | Python 3.13，项目虚拟环境为 `venv` | 项目环境信息 |
| STM32F407ZGT6 | 底盘、巡线、Servo、Stepper 执行 | 通过 CH340 USB 串口连接 Pi | 项目记录为已验证 |
| K230D / CanMV | 颜色/物块视觉协处理 | USB 串口向 Pi 输出 `TARGET` / `NO_TARGET` | 项目记录为已验证 |
| Logitech C270 | AprilTag 相机 | Pi 上默认 `/dev/video0`，640×480 | 项目记录及代码可确认 |
| 麦克纳姆轮底盘 | 全向移动 | 由 STM32 执行 `VEL` 等运动命令 | 项目记录为已验证 |
| Servo 机械臂 | 抓取、搬运、搭建 | 代码使用 Servo 索引 0-4 | 动作序列记录为已实机验证 |
| TB6600 + Stepper | 机械臂升降/直线运动 | STM32 接收步数与 pps | 动作序列记录为已实机验证 |
| 幻尔四路巡线传感器 | 黑线检测 | STM32 软件 I2C：PC4=SCL、PC5=SDA，低 4 位 1=黑线 | 当前固件直线巡线已现场工作；全白输入会继续前进的风险待处理 |
| 底盘电机及驱动 | 底盘执行 | 由 STM32 侧控制 | 电机和驱动器具体型号不在本仓库 |

当前资料还缺少完整 BOM、供电树、接线图以及可追溯的 STM32 固件版本标识；巡线传感器型号、源码引脚和直线巡线能力已确认，T 字路口行为尚未验证和处理。

## 4. 软件架构

### 4.1 Raspberry Pi 职责

- 从 K230D 串口接收物块视觉结果。
- 从 USB Camera 获取图像并检测 tag36h11 AprilTag。
- 根据 `error_x`、目标面积等信息进行轻量对准和接近判断。
- 保存比赛场地区域、Tag 和巡线拓扑世界模型。
- 根据已确认 Tag 更新逻辑位置，并在无坐标拓扑图上规划节点路径。
- 保存比赛任务顺序及未来状态机接口。
- 通过 `CompetitionRunner` 串联任务、导航、视觉接近和机械臂控制器。
- 通过 `NavigationController` 统一记录巡线、材料区目标和 Tag 辅助定位状态。
- 通过 `VisualApproachController` 管理 K230 对准、接近、READY 和安全停车状态。
- 通过 `TaskController` 调度已有机械臂搭建和安全复位动作。
- 通过唯一 STM32 串口连接发送底盘、Servo、Stepper 和 STOP 命令。

新增分层已把 STM32、K230D 和 AprilTag 的“构造”与“连接”分开。创建这些对象不会打开串口或摄像头；硬件连接必须由未来调度入口显式执行。

### 4.2 STM32 职责

- 执行麦克纳姆底盘速度与停车命令。
- 执行本地巡线功能。
- 执行 5 路 Servo 绝对角度动作。
- 执行 Stepper 指定步数运动并报告完成。
- 提供命令 ACK、完成通知和错误回复。

上述职责已由 Pi 端调用以及外部 STM32 源码交叉确认。STM32 源码仍不属于本 Git 仓库；当前直线巡线已有现场结果，但由于外部工程缺少可用 Git 历史和固件标识，仍不能把现场固件精确关联到某次本地构建。

### 4.3 通信方式

- Pi ↔ STM32：CH340 USB 串口，`115200` baud，以 `\n` 结尾的 ASCII 兼容文本命令。
- K230D → Pi：USB 串口，`115200` baud，以行为单位发送 `TARGET,...` 或 `NO_TARGET`。
- USB Camera → Pi：OpenCV `VideoCapture`，默认 `/dev/video0`、请求 640×480。

完整 Pi ↔ STM32 格式见 [PI_STM32_PROTOCOL.md](PI_STM32_PROTOCOL.md)，模块和数据流见 [SOFTWARE_ARCHITECTURE.md](SOFTWARE_ARCHITECTURE.md)。

## 5. 当前完成情况

| 子系统 | 状态 | 当前结论 |
| --- | --- | --- |
| Pi 车载环境与设备接入 | 已完成基础部署 | Python 3.13、虚拟环境和 USB 设备已建立；仓库没有依赖锁定文件 |
| Pi ↔ STM32 串口通信 | 已验证 | 正式 by-id 路径和 115200 波特率已固定；请求/响应代码正在使用 |
| STM32 通信抽象层 | 骨架完成，未实机回归 | `Stm32Link` 封装 HELLO_STM32、STOP、VEL、SERVO_GET、SERVO_SET、STEPPER_MOVE，并要求显式 `connect()` |
| STM32 底盘、电机、巡线 | 直线巡线基本可用 | `LINE_START` 的基础速度已从 0.40 m/s 调整为 0.80 m/s，尚待实车回归；此前 0.40 m/s 直线巡线已现场可用，轻微蛇形暂可接受；全白仍继续前进，T 字路口留待后续处理 |
| Servo、Stepper、机械臂 | 动作层已验证 | `ArmSequence` 可复用，三组搭建动作和安全 HOME 已保留；不要改动参数 |
| K230D 物块视觉 | 单项检测已验证 | 当前主控能解析目标并完成对准/接近；完整橙色、紫色任务切换尚未接入 |
| K230D 接口层 | 骨架完成，未实机回归 | `K230Link` 封装连接生命周期、现有 `TARGET` / `NO_TARGET` 解析格式和最新消息读取 |
| K230D 对准到搭建 | 第一版已接通 | `main_controller_align.py` 可从 ALIGN/APPROACH 到 READY，再经人工 `b` 确认执行 ARM_BUILD |
| competition 视觉接近 | 软件迁移完成，未实机回归 | `VisualApproachController` 保留原阈值、EMA、速度、三帧 READY 和超时 STOP；尚未由 MissionController 自动运行 |
| AprilTag 检测 | 单项现场验证完成 | 场地可稳定输出正确 id、error_x 和像素面积，最远约 2.7 m 可检测；无 GUI，适合 SSH |
| AprilTag 服务层 | 接口完成 | `AprilTagService` 延迟创建原 `AprilTagDetector`，没有修改已现场验证的检测算法和参数 |
| AprilTagLocalization | 基础接口完成 | 每帧选择面积最大 Tag，输出语义名称和面积启发式 confidence |
| 比赛世界模型 | 场地事实纠正并结构化 | Tag 2/Tag 4 分别对应左右橙色材料区且都具备 `ORANGE_PICKUP`；搭建区保持独立，绝对坐标仍为 TODO |
| 拓扑定位与规划 | 第一版纯软件实现完成 | `TopologicalNavigator` 可由已知 Tag 更新逻辑位置，并用 BFS 返回已确认节点路径；不拥有或操作硬件 |
| 导航/任务状态定义 | 基础接口完成 | `competition.MissionState` 新增 FINISH、FAULT；`MissionController` 只负责显式切换和复位，没有运行时调度策略 |
| competition 导航控制 | Tag 辅助到站软件实现完成，待实机联调 | `start_line_to_tag()` 启动 STM32 巡线，`update()` 按目标 Tag 面积连续帧确认并在 STOP ACK 后进入 ARRIVED；阈值由入口传入，尚待现场标定 |
| 巡线到拓扑人工联调 | 入口与纯软件回归完成，尚未实机运行 | `test_navigation_topology_live.py` 只在人工输入 START 后连接一个 Stm32Link 和一个 AprilTagService；ARRIVED 后才把目标 Tag 写入 TopologicalNavigator |
| competition 流程调度 | 第一版模拟流程完成 | `CompetitionRunner` 通过外部事件串联巡线、橙色目标接近、抓取、搭建和结束；默认 simulation mode 不调用硬件控制器 |
| 机械臂任务控制 | 骨架完成，橙色抓取待确认 | `TaskController.build()` 复用 `ArmSequence.run_all()`，`reset_arm()` 复用 `safe_home()`；静态考古发现一个禁用的旧抓取候选，但不足以确认其颜色、存放目标和当前实机有效性 |
| 比赛入口 | 安全模拟入口完成 | `python -m competition.main` 初始化 CompetitionRunner，默认只打印 simulation/START 及子控制器状态，不连接或运行任何硬件 |
| 完整比赛闭环 | 未完成 | Runner 已串起第一版橙色任务模拟流程，但没有真实事件、可执行橙色抓取、紫色任务或 live 生命周期 |

## 6. 当前代码入口

### 6.1 未来比赛入口骨架：`competition/main.py`

该入口创建各分层依赖并将 MissionController、NavigationController、VisualApproachController 和 TaskController 注入 `CompetitionRunner(simulation_mode=True)`。它只读取并打印 Runner 的 START 状态和各连接状态，不调用 `start()`、`run_once()`、`handle_event()`、`connect()` 或任何任务动作，因此不会打开串口、摄像头或发送运动命令。ArmSequence、NavigationController 与 VisualApproachController 继续共享同一个 Stm32Link，没有第二个 STM32 串口所有者。安全检查方式为：

```bash
python -m competition.main
```

当前 TaskController 的真实能力边界：

- `build()` 只委托 `ArmSequence.run_all()`，保持 `group1 -> group2 -> group3` 的已验证动作顺序。
- `reset_arm()` 只委托已有 `safe_home()`。
- `grab_orange()` 和 `grab_purple()` 返回 `NotImplemented`。现有三个 group 都把抓取和放置组合在一起，仓库没有把任何一组定义为某种颜色的独立取料动作，因此不能安全猜测映射。
- `build()` 不隐式调用固定 `startup_raise()`；该动作在现有代码中属于独立入口启动策略，且不是带限位的 Homing。

#### 6.1.1 橙色抓取动作考古结论（2026-09-11）

在 Pi 仓库历史、`legacy/`、`tests/`、`arm_sequence.py`、`build_sequence.py` 和 `main_controller_align.py` 中，没有找到被明确命名为“橙色方块抓取并放入中间盒”的独立动作。外部 STM32 工程 `D:\Code\V1.3_vision\V1.3\Core\Src\main.c` 的 `Test_All()` 中存在一段最接近该语义的旧候选，原样位于 `#if 0`：

```text
delay 500 ms
Servo1=0, Servo2=0, Servo4=135
delay 500 ms
Robot.Move_Distance(0.5f, 2.0f)
Robot.Move_Distance(-0.05f, 1.0f)
delay 500 ms
Stepper +3000 @ 800 pps
delay 500 ms
Servo4=105
delay 500 ms
Servo2=15
delay 500 ms
Stepper -3000 @ 800 pps
delay 500 ms
Robot.Move_Distance(-0.30f, 1.5f)
delay 500 ms
Servo2=0
Servo1 Sweep_To(93, 10 ms/degree)
delay 500 ms
Stepper +5000 @ 800 pps
delay 500 ms
Servo4=135
delay 500 ms
Stepper -5000 @ 800 pps
delay 500 ms
```

该候选不能直接封装为 `grab_orange()`，原因如下：

- 源码没有“橙色”或“中间盒”注释，也没有当前调用入口；正式 `main()` 注释掉了 `Test_All()`，工程文档把其内容定义为旧测试代码。
- 候选同时包含三段底盘 `Robot.Move_Distance()`，并非可不改变顺序地抽取为纯 `ArmSequence` 动作。
- 候选与当前 `ArmSequence.group1()` 不一致：后者使用 Servo4 140/105/135、Servo2 120/180/0、Servo1 Sweep 170 后设置 178/30，以及 Stepper ±3000、±10500；旧候选使用 Servo2 15、Servo1 Sweep 93 和 Stepper ±5000。不能把两者视为同一动作版本。
- STM32 工程目录没有可用的 Git 历史，Pi Git 历史也只有从原 `build_sequence.py` 抽取三组搭建动作的版本，无法用版本关系证明旧候选曾在当前机械结构上完成橙色取料。
- Stepper 只有相对步数，没有原点、限位或编码器。旧候选本身不执行 `startup_raise()`；`build_sequence.py` 固定先执行 -1500 steps @ 800 pps，而 `main_controller_align.py` 默认关闭该动作，两个入口的起始高度并不等价。

因此 `TaskController.grab_orange()` 继续返回 `NotImplemented`，CompetitionRunner 也继续在非模拟 `GRAB_ORANGE` 阶段阻塞。继续实现前至少需要：实机记录或操作者确认旧候选的任务语义；确认其中三段底盘动作是否属于不可分割的已验证顺序；确认动作开始时的 Servo 姿态和 Stepper 机械高度；确认 -1500 @ 800 pps 是否应在该流程前且仅执行一次；以及确认当前实机/固件仍对应这套参数。

当前 VisualApproachController 的真实能力边界：

- 状态仅包含 `STOP`、`ALIGN`、`APPROACH`、`READY`、`FAULT`；到达 READY 后由未来任务层决定是否进入 ARM_BUILD。
- `update()` 通过 `K230Link.read_latest()` 读取当前队列的最新结果；控制器本身不创建或连接串口。
- ALIGN 使用原地 `omega = ±0.45`；误差进入 `±30` 后转 APPROACH。
- APPROACH 使用 `y = 0.20`；误差达到 `±70` 后回到 ALIGN。
- pixels 继续使用 `alpha = 0.20` 的 EMA；面积至少 20000 且误差不超过 30 连续 3 帧后，必须收到 STOP ACK 才认为 `is_ready()` 成立。
- 有效 `NO_TARGET` 不立即停车；目标丢失 0.30 s 或 K230 无有效数据 0.60 s 时通过 Stm32Link STOP 并清理接近状态。

当前 NavigationController 的真实能力边界：

- 使用独立于 MissionState 的 `IDLE`、`LINE_FOLLOWING`、`TAG_SEARCH`、`TAG_ALIGN`、`ARRIVED`、`FAULT` 状态。
- `start_line_follow()` 只通过共享 Stm32Link 发送仓库已有的 `LINE_START`；不等待或推断巡线完成。
- `start_line_to_tag(tag_id, arrival_area, confirm_frames)` 使用同一个 `LINE_START` 启动 STM32 本地巡线；后续每次 `update()` 只读取一帧 AprilTag 结果。
- 目标未出现、出现错误 Tag、面积低于阈值或中间丢帧时继续巡线，并清零连续确认计数；不会因偶发漏检停车。
- 只有指定 Tag 的面积连续 `confirm_frames` 帧达到调用方传入的 `arrival_area`，才要求 STM32 确认 STOP，并在成功后进入 `ARRIVED`。该阈值没有内置最终值，仍需现场标定。
- 状态快照包含目标 id、是否可见、当前 `error_x`、`area`、确认计数和错误。AprilTag 读取异常或 STM32 异常会尝试安全 STOP 并进入 `FAULT`；STOP 本身失败则保持 `FAULT` 可观测。
- `stop_navigation()` 只通过 Stm32Link 发送现有 STOP。
- `navigate_to_material_zone()` 只接受 `ORANGE` / `PURPLE` 并记录目标，不生成路线或底盘命令。
- `align_with_tag()` 只调用 AprilTagService 读取一帧并记录指定 Tag 是否可见；`TAG_ALIGN` 目前不代表已经执行姿态修正。
- `handle_event()` 返回 `NotImplemented`，没有假设 `LINE_DONE`、`TAG_REACHED` 或其他尚未验证的 STM32 事件。

当前 TopologicalNavigator 的真实能力边界：

- `update_from_tag()` 把 Tag 2、3、4 分别映射为 `LEFT_ORANGE_MATERIAL`、`PURPLE_MATERIAL`、`RIGHT_ORANGE_MATERIAL`；未知 Tag 返回 `None`，不会覆盖最近一次已确认位置。
- `plan_route(start, target)` 只在无权、无坐标图上执行 BFS，返回逻辑节点列表；规划本身不会改变 `current_location`。
- 图中已表达启动区、灰色过道、左右材料路口、屋顶材料路口和三个材料区。Tag 5/6 保留为方向参考地标；搭建区因连线未确认而保持独立节点。
- Tag 2 和 Tag 4 所在的两侧墙体建筑材料区域都支持 `ORANGE_PICKUP`。策略层通过 `PREFERRED_ORANGE_SOURCE` 单独指定 Tag 2 一侧为第一版首选，Tag 4 一侧为备用，不把策略优先级写成场地事实。
- 该模块不导入 Stm32Link、AprilTagService 或 K230D，不发送 `LINE_START`、VEL、STOP，也不执行路口动作。NavigationController 到达目标 Tag 后，需要由未来上层显式调用 `update_from_tag(tag_id)`。

当前 CompetitionRunner 的真实能力边界：

- Runner 内部阶段为 `START -> FOLLOW_LINE -> GET_ORANGE -> VISUAL_APPROACH -> GRAB_ORANGE -> BUILD -> FINISH`，任意阶段可接收 `FAULT`；其中视觉接近和抓取是 GET_ORANGE 顶层任务内的细分阶段。
- MissionController 仍只保存顶层 `MissionState`。Runner 负责调用控制器和转换顶层状态，MissionController 不接触 STM32、摄像头或机械臂。
- `handle_event()` 支持外部注入 `LINE_DONE`、`TARGET_READY`、`ARM_DONE`、`BUILD_DONE`、`FAULT`，不使用 sleep 或定时伪造完成。
- 默认 simulation mode 只转换软件状态，不调用 NavigationController、VisualApproachController 或 TaskController。
- 显式关闭 simulation mode 后，`start()` 才调用巡线，`run_once()` 才轮询视觉，相关事件才委托抓取和搭建动作；当前入口没有启用该模式。
- 当前真实 `TaskController.grab_orange()` 返回 `NotImplemented`，因此非模拟流程到达 `GRAB_ORANGE` 后会明确阻塞，不能接受 `ARM_DONE` 继续搭建。

### 6.2 整车联调入口：`main_controller_align.py`

当前实际状态为：

```text
STOP -> ALIGN <-> APPROACH -> READY -> ARM_BUILD -> BUILD_DONE
```

- `ALIGN` 只原地旋转，当前使用 `omega = ±0.45 rad/s`。
- `APPROACH` 只前进，当前使用 `y = 0.20 m/s`。
- 对准退出阈值为 30 像素，重新进入对准阈值为 70 像素。
- 面积阈值暂为 20000，连续 3 帧满足后进入 READY。
- 目标丢失 0.30 s 或 K230D 0.60 s 无有效数据会停车。
- READY 必须收到 `ACK,STOP`；默认禁止自动触发，需人工按 `b` 才执行完整搭建动作。
- `ARM_BUILD` 阶段由 `ArmSequence` 独占 STM32 回复读取权；结束后锁存在 `BUILD_DONE`。

因此它是“视觉接近 + 机械臂搭建联调入口”，不是完整比赛主程序。该文件保持原样并可继续独立实机测试；新 VisualApproachController 是并行迁移的软件组件，不替换或删除这个入口。

### 6.3 机械臂入口：`build_sequence.py`

独立打开 STM32 串口，握手后固定执行一次 startup raise，再通过菜单运行 group 1、2、3 或全部动作。startup raise 为 `-1500 steps @ 800 pps` 的固定移动，不是 Homing，因为没有上限位器。

此入口会驱动真实机械结构，只能在人工确认环境安全后执行。

### 6.4 视觉与导航人工测试入口

```bash
python tests/test_apriltag.py
python tests/test_localization.py
```

二者会持续打开相机，按 Ctrl+C 释放；不会打开 STM32，也不会驱动机器人。

第一版 AprilTag 辅助巡线到站人工入口为：

```bash
python3 tests/test_line_to_tag.py --tag-id 2 --arrival-area 10000 --confirm-frames 3
```

`arrival_area=10000` 仅为显式测试示例，不是最终参数。脚本先打印全部参数并要求人工输入 `START`，确认后才连接设备和发送 `LINE_START`；Ctrl+C、异常和非 ARRIVED 退出路径都会尝试 STOP，再释放相机和 STM32。

巡线、AprilTag 到站和拓扑位置更新的独立人工联调入口为：

```bash
python3 tests/test_navigation_topology_live.py \
  --tag-id 2 \
  --arrival-area 4000 \
  --confirm-frames 3
```

该入口复用一个 `Stm32Link`、一个 `AprilTagService`、`NavigationController` 和 `TopologicalNavigator`。只有指定 Tag 达到原有连续帧阈值且 STOP ACK 成功、NavigationController 进入 `ARRIVED` 后，脚本才调用一次 `update_from_tag()` 并打印 `confirmed_tag` 和 `current_location`；错误 Tag 只作为观测状态打印，不会更新拓扑位置。脚本必须由人工输入严格大写的 `START` 才连接设备；Ctrl+C、异常和 FAULT 路径都会尝试 STOP，随后释放 Camera 和 STM32。

这是会启动真实巡线的人工实机联调入口，当前尚未运行或完成实车验证；它不是 `CompetitionRunner`，不执行 BFS 路线，也没有实现 T 字路口检测或转向。

### 6.5 STM32 入口

STM32 不在本 Git 仓库中。当前外部源码入口为 `D:\Code\V1.3_vision\V1.3\Core\Src\main.c`，Keil 工程为 `D:\Code\V1.3_vision\V1.3\MDK-ARM\f407_ustc_2022_robogame.uvprojx`。2026-09-11 将 `LINENAV_DEFAULT_SPEED_M_S` 从 0.4f 调整为 0.8f 后，已使用 ARMCC V5 全量重编译，结果为 0 error、1 个原有 `Test_All` 未引用警告；该 0.80 m/s 固件尚未烧录或实车验证。

## 7. 当前测试状态

### 7.1 已有验证记录

根据项目说明、代码注释和已保留的实机入口，以下功能已完成过独立或联调验证：

- Pi 供电和车载部署。
- CH340 与 STM32 串口通信。
- K230D 串口输出和物块检测。
- USB Camera 采集与 AprilTag 检测。
- 底盘方向、`VEL`、STOP 和远程命令看门狗。
- Servo、Stepper 与完整三组机械臂搭建动作。
- K230D 视觉对准、前进、停车确认和人工触发搭建的第一版联调。

2026-09-11 后续现场结果确认：当前固件的 `LINE_START` 已能完成直线巡线，轻微蛇形暂可接受，本轮不调整控制参数。已知安全缺口是四路全白时仍继续前进；T 字路口也留待后续单独处理。

同场 AprilTag 单项测试可稳定输出正确 id 和 `error_x`，最远约 2.7 m 可检测。固定现场条件下记录的面积样本为：30 cm≈82767.9、60 cm≈16454.4、90 cm≈9839.2、120 cm≈5120.1、170 cm≈2594.9、200 cm≈1580.6、240 cm≈1307.5、270 cm≈1118.0。像素面积受 Tag、相机固定方式和视角影响，这些样本不能替代本次到站链路的 `arrival_area` 实车标定。

### 7.2 测试代码性质

`tests/` 主要是一组人工硬件诊断脚本。多数旧脚本在导入或启动时立即打开串口，其中 `motor_cmd_test.py` 会直接让底盘运动；不能批量运行，也不能在主控制器占用同一串口时运行。

AprilTag 两个旧脚本同样是相机人工测试，不包含断言。`test_line_to_tag.py` 和 `test_navigation_topology_live.py` 都是带人工启动确认且会驱动底盘的实机入口，不能由自动测试调用。`test_navigation_controller_unit.py` 使用 Fake 服务覆盖到站状态逻辑，`test_topological_navigation_unit.py` 覆盖双橙色区域语义、Tag 定位、策略首选、BFS 和无路径失败，`test_topological_navigation_live_unit.py` 用 Fake 串口与 detector 覆盖人工取消、错误 Tag 后仅由目标 Tag 更新拓扑，以及异常 STOP/资源释放；三者都不连接硬件。仓库仍没有持续集成配置或统一回归清单。

## 8. 当前未完成任务和已知缺口

1. `CompetitionRunner` 已定义第一版事件顺序，但 `LINE_DONE`、`TARGET_READY`、`ARM_DONE`、`BUILD_DONE` 目前均没有真实生产者，只能由测试或未来适配层注入。
2. `competition/main.py` 固定使用 simulation mode，尚未实现显式连接、握手、运行循环、统一退出 STOP 和资源关闭；当前可运动的联调入口仍是 `main_controller_align.py`。
3. 当前 STM32 直线巡线已现场可用，但仍没有 LINE_DONE 或路口事件；Pi 的 `ARRIVED` 仅由指定 AprilTag 面积连续达标并成功 STOP 产生。独立人工脚本已把 ARRIVED 接到拓扑位置更新，但尚未实机运行，也没有接入 CompetitionRunner。
4. 世界模型已有无坐标逻辑图，但仍缺少六个 Tag 中心 `(x, y, yaw)`、黑线节点/线段长度和关键停车点实测值；当前不能生成距离、转向或底盘动作。
5. AprilTag 目前只有像素误差和像素面积，没有相机内参、畸变校正或姿态估计；到站控制器只增加连续帧确认，`arrival_area` 仍需按实际停车距离标定。
6. K230D 的橙色、紫色检测结果如何触发取料尚未定义；TaskController 已有接口，但现有 ArmSequence 没有经过确认的独立橙色/紫色抓取动作。STM32 `Test_All()` 中虽有一个禁用的“±3000/±5000 + 底盘移动”旧候选，但缺少颜色/存放语义、调用证据、起始高度和实机版本确认，不能直接采用。
7. 新增链路层已引用 `config.py`；旧主控和部分诊断脚本仍复制路径或使用 `/dev/ttyUSB*` 等易漂移路径。
8. 没有 `requirements.txt` 或其他依赖锁定文件。已知 Pi 环境包含 `pyserial`、`numpy`、`opencv-python-headless` 和 `pupil_apriltags`，但版本没有记录。
9. `main_controller_align.py` 在模块顶层打开串口，且主 `try/finally` 在串口打开和启动等待之后才开始；它不适合被其他模块 import。
10. 当前主控启动时没有执行 `HELLO_STM32,1` 版本握手；独立机械臂入口会执行。
11. Pi 仓库不包含 STM32 源码；外部工程 `D:\Code\V1.3_vision\V1.3` 已完成本次巡线诊断修改和本地编译，但该目录没有可用 Git 历史，也没有与当前烧录固件/实机测试记录的版本对应关系。
12. `strategy/mission.py` 已明确第一版首选 Tag 2 / 左侧橙色材料区，但这套旧任务元数据尚未迁移到新的 `competition.MissionState`，两层接口暂时并存。

## 9. 现有文档评估

| 文档 | 是否可继续使用 | 需要注意/补充 |
| --- | --- | --- |
| `README.md` | 可作为快速入口 | 目录树和比赛入口说明尚未列出新增的视觉接近、导航与流程调度控制器 |
| `docs/PI_ENV.md` | 可继续使用 | SSH、部署路径、Git 安全流程和正式设备路径仍有效 |
| `docs/project_inventory.md` | 可作为历史盘点 | 它生成于机械臂重构前，依赖图和正式入口分析已经过时 |
| `docs/FIELD_DESCRIPTION.md` | 当前场地事实主文档 | 官方尺寸与 TODO 分界清晰；绝对坐标仍需现场测量 |
| `tests/README.md` | 可作为硬件测试安全说明 | 尚未列出两个 AprilTag 测试脚本 |
| `legacy/README.md` | 可继续使用 | 清楚标出旧脚本和旧协议，不应作为现役入口 |
| `docs/PI_STM32_PROTOCOL.md` | 可继续使用 | 已与外部 STM32 源码交叉验证；现场固件与本地构建的精确版本关联仍缺失 |
| `docs/SOFTWARE_ARCHITECTURE.md` | 本次新增 | 描述当前实际链路和目标整合边界 |

Pi 工作区和 Git 历史中没有 `docs/STM32_PROJECT_MAP.md` 和 `docs/STM32_SAFETY_PATCHES.md`；同名资料存在于外部 STM32 工程，但未纳入 Pi 仓库版本管理。需要跨仓库引用时应记录外部工程版本或构建标识，不能把外部未跟踪文件视为 Pi 仓库内容。

## 10. 下一阶段开发建议

建议按以下顺序推进，每一步先做无运动模拟，再做单项实机验证：

1. 在安全空旷场地单独运行 `test_navigation_topology_live.py`，从保守、可提前停车的 `arrival_area` 开始，回归错误 Tag、偶发丢帧、正常 ARRIVED、拓扑位置更新和 Ctrl+C STOP；该步骤同时承担实际停车距离标定。
2. 在上层路线执行器中把 TopologicalNavigator 的相邻节点对转换为“巡线至路口/地标、选择出口”的动作；真正的 T 字路口检测和转向仍应由 STM32 实时执行层实现并先单独验证，本轮没有实现。
3. 为 CompetitionRunner 设计独立、显式的 live 启动入口，补齐连接、握手、循环、STOP 确认和异常关闭生命周期；在完成安全审查前保持 `competition/main.py` 默认 simulation mode。
4. STM32 端事件经实机确认后，再由协议适配层把它们映射为 Runner 事件；不要直接把尚未验证的字符串加入 Stm32Link 或 NavigationController。
5. 现场测量 Tag、T 字路口和停车点，填写 `field_model.py` 中的 TODO；固定相机后再继续标定面积和误差阈值。
6. 给 K230D 解析、任务转移和协议格式继续增加不连接硬件的单元测试及串口 mock。
7. 统一配置与依赖记录，但迁移时保持已验证设备路径、速度、Servo 角度、Stepper 步数和延时原值不变。

当前最准确的完成度描述是：STM32 直线巡线和 AprilTag 单项检测已经现场可用，Pi 已完成 Tag 辅助停车的软件闭环、无坐标拓扑定位/BFS 规划，以及“ARRIVED 后更新逻辑位置”的人工实机入口和 Fake 回归；该新入口尚未实机运行，也不是 CompetitionRunner。Tag 2、Tag 4 两侧橙色来源已正确建模，第一版仍首选 Tag 2；`arrival_area`、全白安全、T 字路口、搭建区连线、Tag 姿态修正和完整比赛流程仍未完成。
