# RoboGame 当前状态

> 状态快照：2026-09-14
> 分析基准：本地 `main` 分支及当前工作区。
> 本文区分“代码可确认”“项目记录为已实机验证”和“尚待确认”，避免把历史脚本或规划接口误认为现役功能。

## 1. 项目简介

RoboGame 是一套参加 RoboGame 2026 竞技组比赛的移动机器人系统。机器人需要在固定场地中巡线和定位，识别并获取橙色、紫色建筑材料，最终在搭建区完成机械臂搭建任务。

当前采用上下位机分工：Raspberry Pi 5 负责视觉结果处理、比赛状态和任务决策；STM32F407ZGT6 负责底盘、巡线、Servo 和 Stepper 的实时执行。K230D / CanMV 作为独立视觉协处理器，把目标检测结果发送给 Pi；Logitech C270 USB Camera 直接连接 Pi，用于 AprilTag 检测。

项目已从单项硬件调试进入整车联调阶段，但还没有形成一条覆盖“启动—巡线—取橙色—取紫色—搭建”的全自动比赛主流程。2026-09-14 新增、2026-09-15 修正 `competition/grab_mission.py`（LINE_TO_GRAB_MISSION）：固定路线按巡线 0.38 m 到墙、机械臂准备、后退 0.05 m、抓取、抓起后再后退 0.30 m、放置执行；已经消除重复向墙前进 0.38 m 的危险流程（代码完成，待烧录/实车）。AprilTag、K230D 和比赛完整策略仍未接入该任务。

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
│   ├── grab_task.py                # Pi端封装已验证旧版抓取动作，接口 grab_orange()
│   ├── grab_mission.py             # LINE_TO_GRAB_MISSION 完整任务入口 run_grab_mission()
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
│   ├── test_old_grab_sequence.py # 旧抓放候选动作独立测试入口
│   └── test_line_to_wall.py      # 巡线、右转、重捕后定时到墙入口
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

当前资料还缺少完整 BOM、供电树、接线图以及可追溯的 STM32 固件版本标识；巡线传感器型号、源码引脚和直线巡线能力已确认。T 路口状态机现已对称接受 `1110/0111/1111`，能够锁存左右方向并通过编译；2.2 s 前进、左右 11.5 s 转向和重捕线行为仍需烧录实测。

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
| STM32 通信抽象层 | 骨架完成，未实机回归 | `Stm32Link` 封装 HELLO_STM32、STOP、VEL、NEXT_TURN、路口事件、Servo/Stepper，并要求显式 `connect()` |
| STM32 底盘、电机、巡线 | 直线可用，左右 T 待烧录实测 | 普通巡线 PID、0.80 m/s 速度和传感器读取未改；路口门控使用 1500 ms 启动保护、`0110` 连续 20 次 arming、`1110/0111/1111` 连续 8 次确认和 3000 ms cooldown。`1110` 锁存 LEFT、`0111` 锁存 RIGHT，左右均按 11.5 s 转向并同向搜索重捕 |
| Servo、Stepper、机械臂 | 动作层已验证 | `ArmSequence` 可复用，三组搭建动作和安全 HOME 已保留；不要改动参数 |
| 旧抓放候选测试 | 机械臂部分已实机验证，底盘段待实机 | `test_old_grab_sequence.py` 按旧 `main.c:214-248` 的 27 步顺序执行 Servo/Stepper 和三段底盘距离动作；其机械臂部分已在实机验证动作正常（2026-09-14 确认），底盘三段仍是 VEL 开环时间估算，保持原样不作正式封装 |
| K230D 物块视觉 | 单项检测已验证 | 当前主控能解析目标并完成对准/接近；完整橙色、紫色任务切换尚未接入 |
| K230D 接口层 | 骨架完成，未实机回归 | `K230Link` 封装连接生命周期、现有 `TARGET` / `NO_TARGET` 解析格式和最新消息读取 |
| K230D 对准到搭建 | 第一版已接通 | `main_controller_align.py` 可从 ALIGN/APPROACH 到 READY，再经人工 `b` 确认执行 ARM_BUILD |
| competition 视觉接近 | 软件迁移完成，未实机回归 | `VisualApproachController` 保留原阈值、EMA、速度、三帧 READY 和超时 STOP；尚未由 MissionController 自动运行 |
| AprilTag 检测 | 单项现场验证完成 | 场地可稳定输出正确 id、error_x 和像素面积，最远约 2.7 m 可检测；无 GUI，适合 SSH |
| AprilTag 服务层 | 接口完成 | `AprilTagService` 延迟创建原 `AprilTagDetector`，没有修改已现场验证的检测算法和参数 |
| AprilTagLocalization | 基础接口完成 | 每帧选择面积最大 Tag，输出语义名称和面积启发式 confidence |
| 比赛世界模型 | 场地事实纠正并结构化 | Tag 2/Tag 4 分别对应左右橙色材料区且都具备 `ORANGE_PICKUP`；搭建区保持独立，绝对坐标仍为 TODO |
| 拓扑定位与规划 | 第一版纯软件实现完成 | `TopologicalNavigator` 可由已知 Tag 更新逻辑位置，并用 BFS 返回已确认节点路径；不拥有或操作硬件 |
| 路线动作解析 | 轻量适配完成，待实车 | `RouteActionResolver` 使用离散边方向把 BFS 路线中的真实 T 路口转换为 LEFT/RIGHT/STRAIGHT；缺少方向、非路口转向和掉头均明确失败，不默认 STRAIGHT |
| 多路口路线执行 | 软件实现完成，待实车 | `RouteExecutor` 严格按 `JUNCTION_DETECTED -> TURN_STARTED -> TURN_DONE` 推进动作，全部路口完成后复用 NavigationController 做最终 Tag 到站；尚未接入 CompetitionRunner |
| 导航/任务状态定义 | 基础接口完成 | `competition.MissionState` 新增 FINISH、FAULT；`MissionController` 只负责显式切换和复位，没有运行时调度策略 |
| competition 导航控制 | Tag 辅助到站已完成第一轮实机联调 | Tag 3/4/5 均完成“巡线—目标 Tag—STOP ACK—ARRIVED”现场验证；加入 STOPDBG 后的后续测试未再复现早前 Tag 5 超时，timeout 和协议保持不变 |
| 巡线到拓扑人工联调 | Tag 3/4/5 已有实机成功记录 | `test_navigation_topology_live.py` 只在人工输入 START 后连接一个 Stm32Link 和一个 AprilTagService；ARRIVED 后才把目标 Tag 写入 TopologicalNavigator，并新增可选 `--stop-debug` 事务时间线 |
| 巡线传感器现场采样 | 只读工具完成，已有真实数据 | `test_line_sensor_capture.py` 约 10 Hz 发送唯一允许命令 `LINE_STATUS`；已确认普通线 `0110`、左侧扩宽 `1110`、右侧扩宽 `0111` 和全黑 `1111`，工具不启动任何执行器 |
| 路口协议与人工入口 | 软件完成，待烧录/实车 | `test_junction_turn.py` 支持直接指定动作，或由 start/target 规划路线并只取首个动作；严格输入 `START` 后才连接，`TURN_DONE` 后 STOP，不会连续跑多个路口 |
| 巡线到抓取墙测试 | Pi 入口完成，待烧录/实车 | `test_line_to_wall.py` 使用现有协议执行启动 STOP、预设 RIGHT、LINE_START、等待首个 `JUNCTION_DETECTED` 和 `LINE_REACQUIRED`，再保持 LINE_FOLLOW 约 3.22 s 后 LINE_STOP；STM32 已补上 `0111 -> RIGHT`，尚未烧录 |
| 巡线到抓取完整任务 | Pi 入口完成，待烧录/实车 | `competition/grab_mission.py` 在重捕后保持巡线 0.38 m 到墙并 LINE_STOP，不再额外向墙前进；随后准备机械臂、后退 0.05 m、抓取、后退 0.30 m、放置。全程 STOP/ERR/Ctrl+C 安全收尾，未实车 |
| competition 流程调度 | 第一版模拟流程完成 | `CompetitionRunner` 通过外部事件串联巡线、橙色目标接近、抓取、搭建和结束；默认 simulation mode 不调用硬件控制器 |
| 机械臂任务控制 | 橙色抓取已封装，待接入控制器 | `TaskController.build()` 复用 `ArmSequence.run_all()`，`reset_arm()` 复用 `safe_home()`；新增 `competition/grab_task.py` 把已实机验证的旧版机械臂抓取动作封装为 `grab_orange()`，`TaskController.grab_orange()` 保持 NotImplemented，接线留待后续 |
| 比赛入口 | 安全模拟入口完成 | `python -m competition.main` 初始化 CompetitionRunner，默认只打印 simulation/START 及子控制器状态，不连接或运行任何硬件 |
| 完整比赛闭环 | 未完成 | Runner 已串起第一版橙色任务模拟流程；可执行橙色抓取动作已由 `competition/grab_task.py` 提供但未接入 Runner，真实事件、紫色任务和 live 生命周期仍缺失 |

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

2026-09-14 后续：`tests/test_old_grab_sequence.py` 的机械臂动作已在实机验证动作正常。据此新增 `competition/grab_task.py`。2026-09-15 复核发现初版在剔除三段底盘动作时误删了旧步骤 8、18 的两个 500 ms 等待，现已补回；模块当前包含全部 24 个非底盘源码动作，Servo 角度、Stepper 步数/pps、延时和相对顺序均由单元测试与旧动作表逐项比较。模块自带启动/结束 STOP、执行中 ERR/Ctrl+C/异常即时 STOP，并逐命令记录 TX、ACK 和 DONE（Servo 的 DONE 明确标为协议不适用）。它可以自建自关 `Stm32Link`，也可以复用调用方已连接的连接。`TaskController.grab_orange()` 仍返回 `NotImplemented`，没有接入 CompetitionRunner。

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
- `continue_line_to_tag(...)` 只配置相同的 Tag 到站观测，不再次发送 `LINE_START`；它只供已经处于 STM32 LineNav 运行期的 RouteExecutor 在最后一个路口完成后交接使用。
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
- 该模块不导入 Stm32Link、AprilTagService 或 K230D，不发送 `LINE_START`、VEL、STOP，也不执行路口动作。独立人工入口会在 NavigationController 到达目标 Tag 后显式调用 `update_from_tag(tag_id)`；CompetitionRunner 尚未接入这条链路。

当前 RouteActionResolver 的真实能力边界：

- `TopologicalNavigator.plan_route()` 只负责返回节点序列；`RouteActionResolver` 只读取 `field_model` 的节点类型和离散边方向，把相邻边的朝向变化转换为 `LEFT`、`RIGHT`、`STRAIGHT`。
- 动作只在 `kind=junction` 的节点生成。`gray_corridor` 等非路口节点可以传递朝向，但不会占用一个 STM32 pending turn。
- 当前图示方向元数据为：启动区至启动 T、左右材料竖线为 `NORTH`；启动 T 至灰廊、灰廊至左材料 T、左材料 T 至右材料 T 为 `EAST`；右材料 T 至屋顶 T 及屋顶 T 至右橙色区为 `NORTH`；屋顶 T 至紫色区为 `WEST`。反向边由解析器取相反方向。这些只是项目坐标系中的离散关系，没有新增未经测量的 x/y。
- 若路线从一个 T 路口开始，调用方必须提供当前 `NORTH/SOUTH/EAST/WEST` heading；否则第一动作无法可靠判断并抛出 `RouteActionUnknownError`。缺少方向、非相邻节点、非路口处改变方向或 180° 掉头同样明确失败，不会默认选择 `STRAIGHT`。
- `build_zone`、`right_corridor_reference`、`bottom_direction_reference` 仍没有已确认拓扑边，因此不能规划或解析到这些节点。己方半场镜像和现场朝向仍需人工确认，未由示意图像素推算坐标。
- 该解析器不导入硬件层、不保存比赛任务状态，也不发送串口。它和尚未接入 CompetitionRunner 的人工测试入口之间只有纯数据调用。

当前 RouteExecutor 的真实能力边界：

- `prepare_route()` 只调用 TopologicalNavigator 和 RouteActionResolver，保存 route、actions、目标 Tag、显式 `arrival_area` / `confirm_frames` 和当前动作下标，不打开设备或发送运动命令。
- 只有显式 `start()` 才先预设第一个 `NEXT_TURN`，再发送并等待 `ACK,LINE_START`。之后单步 `poll()` 是路口阶段唯一的 STM32 回复读取者。
- 状态依次包含 `IDLE -> PLANNING -> READY -> WAITING_JUNCTION -> TURNING -> WAITING_TURN_DONE`；最后一个匹配的 `TURN_DONE` 后先进入 `ROUTE_TURNS_DONE`，再交接到 `FINAL_TAG_APPROACH -> ARRIVED -> FINISHED`。当前动作下标、当前动作和全部动作均可查询。
- 一个 `TURN_DONE` 只完成当前一个动作。只有方向与当前预期完全一致且事件顺序正确，才递增下标并发送下一个 `NEXT_TURN`；因此不会在前一动作完成前覆盖 STM32 pending turn。方向不匹配、重复或乱序事件均进入 `FAULT`。
- 路口动作全部完成不代表到站。此后 RouteExecutor 停止读取 STM32，由 NavigationController 使用同一个 AprilTagService 逐帧观察目标；它复用既有连续面积确认和 STOP ACK 逻辑，并且不会再次发送 `LINE_START`。
- 只有 NavigationController 已进入 `ARRIVED`、`TopologicalNavigator.update_from_tag(target_tag)` 返回预期目标位置后，RouteExecutor 才进入 `ARRIVED`，下一单步进入 `FINISHED`。没有路口动作的路线会直接调用既有 `start_line_to_tag()` 进入最终 Tag 阶段。
- Pi 侧 watchdog 当前默认等待下一路口 30 s、等待转向事件完成 5 s、等待最终 Tag 30 s；它们不修改 STM32 内部 1200/2000 ms timeout，并可由人工入口显式覆盖。超时、协议错误、规划/解析错误、AprilTag 异常或 NavigationController FAULT 都会锁定 `FAULT` 并尝试 STOP。
- target Tag 必须与 field model 的目标节点映射一致。`arrival_area` 必须由调用方明确给出，没有把不同 Tag 的现场数据硬编码进 RouteExecutor。
- 该执行器未接 K230D、机械臂或 CompetitionRunner，多路口实车验证尚未进行。
- 近距离时 Tag 可能因相机安装较低而移出画面；NavigationController 当前只会把丢帧清零，尚无“曾经很近后突然消失”的专用安全判定。RouteExecutor 的最终 Tag watchdog 会超时停车，但不能替代后续现场确认和视觉安全设计。

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

#### 6.3.1 旧抓放候选动作测试入口

```bash
python3 tests/test_old_grab_sequence.py --dry-run
python3 tests/test_old_grab_sequence.py \
  --command-speed 0.80 \
  --forward-actual-speed 0.118 \
  --backward-actual-speed 0.118
```

`--dry-run` 只打印旧 27 步动作表，不打开串口。实机入口必须输入严格的 `START` 才连接；连接后的第一条命令为 `STOP` 并等待 ACK，随后按原顺序使用 `SERVO_SET`、`STEPPER_MOVE` 和 `VEL`，正常结束再次 `STOP`。每条命令分别打印发送、ACK 和 DONE；`SERVO_SET`、`VEL`、`STOP` 的现有协议没有 DONE，日志会明确标记为不适用，`STEPPER_MOVE` 则必须收到真实 `STEPPER_MOVE_DONE`。Ctrl+C、任意 `ERR` 或其他异常都会优先请求 `STOP` 再退出。

脚本保留旧 `main.c:214-248` 的全部 27 个源码步骤及原延时。UART 语法不变，STM32 只把 `VEL.Y` 合法范围从 ±0.30 放宽到 ±0.80；三段 `Move_Distance` 仍由 Pi 每 100 ms 刷新 `VEL` 并按时间估算。脚本默认发送 ±0.80 命令值，计时则使用独立的前进/后退实测速度参数；以目前唯一可用的巡线实测基线 0.118 m/s 计算，+0.50 m、-0.05 m、-0.30 m 分别持续约 4.24 s、0.42 s、2.54 s，每段结束发送 STOP 并等待 ACK。后退暂沿用前进基线，必须单独实测后更新；该实现不是旧版编码器闭环，实际行程仍会受加减速、打滑、负载和通信延迟影响。旧固件未烧录新上限时会返回 `ERR,VEL`，脚本随后立即 STOP。该入口不表示完整抓放候选或颜色/箱位语义已经验证，也没有修改或复用 `ArmSequence` 的既有搭建动作。

#### 6.3.2 正式抓取任务模块入口

```bash
python3 -m competition.grab_task --dry-run
python3 -m competition.grab_task
```

`competition/grab_task.py` 是旧候选机械臂部分的正式封装，动作表包含旧序列中全部 24 个非底盘动作，也保留原本紧跟 0.05 m、0.30 m 底盘动作后的两个 500 ms 等待。`--dry-run` 只打印动作表，不打开串口。实机入口必须输入严格 `START` 才连接；连接后的第一条命令为 `STOP` 并等待 ACK，动作日志逐项打印 TX、ACK 和 DONE，正常结束再次 `STOP`。Ctrl+C、任意 `ERR`、串口异常都会优先请求 `STOP` 再退出。

函数接口为 `grab_orange(stm32=None, *, output=print, clock=time.monotonic)`：不传连接时自建自关 `Stm32Link`；传入已连接的 `Stm32Link` 时复用它且不关闭。`clock` 只用于无硬件确定性测试，实车默认使用单调时钟。独立入口负责机械臂动作与安全收尾；固定路线任务则调用分段接口，在原位置插入底盘后退动作。

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

该入口已经取得以下现场结果：Tag 4 使用 `arrival_area=10000` 可稳定在约 80 cm 停车并更新为 `RIGHT_ORANGE_MATERIAL`；Tag 5 使用 `arrival_area=10000` 基本可在约 80 cm 停车；Tag 3 因前方线路较短，使用 `arrival_area=20000`、`confirm_frames=3` 连续三次在约 57 cm 停车并更新为 `PURPLE_MATERIAL`。这些是对应现场布置的实测记录，不改成代码内置默认值。

Tag 5 早前测试曾偶发车辆已停车但首次 `ACK,STOP` 等待超时，随后 cleanup STOP 又收到 ACK。为定位该问题，入口新增 `--stop-debug`：只针对 STOP 事务记录单调时钟、写完成耗时、写前 `serial.in_waiting`、每条非空原始回复及最终匹配/超时；加入诊断后的后续实测未再复现异常，尚不足以确认或排除早前假设。本轮继续保持当前 ACK timeout、STOP 协议和 STOP/ARRIVED 条件不变。该入口仍不是 `CompetitionRunner`，不执行 BFS 路线，也不调用新增的 `NEXT_TURN` 路口能力。

下一轮 Tag 5 现场诊断命令为：

```bash
python3 tests/test_navigation_topology_live.py \
  --tag-id 5 \
  --arrival-area 10000 \
  --confirm-frames 3 \
  --stop-debug
```

运行者需要保留从到站前 `confirm_count` 开始，到首次 STOP、FAULT、cleanup STOP 和退出为止的全部 `[STOPDBG]` 与状态行。

当前 STOP 调用链和故障路径的代码结论：

```text
NavigationController.update()
  -> _confirm_arrival()
  -> Stm32Link.stop(require_ack=True, timeout=2.0)
  -> 写入并 flush "STOP\n"
  -> _wait_for("ACK,STOP", 2.0)
     -> 成功：ARRIVED
     -> 超时：NavigationController 置 FAULT 并重新抛出
  -> live test except 打印 FAULT
  -> live test finally 再发送一次 cleanup STOP 并等待 ACK
```

因此到站 STOP 超时时，总计发送两次 STOP：首次到站请求和一次 cleanup 请求，没有第三次隐式 STOP。由于现有协议没有 request id，cleanup 等待到的 `ACK,STOP` 可能是首次 STOP 的迟到回复，也可能是 cleanup STOP 的新回复；仅凭成功字符串无法区分，必须结合新增时间线判断。

### 6.4.1 巡线传感器只读采集入口

新增的人工采样入口为：

```bash
python3 tests/test_line_sensor_capture.py
```

它创建并独占一个 `Stm32Link`，默认约 10 Hz 且只发送现有只读命令 `LINE_STATUS`，把 `timestamp`、`sensor_ok`、`s1..s4`、`active` 输出到终端和 `logs/line_sensor_capture_YYYYMMDD_HHMMSS.csv`。`logs/` 已由 `.gitignore` 忽略。运行中可直接输入 `STRAIGHT`、`LEFT_OFFSET`、`RIGHT_OFFSET`、`T_APPROACH`、`T_ENTRY`、`T_CENTER`、`T_EXIT`、`ALL_WHITE` 或 `ALL_BLACK` 并回车；工具打印 `MARK,<名称>`，并把标记附在下一条传感器样本上。输入 `Q` 或 Ctrl+C 后只关闭串口，不发送 STOP 或任何其他命令。

该工具不切换 ControlMode，不启动 LineNav、底盘、Servo、Stepper，不打开 Camera 或 K230D。正常静止采集时 `active` 应保持 0；`sensor_ok=0` 时四路字段只是 STM32 最近一次成功读取的缓存，必须视为无效样本。其真实 CSV 已用于第一版 T 字路口规则；该采样入口本身仍保持完全只读。

### 6.4.2 路口转向人工入口

```bash
python3 tests/test_junction_turn.py --turn LEFT
```

也可以先做 BFS，并从完整动作序列中只选择第一个动作：

```bash
python3 tests/test_junction_turn.py \
  --start start_zone \
  --target purple_material
```

若 `--start` 自身就是 T 路口，还必须用 `--heading NORTH|SOUTH|EAST|WEST` 给出机器人当前朝向，解析器不会猜测。脚本在确认前打印完整 `route`、`actions`、`next_action` 和 `target`；它只把 `next_action` 写入一次 `NEXT_TURN`。`RIGHT` 与 `STRAIGHT` 仍可直接指定，且 `STRAIGHT` 只有在人工确认路口存在真实直行出口时才允许测试。

只有输入严格大写 `START` 后，脚本才构造/连接 `Stm32Link`，依次等待 `ACK,NEXT_TURN,<turn>` 与 `ACK,LINE_START`，随后打印 `JUNCTION_DETECTED`、`TURN_STARTED,...`、`TURN_DONE,...`。收到 `TURN_DONE` 后立即要求 STOP ACK 并退出，不会自动预设或通过第二个路口。`ERR,TURN_TIMEOUT`、`ERR,REACQUIRE_TIMEOUT`、方向不匹配和其他协议异常都会打印 `FAULT`，并在清理路径尝试 STOP。Ctrl+C 同样 STOP 并关闭串口。该入口不会被自动运行，路线动作适配尚未接入 CompetitionRunner。

### 6.4.3 多路口路线人工入口

第一条建议现场验证的两动作路线使用已验证过到站阈值的 Tag 4：

```bash
python3 tests/test_multi_junction_route.py \
  --start right_material_t_junction \
  --target right_orange_material \
  --target-tag 4 \
  --heading EAST \
  --arrival-area 10000 \
  --confirm-frames 3
```

纯软件预览为 `right_material_t_junction -> roof_material_t_junction -> right_orange_material`，动作是 `LEFT -> STRAIGHT`。`--heading EAST` 描述进入当前起始 T 路口前的实际朝向；现场摆车位置和方向必须与它一致。`arrival_area=10000`、`confirm_frames=3` 只对应已有 Tag 4 布置记录，不应复制给未经验证的 Tag 2 或其他新布置。

脚本在 START 前只规划并打印 start、target、route、actions、target Tag 和到站参数；不是严格大写 `START` 时不会构造或连接 STM32/AprilTag 服务。确认后只创建一个 Stm32Link、一个 AprilTagService 和一个 RouteExecutor。RouteExecutor 在路口阶段独占 STM32 读取，最后一个 `TURN_DONE` 后切换到 NavigationController 的相机单步更新；不存在第二串口线程或并发 `readline()`。

运行输出会依次标出当前 `action i/n`、`JUNCTION_DETECTED`、`TURN_STARTED,...`、`TURN_DONE,...`、`ALL JUNCTION ACTIONS DONE`、`SEARCHING TARGET TAG`、`TAG ... ARRIVED` 和 `ROUTE FINISHED`。只有目标 Tag 连续达标并收到 STOP ACK、拓扑位置更新成功后才会完成。Ctrl+C、watchdog、协议/相机异常和状态错序都会尝试 STOP 并释放相机与串口；完成路径依赖 NavigationController 已确认的 STOP ACK 保持停车。该入口尚未进行多路口实车验证，也没有接入 CompetitionRunner。

### 6.4.4 巡线到抓取墙人工入口

新增独立入口：

```bash
python3 tests/test_line_to_wall.py --dry-run
python3 tests/test_line_to_wall.py
```

实机命令只有在操作者严格输入大写 `START` 后才连接 STM32，单次执行以下流程，不自动循环：

```text
STOP / ACK
  -> NEXT_TURN,RIGHT / ACK
  -> LINE_START / ACK
  -> JUNCTION_DETECTED
  -> 等待 LINE_REACQUIRED
  -> 保持 STM32 LINE_FOLLOW，不发送 VEL，也不重复 LINE_START
  -> 0.38 / 0.118 = 3.22 s 后 LINE_STOP / ACK
```

`TARGET_DISTANCE = 0.38`、`REAL_SPEED = 0.118` 和 `LINE_DURATION = TARGET_DISTANCE / REAL_SPEED` 集中定义在脚本顶部；没有写死 `sleep(3.2)`。从收到 `LINE_REACQUIRED` 的时刻开始计时，计时期间仍持续读取串口，任意 `ERR,...`、通信异常、错误的转向方向或意外第二个 `JUNCTION_DETECTED` 都进入故障清理并尝试全局 STOP。Ctrl+C 同样请求 STOP；正常完成则以 `LINE_STOP` ACK 确认停车，并只输出抓取准备就绪标记，不执行机械臂动作。入口不导入 AprilTag、K230D 或机械臂代码，也没有修改 `main_controller_align.py`、competition 状态机、既有巡线逻辑或 STM32 协议。

当前外部 STM32 源码已补充物理 `0111` 右侧候选和负 Omega 右转状态，并会输出 `TURN_STARTED,RIGHT` / `TURN_DONE,RIGHT`；脚本仍严格校验方向。Pi 脚本本身没有新增协议要求，但车上必须重新烧录本次生成的新 HEX 才能获得该行为；旧固定 LEFT 固件仍会使脚本安全报错并 STOP。本轮没有自动烧录或运行车辆。

### 6.4.5 巡线到抓取完整任务入口（LINE_TO_GRAB_MISSION）

```bash
python3 -m competition.grab_mission --dry-run
python3 -m competition.grab_mission
```

`competition/grab_mission.py` 把已分别验证的巡线、路口和机械臂模块连接为第一条固定路线完整任务，不修改任何已有模块、测试脚本或 STM32 固件。`--dry-run` 只打印流程和参数，不打开串口；实机入口必须输入严格 `START` 才连接，单次执行后不自动循环。

状态机依次为 `IDLE -> LINE_FOLLOW -> WAIT_JUNCTION -> TURNING -> LINE_REACQUIRED -> PREPARE_ARM -> BACKOFF -> PICK -> RETREAT -> PLACE -> DONE`；任意串口异常、STM32 `ERR`、协议错序或 Ctrl+C 进入 `ERROR`，先请求全局 STOP 再重新抛出。

```text
STOP / ACK
  -> NEXT_TURN,RIGHT / ACK
  -> LINE_START / ACK
  -> JUNCTION_DETECTED
  -> TURN_STARTED,RIGHT -> TURN_DONE,RIGHT + LINE_REACQUIRED（两者顺序兼容，方向严格校验）
  -> 保持 LINE_FOLLOW 约 3.22 s、共 0.38 m 到墙 -> LINE_STOP / ACK
  -> 执行旧步骤 1-5 的机械臂准备
  -> VEL,0.00,-0.80,0.00 后撤约 0.42 s（0.05 m）-> STOP / ACK -> 等待 500 ms
  -> 下降、夹取、抬升
  -> VEL,0.00,-0.80,0.00 后撤约 2.54 s（0.30 m）-> STOP / ACK -> 等待 500 ms
  -> 放置、释放、收回 -> STOP / ACK
```

参数集中在模块顶部：`TARGET_DISTANCE=0.38`、`REAL_SPEED=0.118`、`LINE_DURATION=TARGET_DISTANCE/REAL_SPEED`、`BACKOFF_DISTANCE=0.05`、`POST_PICK_RETREAT_DISTANCE=0.30`。0.38 m 只由重捕后的 LINE_FOLLOW 执行一次，不再发送正向 VEL；两个后退段使用 `VEL.Y=-0.80`、每 100 ms 刷新并在结束时等待 STOP ACK。后退实测速度尚未单独标定，暂沿用 0.118 m/s 前进基线，因此 0.05 m 和 0.30 m 距离仍需实车校准。该入口不导入 AprilTag、K230D，也没有新增 STM32 协议要求。

### 6.5 STM32 入口

STM32 不在本 Git 仓库中。当前外部源码入口为 `D:\Code\V1.3_vision\V1.3\Core\Src\main.c`，Keil 工程为 `D:\Code\V1.3_vision\V1.3\MDK-ARM\f407_ustc_2022_robogame.uvprojx`。2026-09-14 的坡道抗误检版本已使用 ARMCC V5.06 update 7 全量重编译，结果为 0 error、1 个原有 `Test_All` 未引用警告；没有自动烧录或运行。

同日按旧抓放候选测试需要，仅将外部 STM32 源码的 `RPI_VEL_Y_MAX_M_S` 从 `0.30f` 放宽到 `0.80f`；X 仍为 ±0.30，Omega 仍为 ±1.50，协议语法、ACK/ERR、REMOTE watchdog 和控制逻辑未改。ARMCC V5.06 update 7 再次全量构建并生成 HEX，结果为 0 error、1 个原有 `Test_All` 未引用警告；尚未烧录或实车验证。未烧录该版本时，旧固件会拒绝 `VEL.Y=±0.80`。

2026-09-14 后续根据实车未识别右侧 T 的结果，候选从 `1110/1111` 扩展为左右对称的 `1110/0111/1111`。新增 raw `0x0E`（物理 `0111`）右侧证据锁存、`TURN_RIGHT` 状态、负 Omega 右转和同方向搜索；右转时间暂沿用同次标定的 11500 ms。纯 C++ 主机测试通过，ARMCC V5.06 update 7 全量构建为 0 error、1 个原有 warning，并重新生成 HEX；尚未烧录或实车验证。

当前 `LINE_START` 非阻塞状态机如下：

```text
LINE_START
  -> JUNCTION_DISABLED (startup guard 1500 ms)
  -> JUNCTION_ARMED (仅在 0110 连续 20 次后进入)
  -> JUNCTION_CONFIRM (首次检测到 1110/0111/1111，只建立候选)
  -> JUNCTION_DETECTED (三种候选连续共 8 次，期间未恢复 0110)
  -> ADVANCE_TO_CENTER (2200 ms，保持当前巡线前进速度)
  -> 1110: TURN_LEFT / 0111: TURN_RIGHT / 1111: pending LEFT或RIGHT消歧
     (左右均11500 ms，Omega分别为+0.80/-0.80)
  -> SEARCH_LINE (保持当前转向，最长 2000 ms)
  -> LINE_REACQUIRED
  -> JUNCTION_COOLDOWN (3000 ms)
  -> JUNCTION_DISABLED -> 重新等待 0110 连续 20 次
```

`JUNCTION_CONFIRM` 把进入候选的第一帧计入 8 次；确认期间出现任意非 `1110/0111/1111` 输入（包括恢复 `0110`）都会取消候选、输出 `JUNCTION_REJECTED` 并回到 `JUNCTION_DISABLED`，随后必须重新取得连续 20 次 `0110` 才能 armed。连续候选中一旦取得单侧证据，后续 `1111` 不会抹掉方向；全黑或左右证据冲突时优先使用 pending LEFT/RIGHT，没有可用预设则保留旧 LEFT 回退。`SEARCH_LINE` 的接受线型、连续次数和 timeout 未改，只把转向符号改为跟随当前方向。重捕成功后开始完整 3000 ms cooldown；事件格式保持不变并新增实际的 RIGHT 方向输出。

这版固件已能依据单侧传感器证据执行 LEFT/RIGHT，并让 `NEXT_TURN,LEFT/RIGHT` 对全黑或冲突候选起消歧作用；它仍不理解 Pi 的完整路线、AprilTag 或比赛任务。`STRAIGHT` 的现有命令枚举仍保留，但本轮没有新增 STRAIGHT 动作状态。右转和 Pi 路线组合尚未实车验证，不能描述为完整多路口闭环。

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

2026-09-11 后续现场结果确认：当前固件的 `LINE_START` 已能完成直线巡线，轻微蛇形暂可接受，本轮不调整控制参数。2026-09-14 坡道抗误检门控及左右候选通过纯 C++ 主机测试和 ARMCC V5 全量编译；主机测试包含 10 分钟 `0110` 仿真、100 轮少于 8 帧的 `1110/0111/1111` 扰动、三种候选的 8 帧确认、左右方向锁存、拒绝与 cooldown 边界。以上是软件仿真而非实车结论，仍需烧录后完成平路、坡道及左右真实 T 测试；Advance 2.2 s 和左右 Turn 11.5 s 均保持现有标定值。普通 LINE_FOLLOW 四路全白时仍继续前进，是继续保留的已知安全缺口。

同场 AprilTag 单项测试可稳定输出正确 id 和 `error_x`，最远约 2.7 m 可检测。固定现场条件下记录的面积样本为：30 cm≈82767.9、60 cm≈16454.4、90 cm≈9839.2、120 cm≈5120.1、170 cm≈2594.9、200 cm≈1580.6、240 cm≈1307.5、270 cm≈1118.0。像素面积受 Tag、相机固定方式和视角影响，这些样本不能替代本次到站链路的 `arrival_area` 实车标定。

后续拓扑入口现场确认：Tag 4 和 Tag 5 使用 `arrival_area=10000` 时约在 80 cm 停车；Tag 3 使用 `arrival_area=20000`、`confirm_frames=3` 连续三次约在 57 cm 停车，并成功输出 `confirmed_tag=3`、`current_location=PURPLE_MATERIAL`。Tag 5 早前偶发的 STOP ACK timeout 在加入诊断后的后续实测中未再出现；根因仍未确认，不应把“迟到 ACK”、UART 发送失败或第二读取者中的任何一个写成既定事实。

### 7.2 测试代码性质

`tests/` 主要是一组人工硬件诊断脚本。多数旧脚本在导入或启动时立即打开串口，其中 `motor_cmd_test.py` 会直接让底盘运动；不能批量运行，也不能在主控制器占用同一串口时运行。

AprilTag 两个旧脚本同样是相机人工测试，不包含断言。`test_line_to_tag.py`、`test_navigation_topology_live.py`、`test_junction_turn.py` 和 `test_multi_junction_route.py` 都是带人工启动确认且会驱动底盘的实机入口，不能由自动测试调用。`test_old_grab_sequence.py` 只有在严格输入 `START` 后才驱动底盘、Servo 和 Stepper，并在启动、每段定时距离结束、Ctrl+C、ERR、异常和正常完成路径请求 STOP；它也不能由自动测试调用。`test_line_sensor_capture.py` 只允许 `LINE_STATUS`，不会驱动任何执行器，但会打开真实 STM32 串口，仍需人工单独运行。

`test_route_executor_unit.py` 使用四类 Fake 覆盖两/三动作严格推进、方向不匹配、重复事件、Pi watchdog、全部路口完成但未到站、最终 ARRIVED 后拓扑更新、NavigationController FAULT、AprilTag 异常、STM32 协议错误、空 route 和零路口路线。`test_multi_junction_route_unit.py` 确认未输入严格 `START` 时 STM32/Camera factory 均不会调用。`test_navigation_controller_unit.py` 新增确认最后路口交接不会重复发送 `LINE_START`。`test_line_to_wall_unit.py` 使用 Fake STM32/时钟覆盖距离公式、无需硬件的 dry-run、严格 START、单次 RIGHT 路口流程、计时期间保持 LINE_FOLLOW、错误方向、ERR 和 Ctrl+C STOP。`test_grab_task_unit.py` 逐项核对 24 个非底盘动作，覆盖 Servo/Stepper ACK/DONE 日志、启动/结束 STOP、异步 ERR STOP 和未连接保护。`test_grab_mission_unit.py` 覆盖完整成功流程，明确断言不存在正向 VEL 第二段，并验证准备、5 cm 后退、抓取、30 cm 后退和放置的顺序；所有单元测试都不连接硬件。

## 8. 当前未完成任务和已知缺口

1. `CompetitionRunner` 已定义第一版事件顺序，但 `LINE_DONE`、`TARGET_READY`、`ARM_DONE`、`BUILD_DONE` 目前均没有真实生产者，只能由测试或未来适配层注入。
2. `competition/main.py` 固定使用 simulation mode，尚未实现显式连接、握手、运行循环、统一退出 STOP 和资源关闭；当前可运动的联调入口仍是 `main_controller_align.py`。
3. 当前 STM32 直线巡线和独立脚本的 ARRIVED→拓扑更新已完成 Tag 3/4/5 第一轮现场验证；新增 `0111 -> RIGHT`、方向锁存和 Pi `test_line_to_wall.py` 组合仅完成软件测试与编译，尚未烧录/实车。STRAIGHT 动作状态仍未在当前 STM32 非阻塞状态机中恢复。早前 Tag 5 偶发 STOP ACK timeout 的诊断能力继续保留，后续实测暂未复现，根因尚未确认。
4. 世界模型已有无坐标逻辑图和离散转向，但仍缺少六个 Tag 中心 `(x, y, yaw)`、黑线节点/线段长度和关键停车点实测值；当前不能生成距离动作或基于坐标的轨迹。
5. AprilTag 目前只有像素误差和像素面积，没有相机内参、畸变校正或姿态估计；到站控制器只增加连续帧确认，`arrival_area` 仍需按实际停车距离标定。近距离 Tag 移出画面后的专用安全判定尚未实现。
6. K230D 的橙色、紫色检测结果如何触发取料尚未定义；旧候选的机械臂部分已实机验证并封装为 `competition/grab_task.py` 的 `grab_orange()`，但 `TaskController`/`CompetitionRunner` 尚未接线，紫色抓取仍没有经过确认的动作。固定路线上的巡线到抓取闭环已由 `competition/grab_mission.py` 组成（代码完成，待实车）；视觉触发、多物块和比赛策略接线仍未完成。
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

1. 烧录前人工复核 1500 ms / 20 次 / 8 次 / 3000 ms 四个检测参数，以及正 Omega 左转、负 Omega 右转符号、2.2 s/11.5 s 定时值和 2 s 搜索 timeout；保留当前固件以便回退。
2. 先架空轮或安全支撑验证 STOP 能在 `ADVANCE_TO_CENTER`、`TURN_LEFT`、`TURN_RIGHT`、`SEARCH_LINE` 任一阶段立即急停，并验证 `1110 -> LEFT`、`0111 -> RIGHT`、全黑加 `NEXT_TURN,RIGHT -> RIGHT` 的事件方向。
3. 再进入低风险场地，依次完成平路直线连续 10 分钟无误触发、上坡扰动不误触发、左右真实 T 都能完整输出 `JUNCTION_DETECTED -> ADVANCE -> TURN -> SEARCH -> LINE_REACQUIRED`；记录传感器序列、实际前进距离、左右转角和重捕用时。
4. 单 T 左右转稳定后使用 `test_line_to_wall.py` 验证右转重捕后仅巡线一次 0.38 m 到墙；随后分阶段验证机械臂准备、0.05 m 后退、抓取、0.30 m 后退和放置，全部单项安全后才运行 `competition/grab_mission.py`。两个后退段应分别测量实际速度，不能直接把当前 0.118 m/s 前进基线视为最终标定。STRAIGHT 和多路口 RouteExecutor 组合仍需另行恢复/验证。
5. 多路口与最终 Tag 到站稳定后，再设计 RouteExecutor 接入 CompetitionRunner 的事件和 live 生命周期；当前 `competition/main.py` 继续固定 simulation mode。
6. 继续单独处理普通 LINE_FOLLOW 下持续 `0000` 和近距离 Tag 移出画面的安全风险，并现场测量 Tag、T 字路口和停车点；本轮未改变 AprilTag 算法或坐标 TODO。

当前最准确的完成度描述是：STM32 直线巡线、AprilTag 单项检测以及独立人工入口的“指定 Tag 到站—STOP ACK—ARRIVED—拓扑位置更新”已有现场结果。STM32 单 T 软件状态机具备 1500 ms 启动保护、稳定中心线 arming、`1110/0111/1111` 8 帧候选确认、左右方向锁存和 3000 ms cooldown；纯检测器主机测试和 Keil 全量编译通过，但新右转尚未烧录实测。Advance、Search、普通巡线 PID 与速度保持原实现，RIGHT 使用与 LEFT 等幅反向 Omega 和相同 11.5 s 初值。Pi 的 `test_line_to_wall.py` 已与 RIGHT 事件格式匹配；STRAIGHT、多路口 RouteExecutor 实车组合、普通巡线全白安全、近距离 Tag 丢失、搭建区连线、Tag 姿态修正和完整比赛流程仍未完成。
