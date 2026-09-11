# RoboGame 软件架构

> 架构快照：2026-09-11。本文描述仓库当前实际状态，同时把尚未接入的规划模块明确标出。

## 1. 系统边界

RoboGame 使用 Raspberry Pi 5 作为比赛上位机，STM32F407ZGT6 作为实时执行控制器。视觉有两条独立输入链路：K230D 输出已识别的物块结果，USB Camera 在 Pi 本机完成 AprilTag 检测。

```text
K230D / CanMV ──USB Serial──┐
                            │
Logitech C270 ──USB Video───┼──> Raspberry Pi 5
                            │    ├─ 视觉解析与定位
                            │    ├─ 状态/任务决策
                            │    ├─ 世界模型
                            │    └─ STM32 唯一串口所有者
                            │              │
                            └──────────────┼── CH340 USB Serial
                                           v
                                     STM32F407ZGT6
                                     ├─ 麦克纳姆底盘
                                     ├─ 本地巡线
                                     ├─ 5 路 Servo
                                     └─ TB6600 Stepper
```

## 2. 当前软件分层

| 层 | 当前文件 | 职责 | 接入状态 |
| --- | --- | --- | --- |
| 设备/协议 | `config.py` | 保存正式设备路径和 baudrate | 存在，但入口仍重复常量 |
| 设备/协议 | `hardware/stm32_link.py` | STM32 串口所有权，HELLO/STOP/VEL/Servo/Stepper 请求与 ACK/DONE/ERR | 骨架完成；显式连接，未迁移旧入口 |
| 外部实时执行 | `D:\Code\V1.3_vision\V1.3` | STM32 USART1 解析、控制权、底盘、巡线、Servo、Stepper | 不在 Pi Git 仓库；当前直线巡线已现场基本验证，全白和 T 字路口待处理 |
| 设备/协议 | `arm_sequence.py` 中的请求方法 | HELLO、STOP、Servo、Stepper 请求/回复 | 已用于正式入口 |
| K230D 输入 | `vision/k230_link.py` | 显式连接 K230D，解析 TARGET/NO_TARGET | 骨架完成；未迁移旧入口 |
| K230D 解析 | `main_controller_align.py::parse_k230` | 解析 TARGET/NO_TARGET | 已用于当前主控 |
| AprilTag 服务 | `vision/apriltag_service.py` | 延迟初始化并调用原 AprilTagDetector | 接口完成；算法不变，已供 Tag 辅助到站控制器使用 |
| AprilTag 视觉 | `vision/apriltag_detector.py` | 采集相机并检测 tag36h11 | 单项现场验证完成，约 2.7 m 仍可检测 |
| 轻量定位 | `navigation/localization.py` | 选择主 Tag，输出语义、error_x、area、confidence | 模块和人工测试完成 |
| 世界模型 | `navigation/field_model.py` | 场地、区域、Tag、巡线拓扑 | 已建立；绝对坐标 TODO |
| 兼容地图接口 | `navigation/field_map.py`、`tag_map.py` | 保持 `get_zone_by_tag()` 和旧 TAG_MAP 可用 | 已完成 |
| 导航状态接口 | `navigation/navigation_state.py` | LINE_FOLLOW、SEARCH_TAG、TAG_ALIGN、TASK_EXECUTE | 只定义，未运行 |
| 比赛策略 | `strategy/mission.py` | START 到 BUILD 的任务顺序和元数据 | 只定义，未运行 |
| 比赛状态 | `competition/mission_state.py` | START、FOLLOW_LINE、GET_ORANGE、GET_PURPLE、BUILD、FINISH、FAULT | 已定义 |
| 比赛状态管理 | `competition/mission_controller.py` | 保存、显式切换、复位顶层状态 | 基础骨架完成；无动作逻辑 |
| 比赛流程调度 | `competition/competition_runner.py` | 通过外部事件编排导航、视觉接近、抓取与搭建 | 第一版模拟流程完成；无真实事件源 |
| 比赛任务控制 | `competition/task_controller.py` | 把比赛任务接口适配到已有 ArmSequence 动作 | build/reset 已接入；独立取料待确认 |
| 视觉接近控制 | `competition/visual_approach_controller.py` | K230 对准、前进、READY 和超时停车 | 原逻辑已迁移；未接入比赛循环、未实机回归 |
| 比赛导航控制 | `competition/navigation_controller.py` | 巡线启动、指定 Tag 面积连续确认、STOP 到站和状态观测 | 第一版软件闭环完成；阈值待标定，未接入 Runner、未实车联调 |
| 比赛入口 | `competition/main.py` | 初始化上述接口并打印状态 | 安全骨架完成；不连接硬件 |
| 局部主控 | `main_controller_align.py` | K230D 对准、接近、停车、人工触发搭建 | 第一版联调完成 |
| 机械动作 | `arm_sequence.py` | 已验证的三组搭建动作 | 已接入局部主控 |
| 独立入口 | `build_sequence.py` | 单独回归机械臂完整动作 | 已实机使用 |

## 3. 当前实际运行链路

### 3.1 比赛入口安全骨架

```text
python -m competition.main
  -> MissionController(START)
  -> Stm32Link(disconnected)
  -> ArmSequence(Stm32Link)
  -> TaskController(ArmSequence)
  -> K230Link(disconnected)
  -> VisualApproachController(K230Link, Stm32Link)
  -> AprilTagService(disconnected)
  -> NavigationController(Stm32Link, AprilTagService)
  -> CompetitionRunner(MissionController, NavigationController,
                       VisualApproachController, TaskController,
                       simulation_mode=True)
  -> 打印状态并退出
```

该链路只验证软件对象可以组合并读取 Runner 的 START 状态，不调用 `connect()`、`CompetitionRunner.start()`、`run_once()`、`handle_event()` 或任务动作，因此不会打开 STM32/K230D 串口或 USB Camera，也不会发送任何控制命令。

TaskController 只负责任务级委托，不保存端口、创建串口或复制机械动作：

```text
TaskController.build()
  -> ArmSequence.run_all()
  -> group1() -> group2() -> group3()

TaskController.reset_arm()
  -> ArmSequence.safe_home()
```

`grab_orange()` 和 `grab_purple()` 当前返回 `NotImplemented`。仓库中的三个已验证 group 都包含抓取和放置，尚无颜色到独立抓取动作的可靠映射。`startup_raise()` 仍是独立入口的启动策略，不由 `build()` 隐式执行。

对外部 STM32 工程的只读动作考古没有改变这一边界。`Core/Src/main.c` 的未调用 `Test_All()` 中有一个 `#if 0` 旧候选：Servo1/2/4 先到 0/0/135，底盘执行两段 `Move_Distance`，Stepper 以 800 pps 执行 +3000，Servo4 到 105、Servo2 到 15，再执行 -3000；底盘随后后退，Servo2 到 0、Servo1 以 10 ms/degree Sweep 到 93，最后 Stepper +5000、Servo4 到 135、Stepper -5000，各主要动作间保持原 500 ms 延时。它没有橙色或中间盒标注，也不在正式运行链中，并且包含不可忽略的底盘动作。

该候选与当前 `ArmSequence.group1()` 的 Servo2、Servo1 和第二段 Stepper 参数明显不同，不能视为同一版本。由于 STM32 工程目录无可用 Git 历史、当前烧录版本未关联、Stepper 无原点/限位/编码器，架构层不会从该候选推导新动作。`grab_orange()` 只有在动作语义、完整顺序和机械起始姿态（尤其 Stepper 当前高度，以及是否先且仅先执行一次 -1500 @ 800 pps）经实机记录确认后，才可沿 `CompetitionRunner -> TaskController -> ArmSequence -> Stm32Link` 接入。

### 3.2 competition 视觉接近组件

```text
K230Link.read_latest()
  -> TARGET / NO_TARGET
  -> VisualApproachController
       STOP -> ALIGN <-> APPROACH -> READY
       异常 -> FAULT
  -> Stm32Link.set_velocity() / stop()
```

该组件把旧主控已验证的视觉行为迁移为无硬件初始化、无后台循环的单步接口：

- ALIGN：只发送 `VEL,0.00,0.00,±0.45`，误差进入 `±30` 后开始前进。
- APPROACH：只发送 `VEL,0.00,0.20,0.00`，误差达到 `±70` 后重新对准。
- READY：pixels EMA 的 alpha 保持 0.20；面积至少 20000 且误差不超过 30 连续 3 帧后发送 STOP，并以 ACK 作为可执行机械臂的必要条件。
- NO_TARGET 不立即停车；目标丢失 0.30 s 或 K230 有效数据超时 0.60 s 后停车并重置 EMA/计数。

VisualApproachController 只负责到 READY，不包含 ARM_BUILD。上层未来应在 `is_ready()` 成立后停止视觉速度输出，再显式调用 TaskController。当前 competition/main.py 只构造该组件，不调用更新接口。

### 3.3 competition 导航控制组件

```text
MissionController（未来调度；当前只管理任务状态）
  -> NavigationController
       ├── Stm32Link.send_command("LINE_START") / stop()
       └── AprilTagService.detect()
  -> IDLE / LINE_FOLLOWING / TAG_SEARCH / TAG_ALIGN / ARRIVED / FAULT
```

NavigationController 不拥有串口或摄像头，只使用入口注入的共享服务：

- `start_line_follow()` 只发送现有人工巡线入口已经使用的 `LINE_START`，成功后记录 `LINE_FOLLOWING`。
- `start_line_to_tag(tag_id, arrival_area, confirm_frames)` 保存调用方给出的目标与阈值，再通过同一 `LINE_START` 启动 STM32 本地巡线；它不创建第二个 detector 或 serial。
- `update()` 每次只调用一次 AprilTagService 检测。未看到目标、看到错误 Tag、面积低于阈值或中间丢帧时清零确认计数并继续巡线。
- 指定 Tag 的原始像素面积连续达到阈值后，控制器调用 `Stm32Link.stop(require_ack=True)`；只有 STOP ACK 成功才进入 `ARRIVED`，后续 `update()` 不会重复停车。
- `get_navigation_status()` 同时提供目标 id、可见性、`error_x`、`area`、阈值和连续确认计数。视觉/链路异常会尝试 STOP 并进入 `FAULT`，STOP 失败也保持 `FAULT` 可观测。
- `stop_navigation()` 委托 Stm32Link STOP，成功后回到 `IDLE`。
- `navigate_to_material_zone()` 只验证并记录 `ORANGE` / `PURPLE`，不推导路线、不发送运动命令。
- `align_with_tag()` 通过 AprilTagService 获取一帧检测；未看到指定 Tag 时为 `TAG_SEARCH`，看到时为 `TAG_ALIGN`。当前没有生成底盘姿态修正。
- `handle_event()` 仅预留并返回 `NotImplemented`。没有定义或消费 `LINE_DONE`、`TAG_REACHED`、`TASK_DONE` 等未验证协议。

`ARRIVED` 现在只表示“指定 Tag 面积连续达标且 Pi 已收到 STOP ACK”，不表示世界坐标到达，也不依赖 STM32 LINE_DONE。`arrival_area` 没有内置最终值，必须由调用方显式提供；已有 `navigation/field_model.py` 仍不能生成真实路线，因为坐标、线段长度、路口位置与停车点尚未标定。

当前外部 STM32 源码中的巡线执行链为：

```text
LINE_START
  -> LineNav.Stop() + Robot.Emergency_Stop()
  -> 只读传感器预检
       失败 -> 保持 STOP，ERR,LINE_SENSOR_IO
       成功 -> ControlMode=LINE_FOLLOW
            -> LineNav.Start_Follow(0.40 m/s)
            -> ACK,LINE_START
  -> main 循环 LineNav.Update()（10 ms 节流）
       -> Follow_Straight() 软件 I2C 读四路传感器
       -> Robot.Apply_Velocity(LINE_FOLLOW, velocity)
       -> 运行期 I2C 失败时 Active=0 + Emergency_Stop()
          并由 RpiUart 一次性上报 ERR,LINE_SENSOR_IO
```

`LINE_STATUS` 是独立只读诊断路径：它只采样四路传感器并回报本次读取结果与 Active，不切换 ControlMode、不启动或停止 LineNav，也不写底盘、Servo、Stepper。该协议已通过编译，尚待烧录和现场验证；它不是 LINE_DONE 或路口事件。

### 3.4 CompetitionRunner 事件调度

```text
CompetitionRunner.start()
  START
    -> FOLLOW_LINE --LINE_DONE-->
    -> GET_ORANGE
    -> VISUAL_APPROACH --TARGET_READY-->
    -> GRAB_ORANGE --ARM_DONE-->
    -> BUILD --BUILD_DONE-->
    -> FINISH

任意阶段 --FAULT--> FAULT
```

Runner 负责细粒度执行阶段，MissionController 仍只保存比赛顶层状态。`VISUAL_APPROACH` 和 `GRAB_ORANGE` 映射到顶层 `MissionState.GET_ORANGE`，没有把设备动作塞入 MissionController。

事件接口不使用硬编码 sleep：

- `LINE_DONE`、`TARGET_READY`、`ARM_DONE`、`BUILD_DONE` 和 `FAULT` 可由模拟器或未来适配层注入。
- `run_once()` 只在非模拟的 VISUAL_APPROACH 阶段调用一次 VisualApproachController；检测到 `is_ready()` 后生成同等的 TARGET_READY 转移。
- 默认 `simulation_mode=True` 时，`start()`、`run_once()` 和事件转移均不调用下层控制器，可安全检查完整状态顺序。
- 只有显式 `simulation_mode=False` 才会调用 NavigationController、VisualApproachController 和 TaskController；当前 main 不启用此模式。
- 非模拟模式遇到当前 `grab_orange() == NotImplemented` 时锁在 GRAB_ORANGE，并拒绝 ARM_DONE，避免伪造抓取成功。
- 乱序事件抛出 CompetitionRunnerError；控制器异常或 FAULT 事件把 Runner 和 MissionController 同步转为 FAULT。

当前这些完成事件只是 Pi 内部类型，不是 STM32 协议。代码没有新增或假设 `LINE_DONE`、`TAG_REACHED`、`TASK_DONE` 等串口消息。

### 3.5 K230D 对准与机械臂搭建模式

```text
K230D 串口
  -> TARGET / NO_TARGET
  -> parse_k230()
  -> ALIGN / APPROACH / READY 状态
  -> VEL 或 STOP
  -> STM32
  -> READY 收到 ACK,STOP
  -> 人工按 b
  -> ARM_BUILD
  -> ArmSequence.run_all()
  -> BUILD_DONE
```

这个模式已经把底盘停车和机械臂动作放在同一进程、同一 STM32 串口连接中。机械臂阶段停止读取 K230D 和发送底盘运动命令，由 `ArmSequence` 独占 STM32 回复。

它仍不是比赛全流程：没有巡线任务切换，没有橙色/紫色两类任务调度，也没有使用 AprilTag 世界模型。

### 3.6 独立机械臂模式

```text
build_sequence.py
  -> 打开 STM32
  -> HELLO_STM32,1
  -> 固定 startup raise
  -> 人工菜单选择 group 1/2/3/all
  -> STOP / 关闭串口
```

该入口用于机械臂回归和分组调试，不应和整车主控同时运行。

### 3.7 AprilTag 人工测试模式

```text
/dev/video0
  -> AprilTagDetector.detect()
  -> List[dict]
  -> AprilTagLocalization.localize()
  -> visible/tag_id/tag_name/error_x/area/confidence
  -> 终端输出
```

此链路目前不发送 STM32 命令。`confidence` 只是根据像素面积计算的启发值，不是检测概率。

## 4. 当前状态模型之间的关系

仓库中存在多套不同抽象层级的状态，它们还没有由统一调度器连接：

| 层级 | 状态 | 作用 |
| --- | --- | --- |
| 当前 K230D 控制状态 | STOP、ALIGN、APPROACH、READY、ARM_BUILD、BUILD_DONE、ERROR | 一次视觉接近和搭建联调 |
| competition 视觉接近状态 | STOP、ALIGN、APPROACH、READY、FAULT | 可复用的 K230 接近行为；READY 后交回上层 |
| 旧导航能力枚举 | `navigation/navigation_state.py` 中 LINE_FOLLOW、SEARCH_TAG、TAG_ALIGN、TASK_EXECUTE | 早期行为分类，仍未运行 |
| competition 导航状态 | IDLE、LINE_FOLLOWING、TAG_SEARCH、TAG_ALIGN、ARRIVED、FAULT | NavigationController 的运行细节；Tag 连续达标并收到 STOP ACK 后可进入 ARRIVED |
| CompetitionRunner 阶段 | START、FOLLOW_LINE、GET_ORANGE、VISUAL_APPROACH、GRAB_ORANGE、BUILD、FINISH、FAULT | 第一版比赛动作编排；完成事件当前只能外部注入 |
| 旧比赛任务元数据 | `strategy/mission.py` 中 START、FOLLOW_LINE、GET_ORANGE、GET_PURPLE、BUILD | 已有任务计划数据，尚未迁移 |
| 新比赛任务状态 | `competition/mission_state.py` 中 START、FOLLOW_LINE、GET_ORANGE、GET_PURPLE、BUILD、FINISH、FAULT | 未来比赛入口使用的顶层状态 |

目标不是把三套枚举简单合并，而是让比赛任务选择一个导航/执行行为，再由行为内部运行短周期控制状态。例如：

```text
MissionState.GET_ORANGE
  -> NavigationController.navigate_to_material_zone(ORANGE)
  -> NavigationController.start_line_to_tag(tag_id, arrival_area, confirm_frames)
  -> NavigationState.LINE_FOLLOWING
  -> update() 逐帧确认指定 Tag
  -> STOP ACK -> NavigationState.ARRIVED
```

该链路目前只由独立人工测试入口调用，CompetitionRunner 仍使用旧 `start_line_follow()`，本轮没有把 ARRIVED 接入 Runner。`MissionController` 只保存、显式切换和复位比赛状态；默认 simulation mode 下三类子控制器仍保持空闲。旧 `strategy/mission.py` 的任务元数据尚未迁移，新流程也暂未包含 GET_PURPLE。

## 5. 世界模型数据流

`navigation/field_model.py` 是详细世界模型的唯一事实来源：

```text
官方规则 PDF
  -> docs/FIELD_DESCRIPTION.md
  -> navigation.field_model
       ├─ FIELD_INFO
       ├─ ZONE_MAP
       ├─ TAG_MAP
       └─ LINE_MAP
            ├─ navigation.tag_map（旧字段兼容）
            └─ navigation.field_map（get_zone_by_tag 接口）
```

- 官方场地尺寸、区域尺寸、Tag 尺寸和巡线拓扑已录入。
- 软件坐标系由项目约定，不是规则定义。
- 规则没有标注的 Tag、路口和线段绝对坐标保持 `None`。
- 不能用规则示意图像素比例伪造厘米坐标。

`from navigation import TAG_MAP` 当前导出的是兼容语义地图；需要完整世界坐标字段时应从 `navigation.field_model` 使用详细 `TAG_MAP`。

## 6. 串口所有权

STM32 回复没有请求 ID；如果多个读取者并发读取，ACK 或 DONE 可能被错误组件消费。因此必须遵守：

```text
一个进程
  -> 一个 serial.Serial(STM32)
       -> 一个阶段性回复读取者
```

当前 `main_controller_align.py` 已让底盘控制和 `ArmSequence` 复用同一串口，并在状态边界切换读取权。这条原则必须延续到未来比赛入口。

禁止的结构：

```text
视觉进程 ----打开 STM32----┐
机械臂进程 --打开 STM32----┼── 同一 CH340
巡线进程 ----打开 STM32----┘
```

已建立的目标骨架：

```text
competition/main.py（当前 simulation mode，不连接）
├── Stm32Link（唯一 STM32 serial.Serial 所有者）
├── K230Link
├── AprilTagService
└── CompetitionRunner
     ├── MissionController
     ├── NavigationController
     │    ├── AprilTagService
     │    └── Stm32Link
     ├── VisualApproachController
     │    ├── K230Link
     │    └── Stm32Link
     └── TaskController
          └── ArmSequence
               └── Stm32Link
```

`Stm32Link` 的构造不会打开端口，只有显式 `connect()` 才创建 `serial.Serial`。它统一保留现有 LF 文本帧、两位小数 VEL、ACK/DONE 前缀等待、ERR 立即异常和超时异常语义。为在不修改已验证 `arm_sequence.py` 的前提下完成依赖接入，Stm32Link 新增受连接状态保护的 `write()`、`flush()`、`readline()` 兼容转发；底层 `serial.Serial` 仍不向 TaskController 或 ArmSequence 暴露，也没有第二个所有者。

旧 `main_controller_align.py` 和 `build_sequence.py` 尚未迁移，当前仍使用各自已经验证的串口对象和调用路径；`arm_sequence.py` 本身保持完全不变，并已可由新 competition 骨架通过 Stm32Link 复用。

`K230Link.read_latest()` 保留旧主控“清空队列、只处理最新一行”的输入语义。VisualApproachController 只持有注入的 K230Link/Stm32Link，不创建 Serial，也不接触 AprilTag 相机。

NavigationController 同样只持有注入的 Stm32Link/AprilTagService。材料区请求只保存目标；Tag 对准接口仍不生成姿态修正；Tag 辅助到站复用已有 `LINE_START` 和 STOP，只增加 Pi 侧逐帧观测与状态转换。CompetitionRunner 再持有三类控制器和 MissionController，不拥有任何端口或摄像头，本轮也未调用新能力。

## 7. 安全和故障流

当前局部主控已有以下安全路径：

- 启动前尝试发送 STOP。
- 目标持续丢失超过 0.30 s 后 STOP。
- K230D 超过 0.60 s 没有有效数据后 STOP。
- READY 阶段收到 `ACK,STOP` 前不进入机械臂动作。
- ARM_BUILD 阶段不再发送底盘运动命令。
- Ctrl+C、运行异常和 finally 路径尝试 STOP 并关闭串口。
- Stepper 动作等待 ACK 和 MOVE_DONE，收到 ERR 立即失败。
- 当前 STM32 源码在 `LINE_START` 前预检传感器；失败保持停车并回复 `ERR,LINE_SENSOR_IO`。运行期读取失败仍立即停车，并一次性上报相同错误。
- `LINE_STATUS` 只读采样，不取得底盘控制权，也不改变 LineNav Active。
- Tag 辅助到站只在指定 Tag 面积连续达标后停车；单帧或少量漏检只清零确认计数，不会突然 STOP。
- 到站 STOP 必须收到 ACK 才进入 `ARRIVED`；AprilTagService/Stm32Link 异常会尝试 STOP 并进入 `FAULT`，STOP 失败保持 `FAULT`。

仍存在的架构风险：

- `main_controller_align.py` 在模块顶层打开两路串口，主 try/finally 没有覆盖打开失败和启动等待全过程。
- 当前主控没有 STM32 协议版本握手。
- 回复协议没有请求 ID，必须严格保持单一读取者。
- `BUILD_DONE` 为锁存终态，只能结束进程，尚无下一比赛阶段。
- 独立测试多为 import 即执行，不能作为自动化测试集合运行。
- 没有上限位器，startup raise 不能视为 Homing。
- CompetitionRunner 已把下层异常映射到 `MissionState.FAULT`，但 main 尚无 live 连接/循环/退出生命周期；统一 STOP、关闭顺序和失败恢复仍需在实机入口前补齐。
- CompetitionRunner 已在 ARM_DONE 后委托 build，但独立橙色和紫色取料动作没有实机确认；当前真实 grab_orange 返回 NotImplemented，非模拟流程会停在 GRAB_ORANGE。STM32 `Test_All()` 的禁用旧候选缺少任务语义、调用/版本证据和机械起始高度，不能作为已验证动作接入。
- CompetitionRunner.run_once() 已定义 VisualApproachController READY/FAULT 的软件映射，但 main 不运行该循环，也没有进行 K230/STM32 实机集成回归。
- NavigationController 仍没有巡线完成/路口协议事件，`handle_event()` 保持未实现；`ARRIVED` 仅是 Pi 的 Tag 面积到站条件，不代表姿态已经对准。
- 当前 `NavigationController` 尚未封装 `LINE_STATUS` 或消费 `ERR,LINE_SENSOR_IO`；本次只在人工诊断脚本中透传验证，避免在实测前扩展比赛状态机。
- STM32 四路全白时继续前进是已知风险，T 字路口也尚未处理；二者不由 Pi 的 Tag 到站逻辑掩盖。

## 8. 配置和依赖

### 8.1 当前配置

`config.py` 保存：

```text
STM32_PORT=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
K230_PORT=/dev/serial/by-id/usb-Kendryte_CanMV_001000000-if00
STM32_BAUDRATE=115200
K230_BAUDRATE=115200
```

新增 `Stm32Link` 和 `K230Link` 已直接引用这些值。现有正式联调入口仍复制相同值；后续迁移必须保持已验证值不变并做实机回归。

### 8.2 当前已知 Python 依赖

- Python 3.13
- `pyserial`
- `numpy`
- `opencv-python-headless`
- `pupil_apriltags`

仓库没有 `requirements.txt`、`pyproject.toml` 或锁文件，也没有记录确切版本；这是部署可重复性的缺口。

## 9. 测试边界

```text
tests/test_apriltag.py       相机人工测试，无机器人运动
tests/test_localization.py   相机+定位人工测试，无机器人运动
tests/test_line_to_tag.py    人工确认后启动巡线，Tag 连续达标后 STOP；会驱动底盘
tests/test_navigation_controller_unit.py  Fake STM32/AprilTag 纯软件状态测试
tests/vel_test.py            会驱动底盘
tests/motor_cmd_test.py      启动后会依次驱动底盘
tests/line_test.py           任意文本命令透传并原样显示回复；可安全发送 LINE_STATUS，也可发送会驱动底盘的命令
tests/servo_test.py          通用命令透传，可能驱动任意执行器
其他 uart/stm32 脚本        链路诊断，部分使用历史设备路径
```

任何硬件测试都必须由人工逐个启动。同一时刻不得运行另一个占用 STM32 串口的程序；运动测试前需清空场地并准备急停。纯软件回归可单独运行 `python -m unittest discover -s tests -p "test_navigation_controller_unit.py" -v`，不会打开串口或相机。

## 10. 已落地骨架与下一步

当前已落地的轻量骨架不引入 ROS 或复杂 SLAM：

```text
输入层
├── K230D：颜色/物块结果
├── USB Camera：AprilTag 结果
└── STM32：ACK、DONE、ERR、LINE_STATUS（巡线完成事件尚未定义）
        │
        v
世界与状态层
├── field_model：区域、Tag、线路
├── MissionController：任务阶段
├── CompetitionRunner：事件顺序与控制器编排
├── TaskController：已验证机械臂任务适配
├── VisualApproachController：K230 目标对准与接近
└── NavigationController：巡线启动、Tag 面积连续确认、STOP 到站与观测状态
        │
        v
执行层
├── chassis：VEL / STOP / line control
└── ArmSequence：Servo / Stepper
        │
        v
STM32 唯一串口链路
```

仍未完成的部分包括：标定并实车回归 `arrival_area`、处理全白安全和 T 字路口、把新 ARRIVED 能力接入 Runner、设计 live 硬件生命周期、定义巡线完成/路口事件、实现路线与 Tag 姿态修正、补入 GET_PURPLE、实机确认独立橙色/紫色取料动作、实现 FINISH/FAULT 的完整安全收尾，以及系统化测试。

实现顺序应继续优先保证协议和串口所有权，其次接入状态转换，最后再标定世界坐标和视觉阈值。不要在全自动状态机未可观测、未可安全停车前一次性叠加全部动作。
