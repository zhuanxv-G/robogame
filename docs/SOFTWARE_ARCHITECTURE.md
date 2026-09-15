# RoboGame 软件架构

> 架构快照：2026-09-12。本文描述仓库当前实际状态，同时把尚未接入的规划模块明确标出。

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
| 设备/协议 | `hardware/stm32_link.py` | STM32 串口所有权，HELLO/STOP/VEL/NEXT_TURN/Servo/Stepper 与路口事件 | 骨架完成；显式连接，未迁移旧入口 |
| 外部实时执行 | `D:\Code\V1.3_vision\V1.3` | STM32 USART1 解析、控制权、底盘、巡线、Servo、Stepper | 不在 Pi Git 仓库；直线巡线已现场验证，第一版路口状态机已编译但待实车；普通全白风险仍存在 |
| 设备/协议 | `arm_sequence.py` 中的请求方法 | HELLO、STOP、Servo、Stepper 请求/回复 | 已用于正式入口 |
| K230D 输入 | `vision/k230_link.py` | 显式连接 K230D，解析 TARGET/NO_TARGET | 骨架完成；未迁移旧入口 |
| K230D 解析 | `main_controller_align.py::parse_k230` | 解析 TARGET/NO_TARGET | 已用于当前主控 |
| AprilTag 服务 | `vision/apriltag_service.py` | 延迟初始化并调用原 AprilTagDetector | 接口完成；算法不变，已供 Tag 辅助到站控制器使用 |
| AprilTag 视觉 | `vision/apriltag_detector.py` | 采集相机并检测 tag36h11 | 单项现场验证完成，约 2.7 m 仍可检测 |
| 轻量定位 | `navigation/localization.py` | 选择主 Tag，输出语义、error_x、area、confidence | 模块和人工测试完成 |
| 世界模型 | `navigation/field_model.py` | 场地事实、区域能力、Tag 地标和已确认拓扑边 | 左右橙色来源已分离；绝对坐标 TODO |
| 兼容地图接口 | `navigation/field_map.py`、`tag_map.py` | 保持 `get_zone_by_tag()` 和旧 TAG_MAP 可用 | 已完成 |
| 拓扑定位/规划 | `navigation/topological_navigation.py` | Tag 到逻辑位置更新、无权图 BFS 路径 | 独立人工入口已完成 Tag 3/4/5 第一轮实机验证；未接入 Runner 或路线执行 |
| 路线动作适配 | `navigation/route_action_resolver.py` | 用离散边方向把路线中的路口转换为 LEFT/RIGHT/STRAIGHT | 纯软件完成；单路口入口取首项，RouteExecutor 可顺序消费全部动作 |
| 多路口路线执行 | `navigation/route_executor.py` | 严格消费 Junction/Turn 事件、逐项预设 NEXT_TURN、最终交接 AprilTag 到站 | Fake 软件测试完成；人工多路口入口已建立，未实车、未接入 Runner |
| 导航状态接口 | `navigation/navigation_state.py` | LINE_FOLLOW、SEARCH_TAG、TAG_ALIGN、TASK_EXECUTE | 只定义，未运行 |
| 比赛策略 | `strategy/mission.py` | START 到 BUILD 的任务顺序和元数据 | 只定义，未运行 |
| 比赛状态 | `competition/mission_state.py` | START、FOLLOW_LINE、GET_ORANGE、GET_PURPLE、BUILD、FINISH、FAULT | 已定义 |
| 比赛状态管理 | `competition/mission_controller.py` | 保存、显式切换、复位顶层状态 | 基础骨架完成；无动作逻辑 |
| 比赛流程调度 | `competition/competition_runner.py` | 通过外部事件编排导航、视觉接近、抓取与搭建 | 第一版模拟流程完成；无真实事件源 |
| 比赛任务控制 | `competition/task_controller.py` | 把比赛任务接口适配到已有 ArmSequence 动作 | build/reset 已接入；独立取料待确认 |
| 视觉接近控制 | `competition/visual_approach_controller.py` | K230 对准、前进、READY 和超时停车 | 原逻辑已迁移；未接入比赛循环、未实机回归 |
| 比赛导航控制 | `competition/navigation_controller.py` | 巡线启动、指定 Tag 面积连续确认、STOP 到站和状态观测 | Tag 3/4/5 第一轮实机联调完成；加入 STOPDBG 后的后续测试未再复现早前超时 |
| 人工导航联调 | `tests/test_navigation_topology_live.py` | 人工 START 后验证巡线、指定 Tag ARRIVED 和拓扑位置更新 | 已取得实机成功结果并增加可选 STOP 时间线；不是 Runner |
| 巡线传感器采样 | `tests/test_line_sensor_capture.py` | 只读轮询 LINE_STATUS、人工标记并保存 CSV | 工具与 Fake 测试完成；真实 CSV 已作为路口第一版输入 |
| 路口协议/人工入口 | `hardware/stm32_link.py`、`tests/test_junction_turn.py` | 直接选动作或规划路线后预设首个 NEXT_TURN，严格 START 后单项实机测试 | 协议和 Fake 测试完成；TURN_DONE 后 STOP，未接入 Runner，未实车验证 |
| 比赛入口 | `competition/main.py` | 初始化上述接口并打印状态 | 安全骨架完成；不连接硬件 |
| 局部主控 | `main_controller_align.py` | K230D 对准、接近、停车、人工触发搭建 | 第一版联调完成 |
| 机械动作 | `arm_sequence.py` | 已验证的三组搭建动作 | 已接入局部主控 |
| 独立入口 | `build_sequence.py` | 单独回归机械臂完整动作 | 已实机使用 |
| 启动准备入口 | `robot_startup.py` | HELLO 后按 `-1500 @ 800 pps` 将升降轴相对上移一次 | 人工单次运行；不是 Homing，不接入比赛状态机 |
| 左盒完整抓放入口 | `tests/test_old_grab_left_sequence.py` | 到墙抓取、放入左盒并进入95/135/90/95/90静止姿态 | 人工单次运行；待实机验证，不接入比赛状态机 |
| 右盒完整抓放入口 | `tests/test_old_grab_right_sequence.py` | 到墙抓取、放入右盒并进入95/135/90/95/90静止姿态 | 人工单次运行；待实机验证，不接入比赛状态机 |
| 视觉辅助抓取实验 | `tests/test_visual_grab_sequence.py` | 撞墙、后撤、K230D SEARCH/ALIGN/READY 定位与已验证抓取/放盒组合 | 人工单次运行；待实机验证，不接入比赛状态机 |

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

`TaskController.grab_orange()` 和 `TaskController.grab_purple()` 当前仍返回 `NotImplemented`，因此 CompetitionRunner 的这两个入口没有接线。与它们分离的固定路线入口 `competition/grab_mission.py` 使用操作者后来确认过的旧候选机械臂动作；它不修改或复用三个搭建 group，也不代表颜色视觉已经接入。`startup_raise()` 仍是独立入口的启动策略，不由 `build()` 隐式执行。

对外部 STM32 工程的只读动作考古没有改变这一边界。`Core/Src/main.c` 的未调用 `Test_All()` 中有一个 `#if 0` 旧候选：Servo1/2/4 先到 0/0/135，底盘执行两段 `Move_Distance`，Stepper 以 800 pps 执行 +3000，Servo4 到 105、Servo2 到 15，再执行 -3000；底盘随后后退，Servo2 到 0、Servo1 以 10 ms/degree Sweep 到 93，最后 Stepper +5000、Servo4 到 135、Stepper -5000，各主要动作间保持原 500 ms 延时。它没有橙色或中间盒标注，也不在正式运行链中，并且包含不可忽略的底盘动作。

该候选与当前 `ArmSequence.group1()` 的 Servo2、Servo1 和第二段 Stepper 参数明显不同，不能视为同一版本。`competition/grab_task.py`保留全部24个非底盘旧动作，并在末尾追加抓取专用静止姿态Servo0..4=95/135/90/95/90；`competition/grab_mission.py`在右转重捕后前伸机械臂，再巡线0.38 m到墙并插入0.05 m和0.30 m后退，放置后执行同一静止姿态。`ArmSequence.safe_home()`保持原样，因为其Servo4=135并不等于本次抓取结束静止角Servo4=90。该固定路线仍须分阶段实测，且暂不接入`CompetitionRunner -> TaskController -> ArmSequence`。

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

新增的独立人工联调入口保持这一判定不变：

```text
人工输入 START
  -> AprilTagService.connect() + Stm32Link.connect()
  -> NavigationController.start_line_to_tag(...)
  -> update() 逐帧观察指定 Tag
  -> 指定 Tag 连续达标 + STOP ACK -> ARRIVED
  -> TopologicalNavigator.update_from_tag(confirmed_tag)
  -> 打印 confirmed_tag / current_location 并退出
```

`tests/test_navigation_topology_live.py` 中的观测包装只保存同一次 `AprilTagService.detect()` 已返回的 Tag id，供低频状态打印使用；它不创建第二个 detector、不重复读取相机，也不参与 NavigationController 的目标过滤和 ARRIVED 判定。错误 Tag 可被打印，但只有 NavigationController 最终确认的目标 Tag 能更新拓扑位置。该脚本在 Ctrl+C、异常和 FAULT 退出路径尝试 STOP，并释放 Camera 与 STM32。

该入口必须人工输入严格大写的 `START` 才会连接设备并启动巡线。现场已确认 Tag 4 在 `arrival_area=10000` 时约 80 cm 停车并更新为 `RIGHT_ORANGE_MATERIAL`，Tag 5 使用同一面积阈值时也基本在约 80 cm 停车；Tag 3 使用 `arrival_area=20000`、`confirm_frames=3` 连续三次约 57 cm 停车并更新为 `PURPLE_MATERIAL`。这些现场参数不作为代码默认值。该入口不是 CompetitionRunner，不调用 `plan_route()`，也不调用新增 `NEXT_TURN` 路口能力。

### 3.3.1 STOP ACK 偶发超时诊断

当前到站 STOP 的 Pi 调用链为：

```text
NavigationController.update()
  -> _confirm_arrival()
  -> Stm32Link.stop(require_ack=True)
  -> send_command("STOP")
     -> serial.write(b"STOP\n")
     -> serial.flush()
  -> _wait_for("ACK,STOP", timeout=2.0)
     -> ACK：NavigationState.ARRIVED
     -> timeout/ERR：NavigationState.FAULT，异常返回 live test
  -> live test finally
     -> Stm32Link.stop(require_ack=True, debug_label="cleanup")
```

`Stm32Link` 构造时的 pyserial `timeout=0.2` 是单次 `readline()` 最长阻塞时间；STOP 的 `timeout=2.0` 是 `_wait_for()` 总体逻辑窗口，从 STOP 的 `write()` 和 `flush()` 完成后开始。循环会在 2 秒内反复 `readline()`；因为截止条件在每次阻塞读取前检查，实际抛出时间理论上可比 2.0 秒多最多一次串口读超时。空行继续等待；非目标非 ERR 行被消费并丢弃；`ERR,...` 立即抛出 `Stm32ProtocolError`；`ACK,STOP` 前缀匹配成功。

人工拓扑入口运行期间只有 NavigationController 经同一个 Stm32Link 同步读取 STM32 回复，没有后台线程、drain 或第二个读取者。仓库中的 `main_controller_align.py`、其他人工测试和 legacy 脚本各有自己的读取循环，但它们只有被错误地并发启动时才会争抢同一物理串口。

到站 STOP 超时在 `_confirm_arrival()` 内已经把 NavigationController 置为 FAULT，因此 `update()` 外层不会再调用 `_enter_fault()` 重发；live test 的 except 只打印错误，finally 再发一次 cleanup STOP。该路径总计两次 STOP，即首次请求加一次额外 cleanup，每次都以 2.0 秒等待 ACK。

现有协议没有请求 ID。第一次 STOP 超时后，其迟到的 `ACK,STOP` 会留在串口输入路径；cleanup 写入第二次 STOP 后，`_wait_for()` 可以把该旧 ACK 当作第二次请求成功。FakeSerial 已稳定复现这种歧义，因此“cleanup STOP 成功”不能单独证明第二次 STOP 获得了自己的 ACK。

`Stm32Link(stop_debug=True)` 现在只增加观测，不改变上述行为或 timeout：STOP 前记录 `time.monotonic_ns()` 和可安全获取的 `serial.in_waiting`，写入/flush 后记录耗时，等待期间记录每条非空 `repr(raw_line)`，最后记录匹配或超时。`test_navigation_topology_live.py --stop-debug` 还会显式标出 cleanup 请求。正常高频视觉循环不会输出 STOPDBG。

外部 STM32 源码提供的静态证据是：完整 STOP 行由 USART1 单字节中断置入独立高优先级邮箱，紧循环主循环首先调用 `RpiUart.Process()`；`Handle_Stop()` 依次停止 Stepper、LineNav 和底盘后，通过 `HAL_UART_Transmit(..., 100)` 发送 `ACK,STOP`，该路径没有显式 delay。值得现场区分的假设包括 UART/USB 回复丢失或异常延迟、STM32 TX 调用失败，以及外部进程误读；其中 STM32 `Send()` 当前忽略 `HAL_UART_Transmit()` 返回值是代码事实，但是否发生过发送失败仍无现场证据。

当前外部 STM32 源码中的巡线执行链为：

```text
LINE_START
  -> LineNav.Stop() + Robot.Emergency_Stop()
  -> 只读传感器预检
       失败 -> 保持 STOP，ERR,LINE_SENSOR_IO
       成功 -> ControlMode=LINE_FOLLOW
            -> LineNav.Start_Follow(0.80 m/s)
            -> ACK,LINE_START
  -> main 循环 LineNav.Update()（10 ms 节流）
       -> Follow_Straight() 软件 I2C 读四路传感器
       -> Robot.Apply_Velocity(LINE_FOLLOW, velocity)
       -> 运行期 I2C 失败时 Active=0 + Emergency_Stop()
          并由 RpiUart 一次性上报 ERR,LINE_SENSOR_IO
```

`LINE_STATUS` 是独立只读诊断路径：它只采样四路传感器并回报本次读取结果与 Active，不切换 ControlMode、不启动或停止 LineNav，也不写底盘、Servo、Stepper。该协议已通过编译，尚待烧录和现场验证；它本身不是 LINE_DONE 或路口事件。

### 3.3.2 巡线传感器现场采样

```text
人工推动静止机器人
  -> test_line_sensor_capture.py
  -> 单一 Stm32Link / 单一串口读取者
  -> 约 10 Hz 发送唯一命令 LINE_STATUS
  -> 解析 sensor_ok,s1,s2,s3,s4,active
  -> 终端低频状态 + 人工 MARK
  -> logs/line_sensor_capture_YYYYMMDD_HHMMSS.csv
```

该入口不导入或调用 NavigationController、TopologicalNavigator、Camera、K230D、Servo、Stepper 或任何底盘接口；连接后不执行握手、STOP、LINE_START、LINE_STOP 或 VEL，退出时只关闭串口。终端标记采用主线程中的 `select()` 等待，不创建串口读取线程；标记附在下一条采样记录上，因此仍保持单一阶段性回复读取者。CSV 的 `sensor_ok=0` 行会原样保留，但四路位只代表 STM32 缓存，分析时不得当作本次有效读数。`logs/` 已在 `.gitignore` 中忽略。

这项工作建立了可重复的现场数据入口；最初 CSV 中的 `0110 -> 1110/1111 -> 0110` 建立了第一版状态机，后续实车又确认右侧 T 可能出现 `0110 -> 0111`，因此现实现已补充对称的右侧候选。采样工具本身仍不执行任何动作。

### 3.3.3 第一版路口执行链

```text
Pi Stm32Link.set_next_turn(LEFT / RIGHT / STRAIGHT)
  -> NEXT_TURN,<turn> / ACK,NEXT_TURN,<turn>
  -> LINE_START（现有0.80 m/s Follow_Straight）
  -> STM32连续8个约10 ms周期匹配1110、0111或1111
  -> JUNCTION_DETECTED
  -> 1110锁存LEFT，0111锁存RIGHT；歧义时读取pending LEFT/RIGHT
  -> ADVANCE_TO_CENTER
  -> TURN_STARTED,<LEFT|RIGHT>并按±0.80 rad/s定时转向
  -> SEARCH_LINE保持相同转向方向
  -> 连续3周期命中宽松线型
  -> LINE_REACQUIRED + TURN_DONE,<LEFT|RIGHT>
  -> 恢复0.80 m/s Follow_Straight
  -> 3000 ms cooldown后重新等待20周期0110完成arming
```

候选逻辑接受“左三路全黑或右三路全黑”，即物理 `1110/0111/1111`，不会把 `0000` 当路口。连续候选内的方向证据会锁存，所以 `0111 -> 1111` 仍保持 RIGHT；全黑或左右证据冲突时优先使用 pending LEFT/RIGHT，没有可用预设则保留旧 LEFT 回退。LEFT/RIGHT 分别使用正/负 omega，当前两边都沿用 11.5 s 的 90°实测时间；搜索阶段保持相同符号。重捕宽松线型 2000 ms 超时会急停并上报 `ERR,REACQUIRE_TIMEOUT`。速度、时间和连续周期均集中为常量，右转仍待烧录和实车标定。

Pi 已在独立纯软件层把 BFS 节点路线转换成路口动作，但没有接入 CompetitionRunner。`tests/test_junction_turn.py` 可直接使用 `--turn ...`，也可使用 `--start ... --target ...` 规划并打印完整 route/actions；路线从路口起步时还要求 `--heading ...`。两种模式都只取第一个动作，只有操作者输入严格大写 `START` 后才连接 STM32、预设动作并启动巡线。`TURN_DONE` 后入口要求 STOP ACK 并退出；timeout、方向不匹配、Ctrl+C 或其他异常也会进入 FAULT/清理 STOP，不会自动连续运行第二个路口。

### 3.3.4 多路口路线执行链

`RouteExecutor` 是 Navigation Layer 内的轻量编排器，不拥有 serial、Camera 或 detector。它持有注入的 TopologicalNavigator、RouteActionResolver、NavigationController 和唯一 Stm32Link：

```text
prepare_route(start, target, target_tag, arrival_area, confirm_frames)
  -> TopologicalNavigator.plan_route()
  -> RouteActionResolver.resolve_route_actions()
  -> READY（无硬件 I/O）

显式 start()
  -> NEXT_TURN,action[0] / ACK
  -> LINE_START / ACK
  -> WAITING_JUNCTION
  -> JUNCTION_DETECTED
  -> TURN_STARTED,action[0]
  -> TURN_DONE,action[0]
  -> current_action_index += 1
  -> 若仍有动作，才发送 NEXT_TURN,action[1]
  -> ...
  -> ROUTE_TURNS_DONE
  -> NavigationController.continue_line_to_tag(...)
  -> FINAL_TAG_APPROACH（不重复 LINE_START）
  -> 目标 Tag 连续面积达标
  -> STOP / ACK,STOP
  -> NavigationController.ARRIVED
  -> TopologicalNavigator.update_from_tag(target_tag)
  -> RouteExecutor.ARRIVED -> FINISHED
```

路口读取阶段只有 RouteExecutor 通过注入的 Stm32Link 同步 `readline()`；NavigationController 此时不读 STM32。最后一个 TURN_DONE 后，RouteExecutor 停止读取 STM32，并只调用 NavigationController 的相机 `update()`。到站时 NavigationController 才通过同一个 Stm32Link 同步等待 STOP ACK，因此全程仍是一个 STM32 串口所有者和一个阶段性读取者，没有后台线程。

事件检查是严格的：WAITING_JUNCTION 只接受 JUNCTION_DETECTED，随后只接受方向匹配的 TURN_STARTED，再接受同方向 TURN_DONE。方向错误、重复/乱序事件、未知 STM32 行和 `ERR,...` 均进入 FAULT；一个 TURN_DONE 最多推进一个下标。下一个 NEXT_TURN 只在前一个匹配 TURN_DONE 后发送，无法提前覆盖 STM32 pending turn。

Pi watchdog 与 STM32 内部动作 timeout 分离：默认等待下一路口 30 s、转向事件完成 5 s、最终 Tag 30 s，人工入口可以显式覆盖；任何 watchdog 到期都尝试 STOP 并锁定 FAULT。全部路口动作完成只是 `ROUTE_TURNS_DONE`，不是到站。只有指定 Tag 达标、STOP ACK 成功且 topology 更新为请求目标后，才能 FINISHED。无路口动作的路线直接调用原 `start_line_to_tag()` 进入最终 Tag 阶段。

`tests/test_multi_junction_route.py` 是唯一新增的多路口实机入口。它在严格 START 前只打印纯软件计划，确认后才连接一个 Stm32Link 和一个 AprilTagService。该链路已完成 Fake 测试，但尚未实车验证，也未接入 CompetitionRunner、K230D 或机械臂。

```bash
python3 tests/test_multi_junction_route.py \
  --start right_material_t_junction \
  --target right_orange_material \
  --target-tag 4 \
  --heading EAST \
  --arrival-area 10000 \
  --confirm-frames 3
```

这条预览得到 `LEFT -> STRAIGHT` 两个动作，最终使用已有 Tag 4 现场阈值。起始摆车朝向和每项 watchdog 仍需现场复核；未经验证的 Tag 不复用该面积参数。

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

### 3.5.1 K230D 视觉辅助单次抓取实验

```text
tests/test_visual_grab_sequence.py
  -> HELLO_STM32,1 / STM32_OK,1 -> Press ENTER -> STOP / ACK
  -> 初始姿态 Servo0 -> 0 + 旧步骤1-5（Servo1/2/4=0/0/135）
  -> 撞墙 0.50 m -> STOP / ACK
  -> 后撤到视觉抓取位置 0.05 m -> STOP / ACK -> 等待 500 ms
  -> SEARCH（NO_TARGET -> VEL,x=-0.10 缓慢左移；超时停车）
  -> ALIGN（TARGET -> 按 error_x 以 VEL,x=±0.10 左右横移）
  -> READY（abs(error_x)<30 且 pixels>0 连续 5 帧 -> STOP / ACK）
  -> Servo0 -> 95 恢复抓取基线 -> PREPARE_ARM（旧步骤1-5）
  -> PICK -> 后撤 0.30 m -> PLACE -> 静止姿态 -> STOP
```

该实验入口复用 `Stm32Link`、`K230Link` 和 `competition/grab_task.py` 的已验证动作段，以横移式 SEARCH/ALIGN 状态机实现视觉定位（与 `main_controller_align.py` 的旋转+前进方式不同），用于验证 K230D 视觉定位能否替代固定抓取位置。它不修改 `arm_sequence.py`、比赛状态机或 STM32 协议；机械臂阶段停止读取 K230D，由已验证抓取动作独占 STM32 回复。K230D 串口断开先以现有 close+connect 重连（默认 3 次、间隔 1 s），重连失败、0.60 s 无有效数据、SEARCH 超时、STM32 ERR 或 Ctrl+C 才请求全局 STOP 并释放两个串口。撞墙与视觉阶段机械臂固定为 Servo0=0、Servo1/2/4=0/0/135（旧步骤1-5初始姿态）避免遮挡 K230D 视野，READY 后恢复 Servo0=95 并重新执行旧步骤1-5确认抓取姿态；PREPARE 在距墙 0.05 m 处重跑，前伸空间待实车确认。

### 3.6 独立机械臂模式

```text
robot_startup.py
  -> Stm32Link.connect()
  -> HELLO_STM32,1 / STM32_OK,1
  -> STEPPER_MOVE,-1500,800 / ACK / STEPPER_MOVE_DONE
  -> STOP / ACK
  -> 关闭串口

build_sequence.py
  -> 打开 STM32
  -> HELLO_STM32,1
  -> 固定 startup raise
  -> 人工菜单选择 group 1/2/3/all
  -> STOP / 关闭串口

tests/test_old_grab_left_sequence.py
  -> 打开 STM32并执行 HELLO_STM32,1
  -> 等待操作者单独按 ENTER
  -> 旧步骤1-18：前伸 -> 前进0.50m -> 后退0.05m -> 抓取 -> 后退0.30m
  -> Servo2=180 -> Servo0=0 -> Servo1 Sweep_To(170,10)
  -> Stepper +5000 -> Servo4=135 -> Stepper -5000
  -> Servo0..4=95/135/90/95/90 -> 等待500ms
  -> STOP / 关闭串口

tests/test_old_grab_right_sequence.py
  -> 打开 STM32并执行 HELLO_STM32,1
  -> 等待操作者单独按 ENTER
  -> 旧步骤1-18：前伸 -> 前进0.50m -> 后退0.05m -> 抓取 -> 后退0.30m
  -> Servo2=180 -> Servo0=180 -> Servo1 Sweep_To(170,10)
  -> Stepper +5000 -> Servo4=135 -> Stepper -5000
  -> Servo0..4=95/135/90/95/90 -> 等待500ms
  -> STOP / 关闭串口
```

`robot_startup.py` 复用 `hardware.stm32_link.Stm32Link`，不自行创建串口协议实现。它只用于人工启动前机械臂预升高，不是真正 Homing：升降轴没有限位开关或编码器，脚本无法判断绝对高度，重复运行会继续向上移动。连接、HELLO、Stepper、ERR 或 Ctrl+C 异常均退出，并在连接仍有效时执行 best-effort STOP。该入口不会被 `CompetitionRunner`、`MissionController` 或任何自动比赛流程调用。

左右盒脚本通过`Stm32Link`向`ArmSequence`提供唯一串口连接。两者复用旧步骤1–18和底盘距离实现，完成到墙、抓取、收回、后撤及目标盒放置后，均依次发送Servo0..4=95/135/90/95/90并等待500 ms，再执行最终STOP。中盒原测试和competition抓取模块使用同一组末端角度。`ArmSequence`既有搭建动作及`safe_home()`未修改。

这些入口用于机械臂回归和分组调试，不应和整车主控同时运行。

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
| 拓扑导航状态 | current_location、current_tag、target_location | 只表示已确认地标和规划目标，不表示底盘已经执行路线 |
| 路线执行状态 | IDLE、PLANNING、READY、WAITING_JUNCTION、TURNING、WAITING_TURN_DONE、ROUTE_TURNS_DONE、FINAL_TAG_APPROACH、ARRIVED、FINISHED、FAULT | 逐路口动作和最终 Tag 到站；当前动作下标可观察 |
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
       │    ├─ navigation.tag_map（旧字段兼容）
       │    └─ navigation.field_map（get_zone_by_tag 接口）
       ├─ LINE_MAP
       ├─ TOPOLOGY_NODES / TOPOLOGY_EDGES
       ├─ TOPOLOGY_EDGE_DIRECTIONS
       └─ TAG_TOPOLOGY_LOCATIONS
             -> TopologicalNavigator
                  ├─ update_from_tag(tag_id)
                  └─ plan_route(start, target) / BFS
                        -> RouteActionResolver
                             └─ junction actions: LEFT / RIGHT / STRAIGHT
                                  -> RouteExecutor
                                       ├─ ordered NEXT_TURN / TURN_DONE
                                       └─ NavigationController final Tag arrival
```

- 官方场地尺寸、区域尺寸、Tag 尺寸和巡线拓扑已录入。
- `left_orange_material`（Tag 2）和 `right_orange_material`（Tag 4）是两个独立逻辑位置，都声明 `ORANGE_PICKUP`；共享的 `orange_material` 保留规则给出的物理尺寸事实。
- `build_zone` 与右侧橙色材料区保持分离。其黑线连接和停止点尚未确认，因此目前没有为它虚构拓扑边。
- `strategy/mission.py` 单独声明 `PREFERRED_ORANGE_SOURCE = LEFT_ORANGE_MATERIAL` 和 Tag 2；Tag 4 仍是可选橙色来源，但第一版流程不主动选择。
- 软件坐标系由项目约定，不是规则定义。
- 规则没有标注的 Tag、路口和线段绝对坐标保持 `None`。
- 不能用规则示意图像素比例伪造厘米坐标。

`from navigation import TAG_MAP` 当前导出的是兼容语义地图；需要完整世界坐标字段时应从 `navigation.field_model` 使用详细 `TAG_MAP`。

TopologicalNavigator 构造时只把上述节点和无向边转换成邻接表，不拥有任何硬件。`update_from_tag(2/3/4)` 分别确认左侧橙色、紫色、右侧橙色逻辑位置；未知 Tag 不改变最近确认的位置。`plan_route()` 用 BFS 返回最少边数的逻辑节点列表，只更新 `target_location`，不会把规划结果伪装成已经移动。当前已确认的典型逻辑路线为：

```text
START_ZONE
  -> START_T_JUNCTION
  -> GRAY_CORRIDOR
  -> LEFT_MATERIAL_T_JUNCTION
  -> LEFT_ORANGE_MATERIAL (Tag 2, preferred)

LEFT_ORANGE_MATERIAL
  -> LEFT_MATERIAL_T_JUNCTION
  -> RIGHT_MATERIAL_T_JUNCTION
  -> ROOF_MATERIAL_T_JUNCTION
  -> PURPLE_MATERIAL (Tag 3)
```

灰色过道已按从西到东实际经过的路口顺序展开为 `START_T_JUNCTION -> GRAY_CORRIDOR -> LEFT_MATERIAL_T_JUNCTION -> RIGHT_MATERIAL_T_JUNCTION`，避免 BFS 从共享 corridor 节点直接跳过一个真实 T 路口。节点和边仍没有距离或 x/y。

`TOPOLOGY_EDGE_DIRECTIONS` 只保存图 3.8/3.9 和项目坐标约定支持的离散方向：三段灰廊连接向东；启动线、两侧材料竖线向北；屋顶线从屋顶 T 至紫色材料区向西。解析器自动得到反向边。`RouteActionResolver` 比较入边与出边方向，只在 `kind=junction` 节点生成动作：同向为 `STRAIGHT`，逆时针 90° 为 `LEFT`，顺时针 90° 为 `RIGHT`。缺失元数据、非路口转向和 180° 掉头均抛出 `RouteActionUnknownError`，绝不默认直行。路线若从路口开始，必须额外传入当前 heading；否则第一条边没有可靠的入向。

TopologicalNavigator 和 RouteActionResolver 仍只进行纯数据转换，不导入 STM32、相机或任务状态。RouteExecutor 位于其上层：只在显式 start 后通过注入的 Stm32Link 逐路口调度 pending turn，再把最终到站交回 NavigationController。STM32 不理解 route；它根据 `1110/0111` 的侧向证据选择 LEFT/RIGHT，并在全黑或证据冲突时使用 pending LEFT/RIGHT 消歧。当前这条连续路线链只接入独立人工入口，尚未接入 CompetitionRunner。

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

NavigationController 同样只持有注入的 Stm32Link/AprilTagService。材料区请求只保存目标；Tag 对准接口仍不生成姿态修正；Tag 辅助到站复用已有 `LINE_START` 和 STOP，只增加 Pi 侧逐帧观测与状态转换。新增 `continue_line_to_tag()` 复用完全相同的到站条件，但用于已有 LineNav 运行期，因此不重复发送 LINE_START。CompetitionRunner 再持有三类控制器和 MissionController，不拥有任何端口或摄像头，本轮也未调用新能力。

TopologicalNavigator 位于 NavigationController 的上层结果消费者位置，但尚未注入 CompetitionRunner：独立人工入口只在 NavigationController 已对指定 Tag 完成 STOP ACK 并进入 ARRIVED 后调用 `update_from_tag(tag_id)`。该分层不会增加第二个 STM32 串口或 Camera 所有者。

RouteExecutor 只组合上述对象，不创建资源。多路口人工入口中的所有依赖关系为：

```text
test_multi_junction_route.py
├── 一个 Stm32Link
├── 一个 AprilTagService
├── NavigationController ──复用两者
└── RouteExecutor
    ├── TopologicalNavigator
    ├── RouteActionResolver
    ├── NavigationController
    └── 同一个 Stm32Link
```

路口阶段的同步读取权属于 RouteExecutor；最终 Tag 阶段没有 STM32事件读取循环，NavigationController 只读取 AprilTag，并在确认到站时同步执行 STOP/ACK。该阶段切换避免两个组件同时调用 `readline()`。

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
- Tag 5 早前偶发出现车辆已停止但首次 STOP ACK 等待超时；加入 STOPDBG 后的后续实测未再复现。当前继续保持 2.0 s timeout 和原 STOP 协议，根因尚未确认。

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
- Stm32Link 已能解析路口事件；独立 RouteExecutor 现在严格消费它们，NavigationController 的通用 `handle_event()` 仍保持未实现。`TURN_DONE` 只完成一个 pending turn，不是 `LINE_DONE`；NavigationController 的 `ARRIVED` 仍只表示目标 Tag 达标并收到 STOP ACK。
- 当前 `NavigationController` 尚未封装 `LINE_STATUS` 或消费 `ERR,LINE_SENSOR_IO`；本次只在人工诊断脚本中透传验证，避免在实测前扩展比赛状态机。
- STM32 普通 LINE_FOLLOW 四路全白时继续前进仍是已知风险；新增路口动作阶段会由 timeout 停车，但不能掩盖普通巡线风险。
- 近距离目标 Tag 可能因低机位移出画面；当前连续帧逻辑会清零确认计数，没有“曾经接近后突然消失”的专用判断。RouteExecutor 最终 Tag watchdog 只能提供较晚的超时停车，不能替代后续实机安全策略。
- TopologicalNavigator 只有认知和规划能力；无路径时抛出 `RouteNotFoundError`，不会补一条假直线、估算坐标或触发运动。

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
tests/test_navigation_topology_live.py 人工确认后验证巡线、Tag ARRIVED 和拓扑更新；会驱动底盘
tests/test_navigation_controller_unit.py  Fake STM32/AprilTag 纯软件状态测试
tests/test_topological_navigation_unit.py 双橙色区域、Tag 定位和 BFS 纯软件测试
tests/test_route_action_resolver_unit.py 边方向、LEFT/RIGHT/STRAIGHT、未知路线和禁止默认猜测的纯软件测试
tests/test_route_executor_unit.py 多路口严格事件顺序、逐项NEXT_TURN、watchdog和最终Tag到站Fake测试
tests/test_multi_junction_route.py 严格START后的多路口+最终Tag实机入口；会驱动底盘并打开Camera
tests/test_multi_junction_route_unit.py START门禁和纯软件路线预览测试
tests/test_topological_navigation_live_unit.py 人工入口取消/错误 Tag/异常清理的纯软件测试
tests/test_stm32_stop_unit.py STOP ACK 时序、无关行、迟到 ACK 歧义、无 ACK 和 ERR 的 FakeSerial 测试
tests/test_line_sensor_capture.py 只读 LINE_STATUS、人工标记和 CSV 现场采样；不驱动执行器
tests/test_line_sensor_capture_unit.py LINE_STATUS 解析、失败状态、唯一命令、CSV/标记纯软件测试
tests/test_junction_turn.py   直接动作或start/target路线模式；严格START后只执行一个真实路口动作
tests/test_junction_turn_unit.py NEXT_TURN/事件解析、路线首动作、START门禁和单路口STOP的FakeSerial纯软件测试
tests/vel_test.py            会驱动底盘
tests/motor_cmd_test.py      启动后会依次驱动底盘
tests/line_test.py           任意文本命令透传并原样显示回复；可安全发送 LINE_STATUS，也可发送会驱动底盘的命令
tests/servo_test.py          通用命令透传，可能驱动任意执行器
tests/test_visual_grab_sequence.py 人工确认后撞墙、K230D视觉定位、单次抓取与放盒；会驱动底盘/机械臂并打开STM32与K230D串口
其他 uart/stm32 脚本        链路诊断，部分使用历史设备路径
```

任何硬件测试都必须由人工逐个启动。同一时刻不得运行另一个占用 STM32 串口的程序；运动测试前需清空场地并准备急停。相关纯软件回归可运行 `python -m unittest tests.test_route_executor_unit tests.test_multi_junction_route_unit tests.test_route_action_resolver_unit tests.test_topological_navigation_unit tests.test_navigation_controller_unit tests.test_junction_turn_unit -v`，不会打开串口或相机；其他 STOP 和传感器单元测试也保持纯软件运行。

## 10. 已落地骨架与下一步

当前已落地的轻量骨架不引入 ROS 或复杂 SLAM：

```text
输入层
├── K230D：颜色/物块结果
├── USB Camera：AprilTag 结果
└── STM32：ACK、DONE、ERR、LINE_STATUS、JUNCTION/TURN事件
        │
        v
世界与状态层
├── field_model：区域、Tag、线路
├── MissionController：任务阶段
├── CompetitionRunner：事件顺序与控制器编排
├── TaskController：已验证机械臂任务适配
├── VisualApproachController：K230 目标对准与接近
├── NavigationController：巡线启动、Tag 面积连续确认、STOP 到站与观测状态
├── TopologicalNavigator：Tag 地标定位与无坐标 BFS 路线（尚未接入 Runner）
├── RouteActionResolver：离散方向到路口动作
└── RouteExecutor：逐项NEXT_TURN、严格TURN_DONE和最终Tag到站（尚未接入Runner）
        │
        v
执行层
├── chassis：VEL / STOP / line control
└── ArmSequence：Servo / Stepper
        │
        v
STM32 唯一串口链路
```

仍未完成的部分包括：实车标定并验证 T 字路口 LEFT/RIGHT/STRAIGHT 和 RouteExecutor 多路口链、处理普通巡线全白与近距离 Tag 移出画面的安全问题、把 RouteExecutor 接入 Runner、确认搭建区连线、设计比赛级 live 生命周期、实现 Tag 姿态修正、补入 GET_PURPLE、实机确认独立橙色/紫色取料动作，以及完整安全收尾和系统化测试。STOPDBG 继续保留供早前偶发 ACK timeout 再次出现时使用。

实现顺序应继续优先保证协议和串口所有权，其次接入状态转换，最后再标定世界坐标和视觉阈值。不要在全自动状态机未可观测、未可安全停车前一次性叠加全部动作。
