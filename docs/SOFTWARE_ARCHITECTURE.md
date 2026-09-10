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
| 设备/协议 | `arm_sequence.py` 中的请求方法 | HELLO、STOP、Servo、Stepper 请求/回复 | 已用于正式入口 |
| K230D 输入 | `vision/k230_link.py` | 显式连接 K230D，解析 TARGET/NO_TARGET | 骨架完成；未迁移旧入口 |
| K230D 解析 | `main_controller_align.py::parse_k230` | 解析 TARGET/NO_TARGET | 已用于当前主控 |
| AprilTag 服务 | `vision/apriltag_service.py` | 延迟初始化并调用原 AprilTagDetector | 骨架完成；算法不变，未接入调度 |
| AprilTag 视觉 | `vision/apriltag_detector.py` | 采集相机并检测 tag36h11 | 模块和人工测试完成 |
| 轻量定位 | `navigation/localization.py` | 选择主 Tag，输出语义、error_x、area、confidence | 模块和人工测试完成 |
| 世界模型 | `navigation/field_model.py` | 场地、区域、Tag、巡线拓扑 | 已建立；绝对坐标 TODO |
| 兼容地图接口 | `navigation/field_map.py`、`tag_map.py` | 保持 `get_zone_by_tag()` 和旧 TAG_MAP 可用 | 已完成 |
| 导航状态接口 | `navigation/navigation_state.py` | LINE_FOLLOW、SEARCH_TAG、TAG_ALIGN、TASK_EXECUTE | 只定义，未运行 |
| 比赛策略 | `strategy/mission.py` | START 到 BUILD 的任务顺序和元数据 | 只定义，未运行 |
| 比赛状态 | `competition/mission_state.py` | START、FOLLOW_LINE、GET_ORANGE、GET_PURPLE、BUILD、FINISH、FAULT | 已定义 |
| 比赛状态管理 | `competition/mission_controller.py` | 保存、显式切换、复位顶层状态 | 基础骨架完成；无动作逻辑 |
| 比赛任务控制 | `competition/task_controller.py` | 把比赛任务接口适配到已有 ArmSequence 动作 | build/reset 已接入；独立取料待确认 |
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
  -> AprilTagService(disconnected)
  -> 打印状态并退出
```

该链路只验证软件对象可以组合，不调用 `connect()`、`build()` 或 `reset_arm()`，因此不会打开 STM32/K230D 串口或 USB Camera，也不会发送任何控制命令。

TaskController 只负责任务级委托，不保存端口、创建串口或复制机械动作：

```text
TaskController.build()
  -> ArmSequence.run_all()
  -> group1() -> group2() -> group3()

TaskController.reset_arm()
  -> ArmSequence.safe_home()
```

`grab_orange()` 和 `grab_purple()` 当前返回 `NotImplemented`。仓库中的三个已验证 group 都包含抓取和放置，尚无颜色到独立抓取动作的可靠映射。`startup_raise()` 仍是独立入口的启动策略，不由 `build()` 隐式执行。

### 3.2 K230D 对准与机械臂搭建模式

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

### 3.3 独立机械臂模式

```text
build_sequence.py
  -> 打开 STM32
  -> HELLO_STM32,1
  -> 固定 startup raise
  -> 人工菜单选择 group 1/2/3/all
  -> STOP / 关闭串口
```

该入口用于机械臂回归和分组调试，不应和整车主控同时运行。

### 3.4 AprilTag 人工测试模式

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

仓库中存在三套不同抽象层级的状态，它们还没有由统一调度器连接：

| 层级 | 状态 | 作用 |
| --- | --- | --- |
| 当前 K230D 控制状态 | STOP、ALIGN、APPROACH、READY、ARM_BUILD、BUILD_DONE、ERROR | 一次视觉接近和搭建联调 |
| 导航能力状态 | LINE_FOLLOW、SEARCH_TAG、TAG_ALIGN、TASK_EXECUTE | 未来可复用的导航行为接口 |
| 旧比赛任务元数据 | `strategy/mission.py` 中 START、FOLLOW_LINE、GET_ORANGE、GET_PURPLE、BUILD | 已有任务计划数据，尚未迁移 |
| 新比赛任务状态 | `competition/mission_state.py` 中 START、FOLLOW_LINE、GET_ORANGE、GET_PURPLE、BUILD、FINISH、FAULT | 未来比赛入口使用的顶层状态 |

目标不是把三套枚举简单合并，而是让比赛任务选择一个导航/执行行为，再由行为内部运行短周期控制状态。例如：

```text
MissionState.GET_ORANGE
  -> NavigationState.LINE_FOLLOW / SEARCH_TAG
  -> 找到 Tag 2
  -> NavigationState.TASK_EXECUTE
  -> K230D orange detection + 抓取动作
```

`MissionController` 现在可以保存、显式切换和复位新比赛状态，但没有自动转移、事件处理或 TaskController 动作调用。旧 `strategy/mission.py` 的任务元数据也尚未迁移到新状态层。

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
competition/main.py（未来唯一所有者；当前不连接）
├── K230D 输入组件
├── AprilTag 输入组件
├── MissionController
├── NavigationController
└── TaskController
     └── ArmSequence
          └── Stm32Link（唯一 STM32 serial.Serial 所有者）
```

`Stm32Link` 的构造不会打开端口，只有显式 `connect()` 才创建 `serial.Serial`。它统一保留现有 LF 文本帧、两位小数 VEL、ACK/DONE 前缀等待、ERR 立即异常和超时异常语义。为在不修改已验证 `arm_sequence.py` 的前提下完成依赖接入，Stm32Link 新增受连接状态保护的 `write()`、`flush()`、`readline()` 兼容转发；底层 `serial.Serial` 仍不向 TaskController 或 ArmSequence 暴露，也没有第二个所有者。

旧 `main_controller_align.py` 和 `build_sequence.py` 尚未迁移，当前仍使用各自已经验证的串口对象和调用路径；`arm_sequence.py` 本身保持完全不变，并已可由新 competition 骨架通过 Stm32Link 复用。

## 7. 安全和故障流

当前局部主控已有以下安全路径：

- 启动前尝试发送 STOP。
- 目标持续丢失超过 0.30 s 后 STOP。
- K230D 超过 0.60 s 没有有效数据后 STOP。
- READY 阶段收到 `ACK,STOP` 前不进入机械臂动作。
- ARM_BUILD 阶段不再发送底盘运动命令。
- Ctrl+C、运行异常和 finally 路径尝试 STOP 并关闭串口。
- Stepper 动作等待 ACK 和 MOVE_DONE，收到 ERR 立即失败。

仍存在的架构风险：

- `main_controller_align.py` 在模块顶层打开两路串口，主 try/finally 没有覆盖打开失败和启动等待全过程。
- 当前主控没有 STM32 协议版本握手。
- 回复协议没有请求 ID，必须严格保持单一读取者。
- `BUILD_DONE` 为锁存终态，只能结束进程，尚无下一比赛阶段。
- 独立测试多为 import 即执行，不能作为自动化测试集合运行。
- 没有上限位器，startup raise 不能视为 Homing。
- 新比赛入口还没有异常到 `MissionState.FAULT` 的映射，也没有退出时的统一 STOP/关闭策略；在真正连接硬件前必须补齐。
- MissionController 尚未定义何时调用 build/reset；独立橙色和紫色取料动作也没有实机确认，不能把任意 group 直接映射为颜色任务。

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
tests/vel_test.py            会驱动底盘
tests/motor_cmd_test.py      启动后会依次驱动底盘
tests/line_test.py           可发送底盘/巡线命令
tests/servo_test.py          通用命令透传，可能驱动任意执行器
其他 uart/stm32 脚本        链路诊断，部分使用历史设备路径
```

任何硬件测试都必须由人工逐个启动。同一时刻不得运行另一个占用 STM32 串口的程序；运动测试前需清空场地并准备急停。

## 10. 已落地骨架与下一步

当前已落地的轻量骨架不引入 ROS 或复杂 SLAM：

```text
输入层
├── K230D：颜色/物块结果
├── USB Camera：AprilTag 结果
└── STM32：ACK、DONE、ERR、巡线事件
        │
        v
世界与状态层
├── field_model：区域、Tag、线路
├── MissionController：任务阶段
├── TaskController：已验证机械臂任务适配
└── NavigationController：当前行为
        │
        v
执行层
├── chassis：VEL / STOP / line control
└── ArmSequence：Servo / Stepper
        │
        v
STM32 唯一串口链路
```

仍未完成的部分包括：把旧调用方迁移为共享一个 `Stm32Link`、为 MissionController 定义事件与合法转移、接入 STM32 巡线事件、实机确认独立橙色/紫色取料动作、连接 K230D/AprilTag 结果与任务调用、实现 BUILD/FINISH/FAULT 的安全收尾，以及系统化无硬件测试和后续实机回归。

实现顺序应继续优先保证协议和串口所有权，其次接入状态转换，最后再标定世界坐标和视觉阈值。不要在全自动状态机未可观测、未可安全停车前一次性叠加全部动作。
