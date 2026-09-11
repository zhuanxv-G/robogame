# RoboGame 当前状态

> 状态快照：2026-09-11
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

仓库中没有 STM32 的 `.c`、`.h`、`.ioc`、链接脚本或 IDE 工程文件；Git 历史中也没有发现这些文件。当前仓库实质上是 Raspberry Pi 端工程以及与 STM32 交互所需的协议调用代码，而不是完整的上下位机单体仓库。

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
| 巡线传感器 | 黑线检测 | 由 STM32 侧处理 | 型号、数量、引脚和算法不在本仓库 |
| 底盘电机及驱动 | 底盘执行 | 由 STM32 侧控制 | 电机和驱动器具体型号不在本仓库 |

当前资料还缺少完整 BOM、供电树、接线图、GPIO/定时器/串口引脚表、传感器型号及 STM32 固件版本标识。

## 4. 软件架构

### 4.1 Raspberry Pi 职责

- 从 K230D 串口接收物块视觉结果。
- 从 USB Camera 获取图像并检测 tag36h11 AprilTag。
- 根据 `error_x`、目标面积等信息进行轻量对准和接近判断。
- 保存比赛场地区域、Tag 和巡线拓扑世界模型。
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

上述职责来自 Pi 端调用、测试脚本和项目已有记录。由于 STM32 源码不在仓库，接收缓冲、参数校验、看门狗、底层电机闭环和中断实现无法在本次分析中复核。

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
| STM32 底盘、电机、巡线 | 底层已验证，上层整合不完整 | 项目明确这些底层模块已完成；仓库仅有 Pi 测试端，没有 STM32 源码 |
| Servo、Stepper、机械臂 | 动作层已验证 | `ArmSequence` 可复用，三组搭建动作和安全 HOME 已保留；不要改动参数 |
| K230D 物块视觉 | 单项检测已验证 | 当前主控能解析目标并完成对准/接近；完整橙色、紫色任务切换尚未接入 |
| K230D 接口层 | 骨架完成，未实机回归 | `K230Link` 封装连接生命周期、现有 `TARGET` / `NO_TARGET` 解析格式和最新消息读取 |
| K230D 对准到搭建 | 第一版已接通 | `main_controller_align.py` 可从 ALIGN/APPROACH 到 READY，再经人工 `b` 确认执行 ARM_BUILD |
| competition 视觉接近 | 软件迁移完成，未实机回归 | `VisualApproachController` 保留原阈值、EMA、速度、三帧 READY 和超时 STOP；尚未由 MissionController 自动运行 |
| AprilTag 检测 | 基础模块完成 | 输出 id、中心、误差、像素面积和角点；无 GUI，适合 SSH |
| AprilTag 服务层 | 骨架完成，未实机回归 | `AprilTagService` 延迟创建原 `AprilTagDetector`，没有修改检测算法和参数 |
| AprilTagLocalization | 基础接口完成 | 每帧选择面积最大 Tag，输出语义名称和面积启发式 confidence |
| 比赛世界模型 | 结构已建立，坐标未标定 | 官方尺寸和拓扑已结构化；绝对区域、Tag、路口坐标仍为 TODO |
| 导航/任务状态定义 | 基础接口完成 | `competition.MissionState` 新增 FINISH、FAULT；`MissionController` 只负责显式切换和复位，没有运行时调度策略 |
| competition 导航控制 | Pi 端抽象完成，未实机回归 | `NavigationController` 可发送已有 `LINE_START`、通过 Stm32Link STOP、记录材料区，并通过 AprilTagService 查询指定 Tag；没有巡线完成事件或自动路线 |
| competition 流程调度 | 第一版模拟流程完成 | `CompetitionRunner` 通过外部事件串联巡线、橙色目标接近、抓取、搭建和结束；默认 simulation mode 不调用硬件控制器 |
| 机械臂任务控制 | 骨架完成，未实机回归 | `TaskController.build()` 复用 `ArmSequence.run_all()`，`reset_arm()` 复用 `safe_home()`；两种独立取料动作尚未确认 |
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
- `stop_navigation()` 只通过 Stm32Link 发送现有 STOP。
- `navigate_to_material_zone()` 只接受 `ORANGE` / `PURPLE` 并记录目标，不生成路线或底盘命令。
- `align_with_tag()` 只调用 AprilTagService 读取一帧并记录指定 Tag 是否可见；`TAG_ALIGN` 目前不代表已经执行姿态修正。
- `handle_event()` 返回 `NotImplemented`，没有假设 `LINE_DONE`、`TAG_REACHED` 或其他尚未验证的 STM32 事件。

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

### 6.4 视觉人工测试入口

```bash
python tests/test_apriltag.py
python tests/test_localization.py
```

二者会持续打开相机，按 Ctrl+C 释放；不会打开 STM32，也不会驱动机器人。

### 6.5 STM32 入口

本仓库没有 STM32 工程，无法给出真实的 `main.c` 路径。必须从固件仓库或工程备份补充入口、工程配置和固件版本。

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

本次只做语法、import、安全入口和 Fake Controller 检查，包括默认模拟零调用、完整事件顺序、真实委托分支、未实现抓取阻塞、乱序事件和 FAULT；没有连接串口、摄像头或运行任何会运动硬件的脚本，因此没有重新执行上述实机测试。

### 7.2 测试代码性质

`tests/` 不是 pytest 自动化测试套件，而是一组人工硬件诊断脚本。多数脚本在导入或启动时立即打开串口，其中 `motor_cmd_test.py` 会直接让底盘运动；不能批量运行、不能由 pytest 自动收集，也不能在主控制器占用同一串口时运行。

AprilTag 两个脚本同样是相机人工测试，不包含断言。当前仓库没有模拟串口、纯逻辑单元测试、持续集成配置或统一回归清单。

## 8. 当前未完成任务和已知缺口

1. `CompetitionRunner` 已定义第一版事件顺序，但 `LINE_DONE`、`TARGET_READY`、`ARM_DONE`、`BUILD_DONE` 目前均没有真实生产者，只能由测试或未来适配层注入。
2. `competition/main.py` 固定使用 simulation mode，尚未实现显式连接、握手、运行循环、统一退出 STOP 和资源关闭；当前可运动的联调入口仍是 `main_controller_align.py`。
3. Pi 端目前只能发送已有 `LINE_START` 和 STOP；STM32 巡线与比赛任务之间仍缺少经过验证的到站、路口、完成和错误事件，`ARRIVED` 不会自动进入。
4. 世界模型缺少六个 Tag 中心 `(x, y, yaw)`、黑线节点/线段长度和关键停车点实测值。
5. AprilTag 目前只有像素误差和像素面积，没有相机内参、畸变校正、姿态估计或跨帧稳定策略。
6. K230D 的橙色、紫色检测结果如何触发取料尚未定义；TaskController 已有接口，但现有 ArmSequence 没有经过确认的独立橙色/紫色抓取动作。
7. 新增链路层已引用 `config.py`；旧主控和部分诊断脚本仍复制路径或使用 `/dev/ttyUSB*` 等易漂移路径。
8. 没有 `requirements.txt` 或其他依赖锁定文件。已知 Pi 环境包含 `pyserial`、`numpy`、`opencv-python-headless` 和 `pupil_apriltags`，但版本没有记录。
9. `main_controller_align.py` 在模块顶层打开串口，且主 `try/finally` 在串口打开和启动等待之后才开始；它不适合被其他模块 import。
10. 当前主控启动时没有执行 `HELLO_STM32,1` 版本握手；独立机械臂入口会执行。
11. 缺少 STM32 源码、固件构建配置和当前烧录版本，无法从仓库复核接收端参数范围与错误码全集。
12. `strategy/mission.py` 的旧任务元数据尚未迁移到新的 `competition.MissionState`，两层接口暂时并存。

## 9. 现有文档评估

| 文档 | 是否可继续使用 | 需要注意/补充 |
| --- | --- | --- |
| `README.md` | 可作为快速入口 | 目录树和比赛入口说明尚未列出新增的视觉接近、导航与流程调度控制器 |
| `docs/PI_ENV.md` | 可继续使用 | SSH、部署路径、Git 安全流程和正式设备路径仍有效 |
| `docs/project_inventory.md` | 可作为历史盘点 | 它生成于机械臂重构前，依赖图和正式入口分析已经过时 |
| `docs/FIELD_DESCRIPTION.md` | 当前场地事实主文档 | 官方尺寸与 TODO 分界清晰；绝对坐标仍需现场测量 |
| `tests/README.md` | 可作为硬件测试安全说明 | 尚未列出两个 AprilTag 测试脚本 |
| `legacy/README.md` | 可继续使用 | 清楚标出旧脚本和旧协议，不应作为现役入口 |
| `docs/PI_STM32_PROTOCOL.md` | 本次新增 | 作为当前 Pi 端协议基线；需用 STM32 源码进一步交叉验证 |
| `docs/SOFTWARE_ARCHITECTURE.md` | 本次新增 | 描述当前实际链路和目标整合边界 |

此前项目上下文曾引用 `docs/STM32_PROJECT_MAP.md` 和 `docs/STM32_SAFETY_PATCHES.md`，但本次工作区和 Git 历史中均未找到这两个文件。若文档位于另一个 STM32 仓库，应复制受控版本或建立明确交叉链接。

## 10. 下一阶段开发建议

建议按以下顺序推进，每一步先做无运动模拟，再做单项实机验证：

1. 找回或关联 STM32 工程和固件版本，补齐接收端协议、参数边界、错误码和巡线事件定义。
2. 为 CompetitionRunner 设计独立、显式的 live 启动入口，补齐连接、握手、循环、STOP 确认和异常关闭生命周期；在完成安全审查前保持 `competition/main.py` 默认 simulation mode。
3. STM32 端事件经实机确认后，再由协议适配层把它们映射为 Runner 事件；不要直接把尚未验证的字符串加入 Stm32Link 或 NavigationController。
4. 现场测量 Tag、T 字路口和停车点，填写 `field_model.py` 中的 TODO；固定相机后再标定面积和误差阈值。
5. 给 K230D 解析、AprilTag 目标选择、任务转移和协议格式增加不连接硬件的单元测试及串口 mock。
6. 统一配置与依赖记录，但迁移时保持已验证设备路径、速度、Servo 角度、Stepper 步数和延时原值不变。

当前最准确的完成度描述是：底层执行和各视觉/机械能力已具备，旧入口局部联调已完成；比赛分层、无副作用模拟入口、第一版事件调度、机械臂任务适配、视觉接近和 Pi 端导航抽象已建立；真实事件源、live 生命周期、自动路线、Tag 姿态修正、独立橙色/紫色取料、实机集成回归和系统化自动测试仍未完成。
