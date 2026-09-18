# RoboGame 2026

## 1. 项目简介

本项目是 RoboGame 2026 竞技组机器人的 Raspberry Pi 上位机工程。比赛任务包括沿场地线路移动和定位、识别并获取橙色与紫色建筑材料，以及在搭建区完成结构搭建。

当前机器人采用 Raspberry Pi 5、STM32F407ZGT6 和 K230D / CanMV 协同工作。Raspberry Pi 负责视觉结果处理、比赛状态和任务决策；STM32 负责底盘、巡线、Servo 和 Stepper 的实时执行；K230D 负责物块视觉检测，USB Camera 用于 AprilTag 检测。

项目正在开发中。单项硬件能力、机械臂搭建动作和局部视觉联调已有验证记录；第一版无视觉固定路线比赛流程已完成代码与 Fake 测试，但尚未整车实测，视觉完整比赛闭环仍未完成。本仓库只包含 Raspberry Pi 端代码，不包含 STM32 工程。

## 2. 系统总体架构

```text
K230D / CanMV ──USB Serial──┐
                            │
USB Camera ─────USB Video───┼──> Raspberry Pi 5
                            │    ├─ K230 / AprilTag 视觉接口
                            │    ├─ MissionController
                            │    ├─ TaskController
                            │    └─ Stm32Link（目标唯一串口所有者）
                            │              │
                            └──────────────┼── CH340 USB Serial
                                           v
                                     STM32F407ZGT6
                                     ├─ 麦克纳姆轮底盘
                                     ├─ 本地巡线
                                     ├─ 5 路 Servo
                                     └─ TB6600 Stepper
```

Raspberry Pi 侧职责：

- 管理比赛顶层状态。
- 接收 K230D 物块检测结果。
- 使用 USB Camera 检测 AprilTag。
- 保存场地区域、Tag 和巡线拓扑信息。
- 调度机械臂任务并向 STM32 发送命令。

STM32 侧职责：

- 执行底盘速度、停车和本地巡线控制。
- 执行 Servo 绝对角度动作。
- 执行 Stepper 指定步数运动。
- 返回 ACK、完成通知和错误信息。

新比赛架构要求一个进程只持有一个 STM32 串口连接。`competition/main.py` 已建立 `TaskController -> ArmSequence -> Stm32Link` 依赖，但目前不会连接硬件；`competition/full_mission.py` 是独立的无视觉固定路线 live 入口，同样只使用一个 `Stm32Link`。现有其他联调入口仍保留原来的串口调用方式，尚未迁移。

## 3. 软件目录结构

```text
robogame/
├── competition/
│   ├── main.py                    # 未来比赛入口，目前只初始化组件
│   ├── full_mission.py            # 无视觉固定路线完整比赛入口
│   ├── mission_state.py           # 比赛顶层状态
│   ├── mission_controller.py      # 基础状态管理
│   └── task_controller.py         # 机械臂任务适配
├── hardware/
│   └── stm32_link.py              # STM32 通信与串口生命周期
├── vision/
│   ├── k230_link.py               # TARGET / NO_TARGET 接口
│   ├── apriltag_service.py        # AprilTag 服务生命周期
│   └── apriltag_detector.py       # 现有 AprilTag 检测算法
├── navigation/
│   ├── localization.py            # AprilTag 轻量定位
│   ├── field_model.py             # 场地世界模型
│   ├── field_map.py               # 区域查询兼容接口
│   ├── tag_map.py                 # Tag 语义兼容接口
│   └── navigation_state.py        # 导航状态定义
├── strategy/
│   └── mission.py                 # 旧任务顺序和元数据
├── docs/                          # 状态、架构、环境和场地文档
├── tests/                         # 人工硬件诊断脚本，并非自动化测试套件
├── legacy/                        # 历史主控和旧协议原型
├── arm_sequence.py                # 已验证机械臂动作序列
├── build_sequence.py              # 独立机械臂搭建入口
├── main_controller_align.py       # 视觉接近与搭建联调入口
├── config.py                      # 正式设备路径和波特率
└── RoboGame2026 竞技组规则手册3_0.pdf
```

详细资料：

- [当前工程状态](docs/CURRENT_STATUS.md)
- [软件架构](docs/SOFTWARE_ARCHITECTURE.md)
- [场地信息](docs/FIELD_DESCRIPTION.md)
- [Raspberry Pi 环境与部署](docs/PI_ENV.md)
- [测试脚本说明](tests/README.md)
- [历史代码说明](legacy/README.md)

## 4. 当前开发状态

### 已完成或已建立

- Pi 与 STM32 的现役文本协议及基础通信已使用过。
- `Stm32Link` 已封装 HELLO、STOP、VEL、Servo、Stepper、ACK、DONE、ERR 和显式连接接口；新抽象层尚未实机回归。
- `K230Link` 已封装 `TARGET` / `NO_TARGET` 解析和串口生命周期。
- AprilTag 检测器、服务接口和轻量定位接口已经建立。
- 场地区域、Tag 和巡线拓扑世界模型已经建立，未知实测坐标保留为 TODO。
- `MissionState`、`MissionController` 和安全的比赛入口骨架已经建立。
- `competition/full_mission.py` 已串联三次 T 路口、现有抓取流程、固定回线右转和搭建区停车；纯软件测试完成，待整车实测。
- `TaskController.build()` 和 `reset_arm()` 分别复用已验证的 `ArmSequence.run_all()` 与 `safe_home()`。
- 机械臂三组搭建动作、Stepper 完成等待和安全 HOME 有实机验证记录。
- `main_controller_align.py` 已完成 K230D 对准、接近、停车确认和人工触发搭建的第一版联调。

### 尚未完成

- `MissionController` 尚未实现比赛事件、合法状态转移和自动任务调度。
- STM32 路口事件已接入固定路线入口，但 RIGHT/LEFT 多路口链和抓取后回线仍待整车实测。
- 独立橙色和紫色取料动作尚未实机确认；对应 TaskController 接口当前返回 `NotImplemented`。
- K230D、AprilTag、巡线和 TaskController 尚未串成完整比赛流程。
- 通用 `competition/main.py` 尚未接入 live 的 BUILD、FINISH、FAULT 生命周期；固定路线入口已有 STOP/关闭路径，但本版不执行搭建动作。
- 新 competition 架构尚未进行整车实机回归。

## 5. 开发流程

项目采用人工确认下的 AI 辅助开发流程：

- GPT：辅助架构设计、问题分析和任务拆分。
- Codex：按明确任务修改代码、执行无硬件验证并同步文档。
- GitHub：保存经过检查的最新工程状态和提交历史。

```text
需求与安全边界确认
        ↓
GPT 分析 / 任务规划
        ↓
Codex 修改代码并进行无硬件检查
        ↓
更新 CURRENT_STATUS.md 和 SOFTWARE_ARCHITECTURE.md
        ↓
人工复核 git diff
        ↓
git commit
        ↓
git push origin main
```

涉及底盘、机械臂、视觉参数、串口协议或 STM32 的修改必须单独说明，并在具备安全条件时由人工安排实机验证。提交时只暂存当前任务文件，不覆盖其他工作区修改，不使用 force push。

## 6. 运行方式

树莓派当前记录的环境为 Python 3.13 和项目 `venv`。已知依赖包括：

- `pyserial`
- `numpy`
- `opencv-python-headless`
- `pupil_apriltags`

仓库暂未提供 `requirements.txt`、`pyproject.toml` 或版本锁定文件。

### 安全的软件骨架入口

```bash
cd ~/robogame
source venv/bin/activate
python3 -m competition.main
```

也可以执行：

```bash
python3 competition/main.py
```

该入口当前只初始化 `MissionController`、`TaskController`、`ArmSequence`、`Stm32Link`、`K230Link` 和 `AprilTagService`，然后打印状态。它默认不会打开串口或摄像头，也不会驱动车辆或机械臂。

### 需要真实硬件的现有入口

```bash
python3 -m competition.full_mission --dry-run
python3 -m competition.full_mission
python3 main_controller_align.py
python3 build_sequence.py
```

- `competition.full_mission --dry-run` 只打印 `RIGHT -> LEFT -> GRAB -> 固定 RIGHT -> RIGHT -> BUILD` 计划，不连接硬件；不带该参数且严格输入 `START` 后，会驱动底盘、Servo 和 Stepper，最后只打印 `BUILD_ZONE_REACHED`，不执行搭建动作。
- `main_controller_align.py` 会打开 K230D 和 STM32 串口，并可能驱动底盘与机械臂。
- `build_sequence.py` 会打开 STM32 串口，并执行固定预抬升和人工选择的机械臂动作。

这些入口只能在人工确认场地、机构和急停条件安全后单独运行。同一时刻不得运行多个占用 STM32 串口的程序。不要把 `tests/` 当作 pytest 测试套件批量执行，其中部分脚本会直接驱动硬件。

## 7. 硬件配置

| 组件 | 当前用途 | 已知接口或配置 |
| --- | --- | --- |
| Raspberry Pi 5 | 上位控制、视觉处理、比赛状态和任务决策 | Python 3.13，项目 `venv` |
| STM32F407ZGT6 | 底盘、巡线、Servo 和 Stepper 实时执行 | CH340 USB 串口，115200 baud |
| K230D / CanMV | 物块视觉协处理 | USB 串口输出 `TARGET` / `NO_TARGET`，115200 baud |
| Logitech C270 USB Camera | AprilTag 图像输入 | `/dev/video0`，请求 640×480 |
| 麦克纳姆轮底盘 | 全向底盘执行 | STM32 执行 `VEL` / `STOP` |
| 5 路 Servo 机械臂 | 抓取、搬运和搭建 | STM32 执行绝对角度命令 |
| TB6600 + Stepper | 机械臂升降/直线运动 | STM32 执行步数和 pps 命令 |
| 巡线传感器 | 黑线检测 | STM32 本地处理；型号和接口未记录在本仓库 |

正式串口配置来自 `config.py`：

```text
STM32: /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
K230D: /dev/serial/by-id/usb-Kendryte_CanMV_001000000-if00
Baudrate: 115200
```

`tests/` 和 `legacy/` 中的 `/dev/ttyUSB*`、`/dev/ttyACM*`、`/dev/serial0` 或 `/dev/ttyAMA0` 仅用于历史或专项诊断，不能替代正式设备身份。

## 8. 已知限制

- 本仓库不包含 STM32 源码、工程配置或当前烧录固件版本，无法在此复核固件参数边界和完整错误码。
- Pi 与 STM32 之间尚无经过确认的巡线完成事件；`LINE_DONE` 不属于当前已定义协议。
- K230D 当前上行协议只有 `TARGET` / `NO_TARGET` 和位置、误差、转向、像素字段，没有颜色类别字段。
- AprilTag 当前以像素中心误差和像素面积提供轻量结果，尚无相机内参、畸变校正、姿态估计或跨帧稳定策略。
- 世界模型中的 Tag、路口、线段和停车点绝对坐标尚未现场标定。
- 机械臂动作依赖已经实机验证的 Servo 角度、Stepper 步数、速度、延时和顺序；未经专项任务和实机回归不得修改。
- 固定 `startup_raise` 没有上限位器，因此不是严格意义上的 Homing。
- STM32 回复没有请求 ID，同一物理串口必须保持单一所有者和单一阶段性读取者。
- 当前缺少可批量运行的自动化测试、依赖锁定和完整比赛级回归流程。
