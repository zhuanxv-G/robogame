# RoboGame Raspberry Pi 控制工程

本仓库保存 RoboGame 的 Raspberry Pi 5 端 Python 代码。Raspberry Pi 读取 K230D / CanMV 的视觉结果，并通过串口向 STM32F407ZGT6 发送底盘、Servo 机械臂和 TB6600 Stepper 控制命令。

当前整理只调整了文档和脚本位置，没有修改视觉算法、机械动作参数、STM32 协议或串口路径。

## 硬件分工

- Raspberry Pi 5：上位控制、视觉结果处理和任务状态机。
- K230D / CanMV：识别目标并通过串口输出视觉数据。
- STM32F407ZGT6：执行底盘、Servo 和 Stepper 控制。
- 麦克纳姆底盘：执行 `VEL` 等运动命令。
- 5 路 Servo + TB6600 Stepper：执行固定搭建流程。

## 串口配置

正式 USB 连接使用稳定的 by-id 路径，波特率均为 `115200`：

```text
STM32: /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
K230D: /dev/serial/by-id/usb-Kendryte_CanMV_001000000-if00
```

路径也记录在 `config.py` 中，但为避免本轮改变已验证脚本的运行行为，现有入口尚未改为导入该配置。`tests/` 和 `legacy/` 中仍有 `/dev/ttyUSB*`、`/dev/ttyACM*`、`/dev/serial0`、`/dev/ttyAMA0` 等历史或硬件诊断路径，运行前必须检查。

## 当前正式入口

### `main_controller_align.py`

当前视觉主控。它直接打开 K230D 和 STM32 串口，运行：

```text
ALIGN -> APPROACH -> READY
```

该脚本包含目标丢失、K230D 超时、READY 停车、Ctrl+C 停车以及最终关闭串口的安全处理。它尚未接入机械臂动作。

### `build_sequence.py`

已经实机跑通的独立机械臂搭建入口。它直接打开 STM32 串口，执行握手、启动预抬升、三组动作、Stepper 完成等待、安全 HOME，以及 Ctrl+C / 异常时发送 `STOP`。

注意：启动预抬升是在没有上限位器的情况下固定向上移动，并不是真正的 Homing。

## 运行

安装 Python 3 和 `pyserial` 后，在仓库根目录执行：

```bash
python3 main_controller_align.py
python3 build_sequence.py
```

硬件诊断脚本按需单独执行，例如：

```bash
python3 tests/stm32_test.py
python3 tests/vel_test.py
```

同一时刻只能运行一个会打开 STM32 串口的程序。尤其不要同时运行 `main_controller_align.py`、`build_sequence.py` 或任何 STM32 测试脚本。

## 目录说明

```text
robogame/
├── main_controller_align.py   # 当前视觉主控
├── build_sequence.py          # 已验证机械臂搭建入口
├── config.py                  # 统一配置候选，当前尚未被入口导入
├── tests/                     # 独立人工硬件诊断脚本，不是自动化测试套件
├── legacy/                    # 旧主控、旧协议和早期原型；日常不要运行
├── docs/                      # 盘点、依赖和整理报告
├── README.md
└── .gitignore
```

测试脚本说明见 `tests/README.md`，历史脚本说明见 `legacy/README.md`，完整文件分类、重复代码和串口所有权分析见 `docs/project_inventory.md`。

## Git 工作流

开发机（Windows + VS Code / Codex）：

```bash
git status
git add <确认过的文件>
git commit -m "说明本次改动"
git push
```

树莓派实机：

```bash
git pull
python3 <入口脚本>
```

提交前先检查 `git diff --check` 和 `git diff --stat`。不要提交日志、缓存、截图或临时输出；不要用 `reset --hard`、force push 或改写 Git 历史来做日常同步。
