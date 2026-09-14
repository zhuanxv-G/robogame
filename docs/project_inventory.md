# RoboGame Raspberry Pi 工程盘点与整理报告

> 历史快照：本文记录 2026-09-05 前后的整理状态。此后项目已经完成 `ArmSequence` 重构、主控机械臂接入、AprilTag、定位和场地世界模型；当前状态请以 `CURRENT_STATUS.md`、`PI_STM32_PROTOCOL.md` 和 `SOFTWARE_ARCHITECTURE.md` 为准。本文的旧文件分类与演进记录仍可用于追溯。

盘点日期：2026-09-05

## 1. 结论

- 当前正式且已经明确有实机价值的入口只有 `main_controller_align.py` 和 `build_sequence.py`。
- `config.py` 是正式配置候选，但当前没有任何脚本导入它；所有现有脚本只导入 `serial`，多数再导入 `time`。
- `main_controller_stable.py` 与 `main_controller_v4.py` 的 SHA-256 完全相同，是确定的重复文件。
- 两个正式入口都会独立打开同一个 STM32 by-id 串口，不能同时运行。未来总状态机必须成为该串口唯一的长期所有者。
- 当前仓库没有日志、截图、运行输出、Python 缓存或 `.vscode` 配置；原 `.gitignore` 已覆盖日志和常见缓存，本轮补充了编辑器、截图和输出目录规则。
- 为避免改变已验证行为，本轮没有修改任何 Python 文件内容，只移动明确的测试/历史文件并补充文档。

## 2. 整理前文件树

```text
robogame/
├── .gitignore
├── README.md
├── baud_test.py
├── build_sequence.py
├── config.py
├── k230_read.py
├── line_test.py
├── main_controller.py
├── main_controller_align.py
├── main_controller_stable.py
├── main_controller_v2.py
├── main_controller_v3.py
├── main_controller_v4.py
├── main_controller_vel.py
├── motor_cmd_test.py
├── motor_test.py
├── servo_test.py
├── stepper_test.py
├── stm32_test.py
├── tset.py
├── uart_read_test.py
├── uart_receive_test.py
├── uart_test.py
├── vel_test.py
├── vision_to_stm32.py
└── src/
    ├── k230_receiver.py       # 空文件
    ├── main.py                # 空文件
    ├── stm32_link_test.py
    ├── stm32_sender.py
    └── uart_loopback_test.py
```

## 3. 本轮整理后文件树

```text
robogame/
├── .gitignore
├── README.md
├── build_sequence.py
├── config.py
├── main_controller_align.py
├── docs/
│   └── project_inventory.md
├── tests/
│   ├── README.md
│   ├── baud_test.py
│   ├── k230_read.py
│   ├── line_test.py
│   ├── motor_cmd_test.py
│   ├── motor_test.py
│   ├── servo_test.py
│   ├── stm32_link_test.py
│   ├── stm32_test.py
│   ├── uart_loopback_test.py
│   ├── uart_read_test.py
│   ├── uart_receive_test.py
│   ├── uart_test.py
│   └── vel_test.py
└── legacy/
    ├── README.md
    ├── main_controller.py
    ├── main_controller_stable.py
    ├── main_controller_v2.py
    ├── main_controller_v3.py
    ├── main_controller_v4.py
    ├── main_controller_vel.py
    ├── stepper_test.py
    ├── tset.py
    ├── vision_to_stm32.py
    └── src/
        ├── k230_receiver.py   # 空文件
        ├── main.py            # 空文件
        └── stm32_sender.py
```

## 4. 分类汇总

### A. 正式主程序

文件：`main_controller_align.py`

用途：读取 K230D 的 `TARGET` / `NO_TARGET` 数据，运行 `ALIGN -> APPROACH -> READY` 状态机，并向 STM32 发送 `VEL` / `STOP`。

状态：当前视觉主控。

建议：保留原位；后续升级为总状态机的视觉控制部分，不在本轮重命名为 `main.py`。

文件：`build_sequence.py`

用途：通过当前正式机械臂协议完成 HELLO、启动预抬升、三组搭建动作、Stepper 完成等待和安全 HOME。

状态：已实机验证的正式独立入口。

建议：保留原位；现阶段仍适合独立测试和回归验证。未来提取 `ArmSequence` 时必须原样保留全部角度、步数、800 pps 和 delay。

### B. 正式模块

文件：`config.py`

用途：记录 STM32 / K230D by-id 路径和 115200 波特率。

状态：正式配置候选，但当前未被任何脚本导入。

建议：保留原位；在下一阶段建立串口模块时再统一使用，避免本轮为“去重复”而改变已验证入口。

### C. 调试/测试脚本

文件：`tests/baud_test.py`

用途：在 `/dev/ttyUSB0` 上遍历多个波特率并发送 `HELLO_STM32,1`。

状态：一次性链路排障工具。

建议：保留在 `tests/`；运行前核对端口，硬件稳定后可列入删除候选。

文件：`tests/k230_read.py`

用途：从 `/dev/ttyACM0` 持续打印 K230D 原始数据。

状态：K230D 原始数据诊断工具。

建议：保留在 `tests/`；未来改用 `config.py` / `K230Link` 后再收敛。

文件：`tests/line_test.py`

用途：交互发送底盘、`LINE_START`、`LINE_STOP` 和 `STOP` 命令。

状态：巡线/底盘人工测试工具。

建议：保留在 `tests/`；未来巡线开发仍有价值。

文件：`tests/motor_cmd_test.py`

用途：自动顺序执行 `FORWARD`、`LEFT`、`RIGHT`、`BACKWARD`，每段结束发送 `STOP`。

状态：底盘动作回归工具。

建议：保留在 `tests/`；因启动即运动，文档中已标出安全风险。

文件：`tests/motor_test.py`

用途：键盘交互发送传统方向命令。

状态：早期但仍可用的底盘人工工具。

建议：保留在 `tests/`；使用 `/dev/ttyUSB0`，未来可与 `line_test.py` 合并后删除。

文件：`tests/servo_test.py`

用途：把用户输入的任意一行透传给 STM32，并打印回复。

状态：通用 STM32 命令控制台；文件名比实际用途更窄。

建议：保留在 `tests/`；未来可重命名为 `stm32_console.py`，本轮不改名以保留使用习惯。

文件：`tests/stm32_link_test.py`

用途：通过 `/dev/ttyAMA0` 发送 `PING` 并期待 `PONG`。

状态：GPIO UART 链路诊断工具，可能对应早期固件。

建议：保留在 `tests/`，但不要把它当作当前 USB by-id 正式协议测试。

文件：`tests/stm32_test.py`

用途：通过 STM32 by-id 重复发送 `HELLO_STM32,1` 并打印原始回复。

状态：当前路径的基础握手诊断工具。

建议：保留在 `tests/`；未来补充 Ctrl+C 关闭逻辑，但本轮不改变行为。

文件：`tests/uart_loopback_test.py`

用途：在 `/dev/ttyAMA0` 上发送并回读 10 条消息。

状态：Raspberry Pi GPIO UART 自环测试，不是 STM32 协议测试。

建议：保留在 `tests/` 作为底层硬件排障工具。

文件：`tests/uart_read_test.py`

用途：持续打印 `/dev/ttyUSB1` 的 RAW 和 HEX 数据。

状态：未注明对端设备的原始串口诊断工具。

建议：保留在 `tests/`，待确认 `/dev/ttyUSB1` 的历史用途后决定是否删除。

文件：`tests/uart_receive_test.py`

用途：在 `/dev/serial0` 上接收并打印文本，输出提示期望 STM32。

状态：GPIO UART 接收诊断工具。

建议：保留在 `tests/`；与当前 USB by-id 链路不同。

文件：`tests/uart_test.py`

用途：在 `/dev/ttyUSB0` 上递增发送 `HELLO_STM32,<count>`。

状态：早期 STM32 握手工具。

建议：保留在 `tests/`；与 `stm32_test.py` 功能重叠，未来可删除一个。

文件：`tests/vel_test.py`

用途：交互验证麦克纳姆底盘 `VEL` 命令、方向、旋转、看门狗和 `STOP`。

状态：有实际价值的底盘硬件测试工具。

建议：保留在 `tests/`，作为视觉主控以外的独立 VEL 回归入口。

### D. 历史废弃版本

文件：`legacy/main_controller.py`

用途：根据 `steering` 阈值发送 `LEFT` / `RIGHT` / `FORWARD` 的最早主控。

状态：旧主控，缺少完整退出清理。

建议：历史归档；当前不要运行。

文件：`legacy/main_controller_v2.py`

用途：加入转向滞回、目标确认、目标丢失/K230D 超时以及 5 Hz 持续刷新。

状态：旧主控迭代。

建议：历史归档；保留状态机演化参考。

文件：`legacy/main_controller_v3.py`

用途：使用固定方向阈值、短暂丢失容忍和持续 5 Hz 刷新。

状态：旧主控迭代。

建议：历史归档。

文件：`legacy/main_controller_v4.py`

用途：只在新视觉结果或超时时发送运动命令。

状态：旧主控；与 `main_controller_stable.py` 字节级完全重复。

建议：历史归档；确认更想保留哪个名字后删除二者之一。

文件：`legacy/main_controller_stable.py`

用途：与 v4 相同的稳定版命名副本。

状态：确定重复文件。

建议：删除候选；本轮保留以免擅自替用户选择历史名称。

文件：`legacy/main_controller_vel.py`

用途：旧的横移对准、前进速度连续控制和 READY 视觉实验。

状态：历史 VEL 方案，已被当前旋转对准策略取代。

建议：历史归档；其中通用解析/限幅思路可供后续参考，但不要把旧参数带回当前算法。

文件：`legacy/vision_to_stm32.py`

用途：把 K230D 结果转换为 `VISION_LEFT` / `VISION_RIGHT` / `VISION_CENTER` / `VISION_NO_TARGET`。

状态：使用易漂移端口与旧消息协议的早期桥接原型。

建议：历史归档。

文件：`legacy/stepper_test.py`

用途：发送 `STEPPER_SLOW` / `STEPPER_NUDGE` 点动命令。

状态：与当前正式 `STEPPER_MOVE,<steps>,<pps>` 协议不一致的旧测试。

建议：历史归档；不要直接用于当前固件。确认固件不再支持后可删除。

文件：`legacy/tset.py`

用途：重复发送 `HELLO_STM32,1`，文件名疑似 `test.py` 的拼写错误。

状态：重复排障遗留。

建议：删除候选；`tests/stm32_test.py` 已覆盖它的用途。

文件：`legacy/src/stm32_sender.py`

用途：在 `/dev/serial0` 上持续发送字符串 `test`。

状态：早期 GPIO UART 原型，不符合当前正式协议。

建议：删除候选；确认不再需要 GPIO UART 实验后删除。

### E. 临时文件/空占位

文件：`legacy/src/main.py`

用途：无内容。

状态：空的早期入口占位。

建议：删除候选。

文件：`legacy/src/k230_receiver.py`

用途：无内容。

状态：空的早期模块占位。

建议：删除候选。

### F. 文档

- `README.md`：项目入口、硬件、串口、运行与 Git 工作流。
- `tests/README.md`：逐个说明人工硬件测试和安全注意事项。
- `legacy/README.md`：说明历史脚本与删除候选。
- `docs/project_inventory.md`：本次完整盘点报告。

### G. 配置文件

- `config.py`：正式设备路径和波特率的唯一配置候选，目前未接线到入口。
- `.gitignore`：忽略 Python 缓存、日志、临时文件、虚拟环境、工具缓存、编辑器设置、截图和输出目录。

## 5. 依赖关系

当前所有非空 Python 文件只依赖第三方 `serial` 和标准库 `time`，没有本地模块之间的 import。因此移动测试/历史脚本不会破坏本地导入关系。

```text
main_controller_align.py
├── pyserial -> K230D serial（脚本直接打开）
└── pyserial -> STM32 serial（脚本直接打开）

build_sequence.py
└── pyserial -> STM32 serial（脚本导入时直接打开）

config.py
└── 当前无使用者

tests/*.py
└── 各脚本独立打开自己的串口

legacy/*.py
└── 各旧主控/原型独立打开自己的串口
```

`main_controller_align.py` 适合在后续升级为总状态机入口，但应先把串口所有权抽离；现在直接改名为 `main.py` 只会隐藏耦合，并不会完成架构整合。

## 6. 串口打开点与所有权

### 正式入口

| 文件 | 变量 | 设备 | 用途 |
| --- | --- | --- | --- |
| `main_controller_align.py` | `k230` | K230D by-id | 读取视觉数据 |
| `main_controller_align.py` | `stm32` | STM32 by-id | 发送 `VEL` / `STOP`，读取 ACK/ERR |
| `build_sequence.py` | `ser` | STM32 by-id | HELLO、Servo、Stepper、STOP；在模块导入时打开 |

### 独立测试工具

| 文件 | 变量 | 设备/路径 | 用途 |
| --- | --- | --- | --- |
| `tests/baud_test.py` | `stm` | `/dev/ttyUSB0` | STM32 波特率/握手扫描 |
| `tests/k230_read.py` | `ser` | `/dev/ttyACM0` | K230D 原始读取 |
| `tests/line_test.py` | `stm32` | STM32 by-id | 底盘/巡线命令控制台 |
| `tests/motor_cmd_test.py` | `ser` | STM32 by-id | 自动底盘动作测试 |
| `tests/motor_test.py` | `ser` | `/dev/ttyUSB0` | 底盘方向控制台 |
| `tests/servo_test.py` | `ser` | STM32 by-id | 通用命令透传控制台 |
| `tests/stm32_link_test.py` | `ser` | `/dev/ttyAMA0` | GPIO UART PING/PONG |
| `tests/stm32_test.py` | `stm` | STM32 by-id | HELLO 握手循环 |
| `tests/uart_loopback_test.py` | `ser` | `/dev/ttyAMA0` | Raspberry Pi UART 自环，不应接 STM32 |
| `tests/uart_read_test.py` | `ser` | `/dev/ttyUSB1` | 未注明设备的原始读取 |
| `tests/uart_receive_test.py` | `ser` | `/dev/serial0` | GPIO UART 接收，提示期望 STM32 |
| `tests/uart_test.py` | `ser` | `/dev/ttyUSB0` | 早期 STM32 HELLO 循环 |
| `tests/vel_test.py` | `stm32` | STM32 by-id | VEL/看门狗/STOP 测试 |

### 历史脚本

| 文件 | 变量 | 设备 | 用途 |
| --- | --- | --- | --- |
| `legacy/main_controller.py` | `k230`, `stm32` | 两个正式 by-id | 旧视觉主控 |
| `legacy/main_controller_v2.py` | `k230`, `stm32` | 两个正式 by-id | 旧视觉主控 |
| `legacy/main_controller_v3.py` | `k230`, `stm32` | 两个正式 by-id | 旧视觉主控 |
| `legacy/main_controller_v4.py` | `k230`, `stm32` | 两个正式 by-id | 旧视觉主控 |
| `legacy/main_controller_stable.py` | `k230`, `stm32` | 两个正式 by-id | v4 重复副本 |
| `legacy/main_controller_vel.py` | `k230`, `stm32` | 两个正式 by-id | 旧 VEL 视觉主控 |
| `legacy/vision_to_stm32.py` | `vision_serial`, `stm32_serial` | `/dev/ttyACM1`, `/dev/ttyUSB0` | 旧视觉消息桥接 |
| `legacy/stepper_test.py` | `ser` | STM32 by-id | 旧 Stepper 协议测试 |
| `legacy/tset.py` | `ser` | STM32 by-id | 重复 HELLO 测试 |
| `legacy/src/stm32_sender.py` | `ser`（由 `uart` 接收返回值） | `/dev/serial0` | 早期字符串发送原型 |

任何两个指向同一个 STM32 的上述程序都可能争用串口；第二个进程通常会打开失败，也可能因重连、服务代理或不同别名指向同一物理设备而产生交错命令。正式架构必须遵守：

```text
main.py（唯一 STM32 串口所有者）
├── Stm32Link（唯一 serial.Serial 实例）
├── VisionController（借用 Stm32Link，不打开串口）
├── ArmSequence（借用 Stm32Link，不打开串口）
└── Line/AprilTag 控制器（借用 Stm32Link，不打开串口）
```

测试工具只在主控制器停止后独占串口运行。

## 7. 重复代码清单

1. 设备常量：STM32/K230D by-id 和 `115200` 在所有主控、`build_sequence.py` 及多个测试中重复；`config.py` 已有相同值但无人导入。
2. 串口生命周期：各脚本重复 `serial.Serial(...)`、初始化等待、清输入缓冲、finally 关闭等代码，且清理完整度不一致。
3. `send_cmd` / `send` / `send_once` / `send_command` / `tx`：本质都是“命令加换行、编码、写入、可选 flush/打印/读回复”。
4. `parse_k230`：`main_controller.py`、v2、v3、v4/stable、`main_controller_vel.py`、`main_controller_align.py` 重复解析 `TARGET` / `NO_TARGET`；字段校验和 `pixels` 支持略有差异。
5. `STOP`：主控、底盘测试、VEL 测试和机械臂流程各自实现，ACK 等待策略不一致。
6. HELLO：`build_sequence.py` 有完整 `hello()` + `wait_for()`；`stm32_test.py`、`uart_test.py`、`tset.py`、`baud_test.py` 内联实现相似握手。
7. 控制循环：所有旧主控共享“读取最新 K230 数据、目标丢失超时、K230D 超时、发送 STOP、关闭两路串口”的骨架。
8. 确定重复文件：`legacy/main_controller_v4.py` 与 `legacy/main_controller_stable.py` 字节级完全相同。

以下函数并没有多个“当前正式实现”，不应为了抽象而强行合并：

- `wait_for()`、`servo_set()`、当前正式 `stepper_move()` 只在 `build_sequence.py` 有完整实现。
- `legacy/stepper_test.py` 的 `send()` 面向旧协议，不能当作正式 `stepper_move()` 的等价实现。
- `read_k230()` / `parse_target()` 没有以这些名称存在；等价逻辑主要叫 `parse_k230()` 或直接 `readline()`。

后续最值得提取的顺序：

1. `hardware/stm32_link.py`：拥有唯一串口并提供 `send`、`wait_for`、`hello`、`stop`、`servo_set`、`stepper_move`。
2. `hardware/k230_link.py`：负责 K230D 串口读取与 `TARGET` / `NO_TARGET` 解析。
3. `controllers/arm_sequence.py`：接受已有 `Stm32Link`，封装 `ArmSequence`，原样迁移动作参数和 delay。

## 8. 两个正式入口的专项分析

### `build_sequence.py`

- 当前在文件顶层打开 STM32 串口；作为独立入口可工作，但直接 import 会抢占串口，不适合作为未来库模块。
- `tx()` / `wait_for()` / `hello()` / `stop()` / `servo_get()` / `servo_set()` / `servo_sweep()` / `stepper_move()` 组成清晰的低层通信层。
- `startup_raise()`、`group1()`、`group2()`、`group3()`、`run_all()` 组成动作层；重复主要是顺序动作表达，不应在没有回归硬件测试时压缩成数据表。
- 仍适合作为独立测试入口。未来把动作层迁入 `ArmSequence`，`build_sequence.py` 只负责创建连接、菜单和安全退出。
- 本轮未改动任何 Servo 角度、Stepper 步数、800 pps、启动预抬升、安全 HOME 或 delay。

### `main_controller_align.py`

- 顶层依次打开 K230D 和 STM32，并清空输入缓冲。
- 状态由字符串常量和主循环内分支维护：`STOP` 首次见目标后进入 `ALIGN` 或 `APPROACH`，误差阈值控制二者切换，连续满足条件进入 `READY`。
- 启动前发送一次 `STOP`；READY 首次进入时发送 `STOP`；目标丢失和 K230D 超时时发送 `STOP` 并清状态；Ctrl+C 后 finally 再发 `STOP` 并关闭两路串口。
- 后续 `ARM_BUILD` 最合适放在顶层任务状态机中：READY 确认且底盘完成 STOP 后，只触发一次 `ARM_BUILD` 转移。不要在当前每帧视觉分支内直接调用 `group1()` / `run_all()`。
- 它适合作为未来 `main.py` 的起点，但必须先让 `main.py` 持有唯一 STM32 连接，再把视觉状态机和机械臂动作变成使用该连接的组件。
- 本轮未改动视觉算法或任何阈值、速度、EMA、超时参数。

## 9. 文件处置建议

### 保留原位

- `main_controller_align.py`
- `build_sequence.py`
- `config.py`
- `README.md`
- `.gitignore`
- `docs/`

### 已移入 `tests/`

- `baud_test.py`
- `k230_read.py`
- `line_test.py`
- `motor_cmd_test.py`
- `motor_test.py`
- `servo_test.py`
- `stm32_link_test.py`
- `stm32_test.py`
- `uart_loopback_test.py`
- `uart_read_test.py`
- `uart_receive_test.py`
- `uart_test.py`
- `vel_test.py`

### 已移入 `legacy/`

- 所有旧 `main_controller*`（当前 `main_controller_align.py` 除外）
- `main_controller_vel.py`
- `vision_to_stm32.py`
- 使用旧协议的 `stepper_test.py`
- 拼写异常且重复的 `tset.py`
- 原 `src/` 中的早期发送器和两个空占位文件

### 删除候选（本轮均未删除）

- `legacy/main_controller_stable.py` 或 `legacy/main_controller_v4.py`：完全重复，选择保留一个。
- `legacy/tset.py`：被 `tests/stm32_test.py` 覆盖。
- `legacy/src/main.py`、`legacy/src/k230_receiver.py`：空文件。
- `legacy/src/stm32_sender.py`：旧 GPIO UART / 非正式协议原型。
- `tests/uart_test.py`：确认 `tests/stm32_test.py` 足够后可删。
- `tests/baud_test.py`：波特率固定并长期稳定后可删。
- `tests/uart_read_test.py`：确认 `/dev/ttyUSB1` 不再对应任何设备后可删。

## 10. 安全清理建议

本轮只报告以下问题，没有修改运行逻辑：

- `main_controller_align.py` 的主 `try/finally` 在两路串口打开、缓冲清理和 3 秒启动等待之后才开始。若打开 STM32 失败或在启动等待期间中断，当前 finally 不会执行；操作系统会在进程退出时回收句柄，但未来仍应把完整生命周期包进统一上下文。
- `main_controller_align.py` 的 READY 是锁存状态：进入 READY 并停车后，不会自行回到 ALIGN/APPROACH。这与“等待上层触发机械臂”相符；未来应由总状态机显式完成 `READY -> ARM_BUILD`，不要在视觉帧循环里隐式复位。
- `build_sequence.py` 在模块导入时就打开 STM32。它作为独立脚本已可用，但未来提取 `ArmSequence` 前必须改为由主入口注入连接，否则仅 import 就会抢占串口。
- `tests/motor_cmd_test.py` 启动后立即让底盘依次运动，虽每段会发 STOP，但没有统一 finally 关闭；运行环境必须人工清场。
- `tests/servo_test.py` 退出时只关闭串口，不主动发送 STOP；它又能透传任意命令，应该视为人工调试终端。
- `tests/k230_read.py`、`tests/stm32_test.py`、`tests/uart_read_test.py` 是无 finally 的无限循环；它们只做读取/握手，但未来仍宜补充一致的 Ctrl+C 关闭逻辑。
- `legacy/main_controller.py` 没有 Ctrl+C / finally STOP；其余 legacy 文件也不应被当作安全入口。
- `tests/` 中脚本多数在 import 时立即打开串口或进入循环。不要批量导入，也不要用 `pytest` 自动收集运行；只按文件显式执行。

清理或合并测试脚本前，应先在树莓派执行 `readlink -f` / `ls -l /dev/serial/by-id/` 核对每个历史设备名的真实指向，避免把同一物理 STM32 的多个别名误认为不同设备。

## 11. 推荐最终结构

当前只有两个正式运行脚本，暂时保持根目录入口最安全。完成下一阶段模块化后再演进为：

```text
robogame/
├── main.py
├── config.py
├── controllers/
│   ├── vision_controller.py
│   └── arm_sequence.py
├── hardware/
│   ├── stm32_link.py
│   └── k230_link.py
├── tests/
├── legacy/
├── docs/
├── README.md
└── .gitignore
```

不要为了这个树形一次性拆分代码。每一步都应保持 `build_sequence.py` 可单独做机械臂回归，并在 Raspberry Pi + STM32 实机验证后再继续。

## 12. 下一阶段顺序（本轮不实施）

1. 整理 `arm_sequence`：先抽 STM32 连接所有权，再原样封装已验证动作。
2. 整合视觉主控：让总入口持有唯一 STM32 连接，视觉控制与 `ARM_BUILD` 通过显式状态转移衔接。
3. 再加入 AprilTag、巡线和比赛总状态机，避免在串口所有权尚未统一时继续叠加入口。
