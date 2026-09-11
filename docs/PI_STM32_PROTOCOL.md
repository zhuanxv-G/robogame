# Pi-STM32 通信协议

> 文档状态：Pi 端协议基线，2026-09-11 与当前外部 STM32 源码同步。
> 证据来源：Pi 端 `main_controller_align.py`、`arm_sequence.py`、`build_sequence.py`、`tests/`，以及外部 STM32 工程 `D:\Code\V1.3_vision\V1.3`。
> 限制：STM32 工程不属于本 Git 仓库且没有可用 Git 历史；新增巡线诊断已经 Keil 编译，但尚未烧录和实车验证，不能据此认定现场固件已经支持。

## 1. 链路参数

| 参数 | 当前值 |
| --- | --- |
| 物理链路 | Raspberry Pi USB ↔ CH340 ↔ STM32 |
| Linux 正式设备路径 | `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` |
| Baudrate | `115200` |
| 数据位/校验/停止位 | pyserial 默认值：8 data bits、no parity、1 stop bit（8N1） |
| 流控 | Pi 端未启用软件或硬件流控 |
| 消息编码 | 当前命令均为 ASCII 字符；Python 使用 `.encode()` |
| 帧结束符 | LF，即 `\n` |
| Pi 读取方式 | `readline()`，接收后 `.strip()` |
| 常用读超时 | 主控 0.2 s；独立机械臂入口 0.1 s |

正式程序应使用 by-id 路径。`tests/` 和 `legacy/` 中的 `/dev/ttyUSB0`、`/dev/ttyAMA0`、`/dev/serial0` 等仅用于历史或专项诊断，不能替代正式设备身份。

## 2. 帧格式

Pi 发送一条以换行结尾的文本命令：

```text
COMMAND[,ARG1,ARG2,...]\n
```

例如：

```text
VEL,0.00,0.20,0.00\n
SERVO_SET,4,135\n
STOP\n
```

STM32 回复同样按行读取。当前 Pi 代码通过逗号字段和固定前缀匹配回复；没有校验和、消息长度、序列号或二进制转义层。发送端不得在字段中加入逗号或换行。

## 3. 当前正式命令

### 3.1 握手

```text
Pi -> STM32: HELLO_STM32,1
STM32 -> Pi: STM32_OK,1
```

- 当前唯一使用的协议版本是 `1`。
- `build_sequence.py` 启动时要求 2 s 内收到 `STM32_OK,1`。
- `main_controller_align.py` 当前没有主动执行握手，这是已知待统一项。

### 3.2 全局停车

```text
Pi -> STM32: STOP
STM32 -> Pi: ACK,STOP
```

- `STOP` 是底盘/系统安全路径使用的全局停车命令。
- 进入机械臂阶段前，当前主控必须等待 `ACK,STOP`，确认停车后才允许状态转移。
- 普通清理路径可发送 STOP 但容忍 ACK 超时；这不代表固件可以省略 ACK。

### 3.3 底盘速度

```text
Pi -> STM32: VEL,<x>,<y>,<omega>
```

坐标与单位按当前 Pi 主控注释定义：

| 字段 | 正方向 | 负方向 | 单位 |
| --- | --- | --- | --- |
| `x` | 向右横移 | 向左横移 | m/s |
| `y` | 向前 | 向后 | m/s |
| `omega` | 左转 | 右转 | rad/s |

当前正式主控用两位小数发送，例如：

```text
VEL,0.00,0.00,0.45
VEL,0.00,0.20,0.00
```

当前 STM32 源码对合法速度回复 `ACK,VEL`；非法格式或越界值回复 `ERR,VEL`。现役 Pi 主循环只清空一般 ACK 并输出 `ERR`，没有逐条校验 ACK。

REMOTE 模式的当前 STM32 看门狗常量为 1000 ms；连续运动需要持续刷新有效运动命令。该看门狗只覆盖 REMOTE，不覆盖 LINE_FOLLOW。

### 3.4 读取 Servo 位置

```text
Pi -> STM32: SERVO_GET,<index>
STM32 -> Pi: SERVO_POS,<index>,<angle>
```

- 当前机械臂使用索引 0-4。
- Pi 把回复的第三个字段解析为整数角度。
- 当前等待超时为 2 s。

### 3.5 设置 Servo 角度

```text
Pi -> STM32: SERVO_SET,<index>,<angle>
STM32 -> Pi: ACK,SERVO_SET,<index>,<angle>
```

- Pi 要求 ACK 中的索引和角度与请求完全一致。
- 已验证动作序列使用索引 0-4，代码中出现的角度范围为 0-180°。
- 0-180°只是当前代码实际使用范围，不等同于已证明的 STM32 参数校验边界；每个关节的机械安全范围也不能由协议范围代替。

### 3.6 Stepper 指定步数运动

```text
Pi -> STM32: STEPPER_MOVE,<steps>,<pps>
STM32 -> Pi: ACK,STEPPER_MOVE,<steps>,<pps>
STM32 -> Pi: STEPPER_MOVE_DONE,<steps>
```

- `steps` 为有符号整数；正负对应相反方向，具体机械方向取决于安装和已验证动作约定。
- `pps` 为每秒脉冲数，当前动作固定使用 800。
- Pi 必须先收到 ACK，再等待 MOVE_DONE，不能只根据 sleep 推断运动结束。
- MOVE_DONE 超时按 `abs(steps) / pps * 2 + 8 s` 计算。
- 当前动作中观察到的步数为 -10500 至 +10500，但这只是现有序列用值，不是固件接受范围。
- startup raise 的 `-1500 steps @ 800 pps` 是固定预抬升，不是限位 Homing。

### 3.7 巡线启动、停止和只读诊断

```text
Pi -> STM32: LINE_START
STM32 -> Pi: ACK,LINE_START
              或 ERR,LINE_SENSOR_IO
              或 ERR,LINE_START_FAILED

Pi -> STM32: LINE_STOP
STM32 -> Pi: ACK,LINE_STOP

Pi -> STM32: LINE_STATUS
STM32 -> Pi: LINE_STATUS,<sensor_ok>,<s1>,<s2>,<s3>,<s4>,<active>
```

- `LINE_START` 先停止旧 LineNav、紧急停车并清除 REMOTE 运动状态，再做一次不会驱动车辆的传感器预检。预检失败时保持 STOP 并回复 `ERR,LINE_SENSOR_IO`；成功后切换到 `LINE_FOLLOW`，以固定 `0.20 m/s` 启动非阻塞巡线并回复 ACK。
- ACK 后由 STM32 主循环每 10 ms 节流调用 `LineNav.Update()`。运行期软件 I2C 失败时立即 `Active=0` 并紧急停车，随后一次性上报 `ERR,LINE_SENSOR_IO`。
- `LINE_STOP` 停止 LineNav、紧急停车并回复 ACK。
- `LINE_STATUS` 每次执行一次软件 I2C 读取，只报告状态；绝不切换 ControlMode、启停 LineNav、写底盘速度或操作 Servo/Stepper。
- `sensor_ok` 表示本次读取是否成功；`s1` 至 `s4` 为从左到右四路逻辑值（`1=黑线`、`0=白色`）；`active` 表示 LineNav 当前是否运行。若 `sensor_ok=0`，四路值是最近一次成功读取的缓存值，不能当作本次有效样本。
- 当前没有 `LINE_DONE`、到站或路口完成回复。

### 3.8 错误回复

```text
STM32 -> Pi: ERR,<details...>
```

`ArmSequence.wait_for()` 对任意 `ERR,` 前缀立即抛出异常。`main_controller_align.py` 在普通运动阶段只打印 `ERR...`。巡线相关已确认错误为 `ERR,LINE_START_FAILED` 和 `ERR,LINE_SENSOR_IO`；其他命令仍按各自格式、范围、不可用或忙状态返回 `ERR,...`。

## 4. 命令与回复总表

| Pi 命令 | 参数 | Pi 当前期待的回复 | 当前用途 |
| --- | --- | --- | --- |
| `HELLO_STM32,1` | 协议版本 | `STM32_OK,1` | 启动握手 |
| `STOP` | 无 | `ACK,STOP` | 全局停车、阶段切换、安全退出 |
| `VEL,x,y,omega` | 3 个浮点数 | `ACK,VEL` | 麦克纳姆底盘速度 |
| `SERVO_GET,index` | 整数索引 | `SERVO_POS,index,angle` | 获取 Servo 当前角度 |
| `SERVO_SET,index,angle` | 整数索引、角度 | 原参数回显 ACK | 设置 Servo 绝对角度 |
| `STEPPER_MOVE,steps,pps` | 有符号步数、正 pps | 原参数回显 ACK，随后 DONE | Stepper 相对运动 |
| `LINE_START` | 无 | `ACK,LINE_START` 或巡线错误 | 以 0.20 m/s 启动连续巡线 |
| `LINE_STOP` | 无 | `ACK,LINE_STOP` | 停止巡线并紧急停车 |
| `LINE_STATUS` | 无 | `LINE_STATUS,ok,s1,s2,s3,s4,active` | 只读巡线传感器/运行状态 |

## 5. 参数边界

以下边界来自当前外部 STM32 源码；机械安全范围仍可能小于协议允许范围：

| 参数 | 当前代码实际使用/测试值 | 固件边界状态 |
| --- | --- | --- |
| VEL `x` | 正式主控固定 0；测试脚本用到 ±0.20 | `-0.30 .. +0.30 m/s` |
| VEL `y` | 正式主控 0 或 +0.20；测试用到 -0.20 | `-0.30 .. +0.30 m/s` |
| VEL `omega` | 正式主控 ±0.45；测试用到 ±0.80 | `-1.50 .. +1.50 rad/s` |
| Servo index | 0-4 | `0 .. SERVO_NUM-1`，当前 `SERVO_NUM=5` |
| Servo angle | 现有动作 0-180° | 协议接受整数 `0 .. 180`；机械安全范围需另行遵守 |
| Stepper steps | 现有动作 -10500 至 +10500 | `-12000 .. +12000`，拒绝 0 |
| Stepper pps | 现有动作固定 800 | `100 .. 800` |

这些是解析器上限，不是扩大现场动作参数的授权；已验证速度、角度、步数和 pps 保持不变。

## 6. 已确认的人工诊断命令

以下命令存在于当前 STM32 源码，但不被现役比赛主控使用。它们是否存在于现场已烧录的旧固件，必须以实测回复为准：

```text
FORWARD
BACKWARD
LEFT
RIGHT
LINE_START
LINE_STOP
LINE_STATUS
```

- `FORWARD/BACKWARD/LEFT/RIGHT` 会切换到 REMOTE 并驱动底盘；仅在清空场地并准备急停后人工测试。
- `LINE_START` 会驱动底盘；`LINE_STOP` 会停车。
- 只有 `LINE_STATUS` 被设计为绝对不驱动车辆的巡线诊断命令。
- 当前源码还支持 `SERVO_NUDGE`、`STEPPER_NUDGE` 和 `STEPPER_SLOW` 人工标定命令；它们会操作执行器，不属于本次巡线诊断范围。

以下内容只见于 `legacy/` 或 GPIO UART 专项测试，不应发送给当前正式 USB 协议：

```text
VISION_LEFT / VISION_RIGHT / VISION_CENTER / VISION_NO_TARGET
PING / PONG
```

## 7. Python 发送与等待规则

最小发送形式：

```python
stm32.write((command + "\n").encode())
```

机械臂请求采用严格的前缀等待：

```text
发送命令
  -> 逐行读取
  -> 收到 ERR 立即失败
  -> 收到期望前缀则成功
  -> 超时则失败
```

底盘 VEL 属于连续控制流，当前主控不会逐条阻塞等待 ACK，而是在每轮清空 STM32 回复，只显示 ERR。Servo 和 Stepper 属于事务式控制流，必须由 `ArmSequence` 独占读取 ACK/DONE。

## 8. 串口所有权和状态切换

同一物理 STM32 串口同一时刻只能有一个进程、一个所有者：

```text
main_controller_align.py（唯一 serial.Serial）
├── 底盘控制：发送 VEL/STOP，非阻塞清理回复
└── ArmSequence：复用同一连接，事务式读取 ACK/DONE
```

进入 `ARM_BUILD` 前必须：

1. 停止发送 VEL。
2. 发送 STOP。
3. 收到 `ACK,STOP`。
4. 停止普通 `drain_stm32_rx()`。
5. 由 `ArmSequence` 独占回复读取，直到搭建结束。

不要同时运行 `main_controller_align.py`、`build_sequence.py` 或任何打开 STM32 的测试脚本，否则会发生端口占用或回复被错误消费者读走。

## 9. K230D → Pi 视觉串口（独立链路）

这不是 Pi ↔ STM32 命令协议，但它是当前主控的上游输入，记录在此以防混淆。

| 参数 | 当前值 |
| --- | --- |
| 设备 | `/dev/serial/by-id/usb-Kendryte_CanMV_001000000-if00` |
| 波特率 | 115200 |
| 无目标 | `NO_TARGET` |
| 有目标 | `TARGET,cx,cy,error_x,steering,pixels` |

当前解析器允许行首存在调试前缀 `[UART TX] `，然后把六个 TARGET 字段解析为整数。字段不足、类型错误或未知消息返回无效结果，不应驱动底盘。

## 10. 待现场确认

1. 将当前 Keil 生成的 HEX 烧录后，确认实机固件确实支持 `LINE_STATUS` 和 `ERR,LINE_SENSOR_IO`。
2. 分别在白地、普通黑线和四路全黑条件下记录 `LINE_STATUS`，确认接线顺序与 `1=黑线` 极性。
3. 若 `sensor_ok=0`，继续检查 PC4/PC5 接线、供电、上拉、设备地址及三个 ACK 阶段；不要绕过传感器强制启动车辆。
4. 建立外部 STM32 源码、Keil 产物、烧录时间和现场结果的版本对应关系。
5. 巡线到站、路口或完成事件仍未定义；本次没有新增 LINE_DONE 状态机。

完成上述验证前，不要让比赛状态机依赖新增巡线诊断，也不要扩大任何运动参数。
