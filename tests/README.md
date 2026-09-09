# 独立硬件诊断脚本

这里保存人工执行的串口和运动测试工具，不是可由 `pytest` 批量运行的自动化测试套件。多数脚本在导入时就会打开串口并进入循环，因此只能按文件逐个执行。

## 安全规则

1. 同一时刻只运行一个占用 STM32 串口的程序。
2. 运动测试前抬起底盘或清空运动区域，并准备人工急停。
3. 先检查脚本中的设备路径和协议是否符合当前接线、固件。
4. 测试完成后确认发送 `STOP` 并关闭串口；没有完善清理逻辑的旧工具要格外谨慎。

## 文件用途

| 文件 | 当前用途 | 路径/协议注意事项 |
| --- | --- | --- |
| `baud_test.py` | 扫描 STM32 握手波特率 | 使用易漂移的 `/dev/ttyUSB0` |
| `k230_read.py` | 打印 K230D 原始串口数据 | 使用 `/dev/ttyACM0`，无退出清理 |
| `line_test.py` | 手动发送底盘和巡线命令 | 使用 STM32 by-id；退出发送 `STOP` |
| `motor_cmd_test.py` | 顺序执行四种底盘运动命令 | 使用 STM32 by-id；运行后立即运动 |
| `motor_test.py` | 键盘交互式底盘测试 | 使用易漂移的 `/dev/ttyUSB0` |
| `servo_test.py` | STM32 通用命令行串口控制台 | 名称虽为 Servo，实际会透传任意命令 |
| `stm32_link_test.py` | GPIO UART 的 `PING` / `PONG` 链路测试 | 使用 `/dev/ttyAMA0`，协议可能不是当前 USB 正式协议 |
| `stm32_test.py` | 重复发送 `HELLO_STM32,1` 的握手诊断 | 使用 STM32 by-id，无正常退出清理 |
| `uart_loopback_test.py` | Raspberry Pi GPIO UART 自环测试 | `/dev/ttyAMA0`，需物理短接 TX/RX，不连接 STM32 |
| `uart_read_test.py` | 原始读取第二个 USB 串口 | 使用 `/dev/ttyUSB1`，设备角色未写明 |
| `uart_receive_test.py` | GPIO UART 接收诊断 | 使用 `/dev/serial0`，注释期望 STM32 |
| `uart_test.py` | 早期 STM32 握手诊断 | 使用易漂移的 `/dev/ttyUSB0` |
| `vel_test.py` | 人工验证 `VEL`、看门狗和 `STOP` | 使用 STM32 by-id，仍有实际硬件测试价值 |

`legacy/stepper_test.py` 没有放在这里，因为它使用 `STEPPER_SLOW` / `STEPPER_NUDGE`，与当前正式 `STEPPER_MOVE,<steps>,<pps>` 协议不一致。
