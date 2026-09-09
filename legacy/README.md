# 历史脚本

本目录保留旧主控、旧协议和早期原型，便于追溯调试过程。它们不是当前正式入口，运行前必须核对串口、协议和运动行为。

| 文件 | 历史用途 | 处理建议 |
| --- | --- | --- |
| `main_controller.py` | 最早的视觉方向控制 | 保留作历史参考 |
| `main_controller_v2.py` | 加入转向滞回、目标确认和持续刷新 | 保留作历史参考 |
| `main_controller_v3.py` | 恢复阈值控制并加入超时 | 保留作历史参考 |
| `main_controller_v4.py` | 仅在新视觉结果/超时时发命令 | 与 `main_controller_stable.py` 完全重复，二选一后可删一个 |
| `main_controller_stable.py` | 标记为 stable 的 v4 副本 | 与 `main_controller_v4.py` 完全重复，二选一后可删一个 |
| `main_controller_vel.py` | 横移 + 前进的旧 VEL 视觉实验 | 参数路线已被旋转对准主控替代，保留作算法历史 |
| `vision_to_stm32.py` | 早期 `VISION_*` 消息桥接 | 使用易漂移端口和旧协议，保留作历史参考 |
| `stepper_test.py` | 早期 Stepper 点动测试 | 使用非当前正式协议，不应直接用于现固件 |
| `tset.py` | 拼写错误的重复 STM32 握手脚本 | 被 `tests/stm32_test.py` 覆盖，可作为删除候选 |
| `src/stm32_sender.py` | GPIO UART 发送 `test` 的早期原型 | 非当前正式协议，可作为删除候选 |
| `src/main.py` | 空的早期入口占位 | 可作为删除候选 |
| `src/k230_receiver.py` | 空的 K230 接收占位 | 可作为删除候选 |

本轮没有删除任何历史文件。
