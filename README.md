# RoboGame

RoboGame 是一个机器人项目，各模块分工如下：

- Raspberry Pi 5 负责上位机控制。
- STM32F407ZGT6 负责底盘与机械臂控制。
- K230D / CanMV 负责视觉。

当前开发方式：在 Windows 上使用 VS Code 和 Codex 修改代码，通过 GitHub 同步；Raspberry Pi 执行 `git pull` 后进行实机测试。

这是一行用于测试 Git 同步的文字。

Git 用于版本管理与 Windows / Raspberry Pi 之间的代码同步。
