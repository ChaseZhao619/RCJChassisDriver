# RCJappforchase

`RCJappforchase` 是 Ubuntu/Qt 上位机程序，用于地图可视化、路径规划、串口控制和外设调试。它通过串口协议控制 STM32 固件，最终把路径拆成 `cmd_dis`、`cmd_turn` 等命令发送给 `RCJchassisdiver`。

## 版权与许可

> 重要：本目录代码随仓库按 GNU General Public License v3.0 only（GPL-3.0-only）授权。分发修改版或基于本目录代码的派生作品时，需要遵守 GPL-3.0 的源代码公开和同许可证分发要求。

Copyright © 2026 ChaseZhao619 and contributors.

完整条款见 [`../LICENSE`](../LICENSE)。Qt 及系统依赖遵循其各自许可证，发布二进制程序时应同时检查相关依赖的分发要求。

## 安全与免责声明

> 危险：上位机会向 STM32 发送运动和执行器命令。错误地图比例、错误初始位姿、错误路径或误触按钮都可能导致机器人运动异常。

使用 GUI 发送路径前，应确认场地安全、急停可用、固件状态正常。本项目不承担因误操作、地图配置错误、路径规划错误或串口通信异常造成的风险。

## 贡献与维护说明

> 重要：上位机改动必须和固件协议保持一致。新增按钮、快捷操作或自动发送命令的功能，都可能直接驱动硬件。

提交前建议说明：

- 依赖的固件命令版本或协议变化。
- 地图坐标、单位、路径分段或避障策略是否变化。
- JSON 配置文件是否兼容旧版本。
- 是否新增按钮、快捷操作或可能驱动硬件的入口。
- 是否测试过无串口、串口断开、CRC 错误和固件返回 `busy` 的情况。

维护规则：

- 修改 STM32 串口协议后，同步更新上位机命令生成、解析逻辑和本文档。
- 修改地图坐标或路径分段策略后，必须验证生成的 `cmd_dis` 与实际场地单位一致。
- 新增外设控制按钮时，先确认固件已有对应命令和错误回复。
- 配置 JSON 格式变化时，应考虑旧配置文件兼容或给出迁移说明。

## 你应该先知道

- 上位机运行在 Ubuntu 本机，不运行在 STM32 上。
- 串口参数固定为 `115200 8N1`。
- 上位机通过 USART6 协议和 STM32 通信，协议说明见 [`../RCJchassisdiver/App/README.md`](../RCJchassisdiver/App/README.md)。
- 地图规划结果会被分段转换为底盘运动命令。
- 首次联调前，应先确认固件能单独执行 `cmd_dis` 和 `cmd_turn`。

## 快速开始

### 1. 安装依赖

```bash
sudo apt install cmake ninja-build qt6-base-dev qt6-serialport-dev
```

### 2. 构建

从仓库根目录执行：

```bash
cmake --preset host-debug
cmake --build --preset host-debug
```

可执行文件：

```text
build/host/debug/RCJappforchase/rcj_appforchase
```

### 3. 启动 GUI

```bash
./build/host/debug/RCJappforchase/rcj_appforchase
```

### 4. 生成一条带 CRC 的测试帧

保留 `--frame` 入口用于协议测试：

```bash
./build/host/debug/RCJappforchase/rcj_appforchase --frame cmd_dis 10 0
```

输出可直接用于串口发送。

## 首次使用流程

1. 启动 GUI。
2. 选择 STM32 对应串口，例如 `/dev/ttyUSB*` 或 `/dev/ttyACM*`。
3. 加载地图，默认地图为 `Pic/map.png`。
4. 通过两点标定设置 cm/px 比例。
5. 设置外围边界，边界外区域视为绝对禁行区。
6. 添加或调整障碍物。
7. 设置小车初始位姿和目标路点。
8. 规划路径。
9. 发送路径，观察 STM32 回复和底盘动作。

> TODO: 添加主界面截图。

## 工作原理

```text
地图与障碍物
    -> 栅格化与 A* 路径规划
    -> 路径分段
    -> 串口发送 cmd_dis / cmd_turn
    -> STM32 固件执行运动
    -> 上位机接收 ok / done / busy / err 回复
```

上位机不直接控制电机。它只负责生成命令帧、发送命令、显示状态和辅助调试；真正的闭环控制在 STM32 固件中完成。

## 功能说明

| 功能 | 说明 |
| --- | --- |
| 串口 | 扫描 Qt 可见端口，例如 `/dev/ttyUSB*`、`/dev/ttyACM*`，参数固定为 `115200 8N1` |
| 地图 | 默认加载 `Pic/map.png`，通过两点标定获得 cm/px 比例 |
| 边界 | 外围边界为绝对禁行区，可在地图上重新绘制边界多边形 |
| 障碍 | 支持矩形、圆形、多边形障碍，可拖动位置，矩形/圆形可调整尺寸 |
| 路径 | 使用 2 cm 栅格 A*，按小车半径 10.5 cm + 2 cm 安全余量避障 |
| 运动 | 按规划路径分段发送 `cmd_dis`，到点后按路点设置发送 `cmd_turn` |
| 外设 | 吸力、踢球、继电器、运动使能、吸球检测、红外、yaw 清零、MCU 复位 |
| 配置 | 可保存 JSON，内容包含标定、边界、障碍、路点和小车初始位姿 |

## 常用操作

### 发送路径前检查

- STM32 固件已烧录并正常运行。
- USART6 接线正确，电平为 3.3 V。
- 已执行 yaw 清零或确认 yaw 参考方向正确。
- 底盘运动使能已打开。
- 地图比例、边界和障碍物设置可信。

### 使用 `--frame` 验证协议

如果 GUI 串口联调有问题，先用 `--frame` 生成命令，再用串口工具直接发送，区分问题来自 GUI、串口链路还是固件协议。

示例：

```bash
./build/host/debug/RCJappforchase/rcj_appforchase --frame cmd_turn 90
```

## 常见问题

### 找不到串口

检查设备是否被系统识别：

```bash
ls /dev/ttyUSB* /dev/ttyACM*
```

如果没有设备，检查 USB 转串口、线缆、供电和驱动。

### 打不开串口

常见原因是权限不足或串口被其他程序占用。可以检查当前用户是否在 `dialout` 组：

```bash
groups
```

加入后需要重新登录：

```bash
sudo usermod -aG dialout $USER
```

### 命令发送后无回复

检查以下项：

- 上位机连接的是 STM32 USART6。
- 串口参数为 `115200 8N1`。
- STM32 主循环正常运行。
- 命令帧 CRC 正确。
- 固件没有停在等待 IMU 或错误状态。

### 地图比例错误导致路径异常

重新做两点标定，并用已知距离验证 cm/px 比例。比例错误会直接影响规划距离和最终发送给固件的 `cmd_dis` 参数。

### 路径能规划但底盘不按预期走

先脱离 GUI，用固件协议单独测试：

1. `cmd_anglecal`
2. `cmd_dis 10 0`
3. `cmd_turn 90`
4. `cmd_request`

如果单独命令也异常，优先调固件和底盘参数；如果单独命令正常，再检查上位机地图坐标、路径分段和串口发送顺序。
