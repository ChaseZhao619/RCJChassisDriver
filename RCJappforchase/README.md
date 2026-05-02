# RCJappforchase

Ubuntu 本地上位机程序，使用 Qt Widgets + Qt SerialPort 实现地图可视化、路径规划、串口控制和外设控制。

## 构建

依赖：

```bash
sudo apt install cmake ninja-build qt6-base-dev qt6-serialport-dev
```

从仓库根目录执行：

```bash
cmake --preset host-debug
cmake --build --preset host-debug
```

可执行文件：

```text
build/host/debug/RCJappforchase/rcj_appforchase
```

## 使用

启动 GUI：

```bash
./build/host/debug/RCJappforchase/rcj_appforchase
```

保留一个 CRC 帧生成入口，便于协议测试：

```bash
./build/host/debug/RCJappforchase/rcj_appforchase --frame cmd_dis 10 0
```

## 功能

- 串口：扫描并选择 `/dev/ttyUSB*`、`/dev/ttyACM*` 等 Qt 可见端口，参数固定为 `115200 8N1`。
- 地图：默认加载 `Pic/map.png`，通过两点标定获得 cm/px 比例。
- 边界：外围边界为绝对禁行区，可在地图上重新绘制边界多边形。
- 障碍：支持矩形、圆形、多边形障碍；可拖动位置，矩形/圆形可调整尺寸。
- 路径：使用 2 cm 栅格 A*，按小车半径 10.5 cm + 2 cm 安全余量避障。
- 运动：按规划路径分段发送 `cmd_dis`，到点后按路点设置发送 `cmd_turn`。
- 外设：吸力、踢球、继电器、运动使能、吸球检测、红外、yaw 清零、MCU 复位。

配置可保存为 JSON，内容包含标定、边界、障碍、路点和小车初始位姿。
