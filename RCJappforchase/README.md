# RCJappforchase

Ubuntu 本地上位机程序。当前提供一个最小 CLI，用于生成固件协议帧，或通过串口发送命令到 STM32。

## 构建

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

只生成带 CRC 的协议帧：

```bash
./build/host/debug/RCJappforchase/rcj_appforchase --frame cmd_dis 10 0
```

通过串口发送到 STM32：

```bash
./build/host/debug/RCJappforchase/rcj_appforchase --port /dev/ttyUSB0 cmd_dis 10 0
```

串口参数固定为 `115200 8N1`，协议帧格式与 `RCJchassisdiver/README.md` 中的上位机/树莓派协议一致。
