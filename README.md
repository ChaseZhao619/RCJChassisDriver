# RCJChassisDriver

RCJ 底盘总工程。当前仓库包含两个子工程：

- `RCJchassisdiver`: STM32F407 底盘固件，使用 STM32CubeMX 生成代码和 CMake/Ninja/arm-none-eabi-gcc 构建。
- `RCJappforchase`: Ubuntu 本地上位机程序，使用 CMake 构建，目前提供串口命令 CLI 骨架。

## 目录结构

```text
.
├── RCJchassisdiver/      # STM32 固件工程
├── RCJappforchase/       # Ubuntu 上位机工程
├── CMakeLists.txt        # 总工程入口
├── CMakePresets.json     # STM32 和 host app 构建预设
└── .vscode/tasks.json    # VS Code 总工程构建任务
```

## STM32 固件构建

Ubuntu 依赖：

```bash
sudo apt install cmake ninja-build gcc-arm-none-eabi
```

从仓库根目录构建 Debug：

```bash
cmake --preset stm32-debug
cmake --build --preset stm32-debug
```

产物位置：

```text
build/stm32/stm32-debug/RCJchassisdiver/RCJchassisdiver.elf
build/stm32/stm32-debug/RCJchassisdiver/RCJchassisdiver.map
```

仍然可以进入 `RCJchassisdiver/` 使用它自己的旧 preset：

```bash
cd RCJchassisdiver
cmake --preset Debug
cmake --build --preset Debug
```

## Ubuntu 上位机构建

从仓库根目录构建 Debug：

```bash
cmake --preset host-debug
cmake --build --preset host-debug
```

当前上位机可执行文件：

```text
build/host/debug/RCJappforchase/rcj_appforchase
```

注意：STM32 是交叉编译，上位机是 Ubuntu 本机编译，两者不能可靠地混在同一个 CMake build tree 里；本仓库用不同 preset 和不同 `build/` 子目录隔离它们。
