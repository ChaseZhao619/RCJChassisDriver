# RCJChassisDriver

RCJChassisDriver 是 RCJ 机器人底盘控制系统仓库，包含 STM32 底盘固件和 Ubuntu/Qt 上位机程序。

本文档是项目总入口。如果你是第一次接触本工程，先按“快速开始”跑通构建；如果你已经在开发功能，直接查看“开发者入口”。

## 版权与许可

> 重要：本项目采用 GNU General Public License v3.0 only（GPL-3.0-only）授权。分发修改版或基于本项目的派生作品时，需要遵守 GPL-3.0 的源代码公开和同许可证分发要求。

Copyright © 2026 ChaseZhao619 and contributors.

完整条款见 [`LICENSE`](LICENSE)。你可以按照 GPL-3.0 的条款使用、复制、修改和分发本项目。

第三方组件遵循其各自许可证：

- STM32 HAL：见 [`firmware/stm32_chassis/Drivers/STM32F4xx_HAL_Driver/LICENSE.txt`](firmware/stm32_chassis/Drivers/STM32F4xx_HAL_Driver/LICENSE.txt)。
- CMSIS：见 [`firmware/stm32_chassis/Drivers/CMSIS/LICENSE.txt`](firmware/stm32_chassis/Drivers/CMSIS/LICENSE.txt)。
- STM32F4xx CMSIS Device：见 [`firmware/stm32_chassis/Drivers/CMSIS/Device/ST/STM32F4xx/LICENSE.txt`](firmware/stm32_chassis/Drivers/CMSIS/Device/ST/STM32F4xx/LICENSE.txt)。

## 安全与免责声明

> 危险：本仓库包含会驱动底盘电机、踢球机构、吸力机构、继电器和电调的代码。烧录、运行或调参前，必须确认急停、架空测试、供电、接线和人员距离安全。

- 首次测试或修改运动控制后，应架空底盘并准备独立急停。
- 不应在人员靠近轮子、踢球机构或吸力机构时执行运动测试。
- 本项目按“现状”提供，不承诺适用于任何特定比赛、课程、商业或安全关键场景。
- 使用者需要自行承担硬件损坏、人身伤害、比赛失误和数据丢失等风险。

如果发现可能导致失控、越权写入、错误复位或硬件损坏的问题，请优先私下联系维护者或在受控范围内提交 issue，避免公开可直接复现危险动作的细节。

## 贡献与维护说明

> 重要：协议、控制参数、硬件引脚和测试宏的改动都可能影响实车安全。提交这类改动时，必须同步说明测试条件和影响范围。

欢迎通过 issue、pull request 或分支提交改进。为了降低硬件项目的联调风险，提交前建议做到：

- 文档改动说明影响范围。
- 固件改动说明测试硬件、测试命令和安全措施。
- 协议改动同步更新固件、上位机和 README。
- 参数改动记录旧值、新值、测试场景和现象。
- 不提交个人密钥、串口日志中的敏感信息、私有节点配置或本机绝对路径。

维护规则：

- `Core/` 下多数文件由 STM32CubeMX 生成，重新生成代码时注意保留 `USER CODE` 区域。
- 新增业务逻辑优先放在 `App/` 或 `Bsp/`，不要混入 CubeMX 生成代码。
- 修改串口协议时，同步更新 `App/Src/app_pi_comm.c` 和 [`App/README.md`](firmware/stm32_chassis/App/README.md)。
- 修改外设引脚时，同步更新 `.ioc`、CubeMX 生成代码和 [`firmware/stm32_chassis/README.md`](firmware/stm32_chassis/README.md) 的外设表。

## 你应该先知道

- `firmware/stm32_chassis` 是 STM32F407 固件，负责电机、传感器、执行器和串口协议。
- `apps/desktop_controller` 是 Ubuntu 本地上位机，负责地图、路径规划、串口控制和调试操作。
- 上位机和树莓派通过串口协议控制 STM32，常用命令包括 `cmd_dis`、`cmd_turn`、`cmd_dkmotor`。
- STM32 固件是交叉编译工程；上位机是 Ubuntu 本机编译工程，两者使用不同 CMake preset 和不同 build 目录。
- 调试底盘前必须先保证急停、架空测试和运动方向检查可用。

## 子工程导航

| 目录 | 面向对象 | 说明 |
| --- | --- | --- |
| [`firmware/stm32_chassis`](firmware/stm32_chassis/README.md) | 固件开发者、嵌入式调试人员 | STM32 底盘固件主文档 |
| [`firmware/stm32_chassis/App`](firmware/stm32_chassis/App/README.md) | 协议开发者、上位机/树莓派开发者 | 应用层状态机与串口协议 |
| [`firmware/stm32_chassis/Bsp`](firmware/stm32_chassis/Bsp/README.md) | 硬件调试人员、控制参数调试人员 | BSP 驱动与调参手册 |
| [`apps/desktop_controller`](apps/desktop_controller/README.md) | 上位机使用者、Qt 开发者 | Ubuntu/Qt 上位机使用说明 |

## 快速开始

### 1. 安装依赖

Ubuntu 环境：

```bash
sudo apt install cmake ninja-build gcc-arm-none-eabi qt6-base-dev qt6-serialport-dev
```

如果只编译 STM32 固件，可以不安装 Qt 依赖；如果只编译上位机，可以不安装 `gcc-arm-none-eabi`。

### 2. 构建 STM32 固件

从仓库根目录执行：

```bash
cmake --preset stm32-debug
cmake --build --preset stm32-debug
```

主要产物：

```text
build/stm32/stm32-debug/firmware/stm32_chassis/stm32_chassis.elf
build/stm32/stm32-debug/firmware/stm32_chassis/stm32_chassis.map
```

也可以进入固件目录使用旧 preset：

```bash
cd firmware/stm32_chassis
cmake --preset Debug
cmake --build --preset Debug
```

### 3. 构建 Ubuntu 上位机

从仓库根目录执行：

```bash
cmake --preset host-debug
cmake --build --preset host-debug
```

可执行文件：

```text
build/host/debug/apps/desktop_controller/rcj_appforchase
```

### 4. 下一步读什么

- 想烧录和调试 STM32：读 [`firmware/stm32_chassis/README.md`](firmware/stm32_chassis/README.md)。
- 想发串口命令控制底盘：读 [`firmware/stm32_chassis/App/README.md`](firmware/stm32_chassis/App/README.md)。
- 想调 PID、方向、里程计：读 [`firmware/stm32_chassis/Bsp/README.md`](firmware/stm32_chassis/Bsp/README.md)。
- 想使用图形上位机：读 [`apps/desktop_controller/README.md`](apps/desktop_controller/README.md)。

## 工作原理

```text
Ubuntu 上位机 / 树莓派
    -> USART6 串口协议
    -> STM32 App 层：命令解析、状态机、完成事件
    -> STM32 Bsp 层：电机、IMU、红外、PWM、GPIO
    -> 底盘电机 / 传感器 / 执行器
```

工程目录：

```text
.
├── firmware/
│   └── stm32_chassis/          # STM32 固件工程
├── apps/
│   └── desktop_controller/     # Ubuntu/Qt 上位机工程
├── assets/
│   └── maps/                   # 地图和车体图片资源
├── docs/
│   ├── firmware/               # 固件设计文档
│   └── hardware/               # 传感器和硬件参考资料
├── tools/                      # 调试和工具脚本
├── CMakeLists.txt              # 总工程入口
├── CMakePresets.json           # STM32 和 host app 构建预设
└── .vscode/tasks.json          # VS Code 总工程构建任务
```

## 开发者入口

| 你要做什么 | 推荐阅读 |
| --- | --- |
| 理解 STM32 固件启动流程 | [`firmware/stm32_chassis/README.md`](firmware/stm32_chassis/README.md) |
| 新增或修改串口命令 | [`firmware/stm32_chassis/App/README.md`](firmware/stm32_chassis/App/README.md) |
| 调整底盘运动参数 | [`firmware/stm32_chassis/App/README.md`](firmware/stm32_chassis/App/README.md) 与 [`firmware/stm32_chassis/Bsp/README.md`](firmware/stm32_chassis/Bsp/README.md) |
| 调整电机方向、PID、里程计 | [`firmware/stm32_chassis/Bsp/README.md`](firmware/stm32_chassis/Bsp/README.md) |
| 使用或开发 Qt 上位机 | [`apps/desktop_controller/README.md`](apps/desktop_controller/README.md) |

## 常见问题

### 为什么编译 STM32 固件失败？

优先检查 `gcc-arm-none-eabi` 是否安装完整，以及当前 shell 能否找到工具链：

```bash
arm-none-eabi-gcc --version
```

如果 CMake 配置成功但编译时报 `stdint.h`、`errno.h`、`sys/stat.h` 缺失，通常是本地 ARM 工具链或 Newlib 安装不完整。

### 为什么串口没有回复？

检查以下项：

- 上位机连接的是 STM32 的 USART6，不是调试用 USART1。
- 串口参数为 `115200 8N1`。
- 命令帧包含正确 CRC，格式为 `payload *CCCC\r\n`。
- 固件主循环持续调用 `AppPiComm_Task()`。

### 为什么底盘不动或一直返回 `busy`？

优先检查 BNO085 是否有有效 yaw。底盘任务在没有有效 IMU 姿态时会停留在等待状态，并拒绝运动命令。

### 为什么底盘运动方向不对？

不要先调 PID。先按 [`BSP 调参手册`](firmware/stm32_chassis/Bsp/README.md) 检查电机编号、旋转方向、反馈方向、陀螺仪方向和运动坐标约定。
