# RCJchassisdiver

`RCJchassisdiver` 是 STM32F407 底盘固件工程，负责底盘闭环运动、CAN 电机控制、BNO085 姿态读取、BE1732 红外读取、吸力电机、踢球电机、继电器和面向上位机/树莓派的串口协议。

这篇文档是固件主入口。完整串口协议请看 [`App/README.md`](App/README.md)，底层硬件调参请看 [`Bsp/README.md`](Bsp/README.md)。

## 你应该先知道

- 固件分为 `App`、`Bsp`、`Core` 三层：`App` 负责业务状态机，`Bsp` 负责硬件抽象，`Core` 主要来自 STM32CubeMX。
- 底盘运动依赖 BNO085 yaw。没有有效 yaw 时，运动命令会被拒绝或停止。
- 上位机/树莓派通过 USART6 发送命令；USART1 默认用于调试打印。
- 调试运动前必须架空底盘，先确认方向，再调控制参数。
- 修改 `.ioc` 后重新生成代码时，只能依赖 `USER CODE` 区域保留用户代码。

## 工作原理

```text
上位机 / 树莓派
    -> USART6 串口协议
    -> App 层任务：命令解析、底盘状态机、完成事件
    -> Bsp 层驱动：CAN、I2C、PWM、GPIO、USART
    -> 电机 / IMU / 红外 / 吸力 / 继电器
```

主控与功能：

- 主控：STM32F407xx。
- 底盘：4 个 CAN 电机组成全向/麦轮底盘，支持机体系速度、角度保持和里程计移动。
- 姿态：BNO085 通过 I2C1 输出 yaw 和 gyro z。
- 红外复眼：BE1732 通过 I2C2 读取 7 路红外光强方向。
- 功能电机：CAN ID 5，支持速度和方向控制。
- 吸力电机：TIM4_CH1 输出 50 Hz PWM，支持 0-100% 速度设置。
- 吸球检测：PB15/xqwd 微动开关输入。
- 继电器：PD0/JD1 数字输出。

## 代码结构

```text
.
├── App/                         # 应用层任务和外部通信协议
│   ├── Inc/
│   │   ├── app_chassis_task.h    # 底盘任务接口、运动参数宏
│   │   └── app_pi_comm.h         # 树莓派/上位机串口通信接口
│   └── Src/
│       ├── app_chassis_task.c    # 底盘状态机：等待 IMU、空闲、移动、转向、持续运动
│       └── app_pi_comm.c         # 串口收包、CRC 校验、命令解析、回复
├── Bsp/                         # 板级支持层
│   ├── Inc/
│   └── Src/
│       ├── bsp_motor.c           # CAN 电机反馈和电流发送
│       ├── bsp_chassis.c         # 底盘运动学、轮速 PID、角度保持
│       ├── bsp_chassis_odom.c    # 里程计估计和目标点控制
│       ├── bsp_bno085.c          # BNO085 初始化、报文读取、yaw 计算
│       ├── bsp_be1732.c          # BE1732 红外复眼 I2C 读取
│       ├── bsp_kick_motor.c      # CAN ID 5 功能/踢球电机速度控制
│       ├── bsp_suction_motor.c   # 吸力电机 PWM 控制
│       ├── bsp_suction_detect.c  # PB15/xqwd 吸球检测
│       ├── bsp_dct.c             # PD0/JD1 继电器输出
│       └── bsp_usart.c           # 串口发送、接收、Printf 封装
├── Core/                         # STM32CubeMX 生成代码和主循环
├── Drivers/                      # STM32 HAL、CMSIS 驱动
├── cmake/                        # 交叉编译工具链和 CubeMX CMake 文件
├── RCJchassisdiver.ioc           # STM32CubeMX 工程配置
├── STM32F407XX_FLASH.ld          # 链接脚本，最后 128K Flash sector 用于运行参数
└── startup_stm32f407xx.s         # 启动文件
```

## 快速开始

### 1. 安装依赖

```bash
sudo apt install cmake ninja-build gcc-arm-none-eabi
```

### 2. 配置并编译

在 `RCJchassisdiver/` 目录执行：

```bash
cmake --preset Debug
cmake --build --preset Debug
```

Release 构建：

```bash
cmake --preset Release
cmake --build --preset Release
```

默认产物：

```text
build/Debug/RCJchassisdiver.elf
build/Debug/RCJchassisdiver.map
```

也可以从仓库根目录构建：

```bash
cmake --preset stm32-debug
cmake --build --preset stm32-debug
```

### 3. 下载运行

常见 ST-Link/OpenOCD 下载示例：

```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program build/Debug/RCJchassisdiver.elf verify reset exit"
```

> 注意：下载前确认底盘处于安全状态。首次烧录或改动运动参数后，建议架空底盘并准备独立急停。

## 初始化与主循环

`Core/Src/main.c` 完成 HAL、系统时钟、GPIO、CAN、USART、I2C、TIM 初始化后，依次初始化：

1. `BspMotor_Init()`：启动 CAN 电机通信。
2. `BspKickMotor_Init()`：初始化 CAN ID 5 功能/踢球电机速度控制。
3. `BspSuctionMotor_Init()`：启动吸力电机 PWM。
4. `BspSuctionDetect_Init()`：初始化吸球检测。
5. `BspBe1732_Init()`：初始化 BE1732 红外复眼。
6. `BspDct_Init()`：关闭 PD0/JD1 继电器输出。
7. `AppChassisTask_Init()`：初始化底盘状态机。
8. `AppPiComm_Init()`：启动 USART6 中断接收。
9. `Bno085_Init()` 和 `Bno085_EnableDefaultReports()`：初始化 IMU 并开启默认报告。

主循环持续执行：

- 处理串口命令。
- 读取和解析 BNO085 数据。
- 处理 `BNO_KEY` 短按/长按。
- 更新底盘任务和踢球电机速度环。
- 执行吸力电机测试任务。

## 外设连接

| 外设 | 引脚/配置 | 用途 |
| --- | --- | --- |
| USART1 | PA9 TX, PA10 RX, 115200 8N1 | 调试打印，默认 `MAIN_DEBUG_USART` |
| USART6 | PC6 TX, PC7 RX, 115200 8N1 | 树莓派/上位机命令通信 |
| CAN1 | PA11 RX, PA12 TX | CAN 电机控制和反馈 |
| I2C1 | PB6 SCL, PB7 SDA, 400 kHz | BNO085 通信 |
| I2C2 | PB10 SCL, PB11 SDA, 100 kHz | BE1732 红外复眼 |
| TIM4_CH1 | PD12, 50 Hz PWM | 吸力电机/电调控制 |
| xqwd | PB15 input pull-up | 吸球微动开关检测，默认低电平表示吸到球 |
| JD1 | PD0 output | 继电器控制 |
| BNO_INT2 | PB1 input | BNO085 中断/就绪检测 |
| BNO_KEY | PE13 input pull-up | 短按 yaw 清零，长按切换底盘运动使能 |
| BNO_NRST | PB8 output | BNO085 复位 |

## 常用操作

### 串口命令摘要

完整格式和 CRC 算法见 [`App/README.md`](App/README.md)。

| 命令 | 作用 | 常见使用场景 |
| --- | --- | --- |
| `cmd_conmotion 1` | 启用底盘运动 | 开始运动测试前 |
| `cmd_anglecal` | 当前姿态设为 yaw 零点 | 上电后校准车头方向 |
| `cmd_dis x y [profile]` | 相对位移，单位 cm | 按坐标移动一小段 |
| `cmd_turn yaw` | 转到绝对 yaw | 调整车头方向 |
| `cmd_dkmotor speed angle [head_lock]` | 持续方向运动 | 手动遥控或连续运动 |
| `cmd_juststop` | 停止当前底盘命令 | 中断持续运动 |
| `cmd_request` | 查询里程增量和 yaw | 上位机同步位置 |
| `cmd_suck speed` | 设置吸力电机速度 | 控制吸球机构 |
| `cmd_tqdj speed reverse` | 设置功能/踢球电机 | 控制 CAN ID 5 电机 |
| `cmd_xqcx` | 查询数字吸球检测 | 判断是否吸到球 |

### BNO_KEY 按键

- 短按：执行 yaw 清零，成功后 USART1 打印 `imu_zero:1`。
- 长按约 1 秒：切换底盘运动使能。

## 参数与调试

常用参数位置：

| 文件 | 参数示例 | 作用 |
| --- | --- | --- |
| `App/Inc/app_chassis_task.h` | `APP_CHASSIS_TASK_MOVE_SPEED_MM_S` | 默认移动速度 |
| `App/Inc/app_chassis_task.h` | `APP_CHASSIS_TASK_ROTATE_TOLERANCE_DEG` | 转向到位角度容差 |
| `App/Inc/app_chassis_task.h` | `APP_CHASSIS_TASK_STOP_STABLE_MS` | 停稳判定时间 |
| `Bsp/Inc/bsp_chassis.h` | `BSP_CHASSIS_ANGLE_KP`、`BSP_CHASSIS_ANGLE_GYRO_KD` | 航向控制 |
| `Bsp/Inc/bsp_chassis.h` | `BSP_CHASSIS_WHEEL_SPEED_KP/KI/KD/KF` | 轮速控制 |
| `Bsp/Inc/bsp_chassis_odom.h` | `BSP_CHASSIS_ODOM_FORWARD_SCALE`、`BSP_CHASSIS_ODOM_LEFT_SCALE` | 里程计比例 |
| `Bsp/Inc/bsp_suction_motor.h` | `BSP_SUCTION_MOTOR_*_US` | 吸力电机 PWM 脉宽范围 |

推荐调试顺序：

1. 架空底盘，确认急停和 `cmd_conmotion 0` 有效。
2. 确认 CAN 电机 ID、旋转方向和反馈方向。
3. 调底盘轮速环。
4. 调 IMU yaw 方向和偏航角保持。
5. 标定里程计比例。
6. 调 App 层 `cmd_dis` 的速度曲线、横向纠偏和停稳阈值。

## 常见问题

### 串口命令一直返回 `busy`

优先检查 BNO085 是否有有效 yaw。没有有效 yaw 时，底盘状态机会等待 IMU，不会执行移动或转向命令。

### 底盘方向或角度闭环相反

先检查方向宏，例如 `BSP_CHASSIS_*_DIR`、`BSP_CHASSIS_YAW_CTRL_DIR`、`BSP_CHASSIS_GYRO_Z_DIR`。方向错误不能靠负 PID 参数修正。

### 里程计距离偏差很大

先确认轮径、减速比、轮速反馈方向和 yaw 方向，再调整 `BSP_CHASSIS_ODOM_FORWARD_SCALE` 与 `BSP_CHASSIS_ODOM_LEFT_SCALE`。

### 红外或吸球检测结果不稳定

先确认接线和 I2C/GPIO 状态，再看 [`Bsp/README.md`](Bsp/README.md) 中的 BE1732、吸球检测消抖和阈值说明。

## 维护说明

- 修改串口协议时，同步更新 `App/Src/app_pi_comm.c` 和 [`App/README.md`](App/README.md)。
- 修改外设引脚时，同步更新 `.ioc`、CubeMX 生成代码和本文档的外设表。
- 新增业务逻辑优先放在 `App/` 或 `Bsp/`。
- 重新生成 CubeMX 代码后，检查 `main.c`、`gpio.c`、`usart.c`、`i2c.c`、`can.c`、`tim.c` 中的 `USER CODE`。
