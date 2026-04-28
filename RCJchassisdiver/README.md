# RCJchassisdiver

STM32F407 底盘驱动工程，用于 RCJ 机器人底盘控制。工程基于 STM32CubeMX 生成的 HAL 初始化代码和 CMake 构建系统，业务层实现了底盘闭环移动、BNO085 姿态读取、CAN 电机控制、吸力电机 PWM 控制，以及面向上位机/树莓派的串口命令协议。

## 功能概览

- 主控：STM32F407xx。
- 底盘：4 个 CAN 电机组成的全向/麦轮底盘，支持机体系速度、角度保持和里程计移动。
- 姿态：BNO085 通过 I2C1 读取 yaw 和 gyro z，用于航向保持和里程计更新。
- 红外复眼：BE-1732 通过 I2C2 读取 7 路红外光强方向。
- 电机通信：CAN1 控制底盘电机和功能电机。
- 吸力电机：TIM4_CH1 输出 PWM，支持 0-100% 速度设置。
- 串口通信：USART6 接收树莓派命令，USART1 默认用于调试打印。

## 代码结构

```text
.
├── App/                         # 应用层任务和外部通信协议
│   ├── Inc/
│   │   ├── app_chassis_task.h    # 底盘任务接口、运动参数宏
│   │   └── app_pi_comm.h         # 树莓派串口通信接口
│   └── Src/
│       ├── app_chassis_task.c    # 底盘状态机：等待 IMU、空闲、移动、转向
│       └── app_pi_comm.c         # 串口收包、CRC 校验、命令解析、回复
├── Bsp/                         # 板级支持层
│   ├── Inc/
│   └── Src/
│       ├── bsp_motor.c           # CAN 电机反馈和电流发送
│       ├── bsp_chassis.c         # 底盘运动学、PID、角度保持
│       ├── bsp_chassis_odom.c    # 里程计估计和目标点控制
│       ├── bsp_bno085.c          # BNO085 初始化、报文读取、yaw 计算
│       ├── bsp_be1732.c          # BE-1732 红外复眼 I2C 读取
│       ├── bsp_suction_motor.c   # 吸力电机 PWM/油门控制
│       └── bsp_usart.c           # 串口发送、接收、Printf 封装
├── Core/                         # STM32CubeMX 生成和用户主循环代码
│   ├── Inc/                      # 外设头文件、main.h、HAL 配置
│   └── Src/
│       ├── main.c                # 初始化流程和主循环调度
│       ├── can.c                 # CAN1 初始化
│       ├── i2c.c                 # I2C1 初始化
│       ├── tim.c                 # TIM4 PWM 初始化
│       ├── usart.c               # USART1/USART6 初始化
│       └── gpio.c                # BNO085 相关 GPIO
├── Drivers/                      # STM32 HAL、CMSIS 驱动
├── cmake/                        # 交叉编译工具链和 CubeMX CMake 文件
├── CMakeLists.txt                # 工程顶层 CMake 配置
├── CMakePresets.json             # Debug/Release 构建预设
├── RCJchassisdiver.ioc           # STM32CubeMX 工程配置
├── STM32F407XX_FLASH.ld          # 链接脚本
└── startup_stm32f407xx.s         # 启动文件
```

## 主程序流程

`Core/Src/main.c` 完成 HAL、系统时钟、GPIO、CAN、USART、I2C、TIM 初始化后，依次初始化：

1. `BspMotor_Init()`：启动 CAN 电机通信。
2. `BspSuctionMotor_Init()`：启动吸力电机 PWM。
3. `AppChassisTask_Init()`：初始化底盘任务状态机。
4. `AppPiComm_Init()`：启动 USART6 中断接收。
5. `Bno085_Init()` 和 `Bno085_EnableDefaultReports()`：初始化 IMU 并开启默认报告。

主循环中持续处理串口命令、读取 BNO085 数据、按按键进行 yaw 清零、更新底盘任务，并执行吸力电机测试任务。

## 外设连接

| 外设 | 引脚/配置 | 用途 |
| --- | --- | --- |
| USART1 | PA9 TX, PA10 RX, 115200 8N1 | 调试打印，默认 `MAIN_DEBUG_USART` |
| USART6 | PC6 TX, PC7 RX, 115200 8N1 | 树莓派/上位机命令通信 |
| CAN1 | PA11 RX, PA12 TX | CAN 电机控制和反馈 |
| I2C1 | PB6 SCL, PB7 SDA, 400 kHz | BNO085 通信 |
| I2C2 | PB10 SCL, PB11 SDA, 100 kHz | BE-1732 红外复眼 |
| TIM4_CH1 | PD12, 50 Hz PWM | 吸力电机/电调控制 |
| BNO_INT2 | PB1 input | BNO085 中断/就绪检测 |
| BNO_KEY | PE13 input pull-up | yaw 清零按键 |
| BNO_NRST | PB8 output | BNO085 复位 |

## 构建方式

依赖：

- CMake 3.22 或更新版本。
- Ninja。
- `arm-none-eabi-gcc` 工具链。

配置并编译 Debug：

```bash
cmake --preset Debug
cmake --build --preset Debug
```

编译 Release：

```bash
cmake --preset Release
cmake --build --preset Release
```

当前默认产物为：

```text
build/Debug/RCJchassisdiver.elf
build/Debug/RCJchassisdiver.map
```

如需生成 `.bin` 或 `.hex`，可以在 CMake 中增加 `arm-none-eabi-objcopy` 的 post-build 命令，或在编译后手动转换。

## 下载和运行

常见 ST-Link/OpenOCD 下载方式示例：

```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program build/Debug/RCJchassisdiver.elf verify reset exit"
```

运行前检查：

- CAN 总线终端电阻、电机 ID 和供电是否正确。
- BNO085 的 I2C 地址、INT、NRST、KEY 引脚是否和本工程一致。
- BE-1732 红外复眼连接到 I2C2，模块地址为 `0x01`。
- 树莓派串口连接到 USART6，电平为 3.3 V。
- 调试串口如需查看日志，连接 USART1，波特率 115200。

## 串口命令协议

树莓派/上位机通过 USART6 发送命令。串口参数：

```text
115200 baud, 8 data bits, no parity, 1 stop bit
```

每条命令一行：

```text
<payload> *<CRC16>\r\n
```

说明：

- `payload` 是实际命令内容，例如 `cmd_dis 10 0`。
- `*` 后面是 4 位大写十六进制 CRC。
- CRC 算法为 CRC16-CCITT，初值 `0xFFFF`，多项式 `0x1021`，无最终异或。
- CRC 只计算 `payload`，不包含 ` *CRC`、`\r`、`\n`。
- 固件回复也使用同样格式：`<payload> *<CRC16>\r\n`。
- 单行最大长度由 `APP_PI_COMM_LINE_SIZE` 控制，当前为 96 字节。

### CRC 计算示例

```c
uint16_t crc16_ccitt(const uint8_t *data, uint16_t size)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < size; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if ((crc & 0x8000) != 0)
            {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}
```

### `cmd_dis`

控制底盘按当前里程计坐标做相对位移，单位为 cm。执行过程中保持当前 yaw。

```text
cmd_dis <x_cm> <y_cm> *<CRC16>
```

示例：

```text
cmd_dis 10 0 *B37E
```

含义：向里程计 x 方向移动 10 cm，y 方向不变。

可能回复：

```text
cmd_dis ok 10 0 *....
cmd_dis busy 10 0 *....
cmd_dis done 10 0 *....
err arg *....
```

### `cmd_turn`

控制底盘转到绝对目标 yaw 角，单位为度。目标角会被归一化到 0-360 度。

```text
cmd_turn <target_yaw_deg> *<CRC16>
```

示例：

```text
cmd_turn 90 *1935
```

可能回复：

```text
cmd_turn ok 90 *....
cmd_turn busy 90 *....
cmd_turn done 90 *....
err arg *....
```

### `cmd_dkmotor`

进入底盘持续速度控制模式。该模式不做加减速规划，只使用轮速闭环；需要停止时发送速度 `0`。

```text
cmd_dkmotor <speed_percent> <move_angle_deg> [head_lock] *<CRC16>
```

参数说明：

- `speed_percent`：速度映射值，范围 `0-100`。当前 `100` 对应 `APP_CHASSIS_TASK_DKMOTOR_MAX_SPEED_MM_S`，默认 `650 mm/s`。
- `move_angle_deg`：运动角度，单位度，`0` 为小车前方，`90` 为小车左方。
- `head_lock`：锁头使能，可省略，默认 `1`。`1` 表示保持当前车头方向不变，按运动角度整体平移；`0` 表示先转到对应角度，再朝小车前方直行。

示例：

```text
cmd_dkmotor 50 0 *277A
cmd_dkmotor 50 90 *A24E
cmd_dkmotor 50 0 1 *6038
cmd_dkmotor 50 90 1 *F282
cmd_dkmotor 50 90 0 *E2A3
cmd_dkmotor 0 0 *F332
```

可能回复：

```text
cmd_dkmotor ok 50 0 *077E
cmd_dkmotor ok 50 90 *822C
cmd_dkmotor ok 50 0 1 *265A
cmd_dkmotor ok 50 90 1 *B880
cmd_dkmotor busy 50 90 1 *684C
err arg *....
```

### `cmd_juststop`

停止当前持续运动，但不关闭底盘运动功能，仍然保持转向环。主要用于 `cmd_dkmotor` 持续运动过程中停车并保持航向。

```text
cmd_juststop *C3E4
```

可能回复：

```text
cmd_juststop ok *E3CE
cmd_juststop busy *AE33
err arg *....
```

### `cmd_suck`

设置吸力电机速度百分比，范围 0-100。

```text
cmd_suck <speed_percent> *<CRC16>
```

示例：

```text
cmd_suck 50 *5752
```

可能回复：

```text
cmd_suck ok 50 *....
cmd_suck busy 50 *....
err arg *....
```

### `cmd_conmotion`

控制底盘运动功能使能。该开关会影响 `cmd_dis`、`cmd_turn` 以及空闲状态下默认的转向环/角度保持输出。

```text
cmd_conmotion <0|1> *<CRC16>
```

示例：

```text
cmd_conmotion 0 *6732
cmd_conmotion 1 *7713
```

说明：

- `0`：失能底盘运动，立即停止底盘电机，并关闭默认转向环输出。
- `1`：使能底盘运动，允许 `cmd_dis`、`cmd_turn` 和空闲角度保持继续工作。

可能回复：

```text
cmd_conmotion ok 0 *87F6
cmd_conmotion ok 1 *97D7
err arg *....
```

### `cmd_request`

查询自上一次 `cmd_request` 以来的里程计增量和当前 yaw。

```text
cmd_request *55E4
```

回复格式：

```text
cmd_request <dx_cm> <dy_cm> <dyaw_deg> <yaw_deg> *<CRC16>
```

字段说明：

- `dx_cm`：距离上次查询的 x 位移，单位 cm。
- `dy_cm`：距离上次查询的 y 位移，单位 cm。
- `dyaw_deg`：距离上次查询的 yaw 变化，单位度。
- `yaw_deg`：当前 yaw，单位度。

第一次查询时，`dx_cm`、`dy_cm`、`dyaw_deg` 返回 0，随后建立增量参考点。

### `cmd_infred`

查询 BE-1732 红外复眼当前模式下最强红外信号所在通道。命令无参数，返回值为 `1-7`。

```text
cmd_infred *8E0C
```

回复格式：

```text
cmd_infred <channel> *<CRC16>
```

示例：

```text
cmd_infred 1 *D98F
cmd_infred 7 *B949
```

如果 I2C 读取失败：

```text
cmd_infred busy <status> <i2cerr> *<CRC16>
```

### `cmd_infred_mode`

切换 BE-1732 红外复眼检测模式。默认使用 `tz` 调制检测模式。

```text
cmd_infred_mode pt *2597
cmd_infred_mode tz *089D
```

参数说明：

- `pt`：普通检测模式，对应手册命令 `13`。
- `tz`：调制检测模式，对应手册命令 `14`。

成功回复：

```text
cmd_infred_mode ok pt *F22C
cmd_infred_mode ok tz *DF26
```

如果 I2C 切换失败：

```text
cmd_infred_mode busy <pt|tz> <status> <i2cerr> *<CRC16>
```

### `cmd_anglecal`

执行 yaw 角度清零，功能与 `BNO_KEY` 按键清零一致。命令无参数。

```text
cmd_anglecal *7932
```

可能回复：

```text
cmd_anglecal ok *6571
cmd_anglecal done *4C90
cmd_anglecal busy *....
cmd_anglecal eror *....
err arg *....
```

说明：

- `ok` 表示已成功执行 BNO085 yaw 清零。
- `done` 会在 `ok` 后立即发送，表示该命令流程结束。
- `busy` 通常表示当前还没有有效 BNO085 姿态数据，无法清零。

### `cmd_mcureset`

执行单片机软件复位。命令无参数。

```text
cmd_mcureset *C427
```

可能回复：

```text
cmd_mcureset ok *2559
cmd_mcureset done *E436
err arg *....
```

固件发送 `ok` 和 `done` 后会短延时，然后调用 `NVIC_SystemReset()` 复位 MCU。

### 错误回复

| 回复 | 含义 |
| --- | --- |
| `err cmd` | 命令名不支持 |
| `err arg` | 参数数量、格式或范围错误 |
| `err long` | 单行命令超过缓冲区长度 |
| `<cmd> eror` | CRC 错误、缺少 `*CRC` 或帧格式错误 |

注意：当前代码中的格式错误回复字符串为 `eror`，不是 `error`。

## 关键参数

多数控制参数通过头文件宏配置，未定义时使用默认值。

| 文件 | 参数示例 | 作用 |
| --- | --- | --- |
| `App/Inc/app_chassis_task.h` | `APP_CHASSIS_TASK_MOVE_SPEED_MM_S` | 默认移动速度 |
| `App/Inc/app_chassis_task.h` | `APP_CHASSIS_TASK_ROTATE_TOLERANCE_DEG` | 转向到位角度容差 |
| `App/Inc/app_chassis_task.h` | `APP_CHASSIS_TASK_STOP_STABLE_MS` | 电机停止稳定判定时间 |
| `Bsp/Inc/bsp_chassis.h` | `BSP_CHASSIS_ANGLE_KP`、`BSP_CHASSIS_ANGLE_GYRO_KD` | 航向控制参数 |
| `Bsp/Inc/bsp_chassis.h` | `BSP_CHASSIS_WHEEL_SPEED_KP/KI/KD/KF` | 轮速控制参数 |
| `Bsp/Inc/bsp_chassis_odom.h` | `BSP_CHASSIS_ODOM_FORWARD_SCALE`、`BSP_CHASSIS_ODOM_LEFT_SCALE` | 里程计标定比例 |
| `Bsp/Inc/bsp_suction_motor.h` | `BSP_SUCTION_MOTOR_*_US` | 吸力电机 PWM 脉宽范围 |

## 调试建议

- `MAIN_IMU_PRINT_ENABLE` 可打开 IMU 周期打印。
- yaw 清零由 `BNO_KEY` 按键触发，成功后 USART1 打印 `imu_zero:1`。
- 如果串口命令一直返回 `busy`，优先检查 BNO085 是否正常输出 yaw；没有有效 yaw 时底盘任务会停留在等待 IMU 状态。
- 如果底盘方向或角度闭环相反，检查 `BSP_CHASSIS_*_DIR`、`BSP_CHASSIS_YAW_CTRL_DIR`、`BSP_CHASSIS_GYRO_Z_DIR` 等方向宏。
- 如果里程计距离偏差较大，调整 `BSP_CHASSIS_ODOM_FORWARD_SCALE` 和 `BSP_CHASSIS_ODOM_LEFT_SCALE`。

## 维护注意事项

- `Core/` 下多数文件由 STM32CubeMX 生成，重新生成代码时注意保留 `USER CODE` 区域内的用户代码。
- 新增业务逻辑优先放在 `App/` 或 `Bsp/`，避免和 CubeMX 生成代码混在一起。
- 修改串口协议时，应同步更新 `App/Src/app_pi_comm.c` 和本 README 的命令说明。
- 修改外设引脚时，应同步更新 `.ioc`、CubeMX 生成代码和本文档的外设连接表。
