# App 应用层与串口协议

`App` 层负责把上位机/树莓派发来的串口命令转换为机器人动作。它不直接操作寄存器，而是调用 `Bsp` 层完成电机、传感器和执行器控制。

这篇文档面向两类人：如果你是学生，先看“快速开始”和“学生常见误解”；如果你在开发协议或调运动逻辑，重点看“工作原理”“完整命令表”和“参数与调试”。

## 你应该先知道

- `app_pi_comm` 负责 USART6 收包、CRC 校验、命令解析和回复。
- `app_chassis_task` 负责底盘状态机，包括等待 IMU、空闲、移动、转向和持续运动。
- 串口命令必须带 CRC，格式错误或 CRC 错误不会触发执行器。
- `cmd_dis` 是相对位移，不是绝对坐标。
- `cmd_dkmotor` 是持续运动命令，需要主动发送停止或速度 0。

## 快速开始

> 注意：第一次测试必须架空底盘，并准备独立急停。

最小可用命令流程：

1. 启用底盘运动：

   ```text
   cmd_conmotion 1 *<CRC16>
   ```

2. 将当前车头方向设为 yaw 零点：

   ```text
   cmd_anglecal *<CRC16>
   ```

3. 向世界坐标 x 方向移动 10 cm：

   ```text
   cmd_dis 10 0 *<CRC16>
   ```

4. 等待固件先回复 `cmd_dis ok ...`，运动完成后再回复 `cmd_dis done ...`。

5. 查询从上次查询到当前的里程增量：

   ```text
   cmd_request *<CRC16>
   ```

调试时可以用上位机的 `--frame` 功能生成带 CRC 的命令帧，见 [`../../RCJappforchase/README.md`](../../RCJappforchase/README.md)。

## 工作原理

```text
USART6 接收中断
    -> app_pi_comm 环形缓冲区
    -> AppPiComm_Task() 解析一行命令和 CRC
    -> AppChassisTask_Command*() / Bsp*()

主循环 IMU 数据
    -> AppChassisTask_Task()
    -> 更新里程计和运动状态机
    -> BspChassis*()
    -> 完成事件
    -> AppPiComm_Task() 发送 done
```

`AppPiComm_Task()` 和 `AppChassisTask_Task()` 都必须在主循环中持续调用。串口中断只接收单字节并写入环形缓冲区，不在中断中解析命令或驱动执行器。

## 坐标、单位与命令语义

- 世界坐标 `x/y` 与 `BspChassisOdom` 一致。
- 距离命令参数使用 `cm`，进入里程计前乘以 10 转为 `mm`。
- `yaw` 和移动方向使用 `deg`。
- 底盘约定 `0 deg` 为前方，`90 deg` 为左方，逆时针为正。
- `cmd_dis x y [profile]` 是相对当前世界坐标的位移。
- `cmd_turn yaw` 的参数是绝对偏航目标，内部归一化到 `[0, 360)`。
- `cmd_request` 返回从上次请求到当前的增量 `dx dy dyaw` 和当前绝对 `yaw`。

## 底盘状态机

| 状态 | 行为 | 退出条件 |
| --- | --- | --- |
| `WAIT_IMU` | 电机停止，等待有效偏航角 | 收到有效 yaw 后初始化里程计 |
| `IDLE` | 保持最后目标航向 | 收到有效运动命令 |
| `MOVE` | 按速度曲线沿线段运动并纠正横向误差 | 到达位置/进度阈值且完成停稳确认 |
| `TURN` | 原地保持目标偏航角 | 角度进入容差且电机持续停稳 |
| `DKMOTOR` | 持续方向运动，可选锁定车头 | 新命令、速度 0 或停止/禁用命令 |

IMU yaw 无效时，状态机会立即回到 `WAIT_IMU`，取消活动命令并发送停车指令。`cmd_conmotion 0` 会取消活动命令和待发送完成事件，不会为被取消的命令发送 `done`。

## 串口协议

每帧是一行 ASCII 文本：

```text
payload *CCCC\r\n
```

- `payload` 是命令内容，例如 `cmd_dis 10 0`。
- `CCCC` 为 4 位大写十六进制 CRC。
- CRC 算法为 CRC-16/CCITT-FALSE：初值 `0xFFFF`，多项式 `0x1021`，不反射，无最终异或。
- CRC 只覆盖 `payload`，不包含 CRC 前的分隔空格、`*CCCC` 或换行。
- 固件回复也使用同样格式。
- 单行最大长度由 `APP_PI_COMM_LINE_SIZE` 控制。

CRC 参考实现：

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

## 完整命令表

危险等级用于提醒调试风险：

- 低：只读或低风险配置。
- 中：会改变执行器状态，但动作可控。
- 高：会驱动底盘或复位 MCU，必须确认现场安全。

| 命令 payload | 参数 | 何时使用 | 危险等级/注意事项 |
| --- | --- | --- | --- |
| `cmd_dis x y [profile]` | `x/y: cm`, `profile: 0..2` | 让底盘移动指定相对距离 | 高；会驱动底盘，等待 `done` 后再发下一条运动命令 |
| `cmd_turn yaw` | `yaw: deg` | 转到绝对偏航角 | 高；依赖 IMU yaw |
| `cmd_dkmotor speed angle [head_lock]` | `speed: 0..100`, `angle: deg`, `head_lock: 0/1` | 持续方向运动或遥控 | 高；必须主动停止 |
| `cmd_juststop` | 无 | 停止当前底盘命令 | 中；不关闭底盘运动使能 |
| `cmd_conmotion enabled` | `0/1` | 启用/禁用底盘运动 | 高；`0` 应作为软件停车手段 |
| `cmd_request` | 无 | 查询里程增量和当前 yaw | 低；适合周期查询，但频率不宜过高 |
| `cmd_suck speed` | `0..100` | 设置吸力电机速度 | 中；注意电调解锁和机械防护 |
| `cmd_tqdj speed reverse` | `speed: 0..100`, `reverse: 0/1` | 控制功能/踢球电机 | 中；速度为 0 时停止输出 |
| `cmd_xqcx` | 无 | 查询数字吸球检测 | 低；返回 `1` 表示吸到球 |
| `cmd_redzhi` | 无 | 查询 BE1732 最大光值 | 低；用于阈值标定 |
| `cmd_xgred threshold` | `0..255` | 设置并保存无球阈值 | 中；会擦写 Flash，不能周期调用 |
| `cmd_infred` | 无 | 查询滤波后的红外通道 | 低；无可靠信号时返回 `-1` |
| `cmd_infred_mode pt/tz` | 普通/调制模式 | 切换 BE1732 模式 | 中；影响红外识别结果 |
| `cmd_dct enabled` | `0/1` | 控制 DCT/继电器输出 | 中；确认外接负载 |
| `cmd_anglecal` | 无 | 将当前姿态设为 yaw 零点 | 中；会改变后续角度参考 |
| `cmd_mcureset` | 无 | 软件复位 MCU | 高；会中断所有控制 |

成功、忙、完成和错误回复也使用 CRC 封装。代码沿用协议中的 `eror` 拼写，上位机必须按实际字符串兼容。

## 常用命令说明

### `cmd_dis`

按当前里程计坐标做相对位移，单位为 cm。执行过程中保持当前 yaw。

```text
cmd_dis <x_cm> <y_cm> [speed_profile] *<CRC16>
```

速度曲线：

- `0`：更激进，更快接近最高速度。
- `1`：默认曲线。
- `2`：更平滑，起停更柔和。

常见回复：

```text
cmd_dis ok <x_cm> <y_cm> [profile] *<CRC16>
cmd_dis busy <x_cm> <y_cm> [profile] *<CRC16>
cmd_dis done <x_cm> <y_cm> *<CRC16>
err arg *<CRC16>
```

### `cmd_turn`

转到绝对目标 yaw，单位为度。

```text
cmd_turn <target_yaw_deg> *<CRC16>
```

常见回复：

```text
cmd_turn ok <target_yaw_deg> *<CRC16>
cmd_turn busy <target_yaw_deg> *<CRC16>
cmd_turn done <target_yaw_deg> *<CRC16>
err arg *<CRC16>
```

### `cmd_dkmotor`

进入持续速度控制模式。该模式不做加减速规划，需要停止时发送速度 `0` 或 `cmd_juststop`。

```text
cmd_dkmotor <speed_percent> <move_angle_deg> [head_lock] *<CRC16>
```

- `speed_percent`：范围 `0-100`。`100` 对应 `APP_CHASSIS_TASK_DKMOTOR_MAX_SPEED_MM_S`。
- `move_angle_deg`：`0` 为车体前方，`90` 为车体左方。
- `head_lock`：默认 `1`。`1` 保持当前车头方向平移；`0` 先转到对应角度再前进。

### `cmd_request`

查询自上一次 `cmd_request` 以来的里程计增量和当前 yaw。

```text
cmd_request *<CRC16>
```

回复格式：

```text
cmd_request <dx_cm> <dy_cm> <dyaw_deg> <yaw_deg> *<CRC16>
```

第一次查询时，`dx_cm`、`dy_cm`、`dyaw_deg` 返回 0，并建立增量参考点。

### 红外、吸球和执行器命令

- `cmd_xqcx`：返回 `cmd_xqcx <0|1>`，`1` 表示吸到球。
- `cmd_infred`：返回最强红外通道 `1-7`，无可靠信号时返回 `-1`。
- `cmd_redzhi`：返回当前最大光值。
- `cmd_xgred <value>`：设置无红外阈值并写入 Flash。
- `cmd_infred_mode pt|tz`：切换普通/调制检测模式。
- `cmd_suck <speed>`：设置吸力电机速度百分比。
- `cmd_tqdj <speed> <reverse>`：设置 CAN ID 5 功能电机速度和方向。
- `cmd_dct <0|1>`：控制 PD0/JD1 输出。
- `cmd_anglecal`：执行 yaw 清零，成功后返回 `ok` 和 `done`。
- `cmd_mcureset`：回复后短延时并复位 MCU。

### 错误回复

| 回复 | 含义 |
| --- | --- |
| `err cmd` | 命令名不支持 |
| `err arg` | 参数数量、格式或范围错误 |
| `err long` | 单行命令超过缓冲区长度 |
| `<cmd> eror` | CRC 错误、缺少 `*CRC` 或帧格式错误 |

## 参数与调试

### 调运动参数前先确认

先不要急着调 `APP_CHASSIS_TASK_*`。如果下面任一项不成立，调 App 参数通常只会掩盖问题：

- 四个底盘电机 ID 正确。
- 正目标速度对应正反馈速度。
- yaw 角度方向正确。
- `cmd_conmotion 0` 能让底盘停止。
- 里程计前后/左右方向和实际运动方向一致。
- 电池、电机、轮胎和场地状态稳定。

### `cmd_dis` 调参顺序

1. 先完成 `Bsp` 层电机方向、轮速环、偏航环和里程计比例标定。
2. 使用较低 `MOVE_SPEED_MM_S` 验证目标坐标和运动方向。
3. 调整 `LINE_CROSS_KP`：增大可更快回到目标线，过大会蛇形振荡。
4. 用 `LINE_CROSS_MAX_MM_S` 限制最大横向纠偏速度。
5. 用 `LINE_CROSS_DEADBAND_MM` 忽略里程计的小幅横向噪声。
6. 调整速度曲线指数和 `PROFILE_MIN_SCALE`，最后再提高最高速度。
7. 根据实际制动距离调整完成进度、停车转速阈值和停稳时间。

### 速度曲线

三种曲线均以 `sin(progress * PI)` 为基础：

- `SHARP(0)`：指数小于 1，更早提升速度，也更晚降低速度。
- `NORMAL(1)`：原始正弦曲线。
- `SMOOTH(2)`：指数大于 1，起步和停车更缓。

`PROFILE_MIN_SCALE` 为曲线的最低比例。数值过小可能因静摩擦无法启动，过大则会增加终点超调。`MIN_SPEED_DISTANCE_MM` 只允许在距离终点较远时强制最低速度，避免接近目标仍被最小速度推动。

### 停稳与完成事件

四个底盘电机必须均在线，且反馈转速绝对值不超过 `STOP_RPM`，持续 `STOP_STABLE_MS` 后才算停稳。`cmd_dis` 到点后至少保持 `HOLD_AFTER_MOVE_MS`；若始终不能确认停稳，达到 `STOP_MAX_WAIT_MS` 后仍结束，避免状态机永久卡住。

`cmd_dis` 和 `cmd_turn` 接收成功先回复 `ok`，状态机完成后异步回复 `done`。新命令会覆盖当前活动命令，因此上位机不应并发发送多个需要 `done` 的运动命令。

## 学生常见误解

### `cmd_dis 10 0` 是移动到 x=10 吗？

不是。它表示“从当前位置开始，沿世界坐标 x 方向再移动 10 cm”。

### `cmd_turn 90` 是左转 90 度吗？

不一定。它表示“转到绝对 yaw=90 deg”。如果当前已经是 80 deg，只会再转到 90 deg。

### `cmd_dkmotor 50 0` 会自己停吗？

不会。它是持续运动命令，需要发送 `cmd_dkmotor 0 0`、`cmd_juststop` 或 `cmd_conmotion 0` 停止。

### 为什么直接输入 `cmd_dis 10 0` 没反应？

协议要求带 CRC，实际发送必须是 `cmd_dis 10 0 *CCCC\r\n`。`CCCC` 需要按 payload 计算。

## 缓冲区与实时性

- RX 环形缓冲区为 `APP_PI_COMM_RX_RING_SIZE` 字节；中断写指针追上读指针时，新字节会被静默丢弃。
- 单行最多 `APP_PI_COMM_LINE_SIZE - 1` 字符；超长行会清空当前行并回复 `err long`。
- 回复也受同一行长度限制，新增命令或参数时必须计算最坏格式化长度。
- 回复使用同步 UART 发送，密集查询会阻塞主循环并影响控制周期。上位机应限制查询频率。
- `cmd_xgred` 会擦写 Flash，不应作为周期命令调用。

## 维护说明

- 新增命令时，同步更新命令解析、回复字符串、CRC 测试和本文档命令表。
- 新增需要 `done` 的运动命令时，必须明确完成事件何时产生、何时取消、是否允许被覆盖。
- 修改坐标或单位约定时，必须同步上位机、树莓派和本文档。
- 修改 `APP_PI_COMM_LINE_SIZE` 后，检查所有回复的最坏长度。
