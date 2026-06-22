# App 模块说明与调参指南

`App` 层负责把上位机命令转换为底盘及执行器动作。它不直接操作寄存器：运动控制由 `app_chassis_task` 编排，串口协议由 `app_pi_comm` 解析，具体硬件动作交给 `Bsp` 层。

## 任务调用关系

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

`AppPiComm_Task()` 和 `AppChassisTask_Task()` 都必须在主循环中持续调用。串口中断只收取单字节并写入环形缓冲区，不在中断中解析命令或驱动执行器。

## 坐标、单位与命令语义

- 世界坐标 `x/y` 与 `BspChassisOdom` 一致，距离命令参数使用 `cm`，进入里程计前乘以 10 转为 `mm`。
- `yaw` 和移动方向使用 `deg`；底盘约定 `0 deg` 为前、`90 deg` 为左、逆时针为正。
- `cmd_dis x y [profile]` 是相对当前世界坐标的位移，不是绝对目标点。
- `cmd_turn yaw` 的参数是绝对偏航目标，内部归一化到 `[0, 360)`。
- `cmd_dkmotor speed angle [head_lock]` 是持续运动命令，直到速度设为 0、收到停止命令或运动总开关关闭。
- `cmd_request` 返回从上次请求到当前的增量 `dx dy dyaw` 和当前绝对 `yaw`；第一次请求的三个增量均为 0。

## 底盘状态机

| 状态 | 行为 | 退出条件 |
| --- | --- | --- |
| `WAIT_IMU` | 电机停止，等待有效偏航角 | 收到有效 yaw 后初始化里程计 |
| `IDLE` | 保持最后目标航向 | 收到有效运动命令 |
| `MOVE` | 按速度曲线沿线段运动并纠正横向误差 | 到达位置/进度阈值且完成停稳确认 |
| `TURN` | 原地保持目标偏航角 | 角度进入容差且电机持续停稳 |
| `DKMOTOR` | 持续方向运动，可选锁定车头 | 新命令、速度 0 或停止/禁用命令 |

IMU yaw 无效时，状态机立即回到 `WAIT_IMU`、取消活动命令并发送停车指令。`cmd_conmotion 0` 会取消活动命令和待发送完成事件，不会为被取消的命令发送 `done`。

## `cmd_dis` 调参顺序

1. 先完成 `Bsp` 层电机方向、轮速环、偏航环和里程计比例标定。
2. 使用较低 `MOVE_SPEED_MM_S` 验证目标坐标和运动方向。
3. 调整 `LINE_CROSS_KP`：增大可更快回到目标线，过大会蛇形振荡。
4. 用 `LINE_CROSS_MAX_MM_S` 限制最大横向纠偏速度；大偏差时动作过猛应降低。
5. 用 `LINE_CROSS_DEADBAND_MM` 忽略里程计的小幅横向噪声；过大会留下稳定路径偏差。
6. 调整速度曲线指数和 `PROFILE_MIN_SCALE`，最后再提高最高速度。
7. 根据实际制动距离调整完成进度、停车转速阈值和停稳时间。

### 速度曲线

三种曲线均以 `sin(progress * PI)` 为基础：

- `SHARP(0)`：指数小于 1，更早提升速度，也更晚降低速度。
- `NORMAL(1)`：原始正弦曲线。
- `SMOOTH(2)`：指数大于 1，起步和停车更缓。

`PROFILE_MIN_SCALE` 为曲线的最低比例。数值过小可能因静摩擦无法启动，过大则会增加终点超调。`MIN_SPEED_DISTANCE_MM` 只允许在距离终点较远时强制最低速度，避免接近目标仍被最小速度推动。

## 停稳与完成事件

四个底盘电机必须均在线，且反馈转速绝对值不超过 `STOP_RPM`，持续 `STOP_STABLE_MS` 后才算停稳。`cmd_dis` 到点后至少保持 `HOLD_AFTER_MOVE_MS`；若始终不能确认停稳，达到 `STOP_MAX_WAIT_MS` 后仍结束，以避免状态机永久卡住。

`cmd_dis` 和 `cmd_turn` 接收成功先回复 `ok`，状态机完成后异步回复 `done`。新命令会覆盖当前活动命令，因此上位机不应并发发送多个需要 `done` 的运动命令。

## 串口协议

每帧是一行 ASCII 文本：

```text
payload *CCCC\r\n
```

- `CCCC` 为 4 位十六进制 CRC-16/CCITT-FALSE。
- 初值 `0xFFFF`，多项式 `0x1021`，不反射，无最终异或。
- CRC 覆盖 `payload` 本身，不包含 CRC 前的分隔空格、`*CCCC` 或换行。
- 接收时会裁剪 payload 末尾空白，因此发送端应使用规范格式避免歧义。

### 命令表

| 命令 payload | 参数 | 作用 |
| --- | --- | --- |
| `cmd_dis x y [profile]` | `x/y: cm`, `profile: 0..2` | 相对位移 |
| `cmd_turn yaw` | `yaw: deg` | 转到绝对偏航角 |
| `cmd_dkmotor speed angle [head_lock]` | `speed: 0..100`, `angle: deg`, `head_lock: 0/1` | 持续运动 |
| `cmd_juststop` | 无 | 停止当前底盘命令 |
| `cmd_conmotion enabled` | `0/1` | 禁用/启用全部底盘运动 |
| `cmd_request` | 无 | 查询里程增量和当前 yaw |
| `cmd_suck speed` | `0..100` | 吸球电机速度百分比 |
| `cmd_tqdj speed reverse` | `speed: 0..100`, `reverse: 0/1` | 功能/踢球电机 |
| `cmd_xqcx` | 无 | 查询数字有球检测 |
| `cmd_redzhi` | 无 | 查询 BE1732 最强信号值 |
| `cmd_xgred threshold` | `0..255` | 设置并持久化无球阈值 |
| `cmd_infred` | 无 | 查询滤波后的红外通道 |
| `cmd_infred_mode pt/tz` | 普通/调制模式 | 切换 BE1732 模式 |
| `cmd_dct enabled` | `0/1` | 控制 DCT 输出 |
| `cmd_anglecal` | 无 | 将当前姿态设为 yaw 零点 |
| `cmd_mcureset` | 无 | 回复后复位 MCU |

成功、忙、完成和错误回复也使用相同 CRC 封装。代码沿用协议中的 `eror` 拼写，上位机必须按实际字符串兼容。

## 缓冲区与实时性

- RX 环形缓冲区为 `APP_PI_COMM_RX_RING_SIZE` 字节；中断写指针追上读指针时，新字节会被静默丢弃。
- 单行最多 `APP_PI_COMM_LINE_SIZE - 1` 字符；超长行会清空当前行并回复 `err long`。
- 回复也受同一行长度限制，新增命令或参数时必须计算最坏格式化长度。
- 回复使用同步 UART 发送，密集查询会阻塞主循环并影响控制周期。上位机应限制查询频率。
- `cmd_xgred` 会擦写 Flash，不应作为周期命令调用。

## 安全调试清单

1. 架空底盘并确认 `cmd_conmotion 0` 能立即停止运动输出。
2. 低速验证 `cmd_dis` 的世界坐标方向和 `cmd_turn` 的旋转正方向。
3. 验证 IMU 断连时底盘停车，恢复后不会自动继续已取消命令。
4. 验证坏 CRC、缺参数、越界参数和超长行不会触发执行器。
5. 落地测试前设置合理电流、速度上限并保留独立硬件急停。
