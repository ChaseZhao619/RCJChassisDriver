# 离散 LQR 底盘实验分支

本页记录 `codex/lqr-soccer-sim` 的理论模型与主机仿真。它不是实车调参结果；未经过轮径、负载、摩擦、IMU 延迟、CAN 时延和场地打滑标定，不应直接在落地机器人上使用。

## 控制层级

`cmd_dis`、`cmd_turn` 和航向保持改用世界坐标下的离散 LQR 外环，输出车体平移与旋转速度目标。`cmd_vel` 与 `cmd_dkmotor` 是持续目标：LQR 的速度状态反馈负责追踪目标，四个轮子的 PID+前馈内环及 CAN 电流控制保持不变。速度上限和故障停车在 LQR 之外执行。

## 名义模型与增益

每个平移轴和航向轴分别采用状态 $[p,v]^T$ 与速度命令 $u$：

$$\dot p=v,\qquad \dot v=(u-v)/\tau.$$

零阶保持离散化，令 $a=e^{-T/\tau}$，得到：

$$x_{k+1}=Ax_k+Bu_k,\quad
A=\begin{bmatrix}1&\tau(1-a)\\0&a\end{bmatrix},\quad
B=\begin{bmatrix}T-\tau(1-a)\\1-a\end{bmatrix}.$$

`T=0.01 s`；平移 `tau=0.12 s`，航向 `tau=0.10 s`。代价为 $\sum_k(x_k^TQx_k+u_k^TRu_k)$，平移 $Q=\mathrm{diag}(25,1)$、航向 $Q=\mathrm{diag}(16,1)$、两者 $R=1$。`tools/lqr_gain.py` 用离散 Riccati 迭代离线求增益；固件使用写入 `app_lqr.c` 的常量。单位分别为米、米每秒、弧度、弧度每秒；串口的平移速度以 `mm/s` 表示，转换只发生在状态机边界。输出再按四轮全向运动学的最大轮缘速度预算同比缩放，保留平移与旋转指令的比例。

控制律 $u=v_{ref}-K_p(p-p_{ref})-K_v(v-v_{ref})$。位置模式跟踪 $p_{ref}$ 且 $v_{ref}=0$；持续速度模式关闭位置误差，仅跟踪 $v_{ref}$。航向误差先归一化到 $[-\pi,\pi)$。这是一阶名义模型，不包含轮胎侧滑、齿轮间隙或电池电压变化，因此仿真优于旧外环并不代表实车也会如此。

## 协议与故障

- `cmd_vel <forward_mm_s> <left_mm_s> <ccw_rad_s>`：车体坐标速度，合成平移速度不超过 `650 mm/s`，角速度绝对值不超过 `2 rad/s`。全零命令停止持续模式。
- `cmd_vel` 和 `cmd_dkmotor` 每隔不超过 `300 ms` 必须收到有效续发帧；桌面端每 `100 ms` 续发。不接受没有 CRC 的命令。
- `cmd_ctrlstat` 返回最近控制周期毫秒数、超过 15 ms 的周期计数、运动许可位。目标周期为 10 ms；若周期超过 50 ms，进入故障停车。主循环中仍有阻塞式 I2C/UART，目标周期不等于硬实时保证。
- IMU yaw/gyro、任一电机反馈过期或 CAN 发送失败会停车并锁定运动许可。排除故障、恢复有效反馈后，发送 `cmd_conmotion 1` 重新使能；原命令不会自动恢复。
- 缺失视觉观测不能由 BE1732 最强通道补成二维球位置。仿真使用合成二维观测，短时丢帧按匀速预测，观测失效超过 0.5 s 后停止追球。

## 主机验证

无需 Qt 或硬件，可运行：

```sh
cmake -S . -B build/control-tests -G Ninja -DRCJ_BUILD_STM32=OFF -DRCJ_BUILD_HOST_APP=OFF -DRCJ_BUILD_CONTROL_TESTS=ON
cmake --build build/control-tests
ctest --test-dir build/control-tests --output-on-failure
```

`build/control-tests/tests/control_trace.csv` 包含原位置 P 外环与 LQR 的名义模型轨迹，以及静止球、移动球、暂时丢帧、目标突变和命令链路丢失的追球轨迹。比较仅在同一个合成模型下成立。实车部署之前仍需辨识每轴时间常数、核对正负方向、测量真实控制周期，并在架空、低电流和独立急停条件下逐步验证。
