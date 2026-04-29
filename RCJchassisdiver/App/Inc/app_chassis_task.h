#ifndef APP_CHASSIS_TASK_H
#define APP_CHASSIS_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

typedef enum
{
    APP_CHASSIS_TASK_DONE_NONE = 0,
    APP_CHASSIS_TASK_DONE_DIS,
    APP_CHASSIS_TASK_DONE_TURN,
} AppChassisTaskDoneEvent;

/* Reserved preset route distance, in mm. Currently not used by cmd_dis/cmd_turn.
 * 预留预设路线距离，单位 mm。当前不影响 cmd_dis/cmd_turn。 */
#ifndef APP_CHASSIS_TASK_FORWARD_1_DISTANCE_MM
#define APP_CHASSIS_TASK_FORWARD_1_DISTANCE_MM  450.0f
#endif

/* Reserved preset route distance, in mm. Currently not used by cmd_dis/cmd_turn.
 * 预留预设路线距离，单位 mm。当前不影响 cmd_dis/cmd_turn。 */
#ifndef APP_CHASSIS_TASK_RIGHT_DISTANCE_MM
#define APP_CHASSIS_TASK_RIGHT_DISTANCE_MM      400.0f
#endif

/* Reserved preset route distance, in mm. Currently not used by cmd_dis/cmd_turn.
 * 预留预设路线距离，单位 mm。当前不影响 cmd_dis/cmd_turn。 */
#ifndef APP_CHASSIS_TASK_LEFT_DISTANCE_MM
#define APP_CHASSIS_TASK_LEFT_DISTANCE_MM       900.0f
#endif

/* Reserved preset route distance, in mm. Currently not used by cmd_dis/cmd_turn.
 * 预留预设路线距离，单位 mm。当前不影响 cmd_dis/cmd_turn。 */
#ifndef APP_CHASSIS_TASK_FORWARD_2_DISTANCE_MM
#define APP_CHASSIS_TASK_FORWARD_2_DISTANCE_MM  1200.0f
#endif

/* Reserved preset clockwise turn angle, in deg. Currently not used by cmd_turn.
 * 预留预设顺时针转角，单位 deg。当前不影响 cmd_turn。 */
#ifndef APP_CHASSIS_TASK_CLOCKWISE_YAW_DEG
#define APP_CHASSIS_TASK_CLOCKWISE_YAW_DEG      35.0f
#endif

/* cmd_dis maximum profile speed, in mm/s. Increase for faster translation.
 * cmd_dis 速度曲线最高速度，单位 mm/s。增大会提高平移速度上限。 */
#ifndef APP_CHASSIS_TASK_MOVE_SPEED_MM_S
#define APP_CHASSIS_TASK_MOVE_SPEED_MM_S        350.0f
#endif

/* Reserved startup delay before preset motion, in ms. Currently not used.
 * 预留预设运动启动前延时，单位 ms。当前代码未使用。 */
#ifndef APP_CHASSIS_TASK_START_DELAY_MS
#define APP_CHASSIS_TASK_START_DELAY_MS         300U
#endif

/* Hold time after cmd_dis reaches target, in ms. Increase for a longer settle time.
 * cmd_dis 到点后的保持时间，单位 ms。增大会让到点后稳定保持更久。 */
#ifndef APP_CHASSIS_TASK_HOLD_AFTER_MOVE_MS
#define APP_CHASSIS_TASK_HOLD_AFTER_MOVE_MS     400U
#endif

/* Motor speed threshold used to judge stopped, in rpm. Increase to accept more residual motion.
 * 停车判定电机转速阈值，单位 rpm。增大会允许更大的残余转速仍视为已停稳。 */
#ifndef APP_CHASSIS_TASK_STOP_RPM
#define APP_CHASSIS_TASK_STOP_RPM               35
#endif

/* Required continuous stopped time, in ms. Increase for stricter stop confirmation.
 * 连续停稳判定时间，单位 ms。增大会让停稳确认更严格、更慢触发完成。 */
#ifndef APP_CHASSIS_TASK_STOP_STABLE_MS
#define APP_CHASSIS_TASK_STOP_STABLE_MS         250U
#endif

/* Maximum wait after cmd_dis reaches target, in ms. Increase to wait longer for full stop.
 * cmd_dis 到点后的最大等待时间，单位 ms。增大会更久等待电机完全停稳。 */
#ifndef APP_CHASSIS_TASK_STOP_MAX_WAIT_MS
#define APP_CHASSIS_TASK_STOP_MAX_WAIT_MS       1500U
#endif

/* Yaw tolerance for angle target, in deg. Increase to finish with looser heading accuracy.
 * 角度目标容差，单位 deg。增大会放宽车头角度到位判定。 */
#ifndef APP_CHASSIS_TASK_ROTATE_TOLERANCE_DEG
#define APP_CHASSIS_TASK_ROTATE_TOLERANCE_DEG   1.0f
#endif

/* Minimum speed scale near start/end of the profile. Increase for higher minimum speed.
 * 速度曲线起点/终点附近的最低速度比例。增大会提高最低速度，减小会让起停更柔和。 */
#ifndef APP_CHASSIS_TASK_PROFILE_MIN_SCALE
#define APP_CHASSIS_TASK_PROFILE_MIN_SCALE      0.08f
#endif

/* Command values for cmd_dis speed_profile. These are protocol values, not tuning gains.
 * cmd_dis speed_profile 的命令取值。这些是协议值，不是调参增益。 */
#define APP_CHASSIS_TASK_PROFILE_SHARP          0U
#define APP_CHASSIS_TASK_PROFILE_NORMAL         1U
#define APP_CHASSIS_TASK_PROFILE_SMOOTH         2U

/*
 * cmd_dis speed profile tuning / cmd_dis 速度曲线调参:
 *
 * - NORMAL keeps the original sin(progress * pi) curve.
 *   NORMAL 档保持原来的 sin(progress * pi) 速度曲线。
 *
 * - SHARP_EXP is used when speed_profile = 0.
 *   Smaller values make acceleration/deceleration more aggressive;
 *   values closer to 1.0f make it closer to NORMAL.
 *   SHARP_EXP 用于 speed_profile = 0。
 *   数值越小，加减速越急；越接近 1.0f，越接近 NORMAL 档。
 *
 * - SMOOTH_EXP is used when speed_profile = 2.
 *   Larger values make acceleration/deceleration gentler;
 *   values closer to 1.0f make it closer to NORMAL.
 *   SMOOTH_EXP 用于 speed_profile = 2。
 *   数值越大，加减速越缓；越接近 1.0f，越接近 NORMAL 档。
 *
 * - PROFILE_MIN_SCALE limits the lowest speed scale near start/end.
 *   Increase it for a higher minimum speed, decrease it for softer start/stop.
 *   PROFILE_MIN_SCALE 限制起点和终点附近的最低速度比例。
 *   增大它会提高最低速度，减小它会让起停更柔和。
 */
/* Exponent for the sharp profile. Smaller than 1.0f means more aggressive.
 * 急档速度曲线指数。小于 1.0f，数值越小加减速越急。 */
#ifndef APP_CHASSIS_TASK_PROFILE_SHARP_EXP
#define APP_CHASSIS_TASK_PROFILE_SHARP_EXP      0.65f
#endif

/* Exponent for the smooth profile. Larger than 1.0f means gentler.
 * 缓档速度曲线指数。大于 1.0f，数值越大加减速越缓。 */
#ifndef APP_CHASSIS_TASK_PROFILE_SMOOTH_EXP
#define APP_CHASSIS_TASK_PROFILE_SMOOTH_EXP     1.75f
#endif

/* Segment progress that marks cmd_dis complete. Increase to require going closer to target.
 * cmd_dis 线段完成进度阈值。增大会要求更接近目标点才判定完成。 */
#ifndef APP_CHASSIS_TASK_SEGMENT_DONE_PROGRESS
#define APP_CHASSIS_TASK_SEGMENT_DONE_PROGRESS  0.990f
#endif

/* Cross-track correction gain. Increase for stronger straight-line correction.
 * 直线横向纠偏比例增益。增大会让路线纠偏更强，但过大可能抖动。 */
#ifndef APP_CHASSIS_TASK_LINE_CROSS_KP
#define APP_CHASSIS_TASK_LINE_CROSS_KP          2.2f
#endif

/* Cross-track correction speed limit, in mm/s. Increase to allow stronger lateral correction.
 * 横向纠偏速度上限，单位 mm/s。增大会允许更大的侧向纠偏速度。 */
#ifndef APP_CHASSIS_TASK_LINE_CROSS_MAX_MM_S
#define APP_CHASSIS_TASK_LINE_CROSS_MAX_MM_S    180.0f
#endif

/* Cross-track deadband, in mm. Increase to ignore small line errors.
 * 横向偏差死区，单位 mm。增大会忽略更大的轻微偏线。 */
#ifndef APP_CHASSIS_TASK_LINE_CROSS_DEADBAND_MM
#define APP_CHASSIS_TASK_LINE_CROSS_DEADBAND_MM 8.0f
#endif

/* Distance where minimum speed clamp is allowed, in mm. Increase to keep minimum speed for longer.
 * 允许最低速度钳位的剩余距离阈值，单位 mm。增大会让最低速度保持到更接近终点。 */
#ifndef APP_CHASSIS_TASK_MIN_SPEED_DISTANCE_MM
#define APP_CHASSIS_TASK_MIN_SPEED_DISTANCE_MM  220.0f
#endif

/* cmd_dkmotor 100 percent speed, in mm/s. Increase to make cmd_dkmotor faster.
 * cmd_dkmotor 100% 对应速度，单位 mm/s。增大会提高持续运动命令的速度上限。 */
#ifndef APP_CHASSIS_TASK_DKMOTOR_MAX_SPEED_MM_S
#define APP_CHASSIS_TASK_DKMOTOR_MAX_SPEED_MM_S 650.0f
#endif

void AppChassisTask_Init(void);
HAL_StatusTypeDef AppChassisTask_SetMotionEnabled(uint8_t enabled);
uint8_t AppChassisTask_IsMotionEnabled(void);
HAL_StatusTypeDef AppChassisTask_CommandJustStop(void);
HAL_StatusTypeDef AppChassisTask_CommandDistanceCm(float x_cm,
                                                  float y_cm,
                                                  uint8_t speed_profile);
HAL_StatusTypeDef AppChassisTask_CommandTurnDeg(float target_yaw_deg);
HAL_StatusTypeDef AppChassisTask_CommandDkMotor(uint8_t speed_percent,
                                                float move_angle_deg,
                                                uint8_t head_lock);
HAL_StatusTypeDef AppChassisTask_GetRequestDelta(float *dx_cm,
                                                 float *dy_cm,
                                                 float *dyaw_deg,
                                                 float *yaw_deg);
AppChassisTaskDoneEvent AppChassisTask_ConsumeDoneEvent(void);
void AppChassisTask_OnYawZero(float yaw_deg);
void AppChassisTask_Task(uint8_t yaw_valid,
                         float yaw_deg,
                         uint8_t gyro_valid,
                         float gyro_z_deg_s);

#ifdef __cplusplus
}
#endif

#endif
