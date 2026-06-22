#ifndef BSP_KICK_MOTOR_H
#define BSP_KICK_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* 功能电机 CAN ID。必须与电调拨码一致，且不能和底盘 1~4 号冲突。 */
#define BSP_KICK_MOTOR_CAN_ID 5U

#ifndef BSP_KICK_MOTOR_TASK_PERIOD_MS
/* 速度环期望调用周期 [ms]；实际 Task 调用频率应不慢于此值。 */
#define BSP_KICK_MOTOR_TASK_PERIOD_MS 10U
#endif

#ifndef BSP_KICK_MOTOR_MAX_RPM
/* 百分比命令映射到的最大目标转速 [rpm]，不是电机机械安全值的自动保证。 */
#define BSP_KICK_MOTOR_MAX_RPM 5000.0f
#endif

#ifndef BSP_KICK_MOTOR_MAX_CURRENT
/* 功能电机电流控制量绝对值上限；需结合电机、电调、机构和温升确定。 */
#define BSP_KICK_MOTOR_MAX_CURRENT 9000
#endif

#ifndef BSP_KICK_MOTOR_SPEED_KP
/* 速度比例增益：增大可改善负载响应，过大会抖动或啸叫。 */
#define BSP_KICK_MOTOR_SPEED_KP 5.5f
#endif

#ifndef BSP_KICK_MOTOR_SPEED_KI
/* 速度积分增益：消除稳态误差，过大会造成饱和后长时间反向恢复。 */
#define BSP_KICK_MOTOR_SPEED_KI 1.0f
#endif

#ifndef BSP_KICK_MOTOR_SPEED_KD
/* 速度微分增益：反馈噪声会被放大，无明确需求时保持为 0。 */
#define BSP_KICK_MOTOR_SPEED_KD 0.0f
#endif

#ifndef BSP_KICK_MOTOR_SPEED_KF
/* 目标转速前馈增益 [电流控制量/rpm]，先于 PI 调整基础匀速输出。 */
#define BSP_KICK_MOTOR_SPEED_KF 1.8f
#endif

#ifndef BSP_KICK_MOTOR_SPEED_I_LIMIT
/* 积分状态绝对值上限，用于抑制堵转或限流时的积分累积。 */
#define BSP_KICK_MOTOR_SPEED_I_LIMIT 4000.0f
#endif

#ifndef BSP_KICK_MOTOR_DIR
/* 输出命令方向，只允许 +1/-1。 */
#define BSP_KICK_MOTOR_DIR 1
#endif

#ifndef BSP_KICK_MOTOR_FB_DIR
/* 转速反馈方向，只允许 +1/-1；必须与逻辑目标方向一致。 */
#define BSP_KICK_MOTOR_FB_DIR 1
#endif

HAL_StatusTypeDef BspKickMotor_Init(void);
HAL_StatusTypeDef BspKickMotor_SetSpeed(uint8_t speed_percent, uint8_t reverse);
HAL_StatusTypeDef BspKickMotor_Stop(void);
void BspKickMotor_Task(void);
uint8_t BspKickMotor_GetSpeedPercent(void);
uint8_t BspKickMotor_GetReverse(void);
float BspKickMotor_GetTargetRpm(void);
int16_t BspKickMotor_GetLastCurrent(void);

#ifdef __cplusplus
}
#endif

#endif
