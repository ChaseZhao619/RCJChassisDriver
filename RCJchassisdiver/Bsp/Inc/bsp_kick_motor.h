#ifndef BSP_KICK_MOTOR_H
#define BSP_KICK_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define BSP_KICK_MOTOR_CAN_ID 5U

#ifndef BSP_KICK_MOTOR_TASK_PERIOD_MS
#define BSP_KICK_MOTOR_TASK_PERIOD_MS 10U
#endif

#ifndef BSP_KICK_MOTOR_MAX_RPM
#define BSP_KICK_MOTOR_MAX_RPM 5000.0f
#endif

#ifndef BSP_KICK_MOTOR_MAX_CURRENT
#define BSP_KICK_MOTOR_MAX_CURRENT 9000
#endif

#ifndef BSP_KICK_MOTOR_SPEED_KP
#define BSP_KICK_MOTOR_SPEED_KP 5.5f
#endif

#ifndef BSP_KICK_MOTOR_SPEED_KI
#define BSP_KICK_MOTOR_SPEED_KI 1.0f
#endif

#ifndef BSP_KICK_MOTOR_SPEED_KD
#define BSP_KICK_MOTOR_SPEED_KD 0.0f
#endif

#ifndef BSP_KICK_MOTOR_SPEED_KF
#define BSP_KICK_MOTOR_SPEED_KF 1.8f
#endif

#ifndef BSP_KICK_MOTOR_SPEED_I_LIMIT
#define BSP_KICK_MOTOR_SPEED_I_LIMIT 4000.0f
#endif

#ifndef BSP_KICK_MOTOR_DIR
#define BSP_KICK_MOTOR_DIR 1
#endif

#ifndef BSP_KICK_MOTOR_FB_DIR
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
