#ifndef BSP_SUCTION_MOTOR_H
#define BSP_SUCTION_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define BSP_SUCTION_MOTOR_INIT_MIN_US   800U
#define BSP_SUCTION_MOTOR_INIT_MAX_US   1050U
#define BSP_SUCTION_MOTOR_RUN_MIN_US    1050U
#define BSP_SUCTION_MOTOR_RUN_MAX_US    2000U

#ifndef BSP_SUCTION_MOTOR_INIT_PULSE_US
#define BSP_SUCTION_MOTOR_INIT_PULSE_US 1000U
#endif

HAL_StatusTypeDef BspSuctionMotor_Init(void);
HAL_StatusTypeDef BspSuctionMotor_SetInitPulse(void);
HAL_StatusTypeDef BspSuctionMotor_SetPulseUs(uint16_t pulse_us);
HAL_StatusTypeDef BspSuctionMotor_SetThrottlePermille(uint16_t throttle_permille);
HAL_StatusTypeDef BspSuctionMotor_SetSpeedPercent(uint8_t speed_percent);
uint16_t BspSuctionMotor_GetPulseUs(void);
uint8_t BspSuctionMotor_GetSpeedPercent(void);

#ifdef __cplusplus
}
#endif

#endif
