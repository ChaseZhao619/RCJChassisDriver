#ifndef BSP_SUCTION_MOTOR_H
#define BSP_SUCTION_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/*
 * ESC PWM 脉宽边界 [us]。INIT 区间用于上电解锁，RUN 区间用于运行。
 * 这些值必须匹配具体电调说明书；扩大范围前先确认定时器周期和机构安全性。
 */
#define BSP_SUCTION_MOTOR_INIT_MIN_US   800U
#define BSP_SUCTION_MOTOR_INIT_MAX_US   1050U
#define BSP_SUCTION_MOTOR_RUN_MIN_US    1050U
#define BSP_SUCTION_MOTOR_RUN_MAX_US    2000U

#ifndef BSP_SUCTION_MOTOR_INIT_PULSE_US
/* 初始化/停机脉宽 [us]，必须位于 INIT_MIN_US~INIT_MAX_US。 */
#define BSP_SUCTION_MOTOR_INIT_PULSE_US 1000U
#endif

HAL_StatusTypeDef BspSuctionMotor_Init(void);
HAL_StatusTypeDef BspSuctionMotor_SetInitPulse(void);
HAL_StatusTypeDef BspSuctionMotor_SetPulseUs(uint16_t pulse_us);
/* 0~1000 线性映射到 RUN_MIN_US~RUN_MAX_US，超范围输入由实现限幅。 */
HAL_StatusTypeDef BspSuctionMotor_SetThrottlePermille(uint16_t throttle_permille);
/* 0~100 线性映射到运行脉宽；百分比不等同于实际转速百分比。 */
HAL_StatusTypeDef BspSuctionMotor_SetSpeedPercent(uint8_t speed_percent);
uint16_t BspSuctionMotor_GetPulseUs(void);
uint8_t BspSuctionMotor_GetSpeedPercent(void);

#ifdef __cplusplus
}
#endif

#endif
