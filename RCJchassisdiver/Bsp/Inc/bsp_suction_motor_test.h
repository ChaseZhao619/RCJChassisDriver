#ifndef BSP_SUCTION_MOTOR_TEST_H
#define BSP_SUCTION_MOTOR_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#ifndef BSP_SUCTION_MOTOR_TEST_ENABLE
#define BSP_SUCTION_MOTOR_TEST_ENABLE 0U
#endif

#ifndef BSP_SUCTION_MOTOR_TEST_PRINT_USART
#define BSP_SUCTION_MOTOR_TEST_PRINT_USART BSP_USART_6
#endif

#ifndef BSP_SUCTION_MOTOR_TEST_INIT_HOLD_MS
#define BSP_SUCTION_MOTOR_TEST_INIT_HOLD_MS 3000U
#endif

#ifndef BSP_SUCTION_MOTOR_TEST_STEP_HOLD_MS
#define BSP_SUCTION_MOTOR_TEST_STEP_HOLD_MS 2000U
#endif

#ifndef BSP_SUCTION_MOTOR_TEST_MAX_PULSE_US
#define BSP_SUCTION_MOTOR_TEST_MAX_PULSE_US 1200U
#endif

void BspSuctionMotorTest_Init(void);
void BspSuctionMotorTest_Task(void);

#ifdef __cplusplus
}
#endif

#endif
