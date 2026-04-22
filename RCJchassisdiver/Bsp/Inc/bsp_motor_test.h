#ifndef BSP_MOTOR_TEST_H
#define BSP_MOTOR_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#ifndef BSP_MOTOR_TEST_ENABLE
#define BSP_MOTOR_TEST_ENABLE 0U
#endif

#ifndef BSP_MOTOR_TEST_CURRENT
#define BSP_MOTOR_TEST_CURRENT 600
#endif

void BspMotorTest_Init(void);
void BspMotorTest_Task(void);

#ifdef __cplusplus
}
#endif

#endif
