#ifndef BSP_MOTOR_TEST_H
#define BSP_MOTOR_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#ifndef BSP_MOTOR_TEST_ENABLE
/* 置 1 后按测试序列驱动电机；正常运行必须置 0。 */
#define BSP_MOTOR_TEST_ENABLE 0U
#endif

#ifndef BSP_MOTOR_TEST_CURRENT
/* 单电机方向检查的电流控制量，宜从能可靠转动的最小值开始。 */
#define BSP_MOTOR_TEST_CURRENT 600
#endif

void BspMotorTest_Init(void);
void BspMotorTest_Task(void);

#ifdef __cplusplus
}
#endif

#endif
