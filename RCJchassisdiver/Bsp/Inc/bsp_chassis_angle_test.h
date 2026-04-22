#ifndef BSP_CHASSIS_ANGLE_TEST_H
#define BSP_CHASSIS_ANGLE_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#ifndef BSP_CHASSIS_ANGLE_TEST_ENABLE
#define BSP_CHASSIS_ANGLE_TEST_ENABLE 1U
#endif

#ifndef BSP_CHASSIS_ANGLE_TEST_MAX_CURRENT
#define BSP_CHASSIS_ANGLE_TEST_MAX_CURRENT 10000
#endif

#ifndef BSP_CHASSIS_ANGLE_TEST_PRINT_MS
#define BSP_CHASSIS_ANGLE_TEST_PRINT_MS 50U
#endif

void BspChassisAngleTest_Init(void);
void BspChassisAngleTest_Task(uint8_t yaw_valid, float yaw_deg);

#ifdef __cplusplus
}
#endif

#endif
