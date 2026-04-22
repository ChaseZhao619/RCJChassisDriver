#ifndef BSP_CHASSIS_ODOM_TEST_H
#define BSP_CHASSIS_ODOM_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#ifndef BSP_CHASSIS_ODOM_TEST_ENABLE
#define BSP_CHASSIS_ODOM_TEST_ENABLE 1U
#endif

#ifndef BSP_CHASSIS_ODOM_TEST_PRINT_ENABLE
#define BSP_CHASSIS_ODOM_TEST_PRINT_ENABLE 0U
#endif

#ifndef BSP_CHASSIS_ODOM_TEST_YAW_TOLERANCE_DEG
#define BSP_CHASSIS_ODOM_TEST_YAW_TOLERANCE_DEG 3.0f
#endif

void BspChassisOdomTest_Init(void);
void BspChassisOdomTest_Task(uint8_t yaw_valid, float yaw_deg);

#ifdef __cplusplus
}
#endif

#endif
