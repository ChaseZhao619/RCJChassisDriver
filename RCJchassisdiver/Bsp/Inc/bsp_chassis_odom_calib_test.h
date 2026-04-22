#ifndef BSP_CHASSIS_ODOM_CALIB_TEST_H
#define BSP_CHASSIS_ODOM_CALIB_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#ifndef BSP_CHASSIS_ODOM_CALIB_DISTANCE_MM
#define BSP_CHASSIS_ODOM_CALIB_DISTANCE_MM 600.0f
#endif

#ifndef BSP_CHASSIS_ODOM_CALIB_MAX_SPEED_MM_S
#define BSP_CHASSIS_ODOM_CALIB_MAX_SPEED_MM_S 260.0f
#endif

#ifndef BSP_CHASSIS_ODOM_CALIB_MIN_SPEED_MM_S
#define BSP_CHASSIS_ODOM_CALIB_MIN_SPEED_MM_S 70.0f
#endif

#ifndef BSP_CHASSIS_ODOM_CALIB_START_DELAY_MS
#define BSP_CHASSIS_ODOM_CALIB_START_DELAY_MS 2000U
#endif

#ifndef BSP_CHASSIS_ODOM_CALIB_HOLD_MS
#define BSP_CHASSIS_ODOM_CALIB_HOLD_MS 3000U
#endif

#ifndef BSP_CHASSIS_ODOM_CALIB_MAX_RUN_MS
#define BSP_CHASSIS_ODOM_CALIB_MAX_RUN_MS 10000U
#endif

#ifndef BSP_CHASSIS_ODOM_CALIB_PRINT_ENABLE
#define BSP_CHASSIS_ODOM_CALIB_PRINT_ENABLE 0U
#endif

void BspChassisOdomCalibTest_Init(void);
void BspChassisOdomCalibTest_Task(uint8_t yaw_valid, float yaw_deg);

#ifdef __cplusplus
}
#endif

#endif
