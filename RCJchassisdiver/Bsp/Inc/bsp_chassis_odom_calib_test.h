#ifndef BSP_CHASSIS_ODOM_CALIB_TEST_H
#define BSP_CHASSIS_ODOM_CALIB_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#ifndef BSP_CHASSIS_ODOM_CALIB_DISTANCE_MM
/* 单段标定目标距离 [mm]；距离越长比例误差越易测量，但需预留制动空间。 */
#define BSP_CHASSIS_ODOM_CALIB_DISTANCE_MM 600.0f
#endif

#ifndef BSP_CHASSIS_ODOM_CALIB_MAX_SPEED_MM_S
/* 标定巡航速度上限 [mm/s]；打滑时应降低。 */
#define BSP_CHASSIS_ODOM_CALIB_MAX_SPEED_MM_S 260.0f
#endif

#ifndef BSP_CHASSIS_ODOM_CALIB_MIN_SPEED_MM_S
/* 接近终点时的最低速度 [mm/s]；过高会造成明显越界。 */
#define BSP_CHASSIS_ODOM_CALIB_MIN_SPEED_MM_S 70.0f
#endif

#ifndef BSP_CHASSIS_ODOM_CALIB_START_DELAY_MS
/* 上电到动作开始的安全等待 [ms]。 */
#define BSP_CHASSIS_ODOM_CALIB_START_DELAY_MS 2000U
#endif

#ifndef BSP_CHASSIS_ODOM_CALIB_HOLD_MS
/* 每段完成后的静止保持时间 [ms]，用于人工测量和机械稳定。 */
#define BSP_CHASSIS_ODOM_CALIB_HOLD_MS 3000U
#endif

#ifndef BSP_CHASSIS_ODOM_CALIB_MAX_RUN_MS
/* 单段最大运行时间 [ms]，防止反馈异常时持续驱动。 */
#define BSP_CHASSIS_ODOM_CALIB_MAX_RUN_MS 10000U
#endif

#ifndef BSP_CHASSIS_ODOM_CALIB_PRINT_ENABLE
/* 输出标定位姿和阶段信息；高频同步打印可能扰动控制周期。 */
#define BSP_CHASSIS_ODOM_CALIB_PRINT_ENABLE 0U
#endif

void BspChassisOdomCalibTest_Init(void);
void BspChassisOdomCalibTest_Task(uint8_t yaw_valid, float yaw_deg);

#ifdef __cplusplus
}
#endif

#endif
