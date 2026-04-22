#ifndef APP_CHASSIS_TASK_H
#define APP_CHASSIS_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#ifndef APP_CHASSIS_TASK_FORWARD_1_DISTANCE_MM
#define APP_CHASSIS_TASK_FORWARD_1_DISTANCE_MM  450.0f
#endif

#ifndef APP_CHASSIS_TASK_RIGHT_DISTANCE_MM
#define APP_CHASSIS_TASK_RIGHT_DISTANCE_MM      400.0f
#endif

#ifndef APP_CHASSIS_TASK_LEFT_DISTANCE_MM
#define APP_CHASSIS_TASK_LEFT_DISTANCE_MM       900.0f
#endif

#ifndef APP_CHASSIS_TASK_FORWARD_2_DISTANCE_MM
#define APP_CHASSIS_TASK_FORWARD_2_DISTANCE_MM  1200.0f
#endif

#ifndef APP_CHASSIS_TASK_CLOCKWISE_YAW_DEG
#define APP_CHASSIS_TASK_CLOCKWISE_YAW_DEG      35.0f
#endif

#ifndef APP_CHASSIS_TASK_MOVE_SPEED_MM_S
#define APP_CHASSIS_TASK_MOVE_SPEED_MM_S        350.0f
#endif

#ifndef APP_CHASSIS_TASK_START_DELAY_MS
#define APP_CHASSIS_TASK_START_DELAY_MS         300U
#endif

#ifndef APP_CHASSIS_TASK_HOLD_AFTER_MOVE_MS
#define APP_CHASSIS_TASK_HOLD_AFTER_MOVE_MS     400U
#endif

#ifndef APP_CHASSIS_TASK_STOP_RPM
#define APP_CHASSIS_TASK_STOP_RPM               35
#endif

#ifndef APP_CHASSIS_TASK_STOP_STABLE_MS
#define APP_CHASSIS_TASK_STOP_STABLE_MS         250U
#endif

#ifndef APP_CHASSIS_TASK_ROTATE_TOLERANCE_DEG
#define APP_CHASSIS_TASK_ROTATE_TOLERANCE_DEG   1.0f
#endif

void AppChassisTask_Init(void);
void AppChassisTask_Task(uint8_t yaw_valid,
                         float yaw_deg,
                         uint8_t gyro_valid,
                         float gyro_z_deg_s);

#ifdef __cplusplus
}
#endif

#endif
