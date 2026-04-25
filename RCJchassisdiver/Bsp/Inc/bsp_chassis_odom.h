#ifndef BSP_CHASSIS_ODOM_H
#define BSP_CHASSIS_ODOM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#ifndef BSP_CHASSIS_ODOM_WHEEL_DIAMETER_MM
#define BSP_CHASSIS_ODOM_WHEEL_DIAMETER_MM      46.6f
#endif

#ifndef BSP_CHASSIS_ODOM_MOTOR_GEAR_RATIO
#define BSP_CHASSIS_ODOM_MOTOR_GEAR_RATIO       36.0f
#endif

#ifndef BSP_CHASSIS_ODOM_POS_KP
#define BSP_CHASSIS_ODOM_POS_KP                 3.0f
#endif

#ifndef BSP_CHASSIS_ODOM_POS_TOLERANCE_MM
#define BSP_CHASSIS_ODOM_POS_TOLERANCE_MM       15.0f
#endif

#ifndef BSP_CHASSIS_ODOM_MIN_SPEED_MM_S
#define BSP_CHASSIS_ODOM_MIN_SPEED_MM_S         80.0f
#endif

#ifndef BSP_CHASSIS_ODOM_DEFAULT_MAX_SPEED_MM_S
#define BSP_CHASSIS_ODOM_DEFAULT_MAX_SPEED_MM_S 350.0f
#endif

#ifndef BSP_CHASSIS_ODOM_MAX_CURRENT
#define BSP_CHASSIS_ODOM_MAX_CURRENT            4000
#endif

#ifndef BSP_CHASSIS_ODOM_FORWARD_SCALE
#define BSP_CHASSIS_ODOM_FORWARD_SCALE          1.28f
#endif

#ifndef BSP_CHASSIS_ODOM_LEFT_SCALE
#define BSP_CHASSIS_ODOM_LEFT_SCALE             1.36f
#endif

typedef struct
{
    float x_mm;
    float y_mm;
    float yaw_deg;
    float vx_mm_s;
    float vy_mm_s;
    float body_forward_mm_s;
    float body_left_mm_s;
} BspChassisOdomPose;

typedef struct
{
    float x_mm;
    float y_mm;
    float yaw_deg;
    float max_speed_mm_s;
    uint32_t hold_ms;
} BspChassisOdomWaypoint;

void BspChassisOdom_Init(float yaw_deg);
void BspChassisOdom_Reset(float x_mm, float y_mm, float yaw_deg);
void BspChassisOdom_Update(float yaw_deg);
HAL_StatusTypeDef BspChassisOdom_DriveTo(float target_x_mm,
                                         float target_y_mm,
                                         float target_yaw_deg,
                                         float max_speed_mm_s,
                                         int16_t max_current);
HAL_StatusTypeDef BspChassisOdom_DriveToGyro(float target_x_mm,
                                             float target_y_mm,
                                             float target_yaw_deg,
                                             float gyro_z_deg_s,
                                             float max_speed_mm_s,
                                             int16_t max_current);
uint8_t BspChassisOdom_IsAt(float target_x_mm, float target_y_mm, float tolerance_mm);
float BspChassisOdom_MmSToMotorRpm(float speed_mm_s);
float BspChassisOdom_MotorRpmToMmS(float motor_rpm);
const BspChassisOdomPose *BspChassisOdom_GetPose(void);

#ifdef __cplusplus
}
#endif

#endif
