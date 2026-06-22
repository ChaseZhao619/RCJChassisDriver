#ifndef BSP_CHASSIS_ODOM_H
#define BSP_CHASSIS_ODOM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#ifndef BSP_CHASSIS_ODOM_WHEEL_DIAMETER_MM
/* 驱动轮有效直径 [mm]。应以带载滚动周长/PI 标定，而非仅测空载外径。 */
#define BSP_CHASSIS_ODOM_WHEEL_DIAMETER_MM      46.6f
#endif

#ifndef BSP_CHASSIS_ODOM_MOTOR_GEAR_RATIO
/* 电机转数/车轮转数。填写减速器总传动比，错误会按比例影响全部距离。 */
#define BSP_CHASSIS_ODOM_MOTOR_GEAR_RATIO       36.0f
#endif

#ifndef BSP_CHASSIS_ODOM_POS_KP
/* 位置误差到平移速度的比例增益 [(mm/s)/mm]；过大会冲过目标点。 */
#define BSP_CHASSIS_ODOM_POS_KP                 3.0f
#endif

#ifndef BSP_CHASSIS_ODOM_POS_TOLERANCE_MM
/* 默认到点距离容差 [mm]；应大于系统静止抖动和单周期位移。 */
#define BSP_CHASSIS_ODOM_POS_TOLERANCE_MM       5.0f
#endif

#ifndef BSP_CHASSIS_ODOM_MIN_SPEED_MM_S
/* 非零位置误差时的最小平移速度 [mm/s]；克服静摩擦，但过大会降低到点精度。 */
#define BSP_CHASSIS_ODOM_MIN_SPEED_MM_S         80.0f
#endif

#ifndef BSP_CHASSIS_ODOM_DEFAULT_MAX_SPEED_MM_S
/* 调用者未给出有效上限时采用的默认平移速度 [mm/s]。 */
#define BSP_CHASSIS_ODOM_DEFAULT_MAX_SPEED_MM_S 350.0f
#endif

#ifndef BSP_CHASSIS_ODOM_MAX_CURRENT
/* 里程计位置控制默认电流控制量上限；调试阶段应从较低值开始。 */
#define BSP_CHASSIS_ODOM_MAX_CURRENT            4000
#endif

#ifndef BSP_CHASSIS_ODOM_FORWARD_SCALE
/* 前后里程比例修正：积分速度乘以该值；新值=旧值*实际距离/记录距离。 */
#define BSP_CHASSIS_ODOM_FORWARD_SCALE          1.28f
#endif

#ifndef BSP_CHASSIS_ODOM_LEFT_SCALE
/* 左右里程比例修正：积分速度乘以该值；需与前后方向分别标定。 */
#define BSP_CHASSIS_ODOM_LEFT_SCALE             1.36f
#endif

typedef struct
{
    /* 世界坐标位置 [mm]，yaw 为偏航角 [deg]。 */
    float x_mm;
    float y_mm;
    float yaw_deg;
    float vx_mm_s;              /* 世界坐标 X 速度 [mm/s]。 */
    float vy_mm_s;              /* 世界坐标 Y 速度 [mm/s]。 */
    float body_forward_mm_s;    /* 车体前向速度 [mm/s]。 */
    float body_left_mm_s;       /* 车体左向速度 [mm/s]。 */
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
