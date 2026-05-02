#ifndef BSP_CHASSIS_H
#define BSP_CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define BSP_CHASSIS_DIAMETER_MM                 210.0f
#define BSP_CHASSIS_CONTACT_OPPOSITE_MM         192.0f
#define BSP_CHASSIS_CONTACT_ADJACENT_MM         140.0f
#define BSP_CHASSIS_ROTATION_RADIUS_MM          (BSP_CHASSIS_CONTACT_OPPOSITE_MM * 0.5f)

#ifndef BSP_CHASSIS_DEFAULT_MAX_CURRENT
#define BSP_CHASSIS_DEFAULT_MAX_CURRENT         3000
#endif

#ifndef BSP_CHASSIS_ANGLE_KP
#define BSP_CHASSIS_ANGLE_KP                    70.0f
#endif

#ifndef BSP_CHASSIS_ANGLE_KD
#define BSP_CHASSIS_ANGLE_KD                    0.0f
#endif

#ifndef BSP_CHASSIS_ANGLE_GYRO_KD
#define BSP_CHASSIS_ANGLE_GYRO_KD               10.0f
#endif

#ifndef BSP_CHASSIS_ANGLE_GYRO_DEADBAND_DPS
#define BSP_CHASSIS_ANGLE_GYRO_DEADBAND_DPS     1.5f
#endif

#ifndef BSP_CHASSIS_GYRO_Z_DIR
#define BSP_CHASSIS_GYRO_Z_DIR                  -1
#endif

#ifndef BSP_CHASSIS_ANGLE_DEADBAND_DEG
#define BSP_CHASSIS_ANGLE_DEADBAND_DEG          0.10f
#endif

#ifndef BSP_CHASSIS_ANGLE_MIN_RPM
#define BSP_CHASSIS_ANGLE_MIN_RPM               70.0f
#endif

#ifndef BSP_CHASSIS_ANGLE_MIN_RPM_FULL_ERR_DEG
#define BSP_CHASSIS_ANGLE_MIN_RPM_FULL_ERR_DEG  1.0f
#endif

#ifndef BSP_CHASSIS_ANGLE_MAX_RPM
#define BSP_CHASSIS_ANGLE_MAX_RPM               3000.0f
#endif

#ifndef BSP_CHASSIS_ANGLE_MAX_CURRENT
#define BSP_CHASSIS_ANGLE_MAX_CURRENT           500
#endif

#ifndef BSP_CHASSIS_WHEEL_SPEED_KP
#define BSP_CHASSIS_WHEEL_SPEED_KP              4.5f
#endif

#ifndef BSP_CHASSIS_WHEEL_SPEED_KI
#define BSP_CHASSIS_WHEEL_SPEED_KI              0.8f
#endif

#ifndef BSP_CHASSIS_WHEEL_SPEED_KD
#define BSP_CHASSIS_WHEEL_SPEED_KD              0.0f
#endif

#ifndef BSP_CHASSIS_WHEEL_SPEED_KF
#define BSP_CHASSIS_WHEEL_SPEED_KF              1.4f
#endif

#ifndef BSP_CHASSIS_WHEEL_SPEED_I_LIMIT
#define BSP_CHASSIS_WHEEL_SPEED_I_LIMIT         4000.0f
#endif

#ifndef BSP_CHASSIS_MOVE_FORWARD_BIAS_RPM
#define BSP_CHASSIS_MOVE_FORWARD_BIAS_RPM       0.0f
#endif

#ifndef BSP_CHASSIS_MOVE_LEFT_BIAS_RPM
#define BSP_CHASSIS_MOVE_LEFT_BIAS_RPM          0.0f
#endif

#ifndef BSP_CHASSIS_MOVE_BIAS_MIN_RPM
#define BSP_CHASSIS_MOVE_BIAS_MIN_RPM           50.0f
#endif

#ifndef BSP_CHASSIS_FORWARD_TO_LEFT_COMP
#define BSP_CHASSIS_FORWARD_TO_LEFT_COMP        -0.08f
#endif

#ifndef BSP_CHASSIS_LEFT_TO_FORWARD_COMP
#define BSP_CHASSIS_LEFT_TO_FORWARD_COMP        0.0f
#endif

#ifndef BSP_CHASSIS_YAW_CTRL_DIR
#define BSP_CHASSIS_YAW_CTRL_DIR                1
#endif

#ifndef BSP_CHASSIS_MOTOR1_DIR
#define BSP_CHASSIS_MOTOR1_DIR                  1
#endif

#ifndef BSP_CHASSIS_MOTOR2_DIR
#define BSP_CHASSIS_MOTOR2_DIR                  1
#endif

#ifndef BSP_CHASSIS_MOTOR3_DIR
#define BSP_CHASSIS_MOTOR3_DIR                  1
#endif

#ifndef BSP_CHASSIS_MOTOR4_DIR
#define BSP_CHASSIS_MOTOR4_DIR                  1
#endif

#ifndef BSP_CHASSIS_MOTOR1_FB_DIR
#define BSP_CHASSIS_MOTOR1_FB_DIR               1
#endif

#ifndef BSP_CHASSIS_MOTOR2_FB_DIR
#define BSP_CHASSIS_MOTOR2_FB_DIR               1
#endif

#ifndef BSP_CHASSIS_MOTOR3_FB_DIR
#define BSP_CHASSIS_MOTOR3_FB_DIR               1
#endif

#ifndef BSP_CHASSIS_MOTOR4_FB_DIR
#define BSP_CHASSIS_MOTOR4_FB_DIR               1
#endif

typedef struct
{
    int16_t motor1;
    int16_t motor2;
    int16_t motor3;
    int16_t motor4;
} BspChassisMotorCurrent;

typedef struct
{
    float motor1;
    float motor2;
    float motor3;
    float motor4;
} BspChassisWheelDemand;

typedef struct
{
    float motor1_rpm;
    float motor2_rpm;
    float motor3_rpm;
    float motor4_rpm;
} BspChassisWheelSpeedTarget;

HAL_StatusTypeDef BspChassis_Stop(void);
void BspChassis_ResetPid(void);
HAL_StatusTypeDef BspChassis_SetMotorCurrents(int16_t motor1,
                                              int16_t motor2,
                                              int16_t motor3,
                                              int16_t motor4);
HAL_StatusTypeDef BspChassis_SetOpenLoop(int16_t forward_current,
                                         int16_t left_current,
                                         int16_t ccw_current);
HAL_StatusTypeDef BspChassis_SetOpenLoopLimited(int16_t forward_current,
                                                int16_t left_current,
                                                int16_t ccw_current,
                                                int16_t max_current);
HAL_StatusTypeDef BspChassis_SetPolarOpenLoop(float move_direction_deg,
                                              int16_t move_current,
                                              int16_t ccw_current,
                                              int16_t max_current);
HAL_StatusTypeDef BspChassis_SetPolarAngleHold(float move_direction_deg,
                                               int16_t move_current,
                                               float target_yaw_deg,
                                               float current_yaw_deg,
                                               int16_t max_current);
HAL_StatusTypeDef BspChassis_SetWheelSpeeds(const BspChassisWheelSpeedTarget *target,
                                            int16_t max_current);
HAL_StatusTypeDef BspChassis_SetPolarSpeed(float move_direction_deg,
                                           float move_rpm,
                                           float ccw_rpm,
                                           int16_t max_current);
HAL_StatusTypeDef BspChassis_SetPolarSpeedAngleHold(float move_direction_deg,
                                                    float move_rpm,
                                                    float target_yaw_deg,
                                                    float current_yaw_deg,
                                                    int16_t max_current);
HAL_StatusTypeDef BspChassis_SetPolarSpeedAngleHoldGyro(float move_direction_deg,
                                                        float move_rpm,
                                                        float target_yaw_deg,
                                                        float current_yaw_deg,
                                                        float gyro_z_deg_s,
                                                        int16_t max_current);
HAL_StatusTypeDef BspChassis_SetBodySpeed(float forward_rpm,
                                          float left_rpm,
                                          float ccw_rpm,
                                          int16_t max_current);
HAL_StatusTypeDef BspChassis_SetBodySpeedAngleHold(float forward_rpm,
                                                   float left_rpm,
                                                   float target_yaw_deg,
                                                   float current_yaw_deg,
                                                   int16_t max_current);
HAL_StatusTypeDef BspChassis_SetBodySpeedAngleHoldGyro(float forward_rpm,
                                                       float left_rpm,
                                                       float target_yaw_deg,
                                                       float current_yaw_deg,
                                                       float gyro_z_deg_s,
                                                       int16_t max_current);
HAL_StatusTypeDef BspChassis_SetVelocity(float vx_mm_s,
                                         float vy_mm_s,
                                         float wz_rad_s,
                                         int16_t max_current);
void BspChassis_CalcWheelDemand(float vx_mm_s,
                                float vy_mm_s,
                                float wz_rad_s,
                                BspChassisWheelDemand *demand);
float BspChassis_WrapAngle360(float angle_deg);
float BspChassis_GetAngleErrorDeg(float target_deg, float current_deg);
int16_t BspChassis_CalcAngleCurrent(float target_yaw_deg, float current_yaw_deg);
float BspChassis_CalcAngleSpeed(float target_yaw_deg, float current_yaw_deg);
float BspChassis_CalcAngleSpeedGyro(float target_yaw_deg,
                                    float current_yaw_deg,
                                    float gyro_z_deg_s);
const BspChassisMotorCurrent *BspChassis_GetLastCurrent(void);
const BspChassisWheelSpeedTarget *BspChassis_GetLastWheelSpeedTarget(void);
float BspChassis_GetLastAngleErrorDeg(void);
float BspChassis_GetLastAngleSpeedRpm(void);

#ifdef __cplusplus
}
#endif

#endif
