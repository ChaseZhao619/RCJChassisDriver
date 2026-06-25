#ifndef BSP_CHASSIS_H
#define BSP_CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/*
 * 四轮底盘几何参数，单位 mm。
 * ROTATION_RADIUS 用于将角速度换算为轮缘线速度；测量误差会按比例影响旋转速度。
 * CONTACT_OPPOSITE 表示相对接触点间距，ADJACENT 仅用于记录相邻轮接触点几何尺寸。
 */
#define BSP_CHASSIS_DIAMETER_MM                 210.0f
#define BSP_CHASSIS_CONTACT_OPPOSITE_MM         192.0f
#define BSP_CHASSIS_CONTACT_ADJACENT_MM         140.0f
#define BSP_CHASSIS_ROTATION_RADIUS_MM          (BSP_CHASSIS_CONTACT_OPPOSITE_MM * 0.5f)

#ifndef BSP_CHASSIS_DEFAULT_MAX_CURRENT
/* 默认单电机电流控制量上限。首次落地测试应降低，确认方向后再逐步增加。 */
#define BSP_CHASSIS_DEFAULT_MAX_CURRENT         3000
#endif

#ifndef BSP_CHASSIS_ANGLE_KP
/* 偏航角比例增益 [目标 rpm/deg]：增大可加快回正，过大会振荡。 */
#define BSP_CHASSIS_ANGLE_KP                    70.0f
#endif

#ifndef BSP_CHASSIS_ANGLE_KD
/* 基于角误差差分的微分增益；会放大角度量化噪声，通常优先使用陀螺仪反馈。 */
#define BSP_CHASSIS_ANGLE_KD                    0.0f
#endif

#ifndef BSP_CHASSIS_ANGLE_GYRO_KD
/* 陀螺仪阻尼增益 [目标 rpm/(deg/s)]：增大可抑制超调，过大会使转向迟钝。 */
#define BSP_CHASSIS_ANGLE_GYRO_KD               10.0f
#endif

#ifndef BSP_CHASSIS_ANGLE_GYRO_DEADBAND_DPS
/* 角速度死区 [deg/s]：应略高于静止噪声，过大会丢失低速阻尼。 */
#define BSP_CHASSIS_ANGLE_GYRO_DEADBAND_DPS     1.5f
#endif

#ifndef BSP_CHASSIS_GYRO_Z_DIR
/* 陀螺仪 Z 轴符号，必须使逻辑正方向与底盘逆时针正方向一致，只允许 +1/-1。 */
#define BSP_CHASSIS_GYRO_Z_DIR                  -1
#endif

#ifndef BSP_CHASSIS_ANGLE_DEADBAND_DEG
/* 偏航角误差死区 [deg]：增大可减少静止抖动，但会降低最终角度精度。 */
#define BSP_CHASSIS_ANGLE_DEADBAND_DEG          0.10f
#endif

#ifndef BSP_CHASSIS_ANGLE_MIN_RPM
/* 克服静摩擦的最小旋转指令 [rpm]；过高会在目标角附近往复摆动。 */
#define BSP_CHASSIS_ANGLE_MIN_RPM               70.0f
#endif

#ifndef BSP_CHASSIS_ANGLE_MIN_RPM_FULL_ERR_DEG
/* 最小转速从 0 线性爬升到完整值所对应的误差 [deg]。 */
#define BSP_CHASSIS_ANGLE_MIN_RPM_FULL_ERR_DEG  1.0f
#endif

#ifndef BSP_CHASSIS_ANGLE_MAX_RPM
/* 角度环允许的最大旋转目标 [rpm]，用于限制大角度误差时的速度。 */
#define BSP_CHASSIS_ANGLE_MAX_RPM               3000.0f
#endif

#ifndef BSP_CHASSIS_ANGLE_MAX_CURRENT
/* CalcAngleCurrent() 开环角度控制的输出限幅；闭环轮速接口仍受调用参数限制。 */
#define BSP_CHASSIS_ANGLE_MAX_CURRENT           500
#endif

#ifndef BSP_CHASSIS_WHEEL_SPEED_KP
/* 轮速比例增益 [电流控制量/rpm]：先在 KI=KD=0 时调到快速且不持续振荡。 */
#define BSP_CHASSIS_WHEEL_SPEED_KP              4.5f
#endif

#ifndef BSP_CHASSIS_WHEEL_SPEED_KI
/* 轮速积分增益：消除负载导致的稳态误差，过大会积分饱和和低频摆动。 */
#define BSP_CHASSIS_WHEEL_SPEED_KI              0.8f
#endif

#ifndef BSP_CHASSIS_WHEEL_SPEED_KD
/* 轮速微分增益：编码器速度噪声较大时保持为 0。 */
#define BSP_CHASSIS_WHEEL_SPEED_KD              0.0f
#endif

#ifndef BSP_CHASSIS_WHEEL_SPEED_KF
/* 轮速前馈 [电流控制量/rpm]：用于承担匀速所需的主要输出。 */
#define BSP_CHASSIS_WHEEL_SPEED_KF              1.4f
#endif

#ifndef BSP_CHASSIS_WHEEL_SPEED_I_LIMIT
/* 积分状态绝对值上限，减小可加快饱和后的恢复，但可能削弱低速带载能力。 */
#define BSP_CHASSIS_WHEEL_SPEED_I_LIMIT         4000.0f
#endif

#ifndef BSP_CHASSIS_MOVE_FORWARD_BIAS_RPM
/* 有前进指令时叠加的固定转速补偿 [rpm]；符号决定补偿方向。 */
#define BSP_CHASSIS_MOVE_FORWARD_BIAS_RPM       0.0f
#endif

#ifndef BSP_CHASSIS_MOVE_LEFT_BIAS_RPM
/* 有左移指令时叠加的固定转速补偿 [rpm]；仅用于可重复的静摩擦偏差。 */
#define BSP_CHASSIS_MOVE_LEFT_BIAS_RPM          0.0f
#endif

#ifndef BSP_CHASSIS_MOVE_BIAS_MIN_RPM
/* 启用固定补偿的最小轴向目标 [rpm]，避免零速附近被补偿项推动。 */
#define BSP_CHASSIS_MOVE_BIAS_MIN_RPM           50.0f
#endif

#ifndef BSP_CHASSIS_FORWARD_TO_LEFT_COMP
/* 前进到左移的线性耦合系数：left += forward * 本系数，用于修正纯前进横漂。 */
#define BSP_CHASSIS_FORWARD_TO_LEFT_COMP        -0.08f
#endif

#ifndef BSP_CHASSIS_LEFT_TO_FORWARD_COMP
/* 左移到前进的线性耦合系数：forward += left * 本系数，用于修正纯左移纵漂。 */
#define BSP_CHASSIS_LEFT_TO_FORWARD_COMP        0.0f
#endif

#ifndef BSP_CHASSIS_YAW_CTRL_DIR
/* 偏航控制输出总符号，只允许 +1/-1；改变它会反转所有自动回正动作。 */
#define BSP_CHASSIS_YAW_CTRL_DIR                1
#endif

#ifndef BSP_CHASSIS_MOTOR1_DIR
/* 电机命令方向，只允许 +1/-1；按 1~4 号轮的实际安装方向分别设置。 */
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
/* 编码器反馈方向，只允许 +1/-1；正目标转速必须得到正反馈转速。 */
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
