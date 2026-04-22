#include "bsp_chassis_odom.h"

#include "bsp_chassis.h"
#include "bsp_motor.h"
#include <math.h>
#include <string.h>

static const float odom_pi = 3.14159265358979323846f;
static BspChassisOdomPose odom_pose;
static uint32_t odom_last_tick;
static uint8_t odom_ready;
static const int8_t odom_motor_fb_dir[4] = {
    BSP_CHASSIS_MOTOR1_FB_DIR,
    BSP_CHASSIS_MOTOR2_FB_DIR,
    BSP_CHASSIS_MOTOR3_FB_DIR,
    BSP_CHASSIS_MOTOR4_FB_DIR,
};

static float GetLogicalMotorRpm(uint8_t can_id)
{
    const BspMotorFeedback *feedback = BspMotor_GetFeedback(can_id);

    if (feedback == NULL)
    {
        return 0.0f;
    }

    return (float)feedback->speed_rpm * (float)odom_motor_fb_dir[can_id - 1U];
}

float BspChassisOdom_MotorRpmToMmS(float motor_rpm)
{
    float wheel_circumference_mm = odom_pi * BSP_CHASSIS_ODOM_WHEEL_DIAMETER_MM;
    float wheel_rpm = motor_rpm / BSP_CHASSIS_ODOM_MOTOR_GEAR_RATIO;

    return wheel_rpm * wheel_circumference_mm / 60.0f;
}

float BspChassisOdom_MmSToMotorRpm(float speed_mm_s)
{
    float wheel_circumference_mm = odom_pi * BSP_CHASSIS_ODOM_WHEEL_DIAMETER_MM;

    if (wheel_circumference_mm < 0.001f)
    {
        return 0.0f;
    }

    return speed_mm_s * 60.0f * BSP_CHASSIS_ODOM_MOTOR_GEAR_RATIO / wheel_circumference_mm;
}

void BspChassisOdom_Init(float yaw_deg)
{
    BspChassisOdom_Reset(0.0f, 0.0f, yaw_deg);
}

void BspChassisOdom_Reset(float x_mm, float y_mm, float yaw_deg)
{
    memset(&odom_pose, 0, sizeof(odom_pose));
    odom_pose.x_mm = x_mm;
    odom_pose.y_mm = y_mm;
    odom_pose.yaw_deg = BspChassis_WrapAngle360(yaw_deg);
    odom_last_tick = HAL_GetTick();
    odom_ready = 1U;
}

void BspChassisOdom_Update(float yaw_deg)
{
    uint32_t now = HAL_GetTick();
    float dt_s;
    float m1_rpm;
    float m2_rpm;
    float m3_rpm;
    float m4_rpm;
    float body_forward_rpm;
    float body_left_rpm;
    float body_forward_mm_s;
    float body_left_mm_s;
    float yaw_rad;
    float cos_yaw;
    float sin_yaw;

    if (odom_ready == 0U)
    {
        BspChassisOdom_Init(yaw_deg);
        return;
    }

    dt_s = (float)(now - odom_last_tick) * 0.001f;
    odom_last_tick = now;
    if ((dt_s <= 0.0f) || (dt_s > 0.2f))
    {
        odom_pose.yaw_deg = BspChassis_WrapAngle360(yaw_deg);
        return;
    }

    m1_rpm = GetLogicalMotorRpm(1U);
    m2_rpm = GetLogicalMotorRpm(2U);
    m3_rpm = GetLogicalMotorRpm(3U);
    m4_rpm = GetLogicalMotorRpm(4U);

    body_forward_rpm = (m1_rpm + m2_rpm - m3_rpm - m4_rpm) * 0.25f;
    body_left_rpm = (-m1_rpm + m2_rpm + m3_rpm - m4_rpm) * 0.25f;

    body_forward_mm_s = BspChassisOdom_MotorRpmToMmS(body_forward_rpm);
    body_left_mm_s = BspChassisOdom_MotorRpmToMmS(body_left_rpm);
    body_forward_mm_s *= BSP_CHASSIS_ODOM_FORWARD_SCALE;
    body_left_mm_s *= BSP_CHASSIS_ODOM_LEFT_SCALE;

    odom_pose.yaw_deg = BspChassis_WrapAngle360(yaw_deg);
    yaw_rad = odom_pose.yaw_deg * odom_pi / 180.0f;
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);

    odom_pose.vx_mm_s = (body_forward_mm_s * cos_yaw) - (body_left_mm_s * sin_yaw);
    odom_pose.vy_mm_s = (body_forward_mm_s * sin_yaw) + (body_left_mm_s * cos_yaw);
    odom_pose.body_forward_mm_s = body_forward_mm_s;
    odom_pose.body_left_mm_s = body_left_mm_s;
    odom_pose.x_mm += odom_pose.vx_mm_s * dt_s;
    odom_pose.y_mm += odom_pose.vy_mm_s * dt_s;
}

HAL_StatusTypeDef BspChassisOdom_DriveTo(float target_x_mm,
                                         float target_y_mm,
                                         float target_yaw_deg,
                                         float max_speed_mm_s,
                                         int16_t max_current)
{
    return BspChassisOdom_DriveToGyro(target_x_mm,
                                      target_y_mm,
                                      target_yaw_deg,
                                      0.0f,
                                      max_speed_mm_s,
                                      max_current);
}

HAL_StatusTypeDef BspChassisOdom_DriveToGyro(float target_x_mm,
                                             float target_y_mm,
                                             float target_yaw_deg,
                                             float gyro_z_deg_s,
                                             float max_speed_mm_s,
                                             int16_t max_current)
{
    float dx = target_x_mm - odom_pose.x_mm;
    float dy = target_y_mm - odom_pose.y_mm;
    float distance = sqrtf((dx * dx) + (dy * dy));
    float speed_mm_s;
    float vx_world_mm_s;
    float vy_world_mm_s;
    float yaw_rad;
    float cos_yaw;
    float sin_yaw;
    float forward_mm_s;
    float left_mm_s;
    float forward_rpm;
    float left_rpm;

    if (distance <= BSP_CHASSIS_ODOM_POS_TOLERANCE_MM)
    {
        return BspChassis_SetBodySpeedAngleHoldGyro(0.0f,
                                                    0.0f,
                                                    target_yaw_deg,
                                                    odom_pose.yaw_deg,
                                                    gyro_z_deg_s,
                                                    max_current);
    }

    if (max_speed_mm_s <= 0.0f)
    {
        max_speed_mm_s = BSP_CHASSIS_ODOM_DEFAULT_MAX_SPEED_MM_S;
    }

    speed_mm_s = distance * BSP_CHASSIS_ODOM_POS_KP;
    if (speed_mm_s > max_speed_mm_s)
    {
        speed_mm_s = max_speed_mm_s;
    }
    if (speed_mm_s < BSP_CHASSIS_ODOM_MIN_SPEED_MM_S)
    {
        speed_mm_s = BSP_CHASSIS_ODOM_MIN_SPEED_MM_S;
    }

    vx_world_mm_s = dx / distance * speed_mm_s;
    vy_world_mm_s = dy / distance * speed_mm_s;

    yaw_rad = odom_pose.yaw_deg * odom_pi / 180.0f;
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);

    forward_mm_s = (vx_world_mm_s * cos_yaw) + (vy_world_mm_s * sin_yaw);
    left_mm_s = (-vx_world_mm_s * sin_yaw) + (vy_world_mm_s * cos_yaw);
    forward_rpm = BspChassisOdom_MmSToMotorRpm(forward_mm_s);
    left_rpm = BspChassisOdom_MmSToMotorRpm(left_mm_s);

    return BspChassis_SetBodySpeedAngleHoldGyro(forward_rpm,
                                                left_rpm,
                                                target_yaw_deg,
                                                odom_pose.yaw_deg,
                                                gyro_z_deg_s,
                                                max_current);
}

uint8_t BspChassisOdom_IsAt(float target_x_mm, float target_y_mm, float tolerance_mm)
{
    float dx = target_x_mm - odom_pose.x_mm;
    float dy = target_y_mm - odom_pose.y_mm;

    if (tolerance_mm <= 0.0f)
    {
        tolerance_mm = BSP_CHASSIS_ODOM_POS_TOLERANCE_MM;
    }

    return (((dx * dx) + (dy * dy)) <= (tolerance_mm * tolerance_mm)) ? 1U : 0U;
}

const BspChassisOdomPose *BspChassisOdom_GetPose(void)
{
    return &odom_pose;
}
