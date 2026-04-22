#include "bsp_chassis.h"

#include "bsp_motor.h"
#include <math.h>

static BspChassisMotorCurrent chassis_last_current;
static BspChassisWheelSpeedTarget chassis_last_speed_target;
static const float chassis_pi = 3.14159265358979323846f;
static const int8_t chassis_motor_dir[4] = {
    BSP_CHASSIS_MOTOR1_DIR,
    BSP_CHASSIS_MOTOR2_DIR,
    BSP_CHASSIS_MOTOR3_DIR,
    BSP_CHASSIS_MOTOR4_DIR,
};
static const int8_t chassis_motor_fb_dir[4] = {
    BSP_CHASSIS_MOTOR1_FB_DIR,
    BSP_CHASSIS_MOTOR2_FB_DIR,
    BSP_CHASSIS_MOTOR3_FB_DIR,
    BSP_CHASSIS_MOTOR4_FB_DIR,
};

typedef struct
{
    float integral;
    float last_error;
    uint32_t last_tick;
    uint8_t initialized;
} ChassisPidState;

static ChassisPidState wheel_speed_pid[4];
static ChassisPidState angle_pid;
static float chassis_last_angle_error_deg;
static float chassis_last_angle_speed_rpm;

static int16_t Abs16(int16_t value)
{
    if (value == INT16_MIN)
    {
        return INT16_MAX;
    }

    return (value < 0) ? (int16_t)-value : value;
}

static float AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int16_t LimitCurrent(int16_t current, int16_t max_current)
{
    int16_t limit = Abs16(max_current);

    if (limit > BSP_MOTOR_C610_MAX_CURRENT)
    {
        limit = BSP_MOTOR_C610_MAX_CURRENT;
    }

    if (current > limit)
    {
        return limit;
    }

    if (current < -limit)
    {
        return (int16_t)-limit;
    }

    return current;
}

static int16_t FloatToCurrent(float value)
{
    if (value > 32767.0f)
    {
        return 32767;
    }

    if (value < -32768.0f)
    {
        return -32768;
    }

    return (int16_t)value;
}

static int16_t ApplyMotorDir(int16_t current, uint8_t motor_index)
{
    if (chassis_motor_dir[motor_index] < 0)
    {
        return (int16_t)-current;
    }

    return current;
}

static float LimitFloat(float value, float limit)
{
    float abs_limit = AbsFloat(limit);

    if (value > abs_limit)
    {
        return abs_limit;
    }

    if (value < -abs_limit)
    {
        return -abs_limit;
    }

    return value;
}

static float GetLogicalMotorRpm(uint8_t motor_index)
{
    const BspMotorFeedback *feedback = BspMotor_GetFeedback((uint8_t)(motor_index + 1U));

    if (feedback == NULL)
    {
        return 0.0f;
    }

    return (float)feedback->speed_rpm * (float)chassis_motor_fb_dir[motor_index];
}

static float CalcPidOutput(ChassisPidState *pid,
                           float error,
                           float kp,
                           float ki,
                           float kd,
                           float integral_limit,
                           uint32_t now)
{
    float dt = 0.01f;
    float derivative = 0.0f;

    if (pid == NULL)
    {
        return 0.0f;
    }

    if (pid->initialized != 0U)
    {
        uint32_t dt_ms = now - pid->last_tick;
        if ((dt_ms > 0U) && (dt_ms < 200U))
        {
            dt = (float)dt_ms * 0.001f;
        }
        derivative = (error - pid->last_error) / dt;
    }
    else
    {
        pid->initialized = 1U;
    }

    pid->integral += error * dt;
    pid->integral = LimitFloat(pid->integral, integral_limit);
    pid->last_error = error;
    pid->last_tick = now;

    return (kp * error) + (ki * pid->integral) + (kd * derivative);
}

static HAL_StatusTypeDef SendLimitedCurrents(int16_t motor1,
                                             int16_t motor2,
                                             int16_t motor3,
                                             int16_t motor4,
                                             int16_t max_current)
{
    chassis_last_current.motor1 = ApplyMotorDir(LimitCurrent(motor1, max_current), 0U);
    chassis_last_current.motor2 = ApplyMotorDir(LimitCurrent(motor2, max_current), 1U);
    chassis_last_current.motor3 = ApplyMotorDir(LimitCurrent(motor3, max_current), 2U);
    chassis_last_current.motor4 = ApplyMotorDir(LimitCurrent(motor4, max_current), 3U);

    return BspMotor_SendChassisCurrents(chassis_last_current.motor1,
                                        chassis_last_current.motor2,
                                        chassis_last_current.motor3,
                                        chassis_last_current.motor4);
}

static HAL_StatusTypeDef SendNormalizedDemand(const BspChassisWheelDemand *demand, int16_t max_current)
{
    float max_abs;
    float scale;

    if (demand == NULL)
    {
        return HAL_ERROR;
    }

    max_abs = AbsFloat(demand->motor1);
    if (AbsFloat(demand->motor2) > max_abs)
    {
        max_abs = AbsFloat(demand->motor2);
    }
    if (AbsFloat(demand->motor3) > max_abs)
    {
        max_abs = AbsFloat(demand->motor3);
    }
    if (AbsFloat(demand->motor4) > max_abs)
    {
        max_abs = AbsFloat(demand->motor4);
    }

    if (max_abs < 0.001f)
    {
        return BspChassis_Stop();
    }

    scale = (float)Abs16(max_current) / max_abs;

    return SendLimitedCurrents((int16_t)(demand->motor1 * scale),
                               (int16_t)(demand->motor2 * scale),
                               (int16_t)(demand->motor3 * scale),
                               (int16_t)(demand->motor4 * scale),
                               max_current);
}

HAL_StatusTypeDef BspChassis_Stop(void)
{
    BspChassis_ResetPid();
    return BspChassis_SetMotorCurrents(0, 0, 0, 0);
}

void BspChassis_ResetPid(void)
{
    uint8_t i;

    for (i = 0U; i < 4U; i++)
    {
        wheel_speed_pid[i].integral = 0.0f;
        wheel_speed_pid[i].last_error = 0.0f;
        wheel_speed_pid[i].last_tick = 0U;
        wheel_speed_pid[i].initialized = 0U;
    }

    angle_pid.integral = 0.0f;
    angle_pid.last_error = 0.0f;
    angle_pid.last_tick = 0U;
    angle_pid.initialized = 0U;
    chassis_last_angle_error_deg = 0.0f;
    chassis_last_angle_speed_rpm = 0.0f;
}

HAL_StatusTypeDef BspChassis_SetMotorCurrents(int16_t motor1,
                                              int16_t motor2,
                                              int16_t motor3,
                                              int16_t motor4)
{
    return SendLimitedCurrents(motor1,
                               motor2,
                               motor3,
                               motor4,
                               BSP_MOTOR_C610_MAX_CURRENT);
}

HAL_StatusTypeDef BspChassis_SetOpenLoop(int16_t forward_current,
                                         int16_t left_current,
                                         int16_t ccw_current)
{
    return BspChassis_SetOpenLoopLimited(forward_current,
                                         left_current,
                                         ccw_current,
                                         BSP_CHASSIS_DEFAULT_MAX_CURRENT);
}

HAL_StatusTypeDef BspChassis_SetOpenLoopLimited(int16_t forward_current,
                                                int16_t left_current,
                                                int16_t ccw_current,
                                                int16_t max_current)
{
    int32_t motor1 = (int32_t)forward_current - left_current - ccw_current;
    int32_t motor2 = (int32_t)forward_current + left_current - ccw_current;
    int32_t motor3 = -(int32_t)forward_current + left_current - ccw_current;
    int32_t motor4 = -(int32_t)forward_current - left_current - ccw_current;
    int32_t max_abs = motor1;
    int32_t limit = Abs16(max_current);

    if (max_abs < 0)
    {
        max_abs = -max_abs;
    }
    if (((motor2 < 0) ? -motor2 : motor2) > max_abs)
    {
        max_abs = (motor2 < 0) ? -motor2 : motor2;
    }
    if (((motor3 < 0) ? -motor3 : motor3) > max_abs)
    {
        max_abs = (motor3 < 0) ? -motor3 : motor3;
    }
    if (((motor4 < 0) ? -motor4 : motor4) > max_abs)
    {
        max_abs = (motor4 < 0) ? -motor4 : motor4;
    }

    if (limit > BSP_MOTOR_C610_MAX_CURRENT)
    {
        limit = BSP_MOTOR_C610_MAX_CURRENT;
    }

    if ((max_abs > limit) && (max_abs > 0))
    {
        motor1 = (motor1 * limit) / max_abs;
        motor2 = (motor2 * limit) / max_abs;
        motor3 = (motor3 * limit) / max_abs;
        motor4 = (motor4 * limit) / max_abs;
    }

    return SendLimitedCurrents((int16_t)motor1,
                               (int16_t)motor2,
                               (int16_t)motor3,
                               (int16_t)motor4,
                               (int16_t)limit);
}

HAL_StatusTypeDef BspChassis_SetPolarOpenLoop(float move_direction_deg,
                                              int16_t move_current,
                                              int16_t ccw_current,
                                              int16_t max_current)
{
    float rad = BspChassis_WrapAngle360(move_direction_deg) * chassis_pi / 180.0f;
    int16_t forward_current = FloatToCurrent(cosf(rad) * (float)move_current);
    int16_t left_current = FloatToCurrent(sinf(rad) * (float)move_current);

    return BspChassis_SetOpenLoopLimited(forward_current,
                                         left_current,
                                         ccw_current,
                                         max_current);
}

HAL_StatusTypeDef BspChassis_SetPolarAngleHold(float move_direction_deg,
                                               int16_t move_current,
                                               float target_yaw_deg,
                                               float current_yaw_deg,
                                               int16_t max_current)
{
    int16_t ccw_current = BspChassis_CalcAngleCurrent(target_yaw_deg, current_yaw_deg);

    return BspChassis_SetPolarOpenLoop(move_direction_deg,
                                       move_current,
                                       ccw_current,
                                       max_current);
}

HAL_StatusTypeDef BspChassis_SetWheelSpeeds(const BspChassisWheelSpeedTarget *target,
                                            int16_t max_current)
{
    uint32_t now = HAL_GetTick();
    float target_rpm[4];
    float current[4];
    uint8_t i;

    if (target == NULL)
    {
        return HAL_ERROR;
    }

    chassis_last_speed_target = *target;
    target_rpm[0] = target->motor1_rpm;
    target_rpm[1] = target->motor2_rpm;
    target_rpm[2] = target->motor3_rpm;
    target_rpm[3] = target->motor4_rpm;

    for (i = 0U; i < 4U; i++)
    {
        float feedback_rpm = GetLogicalMotorRpm(i);
        float error = target_rpm[i] - feedback_rpm;

        current[i] = (target_rpm[i] * BSP_CHASSIS_WHEEL_SPEED_KF) +
                     CalcPidOutput(&wheel_speed_pid[i],
                                   error,
                                   BSP_CHASSIS_WHEEL_SPEED_KP,
                                   BSP_CHASSIS_WHEEL_SPEED_KI,
                                   BSP_CHASSIS_WHEEL_SPEED_KD,
                                   BSP_CHASSIS_WHEEL_SPEED_I_LIMIT,
                                   now);
    }

    return SendLimitedCurrents(FloatToCurrent(current[0]),
                               FloatToCurrent(current[1]),
                               FloatToCurrent(current[2]),
                               FloatToCurrent(current[3]),
                               max_current);
}

HAL_StatusTypeDef BspChassis_SetPolarSpeed(float move_direction_deg,
                                           float move_rpm,
                                           float ccw_rpm,
                                           int16_t max_current)
{
    float rad = BspChassis_WrapAngle360(move_direction_deg) * chassis_pi / 180.0f;
    float forward_rpm = cosf(rad) * move_rpm;
    float left_rpm = sinf(rad) * move_rpm;

    if (AbsFloat(move_rpm) >= BSP_CHASSIS_MOVE_BIAS_MIN_RPM)
    {
        forward_rpm += BSP_CHASSIS_MOVE_FORWARD_BIAS_RPM;
        left_rpm += BSP_CHASSIS_MOVE_LEFT_BIAS_RPM;
    }

    return BspChassis_SetBodySpeed(forward_rpm, left_rpm, ccw_rpm, max_current);
}

HAL_StatusTypeDef BspChassis_SetBodySpeed(float forward_rpm,
                                          float left_rpm,
                                          float ccw_rpm,
                                          int16_t max_current)
{
    BspChassisWheelSpeedTarget target;
    float corrected_forward_rpm = forward_rpm + (left_rpm * BSP_CHASSIS_LEFT_TO_FORWARD_COMP);
    float corrected_left_rpm = left_rpm + (forward_rpm * BSP_CHASSIS_FORWARD_TO_LEFT_COMP);

    target.motor1_rpm = corrected_forward_rpm - corrected_left_rpm - ccw_rpm;
    target.motor2_rpm = corrected_forward_rpm + corrected_left_rpm - ccw_rpm;
    target.motor3_rpm = -corrected_forward_rpm + corrected_left_rpm - ccw_rpm;
    target.motor4_rpm = -corrected_forward_rpm - corrected_left_rpm - ccw_rpm;

    return BspChassis_SetWheelSpeeds(&target, max_current);
}

HAL_StatusTypeDef BspChassis_SetPolarSpeedAngleHold(float move_direction_deg,
                                                    float move_rpm,
                                                    float target_yaw_deg,
                                                    float current_yaw_deg,
                                                    int16_t max_current)
{
    float ccw_rpm = BspChassis_CalcAngleSpeed(target_yaw_deg, current_yaw_deg);

    return BspChassis_SetPolarSpeed(move_direction_deg,
                                    move_rpm,
                                    ccw_rpm,
                                    max_current);
}

HAL_StatusTypeDef BspChassis_SetBodySpeedAngleHold(float forward_rpm,
                                                   float left_rpm,
                                                   float target_yaw_deg,
                                                   float current_yaw_deg,
                                                   int16_t max_current)
{
    float ccw_rpm = BspChassis_CalcAngleSpeedGyro(target_yaw_deg, current_yaw_deg, 0.0f);

    return BspChassis_SetBodySpeed(forward_rpm,
                                   left_rpm,
                                   ccw_rpm,
                                   max_current);
}

HAL_StatusTypeDef BspChassis_SetBodySpeedAngleHoldGyro(float forward_rpm,
                                                       float left_rpm,
                                                       float target_yaw_deg,
                                                       float current_yaw_deg,
                                                       float gyro_z_deg_s,
                                                       int16_t max_current)
{
    float ccw_rpm = BspChassis_CalcAngleSpeedGyro(target_yaw_deg,
                                                  current_yaw_deg,
                                                  gyro_z_deg_s);

    return BspChassis_SetBodySpeed(forward_rpm,
                                   left_rpm,
                                   ccw_rpm,
                                   max_current);
}

HAL_StatusTypeDef BspChassis_SetVelocity(float vx_mm_s,
                                         float vy_mm_s,
                                         float wz_rad_s,
                                         int16_t max_current)
{
    BspChassisWheelDemand demand;

    BspChassis_CalcWheelDemand(vx_mm_s, vy_mm_s, wz_rad_s, &demand);
    return SendNormalizedDemand(&demand, max_current);
}

void BspChassis_CalcWheelDemand(float vx_mm_s,
                                float vy_mm_s,
                                float wz_rad_s,
                                BspChassisWheelDemand *demand)
{
    float rotate_mm_s;

    if (demand == NULL)
    {
        return;
    }

    rotate_mm_s = BSP_CHASSIS_ROTATION_RADIUS_MM * wz_rad_s;

    demand->motor1 = vx_mm_s - vy_mm_s - rotate_mm_s;
    demand->motor2 = vx_mm_s + vy_mm_s - rotate_mm_s;
    demand->motor3 = -vx_mm_s + vy_mm_s - rotate_mm_s;
    demand->motor4 = -vx_mm_s - vy_mm_s - rotate_mm_s;
}

float BspChassis_WrapAngle360(float angle_deg)
{
    while (angle_deg >= 360.0f)
    {
        angle_deg -= 360.0f;
    }

    while (angle_deg < 0.0f)
    {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

float BspChassis_GetAngleErrorDeg(float target_deg, float current_deg)
{
    float error = BspChassis_WrapAngle360(target_deg) - BspChassis_WrapAngle360(current_deg);

    while (error >= 180.0f)
    {
        error -= 360.0f;
    }

    while (error < -180.0f)
    {
        error += 360.0f;
    }

    return error;
}

int16_t BspChassis_CalcAngleCurrent(float target_yaw_deg, float current_yaw_deg)
{
    float error = BspChassis_GetAngleErrorDeg(target_yaw_deg, current_yaw_deg);
    float output;

    if ((error < BSP_CHASSIS_ANGLE_DEADBAND_DEG) &&
        (error > -BSP_CHASSIS_ANGLE_DEADBAND_DEG))
    {
        return 0;
    }

    output = error * BSP_CHASSIS_ANGLE_KP * (float)BSP_CHASSIS_YAW_CTRL_DIR;

    if (output > (float)BSP_CHASSIS_ANGLE_MAX_CURRENT)
    {
        output = (float)BSP_CHASSIS_ANGLE_MAX_CURRENT;
    }

    if (output < -(float)BSP_CHASSIS_ANGLE_MAX_CURRENT)
    {
        output = -(float)BSP_CHASSIS_ANGLE_MAX_CURRENT;
    }

    return FloatToCurrent(output);
}

float BspChassis_CalcAngleSpeed(float target_yaw_deg, float current_yaw_deg)
{
    return BspChassis_CalcAngleSpeedGyro(target_yaw_deg, current_yaw_deg, 0.0f);
}

float BspChassis_CalcAngleSpeedGyro(float target_yaw_deg,
                                    float current_yaw_deg,
                                    float gyro_z_deg_s)
{
    uint32_t now = HAL_GetTick();
    float error = BspChassis_GetAngleErrorDeg(target_yaw_deg, current_yaw_deg);
    float gyro_feedback = gyro_z_deg_s * (float)BSP_CHASSIS_GYRO_Z_DIR;
    float output;

    chassis_last_angle_error_deg = error;

    if ((error < BSP_CHASSIS_ANGLE_DEADBAND_DEG) &&
        (error > -BSP_CHASSIS_ANGLE_DEADBAND_DEG))
    {
        angle_pid.integral = 0.0f;
        angle_pid.last_error = error;
        angle_pid.last_tick = now;
        angle_pid.initialized = 1U;
        if ((gyro_feedback < BSP_CHASSIS_ANGLE_GYRO_DEADBAND_DPS) &&
            (gyro_feedback > -BSP_CHASSIS_ANGLE_GYRO_DEADBAND_DPS))
        {
            chassis_last_angle_speed_rpm = 0.0f;
            return 0.0f;
        }
    }

    output = CalcPidOutput(&angle_pid,
                           error,
                           BSP_CHASSIS_ANGLE_KP,
                           0.0f,
                           BSP_CHASSIS_ANGLE_KD,
                           0.0f,
                           now);
    output -= gyro_feedback * BSP_CHASSIS_ANGLE_GYRO_KD;
    output *= (float)BSP_CHASSIS_YAW_CTRL_DIR;
    if (output > 0.0f)
    {
        float min_output = BSP_CHASSIS_ANGLE_MIN_RPM *
                           LimitFloat(error / BSP_CHASSIS_ANGLE_MIN_RPM_FULL_ERR_DEG, 1.0f);
        if (output < min_output)
        {
            output = min_output;
        }
    }
    else if (output < 0.0f)
    {
        float min_output = BSP_CHASSIS_ANGLE_MIN_RPM *
                           LimitFloat((-error) / BSP_CHASSIS_ANGLE_MIN_RPM_FULL_ERR_DEG, 1.0f);
        if (output > -min_output)
        {
            output = -min_output;
        }
    }
    output = LimitFloat(output, BSP_CHASSIS_ANGLE_MAX_RPM);
    chassis_last_angle_speed_rpm = output;

    return output;
}

const BspChassisMotorCurrent *BspChassis_GetLastCurrent(void)
{
    return &chassis_last_current;
}

const BspChassisWheelSpeedTarget *BspChassis_GetLastWheelSpeedTarget(void)
{
    return &chassis_last_speed_target;
}

float BspChassis_GetLastAngleErrorDeg(void)
{
    return chassis_last_angle_error_deg;
}

float BspChassis_GetLastAngleSpeedRpm(void)
{
    return chassis_last_angle_speed_rpm;
}
