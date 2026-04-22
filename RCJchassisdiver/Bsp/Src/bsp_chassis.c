#include "bsp_chassis.h"

#include "bsp_motor.h"

static BspChassisMotorCurrent chassis_last_current;
static const int8_t chassis_motor_dir[4] = {
    BSP_CHASSIS_MOTOR1_DIR,
    BSP_CHASSIS_MOTOR2_DIR,
    BSP_CHASSIS_MOTOR3_DIR,
    BSP_CHASSIS_MOTOR4_DIR,
};

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

static int16_t ApplyMotorDir(int16_t current, uint8_t motor_index)
{
    if (chassis_motor_dir[motor_index] < 0)
    {
        return (int16_t)-current;
    }

    return current;
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
    return BspChassis_SetMotorCurrents(0, 0, 0, 0);
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

const BspChassisMotorCurrent *BspChassis_GetLastCurrent(void)
{
    return &chassis_last_current;
}
