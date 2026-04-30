#include "bsp_kick_motor.h"

#include "bsp_motor.h"

typedef struct
{
    float integral;
    float last_error;
    uint32_t last_tick;
    uint8_t initialized;
} BspKickMotorPidState;

static BspKickMotorPidState kick_motor_pid;
static uint8_t kick_motor_speed_percent;
static uint8_t kick_motor_reverse;
static float kick_motor_target_rpm;
static int16_t kick_motor_last_current;
static uint32_t kick_motor_last_task_tick;

static float AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
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

static int16_t LimitCurrent(int16_t current)
{
    int16_t limit = BSP_KICK_MOTOR_MAX_CURRENT;

    if (limit < 0)
    {
        limit = (int16_t)-limit;
    }
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

static void ResetPid(void)
{
    kick_motor_pid.integral = 0.0f;
    kick_motor_pid.last_error = 0.0f;
    kick_motor_pid.last_tick = 0U;
    kick_motor_pid.initialized = 0U;
}

static float CalcPidOutput(float error, float target_rpm, uint32_t now)
{
    float dt = 0.01f;
    float derivative = 0.0f;

    if (kick_motor_pid.initialized != 0U)
    {
        uint32_t dt_ms = now - kick_motor_pid.last_tick;
        if ((dt_ms > 0U) && (dt_ms < 200U))
        {
            dt = (float)dt_ms * 0.001f;
        }
        derivative = (error - kick_motor_pid.last_error) / dt;
    }
    else
    {
        kick_motor_pid.initialized = 1U;
    }

    kick_motor_pid.integral += error * dt;
    kick_motor_pid.integral = LimitFloat(kick_motor_pid.integral,
                                         BSP_KICK_MOTOR_SPEED_I_LIMIT);
    kick_motor_pid.last_error = error;
    kick_motor_pid.last_tick = now;

    return (BSP_KICK_MOTOR_SPEED_KP * error) +
           (BSP_KICK_MOTOR_SPEED_KI * kick_motor_pid.integral) +
           (BSP_KICK_MOTOR_SPEED_KD * derivative) +
           (BSP_KICK_MOTOR_SPEED_KF * target_rpm);
}

static float GetCurrentRpm(void)
{
    const BspMotorFeedback *feedback = BspMotor_GetFeedback(BSP_KICK_MOTOR_CAN_ID);

    if (feedback == NULL)
    {
        return 0.0f;
    }

    return (float)feedback->speed_rpm * (float)BSP_KICK_MOTOR_FB_DIR;
}

HAL_StatusTypeDef BspKickMotor_Init(void)
{
    kick_motor_speed_percent = 0U;
    kick_motor_reverse = 0U;
    kick_motor_target_rpm = 0.0f;
    kick_motor_last_current = 0;
    kick_motor_last_task_tick = 0U;
    ResetPid();

    return BspMotor_SendFunctionCurrent(0);
}

HAL_StatusTypeDef BspKickMotor_SetSpeed(uint8_t speed_percent, uint8_t reverse)
{
    float target_rpm;

    if ((speed_percent > 100U) || (reverse > 1U))
    {
        return HAL_ERROR;
    }

    if (speed_percent == 0U)
    {
        return BspKickMotor_Stop();
    }

    target_rpm = ((float)speed_percent * BSP_KICK_MOTOR_MAX_RPM) / 100.0f;
    if (reverse != 0U)
    {
        target_rpm = -target_rpm;
    }
    target_rpm *= (float)BSP_KICK_MOTOR_DIR;

    if ((kick_motor_speed_percent != speed_percent) ||
        (kick_motor_reverse != reverse))
    {
        ResetPid();
    }

    kick_motor_speed_percent = speed_percent;
    kick_motor_reverse = reverse;
    kick_motor_target_rpm = target_rpm;

    return HAL_OK;
}

HAL_StatusTypeDef BspKickMotor_Stop(void)
{
    kick_motor_speed_percent = 0U;
    kick_motor_target_rpm = 0.0f;
    kick_motor_last_current = 0;
    ResetPid();

    return BspMotor_SendFunctionCurrent(0);
}

void BspKickMotor_Task(void)
{
    uint32_t now = HAL_GetTick();
    float current_rpm;
    float current_output;

    if (kick_motor_speed_percent == 0U)
    {
        return;
    }

    if ((now - kick_motor_last_task_tick) < BSP_KICK_MOTOR_TASK_PERIOD_MS)
    {
        return;
    }
    kick_motor_last_task_tick = now;

    current_rpm = GetCurrentRpm();
    current_output = CalcPidOutput(kick_motor_target_rpm - current_rpm,
                                   kick_motor_target_rpm,
                                   now);
    kick_motor_last_current = LimitCurrent(FloatToCurrent(current_output));

    (void)BspMotor_SendFunctionCurrent(kick_motor_last_current);
}

uint8_t BspKickMotor_GetSpeedPercent(void)
{
    return kick_motor_speed_percent;
}

uint8_t BspKickMotor_GetReverse(void)
{
    return kick_motor_reverse;
}

float BspKickMotor_GetTargetRpm(void)
{
    return kick_motor_target_rpm;
}

int16_t BspKickMotor_GetLastCurrent(void)
{
    return kick_motor_last_current;
}
