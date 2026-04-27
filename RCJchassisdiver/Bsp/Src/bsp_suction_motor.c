#include "bsp_suction_motor.h"

#include "tim.h"

static uint16_t suction_motor_pulse_us = BSP_SUCTION_MOTOR_INIT_PULSE_US;
static uint8_t suction_motor_speed_percent;
static uint8_t suction_motor_started;

static HAL_StatusTypeDef StartPwmIfNeeded(void)
{
    HAL_StatusTypeDef status;

    if (suction_motor_started != 0U)
    {
        return HAL_OK;
    }

    status = HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    if (status == HAL_OK)
    {
        suction_motor_started = 1U;
    }

    return status;
}

HAL_StatusTypeDef BspSuctionMotor_Init(void)
{
    HAL_StatusTypeDef status;

    status = BspSuctionMotor_SetInitPulse();
    if (status != HAL_OK)
    {
        return status;
    }

    return StartPwmIfNeeded();
}

HAL_StatusTypeDef BspSuctionMotor_SetInitPulse(void)
{
    return BspSuctionMotor_SetPulseUs(BSP_SUCTION_MOTOR_INIT_PULSE_US);
}

HAL_StatusTypeDef BspSuctionMotor_SetPulseUs(uint16_t pulse_us)
{
    if ((pulse_us < BSP_SUCTION_MOTOR_INIT_MIN_US) ||
        (pulse_us > BSP_SUCTION_MOTOR_RUN_MAX_US))
    {
        return HAL_ERROR;
    }

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse_us);
    suction_motor_pulse_us = pulse_us;
    suction_motor_speed_percent = 0U;

    return StartPwmIfNeeded();
}

HAL_StatusTypeDef BspSuctionMotor_SetThrottlePermille(uint16_t throttle_permille)
{
    uint32_t pulse_us;

    if (throttle_permille > 1000U)
    {
        return HAL_ERROR;
    }

    pulse_us = BSP_SUCTION_MOTOR_RUN_MIN_US +
               (((uint32_t)(BSP_SUCTION_MOTOR_RUN_MAX_US - BSP_SUCTION_MOTOR_RUN_MIN_US) *
                 throttle_permille) /
                1000U);

    return BspSuctionMotor_SetPulseUs((uint16_t)pulse_us);
}

HAL_StatusTypeDef BspSuctionMotor_SetSpeedPercent(uint8_t speed_percent)
{
    uint32_t pulse_us;

    if (speed_percent > 100U)
    {
        return HAL_ERROR;
    }

    if (speed_percent == 0U)
    {
        return BspSuctionMotor_SetInitPulse();
    }

    if (speed_percent >= 100U)
    {
        pulse_us = BSP_SUCTION_MOTOR_RUN_MAX_US;
    }
    else
    {
        pulse_us = BSP_SUCTION_MOTOR_RUN_MIN_US +
                   (((uint32_t)(BSP_SUCTION_MOTOR_RUN_MAX_US - BSP_SUCTION_MOTOR_RUN_MIN_US) *
                     (uint32_t)(speed_percent - 1U)) /
                    99U);
    }

    if (BspSuctionMotor_SetPulseUs((uint16_t)pulse_us) != HAL_OK)
    {
        return HAL_ERROR;
    }

    suction_motor_speed_percent = speed_percent;
    return HAL_OK;
}

uint16_t BspSuctionMotor_GetPulseUs(void)
{
    return suction_motor_pulse_us;
}

uint8_t BspSuctionMotor_GetSpeedPercent(void)
{
    return suction_motor_speed_percent;
}
