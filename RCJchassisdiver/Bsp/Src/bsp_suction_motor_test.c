#include "bsp_suction_motor_test.h"

#include "bsp_suction_motor.h"
#include "bsp_usart.h"

#if BSP_SUCTION_MOTOR_TEST_ENABLE
typedef struct
{
    uint16_t pulse_us;
    uint32_t duration_ms;
    const char *name;
} BspSuctionMotorTestStep;

static const BspSuctionMotorTestStep suction_motor_test_steps[] = {
    {BSP_SUCTION_MOTOR_INIT_PULSE_US, BSP_SUCTION_MOTOR_TEST_INIT_HOLD_MS, "init"},
    {BSP_SUCTION_MOTOR_RUN_MIN_US, BSP_SUCTION_MOTOR_TEST_STEP_HOLD_MS, "start"},
    {1100U, BSP_SUCTION_MOTOR_TEST_STEP_HOLD_MS, "low"},
    {BSP_SUCTION_MOTOR_TEST_MAX_PULSE_US, BSP_SUCTION_MOTOR_TEST_STEP_HOLD_MS, "test_max"},
    {BSP_SUCTION_MOTOR_INIT_PULSE_US, BSP_SUCTION_MOTOR_TEST_INIT_HOLD_MS, "idle"},
};

static uint8_t suction_motor_test_step;
static uint32_t suction_motor_test_step_tick;
static uint32_t suction_motor_test_print_tick;

static void ApplyStep(const BspSuctionMotorTestStep *step)
{
    if (step == NULL)
    {
        return;
    }

    (void)BspSuctionMotor_SetPulseUs(step->pulse_us);
}
#endif

void BspSuctionMotorTest_Init(void)
{
#if BSP_SUCTION_MOTOR_TEST_ENABLE
    suction_motor_test_step = 0U;
    suction_motor_test_step_tick = HAL_GetTick();
    suction_motor_test_print_tick = 0U;
    ApplyStep(&suction_motor_test_steps[suction_motor_test_step]);
    Printf(BSP_SUCTION_MOTOR_TEST_PRINT_USART,
           "suction_motor_test start pulse=%u\r\n",
           BspSuctionMotor_GetPulseUs());
#endif
}

void BspSuctionMotorTest_Task(void)
{
#if BSP_SUCTION_MOTOR_TEST_ENABLE
    uint32_t now = HAL_GetTick();
    const BspSuctionMotorTestStep *step = &suction_motor_test_steps[suction_motor_test_step];

    if ((now - suction_motor_test_step_tick) >= step->duration_ms)
    {
        suction_motor_test_step++;
        if (suction_motor_test_step >=
            (sizeof(suction_motor_test_steps) / sizeof(suction_motor_test_steps[0])))
        {
            suction_motor_test_step = 0U;
        }

        step = &suction_motor_test_steps[suction_motor_test_step];
        suction_motor_test_step_tick = now;
        ApplyStep(step);
        Printf(BSP_SUCTION_MOTOR_TEST_PRINT_USART,
               "suction_motor_test step=%s pulse=%u\r\n",
               step->name,
               BspSuctionMotor_GetPulseUs());
    }

    if ((now - suction_motor_test_print_tick) >= 500U)
    {
        suction_motor_test_print_tick = now;
        Printf(BSP_SUCTION_MOTOR_TEST_PRINT_USART,
               "suction_motor_test step=%s pulse=%u\r\n",
               step->name,
               BspSuctionMotor_GetPulseUs());
    }
#endif
}
