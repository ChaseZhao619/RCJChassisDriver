#include "bsp_motor_test.h"

#include "bsp_motor.h"
#include "bsp_usart.h"

typedef struct
{
    int16_t current[BSP_MOTOR_COUNT];
    uint32_t duration_ms;
    const char *name;
} BspMotorTestStep;

static const BspMotorTestStep test_steps[] = {
    {{0, 0, 0, 0, 0}, 1000U, "idle"},
    {{BSP_MOTOR_TEST_CURRENT, 0, 0, 0, 0}, 1500U, "m1+"},
    {{0, 0, 0, 0, 0}, 500U, "zero"},
    {{0, BSP_MOTOR_TEST_CURRENT, 0, 0, 0}, 1500U, "m2+"},
    {{0, 0, 0, 0, 0}, 500U, "zero"},
    {{0, 0, BSP_MOTOR_TEST_CURRENT, 0, 0}, 1500U, "m3+"},
    {{0, 0, 0, 0, 0}, 500U, "zero"},
    {{0, 0, 0, BSP_MOTOR_TEST_CURRENT, 0}, 1500U, "m4+"},
    {{0, 0, 0, 0, 0}, 500U, "zero"},
    {{0, 0, 0, 0, BSP_MOTOR_TEST_CURRENT}, 1500U, "m5+"},
    {{0, 0, 0, 0, 0}, 1000U, "idle"},
    {{-BSP_MOTOR_TEST_CURRENT, 0, 0, 0, 0}, 1500U, "m1-"},
    {{0, -BSP_MOTOR_TEST_CURRENT, 0, 0, 0}, 1500U, "m2-"},
    {{0, 0, -BSP_MOTOR_TEST_CURRENT, 0, 0}, 1500U, "m3-"},
    {{0, 0, 0, -BSP_MOTOR_TEST_CURRENT, 0}, 1500U, "m4-"},
    {{0, 0, 0, 0, -BSP_MOTOR_TEST_CURRENT}, 1500U, "m5-"},
    {{0, 0, 0, 0, 0}, 1500U, "idle"},
};

static uint8_t current_step;
static uint32_t step_start_tick;
static uint32_t last_send_tick;
static uint32_t last_print_tick;

static void SendCurrentStep(const BspMotorTestStep *step)
{
    (void)BspMotor_SetCurrents(step->current[0],
                              step->current[1],
                              step->current[2],
                              step->current[3],
                              step->current[4]);
}

static void PrintMotorFeedback(uint8_t can_id)
{
    const BspMotorFeedback *feedback = BspMotor_GetFeedback(can_id);

    if (feedback == NULL)
    {
        return;
    }

    Printf(BSP_USART_6,
           "m%u:on=%u ecd=%u rpm=%d cur=%d temp=%u total=%ld\r\n",
           can_id,
           BspMotor_IsOnline(can_id, 200U),
           feedback->ecd,
           feedback->speed_rpm,
           feedback->given_current,
           feedback->temperature,
           (long)feedback->total_ecd);
}

void BspMotorTest_Init(void)
{
#if BSP_MOTOR_TEST_ENABLE
    current_step = 0U;
    step_start_tick = HAL_GetTick();
    last_send_tick = 0U;
    last_print_tick = 0U;
    SendCurrentStep(&test_steps[current_step]);
    Printf(BSP_USART_6, "motor_test start current=%d\r\n", BSP_MOTOR_TEST_CURRENT);
#else
    (void)BspMotor_SetCurrents(0, 0, 0, 0, 0);
#endif
}

void BspMotorTest_Task(void)
{
#if BSP_MOTOR_TEST_ENABLE
    uint32_t now = HAL_GetTick();
    const BspMotorTestStep *step = &test_steps[current_step];

    if ((now - step_start_tick) >= step->duration_ms)
    {
        current_step++;
        if (current_step >= (sizeof(test_steps) / sizeof(test_steps[0])))
        {
            current_step = 0U;
        }

        step = &test_steps[current_step];
        step_start_tick = now;
        SendCurrentStep(step);
        Printf(BSP_USART_6, "motor_test step=%s\r\n", step->name);
    }

    if ((now - last_send_tick) >= 10U)
    {
        last_send_tick = now;
        SendCurrentStep(step);
    }

    if ((now - last_print_tick) >= 250U)
    {
        last_print_tick = now;
        Printf(BSP_USART_6,
               "motor_test step=%s tx=%d,%d,%d,%d,%d\r\n",
               step->name,
               step->current[0],
               step->current[1],
               step->current[2],
               step->current[3],
               step->current[4]);
        PrintMotorFeedback(1U);
        PrintMotorFeedback(2U);
        PrintMotorFeedback(3U);
        PrintMotorFeedback(4U);
        PrintMotorFeedback(5U);
    }
#else
    (void)BspMotor_SetCurrents(0, 0, 0, 0, 0);
#endif
}
