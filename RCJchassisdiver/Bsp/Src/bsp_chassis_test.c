#include "bsp_chassis_test.h"

#include "bsp_chassis.h"
#include "bsp_motor.h"
#include "bsp_usart.h"

typedef struct
{
    int16_t forward_current;
    int16_t left_current;
    int16_t ccw_current;
    uint32_t duration_ms;
    const char *name;
} BspChassisTestStep;

static const BspChassisTestStep test_steps[] = {
    {0, 0, 0, 1500U, "idle"},
    {BSP_CHASSIS_TEST_CURRENT, 0, 0, 1800U, "forward"},
    {0, 0, 0, 600U, "zero"},
    {-BSP_CHASSIS_TEST_CURRENT, 0, 0, 1800U, "backward"},
    {0, 0, 0, 600U, "zero"},
    {0, BSP_CHASSIS_TEST_CURRENT, 0, 1800U, "left"},
    {0, 0, 0, 600U, "zero"},
    {0, -BSP_CHASSIS_TEST_CURRENT, 0, 1800U, "right"},
    {0, 0, 0, 600U, "zero"},
    {0, 0, BSP_CHASSIS_TEST_CURRENT, 1800U, "ccw"},
    {0, 0, 0, 600U, "zero"},
    {0, 0, -BSP_CHASSIS_TEST_CURRENT, 1800U, "cw"},
    {0, 0, 0, 1500U, "idle"},
};

static uint8_t current_step;
static uint32_t step_start_tick;
static uint32_t last_send_tick;
static uint32_t last_print_tick;

static void SendStep(const BspChassisTestStep *step)
{
    (void)BspChassis_SetOpenLoopLimited(step->forward_current,
                                        step->left_current,
                                        step->ccw_current,
                                        BSP_CHASSIS_TEST_MAX_CURRENT);
}

static void PrintMotorFeedback(uint8_t can_id)
{
    const BspMotorFeedback *feedback = BspMotor_GetFeedback(can_id);

    if (feedback == NULL)
    {
        return;
    }

    Printf(BSP_USART_6,
           "m%u:on=%u rpm=%d cur=%d temp=%u\r\n",
           can_id,
           BspMotor_IsOnline(can_id, 200U),
           feedback->speed_rpm,
           feedback->given_current,
           feedback->temperature);
}

static void PrintStep(const BspChassisTestStep *step)
{
    const BspChassisMotorCurrent *current = BspChassis_GetLastCurrent();

    if (current == NULL)
    {
        return;
    }

    Printf(BSP_USART_6,
           "chassis_test step=%s input=%d,%d,%d tx=%d,%d,%d,%d\r\n",
           step->name,
           step->forward_current,
           step->left_current,
           step->ccw_current,
           current->motor1,
           current->motor2,
           current->motor3,
           current->motor4);
    PrintMotorFeedback(1U);
    PrintMotorFeedback(2U);
    PrintMotorFeedback(3U);
    PrintMotorFeedback(4U);
}

void BspChassisTest_Init(void)
{
#if BSP_CHASSIS_TEST_ENABLE
    current_step = 0U;
    step_start_tick = HAL_GetTick();
    last_send_tick = 0U;
    last_print_tick = 0U;
    SendStep(&test_steps[current_step]);
    Printf(BSP_USART_6,
           "chassis_test start current=%d max=%d\r\n",
           BSP_CHASSIS_TEST_CURRENT,
           BSP_CHASSIS_TEST_MAX_CURRENT);
#else
    (void)BspChassis_Stop();
#endif
}

void BspChassisTest_Task(void)
{
#if BSP_CHASSIS_TEST_ENABLE
    uint32_t now = HAL_GetTick();
    const BspChassisTestStep *step = &test_steps[current_step];

    if ((now - step_start_tick) >= step->duration_ms)
    {
        current_step++;
        if (current_step >= (sizeof(test_steps) / sizeof(test_steps[0])))
        {
            current_step = 0U;
        }

        step = &test_steps[current_step];
        step_start_tick = now;
        SendStep(step);
        Printf(BSP_USART_6, "chassis_test step=%s\r\n", step->name);
    }

    if ((now - last_send_tick) >= 10U)
    {
        last_send_tick = now;
        SendStep(step);
    }

    if ((now - last_print_tick) >= 300U)
    {
        last_print_tick = now;
        PrintStep(step);
    }
#else
    (void)BspChassis_Stop();
#endif
}
