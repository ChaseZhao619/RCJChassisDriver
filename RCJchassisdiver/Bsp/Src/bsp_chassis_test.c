#include "bsp_chassis_test.h"

#include "bsp_chassis.h"
#include "bsp_motor.h"
#include "bsp_usart.h"

typedef struct
{
    float move_direction_deg;
    float move_rpm;
    uint32_t duration_ms;
    const char *name;
} BspChassisTestStep;

static const BspChassisTestStep test_steps[] = {
    {0.0f, 0.0f, 1500U, "idle"},
    {0.0f, BSP_CHASSIS_TEST_MOVE_RPM, 2500U, "dir_0_forward"},
    {45.0f, BSP_CHASSIS_TEST_MOVE_RPM, 2500U, "dir_45"},
    {90.0f, BSP_CHASSIS_TEST_MOVE_RPM, 2500U, "dir_90_left"},
    {135.0f, BSP_CHASSIS_TEST_MOVE_RPM, 2500U, "dir_135"},
    {180.0f, BSP_CHASSIS_TEST_MOVE_RPM, 2500U, "dir_180_back"},
    {225.0f, BSP_CHASSIS_TEST_MOVE_RPM, 2500U, "dir_225"},
    {270.0f, BSP_CHASSIS_TEST_MOVE_RPM, 2500U, "dir_270_right"},
    {315.0f, BSP_CHASSIS_TEST_MOVE_RPM, 2500U, "dir_315"},
    {0.0f, 0.0f, 1500U, "idle"},
};

static uint8_t current_step;
static uint8_t target_yaw_ready;
static float target_yaw_deg;
static uint32_t step_start_tick;
static uint32_t last_send_tick;
static uint32_t last_print_tick;

static void SendStep(const BspChassisTestStep *step, float yaw_deg)
{
    (void)BspChassis_SetPolarSpeedAngleHold(step->move_direction_deg,
                                            step->move_rpm,
                                            target_yaw_deg,
                                            yaw_deg,
                                            BSP_CHASSIS_TEST_MAX_CURRENT);
}

#if BSP_CHASSIS_TEST_PRINT_ENABLE
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

static void PrintStep(const BspChassisTestStep *step, float yaw_deg)
{
    const BspChassisMotorCurrent *current = BspChassis_GetLastCurrent();
    const BspChassisWheelSpeedTarget *target = BspChassis_GetLastWheelSpeedTarget();
    float error_deg = BspChassis_GetLastAngleErrorDeg();
    float ccw_rpm = BspChassis_GetLastAngleSpeedRpm();
    const BspMotorFeedback *m1 = BspMotor_GetFeedback(1U);
    const BspMotorFeedback *m2 = BspMotor_GetFeedback(2U);
    const BspMotorFeedback *m3 = BspMotor_GetFeedback(3U);
    const BspMotorFeedback *m4 = BspMotor_GetFeedback(4U);

    if ((current == NULL) || (target == NULL) ||
        (m1 == NULL) || (m2 == NULL) || (m3 == NULL) || (m4 == NULL))
    {
        return;
    }

    Printf(BSP_USART_6,
           "chassis_pid step=%s dir=%.1f move=%.1f yaw=%.2f target=%.2f err=%.2f ccw=%.1f tx=%d,%d,%d,%d\r\n",
           step->name,
           step->move_direction_deg,
           step->move_rpm,
           yaw_deg,
           target_yaw_deg,
           error_deg,
           ccw_rpm,
           current->motor1,
           current->motor2,
           current->motor3,
           current->motor4);
    Printf(BSP_USART_6,
           "wheel target=%.0f,%.0f,%.0f,%.0f rpm=%d,%d,%d,%d\r\n",
           target->motor1_rpm,
           target->motor2_rpm,
           target->motor3_rpm,
           target->motor4_rpm,
           m1->speed_rpm,
           m2->speed_rpm,
           m3->speed_rpm,
           m4->speed_rpm);
    PrintMotorFeedback(1U);
    PrintMotorFeedback(2U);
    PrintMotorFeedback(3U);
    PrintMotorFeedback(4U);
}
#endif

void BspChassisTest_Init(void)
{
#if BSP_CHASSIS_TEST_ENABLE
    current_step = 0U;
    step_start_tick = HAL_GetTick();
    last_send_tick = 0U;
    last_print_tick = 0U;
    target_yaw_ready = 0U;
    target_yaw_deg = 0.0f;
    (void)BspChassis_Stop();
#if BSP_CHASSIS_TEST_PRINT_ENABLE
    Printf(BSP_USART_6,
           "chassis_pid_test start move_rpm=%.1f max_current=%d\r\n",
           BSP_CHASSIS_TEST_MOVE_RPM,
           BSP_CHASSIS_TEST_MAX_CURRENT);
    Printf(BSP_USART_6,
           "angle_wrap test 1-359=%.2f 359-1=%.2f\r\n",
           BspChassis_GetAngleErrorDeg(1.0f, 359.0f),
           BspChassis_GetAngleErrorDeg(359.0f, 1.0f));
#endif
#else
    (void)BspChassis_Stop();
#endif
}

void BspChassisTest_Task(uint8_t yaw_valid, float yaw_deg)
{
#if BSP_CHASSIS_TEST_ENABLE
    uint32_t now = HAL_GetTick();
    const BspChassisTestStep *step = &test_steps[current_step];

    if (yaw_valid == 0U)
    {
        (void)BspChassis_Stop();
        return;
    }

    yaw_deg = BspChassis_WrapAngle360(yaw_deg);

    if (target_yaw_ready == 0U)
    {
        target_yaw_ready = 1U;
        target_yaw_deg = yaw_deg;
        step_start_tick = now;
#if BSP_CHASSIS_TEST_PRINT_ENABLE
        Printf(BSP_USART_6, "chassis_test target_yaw=%.2f\r\n", target_yaw_deg);
#endif
    }

    if ((now - step_start_tick) >= step->duration_ms)
    {
        current_step++;
        if (current_step >= (sizeof(test_steps) / sizeof(test_steps[0])))
        {
            current_step = 0U;
        }

        step = &test_steps[current_step];
        step_start_tick = now;
        SendStep(step, yaw_deg);
#if BSP_CHASSIS_TEST_PRINT_ENABLE
        Printf(BSP_USART_6, "chassis_test step=%s\r\n", step->name);
#endif
    }

    if ((now - last_send_tick) >= 10U)
    {
        last_send_tick = now;
        SendStep(step, yaw_deg);
    }

    if ((now - last_print_tick) >= 300U)
    {
        last_print_tick = now;
#if BSP_CHASSIS_TEST_PRINT_ENABLE
        PrintStep(step, yaw_deg);
#endif
    }
#else
    (void)BspChassis_Stop();
#endif
}
