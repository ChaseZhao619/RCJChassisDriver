#include "bsp_chassis_angle_test.h"

#include "bsp_chassis.h"
#include "bsp_usart.h"

static uint8_t angle_test_ready;
static float angle_test_zero_yaw_deg;
static float angle_test_target_yaw_deg;
static uint32_t last_print_tick;

static void PrintHelp(void)
{
    Printf(BSP_USART_6,
           "angle_test: z=zero h=hold 0/9/1/2=0/90/180/270 [ ]=-/+10deg\r\n");
}

static void HandleCommand(uint8_t cmd, float yaw_deg)
{
    switch (cmd)
    {
    case 'z':
    case 'Z':
        angle_test_zero_yaw_deg = yaw_deg;
        angle_test_target_yaw_deg = yaw_deg;
        BspChassis_ResetPid();
        Printf(BSP_USART_6, "angle_test zero=%.2f\r\n", yaw_deg);
        break;
    case 'h':
    case 'H':
        angle_test_target_yaw_deg = yaw_deg;
        BspChassis_ResetPid();
        Printf(BSP_USART_6, "angle_test hold=%.2f\r\n", yaw_deg);
        break;
    case '0':
        angle_test_target_yaw_deg = angle_test_zero_yaw_deg;
        BspChassis_ResetPid();
        break;
    case '9':
        angle_test_target_yaw_deg = BspChassis_WrapAngle360(angle_test_zero_yaw_deg + 90.0f);
        BspChassis_ResetPid();
        break;
    case '1':
        angle_test_target_yaw_deg = BspChassis_WrapAngle360(angle_test_zero_yaw_deg + 180.0f);
        BspChassis_ResetPid();
        break;
    case '2':
        angle_test_target_yaw_deg = BspChassis_WrapAngle360(angle_test_zero_yaw_deg + 270.0f);
        BspChassis_ResetPid();
        break;
    case '[':
        angle_test_target_yaw_deg = BspChassis_WrapAngle360(angle_test_target_yaw_deg - 10.0f);
        BspChassis_ResetPid();
        break;
    case ']':
        angle_test_target_yaw_deg = BspChassis_WrapAngle360(angle_test_target_yaw_deg + 10.0f);
        BspChassis_ResetPid();
        break;
    case '?':
        PrintHelp();
        break;
    default:
        break;
    }
}

void BspChassisAngleTest_Init(void)
{
#if BSP_CHASSIS_ANGLE_TEST_ENABLE
    angle_test_ready = 0U;
    angle_test_zero_yaw_deg = 0.0f;
    angle_test_target_yaw_deg = 0.0f;
    last_print_tick = 0U;
    (void)BspChassis_SetBodySpeed(0.0f, 0.0f, 0.0f, BSP_CHASSIS_ANGLE_TEST_MAX_CURRENT);
    PrintHelp();
#endif
}

void BspChassisAngleTest_Task(uint8_t yaw_valid, float yaw_deg)
{
#if BSP_CHASSIS_ANGLE_TEST_ENABLE
    uint8_t cmd;
    uint32_t now = HAL_GetTick();

    if (yaw_valid == 0U)
    {
        (void)BspChassis_SetBodySpeed(0.0f, 0.0f, 0.0f, BSP_CHASSIS_ANGLE_TEST_MAX_CURRENT);
        return;
    }

    yaw_deg = BspChassis_WrapAngle360(yaw_deg);
    if (angle_test_ready == 0U)
    {
        angle_test_ready = 1U;
        angle_test_zero_yaw_deg = yaw_deg;
        angle_test_target_yaw_deg = yaw_deg;
        BspChassis_ResetPid();
        Printf(BSP_USART_6, "angle_test ready zero=%.2f\r\n", yaw_deg);
    }

    if (BspUsart_Receive(BSP_USART_6, &cmd, 1U, 0U) == HAL_OK)
    {
        HandleCommand(cmd, yaw_deg);
    }

    (void)BspChassis_SetBodySpeedAngleHold(0.0f,
                                           0.0f,
                                           angle_test_target_yaw_deg,
                                           yaw_deg,
                                           BSP_CHASSIS_ANGLE_TEST_MAX_CURRENT);

    if ((now - last_print_tick) >= BSP_CHASSIS_ANGLE_TEST_PRINT_MS)
    {
        last_print_tick = now;
        Printf(BSP_USART_6,
               "angle target=%.2f yaw=%.2f err=%.2f ccw=%.1f\r\n",
               angle_test_target_yaw_deg,
               yaw_deg,
               BspChassis_GetLastAngleErrorDeg(),
               BspChassis_GetLastAngleSpeedRpm());
    }
#else
    (void)yaw_valid;
    (void)yaw_deg;
#endif
}
