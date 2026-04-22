#include "bsp_chassis_odom_calib_test.h"

#include "bsp_chassis.h"
#include "bsp_chassis_odom.h"
#include "bsp_usart.h"

typedef enum
{
    ODOM_CALIB_WAIT_YAW = 0,
    ODOM_CALIB_START_DELAY,
    ODOM_CALIB_FORWARD,
    ODOM_CALIB_FORWARD_HOLD,
    ODOM_CALIB_LEFT,
    ODOM_CALIB_LEFT_HOLD,
    ODOM_CALIB_DONE
} OdomCalibPhase;

static OdomCalibPhase calib_phase;
static float target_yaw_deg;
static uint32_t phase_start_tick;
static uint32_t last_print_tick;

static float AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float ClampSpeed(float speed_mm_s)
{
    if (speed_mm_s > BSP_CHASSIS_ODOM_CALIB_MAX_SPEED_MM_S)
    {
        speed_mm_s = BSP_CHASSIS_ODOM_CALIB_MAX_SPEED_MM_S;
    }

    if ((speed_mm_s > 0.0f) && (speed_mm_s < BSP_CHASSIS_ODOM_CALIB_MIN_SPEED_MM_S))
    {
        speed_mm_s = BSP_CHASSIS_ODOM_CALIB_MIN_SPEED_MM_S;
    }

    return speed_mm_s;
}

static void StopNoReset(void)
{
    (void)BspChassis_SetBodySpeed(0.0f,
                                  0.0f,
                                  0.0f,
                                  BSP_CHASSIS_ODOM_MAX_CURRENT);
}

static void PrintPhase(const char *name, const BspChassisOdomPose *pose)
{
#if BSP_CHASSIS_ODOM_CALIB_PRINT_ENABLE
    if (pose == NULL)
    {
        return;
    }

    Printf(BSP_USART_6,
           "odom_calib %s pose=%.1f,%.1f yaw=%.2f\r\n",
           name,
           pose->x_mm,
           pose->y_mm,
           pose->yaw_deg);
#else
    (void)name;
    (void)pose;
#endif
}

void BspChassisOdomCalibTest_Init(void)
{
    calib_phase = ODOM_CALIB_WAIT_YAW;
    target_yaw_deg = 0.0f;
    phase_start_tick = 0U;
    last_print_tick = 0U;
    StopNoReset();
}

void BspChassisOdomCalibTest_Task(uint8_t yaw_valid, float yaw_deg)
{
    uint32_t now = HAL_GetTick();
    const BspChassisOdomPose *pose;
    float remaining_mm;
    float speed_mm_s;
    float speed_rpm;

    if (yaw_valid == 0U)
    {
        StopNoReset();
        return;
    }

    yaw_deg = BspChassis_WrapAngle360(yaw_deg);

    if (calib_phase == ODOM_CALIB_WAIT_YAW)
    {
        target_yaw_deg = yaw_deg;
        BspChassisOdom_Reset(0.0f, 0.0f, yaw_deg);
        phase_start_tick = now;
        calib_phase = ODOM_CALIB_START_DELAY;
        PrintPhase("start_delay", BspChassisOdom_GetPose());
    }

    BspChassisOdom_Update(yaw_deg);
    pose = BspChassisOdom_GetPose();

    switch (calib_phase)
    {
    case ODOM_CALIB_START_DELAY:
        StopNoReset();
        if ((now - phase_start_tick) >= BSP_CHASSIS_ODOM_CALIB_START_DELAY_MS)
        {
            BspChassisOdom_Reset(0.0f, 0.0f, yaw_deg);
            BspChassis_ResetPid();
            phase_start_tick = now;
            calib_phase = ODOM_CALIB_FORWARD;
            PrintPhase("forward_begin", BspChassisOdom_GetPose());
        }
        break;

    case ODOM_CALIB_FORWARD:
        remaining_mm = BSP_CHASSIS_ODOM_CALIB_DISTANCE_MM - pose->x_mm;
        if ((remaining_mm <= 0.0f) ||
            ((now - phase_start_tick) >= BSP_CHASSIS_ODOM_CALIB_MAX_RUN_MS))
        {
            StopNoReset();
            phase_start_tick = now;
            calib_phase = ODOM_CALIB_FORWARD_HOLD;
            PrintPhase("forward_done", pose);
            break;
        }

        speed_mm_s = ClampSpeed(remaining_mm * 1.5f);
        speed_rpm = BspChassisOdom_MmSToMotorRpm(speed_mm_s);
        (void)BspChassis_SetBodySpeedAngleHold(speed_rpm,
                                               0.0f,
                                               target_yaw_deg,
                                               yaw_deg,
                                               BSP_CHASSIS_ODOM_MAX_CURRENT);
        break;

    case ODOM_CALIB_FORWARD_HOLD:
        StopNoReset();
        if ((now - phase_start_tick) >= BSP_CHASSIS_ODOM_CALIB_HOLD_MS)
        {
            BspChassisOdom_Reset(0.0f, 0.0f, yaw_deg);
            BspChassis_ResetPid();
            phase_start_tick = now;
            calib_phase = ODOM_CALIB_LEFT;
            PrintPhase("left_begin", BspChassisOdom_GetPose());
        }
        break;

    case ODOM_CALIB_LEFT:
        remaining_mm = BSP_CHASSIS_ODOM_CALIB_DISTANCE_MM - pose->y_mm;
        if ((remaining_mm <= 0.0f) ||
            ((now - phase_start_tick) >= BSP_CHASSIS_ODOM_CALIB_MAX_RUN_MS))
        {
            StopNoReset();
            phase_start_tick = now;
            calib_phase = ODOM_CALIB_LEFT_HOLD;
            PrintPhase("left_done", pose);
            break;
        }

        speed_mm_s = ClampSpeed(remaining_mm * 1.5f);
        speed_rpm = BspChassisOdom_MmSToMotorRpm(speed_mm_s);
        (void)BspChassis_SetBodySpeedAngleHold(0.0f,
                                               speed_rpm,
                                               target_yaw_deg,
                                               yaw_deg,
                                               BSP_CHASSIS_ODOM_MAX_CURRENT);
        break;

    case ODOM_CALIB_LEFT_HOLD:
        StopNoReset();
        if ((now - phase_start_tick) >= BSP_CHASSIS_ODOM_CALIB_HOLD_MS)
        {
            calib_phase = ODOM_CALIB_DONE;
            PrintPhase("done", pose);
        }
        break;

    case ODOM_CALIB_DONE:
    default:
        StopNoReset();
        break;
    }

#if BSP_CHASSIS_ODOM_CALIB_PRINT_ENABLE
    if ((now - last_print_tick) >= 200U)
    {
        last_print_tick = now;
        Printf(BSP_USART_6,
               "odom_calib phase=%u pose=%.1f,%.1f dist=%.1f\r\n",
               (unsigned int)calib_phase,
               pose->x_mm,
               pose->y_mm,
               AbsFloat(pose->x_mm) + AbsFloat(pose->y_mm));
    }
#else
    (void)last_print_tick;
    (void)AbsFloat(0.0f);
#endif
}
