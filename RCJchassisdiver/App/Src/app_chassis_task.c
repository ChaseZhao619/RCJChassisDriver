#include "app_chassis_task.h"

#include "bsp_chassis.h"
#include "bsp_chassis_odom.h"
#include "bsp_motor.h"

typedef enum
{
    APP_CHASSIS_STEP_WAIT_IMU = 0,
    APP_CHASSIS_STEP_START_DELAY,
    APP_CHASSIS_STEP_MOVE_FORWARD_1,
    APP_CHASSIS_STEP_HOLD_FORWARD_1,
    APP_CHASSIS_STEP_MOVE_RIGHT,
    APP_CHASSIS_STEP_HOLD_RIGHT,
    APP_CHASSIS_STEP_MOVE_LEFT,
    APP_CHASSIS_STEP_HOLD_LEFT,
    APP_CHASSIS_STEP_MOVE_FORWARD_2,
    APP_CHASSIS_STEP_HOLD_FORWARD_2,
    APP_CHASSIS_STEP_ROTATE_CLOCKWISE,
    APP_CHASSIS_STEP_DONE,
} AppChassisStep;

#define APP_TARGET_FORWARD_1_X_MM (APP_CHASSIS_TASK_FORWARD_1_DISTANCE_MM)
#define APP_TARGET_FORWARD_1_Y_MM (0.0f)
#define APP_TARGET_RIGHT_X_MM     (APP_CHASSIS_TASK_FORWARD_1_DISTANCE_MM)
#define APP_TARGET_RIGHT_Y_MM     (-APP_CHASSIS_TASK_RIGHT_DISTANCE_MM)
#define APP_TARGET_LEFT_X_MM      (APP_CHASSIS_TASK_FORWARD_1_DISTANCE_MM)
#define APP_TARGET_LEFT_Y_MM      (APP_CHASSIS_TASK_LEFT_DISTANCE_MM - APP_CHASSIS_TASK_RIGHT_DISTANCE_MM)
#define APP_TARGET_FORWARD_2_X_MM (APP_CHASSIS_TASK_FORWARD_1_DISTANCE_MM + APP_CHASSIS_TASK_FORWARD_2_DISTANCE_MM)
#define APP_TARGET_FORWARD_2_Y_MM (APP_CHASSIS_TASK_LEFT_DISTANCE_MM - APP_CHASSIS_TASK_RIGHT_DISTANCE_MM)

static AppChassisStep app_step;
static float app_start_yaw_deg;
static float app_rotate_target_yaw_deg;
static uint32_t app_step_tick;
static uint32_t app_stop_tick;

static int16_t Abs16(int16_t value)
{
    if (value == INT16_MIN)
    {
        return INT16_MAX;
    }

    return (value < 0) ? (int16_t)-value : value;
}

static uint8_t AreChassisMotorsStopped(void)
{
    uint8_t can_id;

    for (can_id = 1U; can_id <= BSP_MOTOR_CHASSIS_COUNT; can_id++)
    {
        const BspMotorFeedback *feedback = BspMotor_GetFeedback(can_id);

        if ((feedback == NULL) ||
            (BspMotor_IsOnline(can_id, 200U) == 0U) ||
            (Abs16(feedback->speed_rpm) > APP_CHASSIS_TASK_STOP_RPM))
        {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t IsStoppedStable(uint32_t now)
{
    if (AreChassisMotorsStopped() != 0U)
    {
        if (app_stop_tick == 0U)
        {
            app_stop_tick = now;
        }
    }
    else
    {
        app_stop_tick = 0U;
    }

    return ((app_stop_tick != 0U) &&
            ((now - app_stop_tick) >= APP_CHASSIS_TASK_STOP_STABLE_MS)) ? 1U : 0U;
}

static void SetStep(AppChassisStep step, uint32_t now)
{
    app_step = step;
    app_step_tick = now;
    app_stop_tick = 0U;
    BspChassis_ResetPid();
}

static void HoldThenNext(uint32_t now, AppChassisStep next_step)
{
    (void)BspChassis_Stop();

    if (((now - app_step_tick) >= APP_CHASSIS_TASK_HOLD_AFTER_MOVE_MS) &&
        (IsStoppedStable(now) != 0U))
    {
        SetStep(next_step, now);
    }
}

static uint8_t IsYawAtTarget(float target_yaw_deg, float current_yaw_deg)
{
    float error = BspChassis_GetAngleErrorDeg(target_yaw_deg, current_yaw_deg);

    return ((error <= APP_CHASSIS_TASK_ROTATE_TOLERANCE_DEG) &&
            (error >= -APP_CHASSIS_TASK_ROTATE_TOLERANCE_DEG)) ? 1U : 0U;
}

void AppChassisTask_Init(void)
{
    app_step = APP_CHASSIS_STEP_WAIT_IMU;
    app_start_yaw_deg = 0.0f;
    app_rotate_target_yaw_deg = 0.0f;
    app_step_tick = 0U;
    app_stop_tick = 0U;
    (void)BspChassis_Stop();
}

void AppChassisTask_Task(uint8_t yaw_valid,
                         float yaw_deg,
                         uint8_t gyro_valid,
                         float gyro_z_deg_s)
{
    uint32_t now = HAL_GetTick();

    if (yaw_valid == 0U)
    {
        (void)BspChassis_Stop();
        return;
    }

    yaw_deg = BspChassis_WrapAngle360(yaw_deg);
    if (gyro_valid == 0U)
    {
        gyro_z_deg_s = 0.0f;
    }

    if (app_step != APP_CHASSIS_STEP_WAIT_IMU)
    {
        BspChassisOdom_Update(yaw_deg);
    }

    switch (app_step)
    {
    case APP_CHASSIS_STEP_WAIT_IMU:
        app_start_yaw_deg = yaw_deg;
        app_rotate_target_yaw_deg = BspChassis_WrapAngle360(app_start_yaw_deg -
                                                            APP_CHASSIS_TASK_CLOCKWISE_YAW_DEG);
        BspChassisOdom_Reset(0.0f, 0.0f, yaw_deg);
        SetStep(APP_CHASSIS_STEP_START_DELAY, now);
        break;

    case APP_CHASSIS_STEP_START_DELAY:
        (void)BspChassis_Stop();
        if ((now - app_step_tick) >= APP_CHASSIS_TASK_START_DELAY_MS)
        {
            SetStep(APP_CHASSIS_STEP_MOVE_FORWARD_1, now);
        }
        break;

    case APP_CHASSIS_STEP_MOVE_FORWARD_1:
        if (BspChassisOdom_IsAt(APP_TARGET_FORWARD_1_X_MM,
                                APP_TARGET_FORWARD_1_Y_MM,
                                BSP_CHASSIS_ODOM_POS_TOLERANCE_MM) != 0U)
        {
            SetStep(APP_CHASSIS_STEP_HOLD_FORWARD_1, now);
        }
        else
        {
            (void)BspChassisOdom_DriveToGyro(APP_TARGET_FORWARD_1_X_MM,
                                             APP_TARGET_FORWARD_1_Y_MM,
                                             app_start_yaw_deg,
                                             gyro_z_deg_s,
                                             APP_CHASSIS_TASK_MOVE_SPEED_MM_S,
                                             BSP_CHASSIS_ODOM_MAX_CURRENT);
        }
        break;

    case APP_CHASSIS_STEP_HOLD_FORWARD_1:
        HoldThenNext(now, APP_CHASSIS_STEP_MOVE_RIGHT);
        break;

    case APP_CHASSIS_STEP_MOVE_RIGHT:
        if (BspChassisOdom_IsAt(APP_TARGET_RIGHT_X_MM,
                                APP_TARGET_RIGHT_Y_MM,
                                BSP_CHASSIS_ODOM_POS_TOLERANCE_MM) != 0U)
        {
            SetStep(APP_CHASSIS_STEP_HOLD_RIGHT, now);
        }
        else
        {
            (void)BspChassisOdom_DriveToGyro(APP_TARGET_RIGHT_X_MM,
                                             APP_TARGET_RIGHT_Y_MM,
                                             app_start_yaw_deg,
                                             gyro_z_deg_s,
                                             APP_CHASSIS_TASK_MOVE_SPEED_MM_S,
                                             BSP_CHASSIS_ODOM_MAX_CURRENT);
        }
        break;

    case APP_CHASSIS_STEP_HOLD_RIGHT:
        HoldThenNext(now, APP_CHASSIS_STEP_MOVE_LEFT);
        break;

    case APP_CHASSIS_STEP_MOVE_LEFT:
        if (BspChassisOdom_IsAt(APP_TARGET_LEFT_X_MM,
                                APP_TARGET_LEFT_Y_MM,
                                BSP_CHASSIS_ODOM_POS_TOLERANCE_MM) != 0U)
        {
            SetStep(APP_CHASSIS_STEP_HOLD_LEFT, now);
        }
        else
        {
            (void)BspChassisOdom_DriveToGyro(APP_TARGET_LEFT_X_MM,
                                             APP_TARGET_LEFT_Y_MM,
                                             app_start_yaw_deg,
                                             gyro_z_deg_s,
                                             APP_CHASSIS_TASK_MOVE_SPEED_MM_S,
                                             BSP_CHASSIS_ODOM_MAX_CURRENT);
        }
        break;

    case APP_CHASSIS_STEP_HOLD_LEFT:
        HoldThenNext(now, APP_CHASSIS_STEP_MOVE_FORWARD_2);
        break;

    case APP_CHASSIS_STEP_MOVE_FORWARD_2:
        if (BspChassisOdom_IsAt(APP_TARGET_FORWARD_2_X_MM,
                                APP_TARGET_FORWARD_2_Y_MM,
                                BSP_CHASSIS_ODOM_POS_TOLERANCE_MM) != 0U)
        {
            SetStep(APP_CHASSIS_STEP_HOLD_FORWARD_2, now);
        }
        else
        {
            (void)BspChassisOdom_DriveToGyro(APP_TARGET_FORWARD_2_X_MM,
                                             APP_TARGET_FORWARD_2_Y_MM,
                                             app_start_yaw_deg,
                                             gyro_z_deg_s,
                                             APP_CHASSIS_TASK_MOVE_SPEED_MM_S,
                                             BSP_CHASSIS_ODOM_MAX_CURRENT);
        }
        break;

    case APP_CHASSIS_STEP_HOLD_FORWARD_2:
        HoldThenNext(now, APP_CHASSIS_STEP_ROTATE_CLOCKWISE);
        break;

    case APP_CHASSIS_STEP_ROTATE_CLOCKWISE:
        if ((IsYawAtTarget(app_rotate_target_yaw_deg, yaw_deg) != 0U) &&
            (IsStoppedStable(now) != 0U))
        {
            SetStep(APP_CHASSIS_STEP_DONE, now);
        }
        else
        {
            (void)BspChassis_SetBodySpeedAngleHoldGyro(0.0f,
                                                       0.0f,
                                                       app_rotate_target_yaw_deg,
                                                       yaw_deg,
                                                       gyro_z_deg_s,
                                                       BSP_CHASSIS_ODOM_MAX_CURRENT);
        }
        break;

    case APP_CHASSIS_STEP_DONE:
    default:
        (void)BspChassis_Stop();
        break;
    }
}
