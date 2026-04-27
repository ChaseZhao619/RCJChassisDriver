#include "app_chassis_task.h"

#include "bsp_chassis.h"
#include "bsp_chassis_odom.h"
#include "bsp_motor.h"
#include <math.h>

typedef enum
{
    APP_CHASSIS_MODE_WAIT_IMU = 0,
    APP_CHASSIS_MODE_IDLE,
    APP_CHASSIS_MODE_MOVE,
    APP_CHASSIS_MODE_TURN,
} AppChassisMode;

static AppChassisMode app_mode;
static float app_target_x_mm;
static float app_target_y_mm;
static float app_target_yaw_deg;
static float app_segment_start_x_mm;
static float app_segment_start_y_mm;
static uint32_t app_stop_tick;
static uint32_t app_hold_tick;
static uint8_t app_odom_ready;
static uint8_t app_move_reached;
static uint8_t app_hold_started;
static AppChassisTaskDoneEvent app_active_command;
static AppChassisTaskDoneEvent app_done_event;
static float app_request_last_x_mm;
static float app_request_last_y_mm;
static float app_request_last_yaw_deg;
static uint8_t app_request_last_valid;

static const float app_pi = 3.14159265358979323846f;

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

static void SetMode(AppChassisMode mode)
{
    app_mode = mode;
    app_stop_tick = 0U;
    app_hold_tick = 0U;
    app_hold_started = 0U;
    BspChassis_ResetPid();
}

static void MarkActiveCommandDone(void)
{
    if (app_active_command != APP_CHASSIS_TASK_DONE_NONE)
    {
        app_done_event = app_active_command;
        app_active_command = APP_CHASSIS_TASK_DONE_NONE;
    }
}

static uint8_t IsYawAtTarget(float target_yaw_deg, float current_yaw_deg)
{
    float error = BspChassis_GetAngleErrorDeg(target_yaw_deg, current_yaw_deg);

    return ((error <= APP_CHASSIS_TASK_ROTATE_TOLERANCE_DEG) &&
            (error >= -APP_CHASSIS_TASK_ROTATE_TOLERANCE_DEG)) ? 1U : 0U;
}

static float CalcSegmentProgress(const BspChassisOdomPose *pose)
{
    float total_dx = app_target_x_mm - app_segment_start_x_mm;
    float total_dy = app_target_y_mm - app_segment_start_y_mm;
    float done_dx;
    float done_dy;
    float total_dist2 = (total_dx * total_dx) + (total_dy * total_dy);

    if ((pose == NULL) || (total_dist2 < 1.0f))
    {
        return 1.0f;
    }

    done_dx = pose->x_mm - app_segment_start_x_mm;
    done_dy = pose->y_mm - app_segment_start_y_mm;

    return ((done_dx * total_dx) + (done_dy * total_dy)) / total_dist2;
}

static uint8_t IsMoveTargetReached(const BspChassisOdomPose *pose)
{
    if (BspChassisOdom_IsAt(app_target_x_mm,
                            app_target_y_mm,
                            BSP_CHASSIS_ODOM_POS_TOLERANCE_MM) != 0U)
    {
        return 1U;
    }

    return (CalcSegmentProgress(pose) >= APP_CHASSIS_TASK_SEGMENT_DONE_PROGRESS) ? 1U : 0U;
}

static float CalcProfileMaxSpeed(const BspChassisOdomPose *pose)
{
    float total_dx = app_target_x_mm - app_segment_start_x_mm;
    float total_dy = app_target_y_mm - app_segment_start_y_mm;
    float progress;
    float scale;

    if (((total_dx * total_dx) + (total_dy * total_dy)) < 1.0f)
    {
        return APP_CHASSIS_TASK_MOVE_SPEED_MM_S;
    }

    progress = CalcSegmentProgress(pose);
    if (progress < 0.0f)
    {
        progress = 0.0f;
    }
    if (progress > 1.0f)
    {
        progress = 1.0f;
    }

    scale = sinf(progress * app_pi);
    if (scale < APP_CHASSIS_TASK_PROFILE_MIN_SCALE)
    {
        scale = APP_CHASSIS_TASK_PROFILE_MIN_SCALE;
    }

    return APP_CHASSIS_TASK_MOVE_SPEED_MM_S * scale;
}

static HAL_StatusTypeDef DriveAlongSegmentGyro(const BspChassisOdomPose *pose,
                                               float gyro_z_deg_s,
                                               float max_speed_mm_s)
{
    float total_dx = app_target_x_mm - app_segment_start_x_mm;
    float total_dy = app_target_y_mm - app_segment_start_y_mm;
    float total_dist = sqrtf((total_dx * total_dx) + (total_dy * total_dy));
    float ux;
    float uy;
    float nx;
    float ny;
    float done_dx;
    float done_dy;
    float along_done;
    float along_remaining;
    float cross_error;
    float along_speed;
    float cross_speed;
    float vx_world_mm_s;
    float vy_world_mm_s;
    float yaw_rad;
    float cos_yaw;
    float sin_yaw;
    float forward_mm_s;
    float left_mm_s;

    if ((pose == NULL) || (total_dist < 1.0f))
    {
        return BspChassisOdom_DriveToGyro(app_target_x_mm,
                                          app_target_y_mm,
                                          app_target_yaw_deg,
                                          gyro_z_deg_s,
                                          max_speed_mm_s,
                                          BSP_CHASSIS_ODOM_MAX_CURRENT);
    }

    ux = total_dx / total_dist;
    uy = total_dy / total_dist;
    nx = -uy;
    ny = ux;

    done_dx = pose->x_mm - app_segment_start_x_mm;
    done_dy = pose->y_mm - app_segment_start_y_mm;
    along_done = (done_dx * ux) + (done_dy * uy);
    along_remaining = total_dist - along_done;
    if (along_remaining < 0.0f)
    {
        along_remaining = 0.0f;
    }

    cross_error = (done_dx * nx) + (done_dy * ny);
    if (AbsFloat(cross_error) <= APP_CHASSIS_TASK_LINE_CROSS_DEADBAND_MM)
    {
        cross_error = 0.0f;
    }

    along_speed = along_remaining * BSP_CHASSIS_ODOM_POS_KP;
    if (along_speed > max_speed_mm_s)
    {
        along_speed = max_speed_mm_s;
    }
    if ((along_remaining > APP_CHASSIS_TASK_MIN_SPEED_DISTANCE_MM) &&
        (along_speed < BSP_CHASSIS_ODOM_MIN_SPEED_MM_S))
    {
        along_speed = BSP_CHASSIS_ODOM_MIN_SPEED_MM_S;
    }

    cross_speed = LimitFloat(-cross_error * APP_CHASSIS_TASK_LINE_CROSS_KP,
                             APP_CHASSIS_TASK_LINE_CROSS_MAX_MM_S);
    vx_world_mm_s = (ux * along_speed) + (nx * cross_speed);
    vy_world_mm_s = (uy * along_speed) + (ny * cross_speed);

    yaw_rad = pose->yaw_deg * app_pi / 180.0f;
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);
    forward_mm_s = (vx_world_mm_s * cos_yaw) + (vy_world_mm_s * sin_yaw);
    left_mm_s = (-vx_world_mm_s * sin_yaw) + (vy_world_mm_s * cos_yaw);

    return BspChassis_SetBodySpeedAngleHoldGyro(BspChassisOdom_MmSToMotorRpm(forward_mm_s),
                                                BspChassisOdom_MmSToMotorRpm(left_mm_s),
                                                app_target_yaw_deg,
                                                pose->yaw_deg,
                                                gyro_z_deg_s,
                                                BSP_CHASSIS_ODOM_MAX_CURRENT);
}

static HAL_StatusTypeDef HoldTargetYaw(float yaw_deg, float gyro_z_deg_s)
{
    return BspChassis_SetBodySpeedAngleHoldGyro(0.0f,
                                                0.0f,
                                                app_target_yaw_deg,
                                                yaw_deg,
                                                gyro_z_deg_s,
                                                BSP_CHASSIS_ODOM_MAX_CURRENT);
}

static void HoldReachedMove(uint32_t now, float yaw_deg, float gyro_z_deg_s)
{
    (void)HoldTargetYaw(yaw_deg, gyro_z_deg_s);

    if (app_hold_started == 0U)
    {
        app_hold_started = 1U;
        app_hold_tick = now;
        app_stop_tick = 0U;
    }

    if (((now - app_hold_tick) >= APP_CHASSIS_TASK_HOLD_AFTER_MOVE_MS) &&
        ((IsStoppedStable(now) != 0U) ||
         ((now - app_hold_tick) >= APP_CHASSIS_TASK_STOP_MAX_WAIT_MS)))
    {
        MarkActiveCommandDone();
        SetMode(APP_CHASSIS_MODE_IDLE);
        app_move_reached = 0U;
    }
}

void AppChassisTask_Init(void)
{
    app_mode = APP_CHASSIS_MODE_WAIT_IMU;
    app_target_x_mm = 0.0f;
    app_target_y_mm = 0.0f;
    app_target_yaw_deg = 0.0f;
    app_segment_start_x_mm = 0.0f;
    app_segment_start_y_mm = 0.0f;
    app_stop_tick = 0U;
    app_hold_tick = 0U;
    app_odom_ready = 0U;
    app_move_reached = 0U;
    app_hold_started = 0U;
    app_active_command = APP_CHASSIS_TASK_DONE_NONE;
    app_done_event = APP_CHASSIS_TASK_DONE_NONE;
    app_request_last_x_mm = 0.0f;
    app_request_last_y_mm = 0.0f;
    app_request_last_yaw_deg = 0.0f;
    app_request_last_valid = 0U;
    (void)BspChassis_Stop();
}

HAL_StatusTypeDef AppChassisTask_CommandDistanceCm(float x_cm, float y_cm)
{
    const BspChassisOdomPose *pose;

    if (app_odom_ready == 0U)
    {
        return HAL_BUSY;
    }

    pose = BspChassisOdom_GetPose();
    app_segment_start_x_mm = pose->x_mm;
    app_segment_start_y_mm = pose->y_mm;
    app_target_x_mm = pose->x_mm + (x_cm * 10.0f);
    app_target_y_mm = pose->y_mm + (y_cm * 10.0f);
    app_target_yaw_deg = pose->yaw_deg;
    app_move_reached = 0U;
    app_done_event = APP_CHASSIS_TASK_DONE_NONE;
    app_active_command = APP_CHASSIS_TASK_DONE_DIS;
    SetMode(APP_CHASSIS_MODE_MOVE);

    return HAL_OK;
}

HAL_StatusTypeDef AppChassisTask_CommandTurnDeg(float target_yaw_deg)
{
    if (app_odom_ready == 0U)
    {
        return HAL_BUSY;
    }

    app_target_yaw_deg = BspChassis_WrapAngle360(target_yaw_deg);
    app_move_reached = 0U;
    app_done_event = APP_CHASSIS_TASK_DONE_NONE;
    app_active_command = APP_CHASSIS_TASK_DONE_TURN;
    SetMode(APP_CHASSIS_MODE_TURN);

    return HAL_OK;
}

HAL_StatusTypeDef AppChassisTask_GetRequestDelta(float *dx_cm,
                                                 float *dy_cm,
                                                 float *dyaw_deg,
                                                 float *yaw_deg)
{
    const BspChassisOdomPose *pose;

    if ((dx_cm == NULL) || (dy_cm == NULL) || (dyaw_deg == NULL) || (yaw_deg == NULL))
    {
        return HAL_ERROR;
    }

    if (app_odom_ready == 0U)
    {
        return HAL_BUSY;
    }

    pose = BspChassisOdom_GetPose();
    if (app_request_last_valid == 0U)
    {
        *dx_cm = 0.0f;
        *dy_cm = 0.0f;
        *dyaw_deg = 0.0f;
        app_request_last_valid = 1U;
    }
    else
    {
        *dx_cm = (pose->x_mm - app_request_last_x_mm) * 0.1f;
        *dy_cm = (pose->y_mm - app_request_last_y_mm) * 0.1f;
        *dyaw_deg = BspChassis_GetAngleErrorDeg(pose->yaw_deg, app_request_last_yaw_deg);
    }
    *yaw_deg = pose->yaw_deg;

    app_request_last_x_mm = pose->x_mm;
    app_request_last_y_mm = pose->y_mm;
    app_request_last_yaw_deg = pose->yaw_deg;

    return HAL_OK;
}

AppChassisTaskDoneEvent AppChassisTask_ConsumeDoneEvent(void)
{
    AppChassisTaskDoneEvent event = app_done_event;

    app_done_event = APP_CHASSIS_TASK_DONE_NONE;
    return event;
}

void AppChassisTask_OnYawZero(float yaw_deg)
{
    const BspChassisOdomPose *pose;
    float x_mm;
    float y_mm;

    yaw_deg = BspChassis_WrapAngle360(yaw_deg);
    app_target_yaw_deg = yaw_deg;
    app_move_reached = 0U;
    app_active_command = APP_CHASSIS_TASK_DONE_NONE;
    app_done_event = APP_CHASSIS_TASK_DONE_NONE;

    if (app_odom_ready != 0U)
    {
        pose = BspChassisOdom_GetPose();
        x_mm = pose->x_mm;
        y_mm = pose->y_mm;
        BspChassisOdom_Reset(x_mm, y_mm, yaw_deg);
        app_segment_start_x_mm = x_mm;
        app_segment_start_y_mm = y_mm;
        app_target_x_mm = x_mm;
        app_target_y_mm = y_mm;
    }

    SetMode(APP_CHASSIS_MODE_IDLE);
}

void AppChassisTask_Task(uint8_t yaw_valid,
                         float yaw_deg,
                         uint8_t gyro_valid,
                         float gyro_z_deg_s)
{
    uint32_t now = HAL_GetTick();
    const BspChassisOdomPose *pose;

    if (yaw_valid == 0U)
    {
        app_odom_ready = 0U;
        app_active_command = APP_CHASSIS_TASK_DONE_NONE;
        SetMode(APP_CHASSIS_MODE_WAIT_IMU);
        (void)BspChassis_Stop();
        return;
    }

    yaw_deg = BspChassis_WrapAngle360(yaw_deg);
    if (gyro_valid == 0U)
    {
        gyro_z_deg_s = 0.0f;
    }

    if (app_odom_ready == 0U)
    {
        BspChassisOdom_Reset(0.0f, 0.0f, yaw_deg);
        app_target_yaw_deg = yaw_deg;
        app_odom_ready = 1U;
        SetMode(APP_CHASSIS_MODE_IDLE);
    }
    else
    {
        BspChassisOdom_Update(yaw_deg);
    }
    pose = BspChassisOdom_GetPose();

    switch (app_mode)
    {
    case APP_CHASSIS_MODE_MOVE:
        if (app_move_reached != 0U)
        {
            HoldReachedMove(now, yaw_deg, gyro_z_deg_s);
        }
        else if (IsMoveTargetReached(pose) != 0U)
        {
            app_move_reached = 1U;
            BspChassis_ResetPid();
            HoldReachedMove(now, yaw_deg, gyro_z_deg_s);
        }
        else
        {
            (void)DriveAlongSegmentGyro(pose,
                                        gyro_z_deg_s,
                                        CalcProfileMaxSpeed(pose));
        }
        break;

    case APP_CHASSIS_MODE_TURN:
        if ((IsYawAtTarget(app_target_yaw_deg, yaw_deg) != 0U) &&
            (IsStoppedStable(now) != 0U))
        {
            MarkActiveCommandDone();
            SetMode(APP_CHASSIS_MODE_IDLE);
            (void)HoldTargetYaw(yaw_deg, gyro_z_deg_s);
        }
        else
        {
            (void)HoldTargetYaw(yaw_deg, gyro_z_deg_s);
        }
        break;

    case APP_CHASSIS_MODE_WAIT_IMU:
        (void)BspChassis_Stop();
        break;

    case APP_CHASSIS_MODE_IDLE:
    default:
        (void)HoldTargetYaw(yaw_deg, gyro_z_deg_s);
        break;
    }
}
