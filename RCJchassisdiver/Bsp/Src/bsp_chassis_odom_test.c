#include "bsp_chassis_odom_test.h"

#include "bsp_chassis.h"
#include "bsp_chassis_odom.h"
#include "bsp_motor.h"
#include "bsp_usart.h"
#include <math.h>

#define FIELD_PATH_X_MIN (BSP_CHASSIS_ODOM_TEST_LINE_MARGIN_MM)
#define FIELD_PATH_Y_MIN (BSP_CHASSIS_ODOM_TEST_LINE_MARGIN_MM)
#define FIELD_PATH_X_MAX (BSP_CHASSIS_ODOM_TEST_FIELD_LENGTH_MM - BSP_CHASSIS_DIAMETER_MM - BSP_CHASSIS_ODOM_TEST_LINE_MARGIN_MM)
#define FIELD_PATH_Y_MAX (BSP_CHASSIS_ODOM_TEST_FIELD_WIDTH_MM - BSP_CHASSIS_DIAMETER_MM - BSP_CHASSIS_ODOM_TEST_LINE_MARGIN_MM)

static const BspChassisOdomWaypoint odom_path[] = {
    {0.0f, 0.0f, 0.0f, 0.0f, 300U},
    {FIELD_PATH_X_MIN, FIELD_PATH_Y_MIN, 0.0f, 180.0f, 300U},
    {FIELD_PATH_X_MAX, FIELD_PATH_Y_MIN, 0.0f, BSP_CHASSIS_ODOM_TEST_BORDER_SPEED_MM_S, 300U},
    {FIELD_PATH_X_MAX, FIELD_PATH_Y_MAX, 0.0f, BSP_CHASSIS_ODOM_TEST_BORDER_SPEED_MM_S, 300U},
    {FIELD_PATH_X_MIN, FIELD_PATH_Y_MAX, 0.0f, BSP_CHASSIS_ODOM_TEST_BORDER_SPEED_MM_S, 300U},
    {FIELD_PATH_X_MIN, FIELD_PATH_Y_MIN, 0.0f, BSP_CHASSIS_ODOM_TEST_BORDER_SPEED_MM_S, 600U},
};

static uint8_t odom_test_ready;
static uint8_t waypoint_index;
static uint8_t waypoint_hold_started;
static float target_yaw_deg;
static float segment_start_x_mm;
static float segment_start_y_mm;
static uint32_t waypoint_hold_tick;
static uint32_t motor_stop_tick;
static uint32_t last_print_tick;
static uint8_t waypoint_reached;

static int16_t Abs16(int16_t value)
{
    if (value == INT16_MIN)
    {
        return INT16_MAX;
    }

    return (value < 0) ? (int16_t)-value : value;
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

static uint8_t AreChassisMotorsStopped(void)
{
    uint8_t can_id;

    for (can_id = 1U; can_id <= BSP_MOTOR_CHASSIS_COUNT; can_id++)
    {
        const BspMotorFeedback *feedback = BspMotor_GetFeedback(can_id);

        if ((feedback == NULL) ||
            (BspMotor_IsOnline(can_id, 200U) == 0U) ||
            (Abs16(feedback->speed_rpm) > BSP_CHASSIS_ODOM_TEST_STOP_RPM))
        {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t IsYawAtTarget(float target_yaw, float current_yaw)
{
    float error = BspChassis_GetAngleErrorDeg(target_yaw, current_yaw);

    return ((error <= BSP_CHASSIS_ODOM_TEST_YAW_TOLERANCE_DEG) &&
            (error >= -BSP_CHASSIS_ODOM_TEST_YAW_TOLERANCE_DEG)) ? 1U : 0U;
}

static float CalcSegmentProgress(const BspChassisOdomPose *pose,
                                 const BspChassisOdomWaypoint *waypoint)
{
    float total_dx = waypoint->x_mm - segment_start_x_mm;
    float total_dy = waypoint->y_mm - segment_start_y_mm;
    float done_dx;
    float done_dy;
    float total_dist2 = (total_dx * total_dx) + (total_dy * total_dy);

    if ((pose == NULL) || (total_dist2 < 1.0f))
    {
        return 1.0f;
    }

    done_dx = pose->x_mm - segment_start_x_mm;
    done_dy = pose->y_mm - segment_start_y_mm;

    return ((done_dx * total_dx) + (done_dy * total_dy)) / total_dist2;
}

static uint8_t IsWaypointReached(const BspChassisOdomPose *pose,
                                 const BspChassisOdomWaypoint *waypoint)
{
    if (BspChassisOdom_IsAt(waypoint->x_mm,
                            waypoint->y_mm,
                            BSP_CHASSIS_ODOM_POS_TOLERANCE_MM) != 0U)
    {
        return 1U;
    }

    return (CalcSegmentProgress(pose, waypoint) >= BSP_CHASSIS_ODOM_TEST_SEGMENT_DONE_PROGRESS) ? 1U : 0U;
}

static void MoveToNextWaypoint(const BspChassisOdomPose **pose, float yaw_deg)
{
    waypoint_index++;
    if (waypoint_index >= (sizeof(odom_path) / sizeof(odom_path[0])))
    {
        waypoint_index = 0U;
        BspChassisOdom_Reset(0.0f, 0.0f, yaw_deg);
        *pose = BspChassisOdom_GetPose();
    }

    segment_start_x_mm = (*pose)->x_mm;
    segment_start_y_mm = (*pose)->y_mm;
    waypoint_hold_started = 0U;
    motor_stop_tick = 0U;
    waypoint_reached = 0U;
    BspChassis_ResetPid();
}

static void HoldReachedWaypoint(uint32_t now, const BspChassisOdomPose **pose, float yaw_deg)
{
    (void)BspChassis_Stop();

    if (waypoint_hold_started == 0U)
    {
        waypoint_hold_started = 1U;
        waypoint_hold_tick = now;
        motor_stop_tick = 0U;
    }

    if (AreChassisMotorsStopped() != 0U)
    {
        if (motor_stop_tick == 0U)
        {
            motor_stop_tick = now;
        }
    }
    else
    {
        motor_stop_tick = 0U;
    }

    if (((now - waypoint_hold_tick) >= odom_path[waypoint_index].hold_ms) &&
        (((motor_stop_tick != 0U) &&
          ((now - motor_stop_tick) >= BSP_CHASSIS_ODOM_TEST_STOP_STABLE_MS)) ||
         ((now - waypoint_hold_tick) >= BSP_CHASSIS_ODOM_TEST_STOP_MAX_WAIT_MS)))
    {
        MoveToNextWaypoint(pose, yaw_deg);
    }
}

static float CalcProfileMaxSpeed(const BspChassisOdomPose *pose,
                                 const BspChassisOdomWaypoint *waypoint)
{
    float total_dx = waypoint->x_mm - segment_start_x_mm;
    float total_dy = waypoint->y_mm - segment_start_y_mm;
    float progress;
    float scale;

    if (((total_dx * total_dx) + (total_dy * total_dy)) < 1.0f)
    {
        return waypoint->max_speed_mm_s;
    }

    progress = CalcSegmentProgress(pose, waypoint);
    if (progress < 0.0f)
    {
        progress = 0.0f;
    }
    if (progress > 1.0f)
    {
        progress = 1.0f;
    }

    scale = sinf(progress * 3.14159265358979323846f);
    if (scale < BSP_CHASSIS_ODOM_TEST_PROFILE_MIN_SCALE)
    {
        scale = BSP_CHASSIS_ODOM_TEST_PROFILE_MIN_SCALE;
    }

    return waypoint->max_speed_mm_s * scale;
}

static HAL_StatusTypeDef DriveAlongSegmentGyro(const BspChassisOdomPose *pose,
                                               const BspChassisOdomWaypoint *waypoint,
                                               float target_yaw_deg,
                                               float gyro_z_deg_s,
                                               float max_speed_mm_s)
{
    float total_dx = waypoint->x_mm - segment_start_x_mm;
    float total_dy = waypoint->y_mm - segment_start_y_mm;
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

    if ((pose == NULL) || (waypoint == NULL) || (total_dist < 1.0f))
    {
        return BspChassisOdom_DriveToGyro(waypoint->x_mm,
                                          waypoint->y_mm,
                                          target_yaw_deg,
                                          gyro_z_deg_s,
                                          max_speed_mm_s,
                                          BSP_CHASSIS_ODOM_MAX_CURRENT);
    }

    if (max_speed_mm_s <= 0.0f)
    {
        max_speed_mm_s = BSP_CHASSIS_ODOM_DEFAULT_MAX_SPEED_MM_S;
    }

    ux = total_dx / total_dist;
    uy = total_dy / total_dist;
    nx = -uy;
    ny = ux;

    done_dx = pose->x_mm - segment_start_x_mm;
    done_dy = pose->y_mm - segment_start_y_mm;
    along_done = (done_dx * ux) + (done_dy * uy);
    along_remaining = total_dist - along_done;
    if (along_remaining < 0.0f)
    {
        along_remaining = 0.0f;
    }

    cross_error = (done_dx * nx) + (done_dy * ny);
    if (AbsFloat(cross_error) <= BSP_CHASSIS_ODOM_TEST_LINE_CROSS_DEADBAND_MM)
    {
        cross_error = 0.0f;
    }

    along_speed = along_remaining * BSP_CHASSIS_ODOM_POS_KP;
    if (along_speed > max_speed_mm_s)
    {
        along_speed = max_speed_mm_s;
    }
    if ((along_remaining > BSP_CHASSIS_ODOM_TEST_MIN_SPEED_DISTANCE_MM) &&
        (along_speed < BSP_CHASSIS_ODOM_MIN_SPEED_MM_S))
    {
        along_speed = BSP_CHASSIS_ODOM_MIN_SPEED_MM_S;
    }

    cross_speed = LimitFloat(-cross_error * BSP_CHASSIS_ODOM_TEST_LINE_CROSS_KP,
                             BSP_CHASSIS_ODOM_TEST_LINE_CROSS_MAX_MM_S);

    vx_world_mm_s = (ux * along_speed) + (nx * cross_speed);
    vy_world_mm_s = (uy * along_speed) + (ny * cross_speed);

    yaw_rad = pose->yaw_deg * 3.14159265358979323846f / 180.0f;
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);
    forward_mm_s = (vx_world_mm_s * cos_yaw) + (vy_world_mm_s * sin_yaw);
    left_mm_s = (-vx_world_mm_s * sin_yaw) + (vy_world_mm_s * cos_yaw);

    return BspChassis_SetBodySpeedAngleHoldGyro(BspChassisOdom_MmSToMotorRpm(forward_mm_s),
                                                BspChassisOdom_MmSToMotorRpm(left_mm_s),
                                                target_yaw_deg,
                                                pose->yaw_deg,
                                                gyro_z_deg_s,
                                                BSP_CHASSIS_ODOM_MAX_CURRENT);
}

void BspChassisOdomTest_Init(void)
{
#if BSP_CHASSIS_ODOM_TEST_ENABLE
    odom_test_ready = 0U;
    waypoint_index = 0U;
    waypoint_hold_started = 0U;
    target_yaw_deg = 0.0f;
    segment_start_x_mm = 0.0f;
    segment_start_y_mm = 0.0f;
    waypoint_hold_tick = 0U;
    motor_stop_tick = 0U;
    last_print_tick = 0U;
    waypoint_reached = 0U;
    (void)BspChassis_Stop();
#endif
}

void BspChassisOdomTest_Task(uint8_t yaw_valid,
                             float yaw_deg,
                             uint8_t gyro_valid,
                             float gyro_z_deg_s)
{
#if BSP_CHASSIS_ODOM_TEST_ENABLE
    uint32_t now = HAL_GetTick();
    const BspChassisOdomWaypoint *waypoint = &odom_path[waypoint_index];
    const BspChassisOdomPose *pose;
    float profile_max_speed;

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
    if (odom_test_ready == 0U)
    {
        target_yaw_deg = yaw_deg;
        BspChassisOdom_Init(yaw_deg);
        segment_start_x_mm = 0.0f;
        segment_start_y_mm = 0.0f;
        odom_test_ready = 1U;
    }

    BspChassisOdom_Update(yaw_deg);
    pose = BspChassisOdom_GetPose();
    waypoint = &odom_path[waypoint_index];

    if (IsWaypointReached(pose, waypoint) != 0U)
    {
        if (waypoint_reached == 0U)
        {
            waypoint_reached = 1U;
            BspChassis_ResetPid();
        }
        if (IsYawAtTarget(target_yaw_deg, yaw_deg) != 0U)
        {
            HoldReachedWaypoint(now, &pose, yaw_deg);
        }
        else
        {
            waypoint_hold_started = 0U;
            motor_stop_tick = 0U;
            (void)BspChassis_SetBodySpeedAngleHoldGyro(0.0f,
                                                       0.0f,
                                                       target_yaw_deg,
                                                       yaw_deg,
                                                       gyro_z_deg_s,
                                                       BSP_CHASSIS_ODOM_MAX_CURRENT);
        }
    }
    else
    {
        if (waypoint_reached != 0U)
        {
            HoldReachedWaypoint(now, &pose, yaw_deg);
        }
        else
        {
            waypoint_hold_started = 0U;
            motor_stop_tick = 0U;
            profile_max_speed = CalcProfileMaxSpeed(pose, waypoint);
            (void)DriveAlongSegmentGyro(pose,
                                        waypoint,
                                        target_yaw_deg,
                                        gyro_z_deg_s,
                                        profile_max_speed);
        }
    }

#if BSP_CHASSIS_ODOM_TEST_PRINT_ENABLE
    if ((now - last_print_tick) >= 200U)
    {
        last_print_tick = now;
        Printf(BSP_USART_6,
               "odom wp=%u pose=%.0f,%.0f yaw=%.1f target=%.0f,%.0f v=%.0f,%.0f\r\n",
               waypoint_index,
               pose->x_mm,
               pose->y_mm,
               pose->yaw_deg,
               waypoint->x_mm,
               waypoint->y_mm,
               pose->vx_mm_s,
               pose->vy_mm_s);
    }
#else
    (void)pose;
#endif
#else
    (void)yaw_valid;
    (void)yaw_deg;
    (void)gyro_valid;
    (void)gyro_z_deg_s;
    (void)BspChassis_Stop();
#endif
}
