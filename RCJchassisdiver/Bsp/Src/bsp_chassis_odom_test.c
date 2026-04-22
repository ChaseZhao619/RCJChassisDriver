#include "bsp_chassis_odom_test.h"

#include "bsp_chassis.h"
#include "bsp_chassis_odom.h"
#include "bsp_usart.h"
#include <math.h>

static const BspChassisOdomWaypoint odom_path[] = {
    {0.0f, 0.0f, 0.0f, 0.0f, 100U},
    {600.0f, 0.0f, 0.0f, 350.0f, 50U},
    {0.0f, 0.0f, 0.0f, 350.0f, 50U},
    {0.0f, 600.0f, 0.0f, 350.0f, 50U},
    {0.0f, 0.0f, 0.0f, 350.0f, 50U},
    {-600.0f, 0.0f, 0.0f, 350.0f, 50U},
    {0.0f, 0.0f, 0.0f, 350.0f, 50U},
    {0.0f, -600.0f, 0.0f, 350.0f, 50U},
    {0.0f, 0.0f, 0.0f, 350.0f, 150U},
    {450.0f, 450.0f, 0.0f, 400.0f, 50U},
    {-450.0f, 450.0f, 0.0f, 400.0f, 50U},
    {-450.0f, -450.0f, 0.0f, 400.0f, 50U},
    {450.0f, -450.0f, 0.0f, 400.0f, 50U},
    {0.0f, 0.0f, 0.0f, 400.0f, 300U},
};

static uint8_t odom_test_ready;
static uint8_t waypoint_index;
static uint8_t waypoint_hold_started;
static float target_yaw_deg;
static float segment_start_x_mm;
static float segment_start_y_mm;
static uint32_t waypoint_hold_tick;
static uint32_t last_print_tick;

static uint8_t IsYawAtTarget(float target_yaw, float current_yaw)
{
    float error = BspChassis_GetAngleErrorDeg(target_yaw, current_yaw);

    return ((error <= BSP_CHASSIS_ODOM_TEST_YAW_TOLERANCE_DEG) &&
            (error >= -BSP_CHASSIS_ODOM_TEST_YAW_TOLERANCE_DEG)) ? 1U : 0U;
}

static float CalcProfileMaxSpeed(const BspChassisOdomPose *pose,
                                 const BspChassisOdomWaypoint *waypoint)
{
    float total_dx = waypoint->x_mm - segment_start_x_mm;
    float total_dy = waypoint->y_mm - segment_start_y_mm;
    float done_dx = pose->x_mm - segment_start_x_mm;
    float done_dy = pose->y_mm - segment_start_y_mm;
    float total_dist2 = (total_dx * total_dx) + (total_dy * total_dy);
    float progress;
    float scale;

    if (total_dist2 < 1.0f)
    {
        return waypoint->max_speed_mm_s;
    }

    progress = ((done_dx * total_dx) + (done_dy * total_dy)) / total_dist2;
    if (progress < 0.0f)
    {
        progress = 0.0f;
    }
    if (progress > 1.0f)
    {
        progress = 1.0f;
    }

    scale = sinf(progress * 3.14159265358979323846f);
    if (scale < 0.25f)
    {
        scale = 0.25f;
    }

    return waypoint->max_speed_mm_s * scale;
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
    last_print_tick = 0U;
    (void)BspChassis_Stop();
#endif
}

void BspChassisOdomTest_Task(uint8_t yaw_valid, float yaw_deg)
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

    if (BspChassisOdom_IsAt(waypoint->x_mm,
                            waypoint->y_mm,
                            BSP_CHASSIS_ODOM_POS_TOLERANCE_MM) != 0U)
    {
        if (IsYawAtTarget(target_yaw_deg, yaw_deg) != 0U)
        {
            (void)BspChassis_SetBodySpeed(0.0f, 0.0f, 0.0f, BSP_CHASSIS_ODOM_MAX_CURRENT);
            if (waypoint_hold_started == 0U)
            {
                waypoint_hold_started = 1U;
                waypoint_hold_tick = now;
            }
            else if ((now - waypoint_hold_tick) >= waypoint->hold_ms)
            {
                waypoint_index++;
                if (waypoint_index >= (sizeof(odom_path) / sizeof(odom_path[0])))
                {
                    waypoint_index = 0U;
                    BspChassisOdom_Reset(0.0f, 0.0f, yaw_deg);
                    pose = BspChassisOdom_GetPose();
                }
                segment_start_x_mm = pose->x_mm;
                segment_start_y_mm = pose->y_mm;
                waypoint_hold_started = 0U;
            }
        }
        else
        {
            waypoint_hold_started = 0U;
            (void)BspChassis_SetBodySpeedAngleHold(0.0f,
                                                   0.0f,
                                                   target_yaw_deg,
                                                   yaw_deg,
                                                   BSP_CHASSIS_ODOM_MAX_CURRENT);
        }
    }
    else
    {
        waypoint_hold_started = 0U;
        profile_max_speed = CalcProfileMaxSpeed(pose, waypoint);
        (void)BspChassisOdom_DriveTo(waypoint->x_mm,
                                     waypoint->y_mm,
                                     target_yaw_deg,
                                     profile_max_speed,
                                     BSP_CHASSIS_ODOM_MAX_CURRENT);
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
    (void)BspChassis_Stop();
#endif
}
