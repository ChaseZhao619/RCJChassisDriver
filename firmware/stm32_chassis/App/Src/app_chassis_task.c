#include "app_chassis_task.h"
#include "app_lqr.h"
#include "app_control_safety.h"

#include "bsp_chassis.h"
#include "bsp_chassis_odom.h"
#include "bsp_motor.h"
#include <math.h>

/*
 * 底盘应用状态机。
 * 本模块保存“当前唯一活动命令”，把相对位移/绝对转角/持续运动转换为 Bsp 层调用，
 * 并在到位和停稳后产生单槽完成事件。新运动命令会覆盖旧命令，不支持命令排队。
 * 坐标、状态转换和调参顺序见 App/README.md。
 */
typedef enum
{
    APP_CHASSIS_MODE_WAIT_IMU = 0, /* yaw 无效：取消活动命令并保持停车。 */
    APP_CHASSIS_MODE_IDLE,         /* 无活动命令：零平移并保持目标 yaw。 */
    APP_CHASSIS_MODE_MOVE,         /* cmd_dis：沿世界坐标线段移动。 */
    APP_CHASSIS_MODE_TURN,         /* cmd_turn：原地转到绝对 yaw。 */
    APP_CHASSIS_MODE_DKMOTOR,      /* cmd_dkmotor：持续运动，无自动完成事件。 */
    APP_CHASSIS_MODE_VELOCITY,     /* cmd_vel：车体速度命令，需周期续发。 */
} AppChassisMode;

static AppChassisMode app_mode;
static float app_target_x_mm;       /* MOVE 的世界坐标目标 X [mm]。 */
static float app_target_y_mm;       /* MOVE 的世界坐标目标 Y [mm]。 */
static float app_target_yaw_deg;    /* IDLE/MOVE/TURN/DKMOTOR 的航向目标 [deg]。 */
static float app_segment_start_x_mm;/* 当前 MOVE 线段起点 X [mm]。 */
static float app_segment_start_y_mm;/* 当前 MOVE 线段起点 Y [mm]。 */
static uint32_t app_stop_tick;      /* 连续满足停车转速阈值的起始时刻 [ms]。 */
static uint32_t app_hold_tick;      /* MOVE 到点后开始航向保持的时刻 [ms]。 */
static uint8_t app_odom_ready;      /* 已用有效 yaw 初始化里程计。 */
static uint8_t app_move_reached;    /* MOVE 已满足位置或投影进度条件。 */
static uint8_t app_hold_started;    /* MOVE 到点保持计时已开始。 */
static AppChassisTaskDoneEvent app_active_command; /* 结束时应生成哪类事件。 */
static AppChassisTaskDoneEvent app_done_event;     /* 等待通信任务消费的单槽事件。 */
static float app_request_last_x_mm;
static float app_request_last_y_mm;
static float app_request_last_yaw_deg;
static uint8_t app_request_last_valid;
static uint8_t app_motion_enabled;  /* 全局软件运动许可。 */
static float app_dkmotor_speed_rpm; /* 百分比换算后的电机轴目标 [rpm]。 */
static float app_dkmotor_angle_deg; /* 持续运动方向 [deg]。 */
static uint8_t app_dkmotor_head_lock; /* 1=保持下发时航向，0=转向后前进。 */
static uint8_t app_move_profile;    /* 当前 MOVE 的 SHARP/NORMAL/SMOOTH 档位。 */
static float app_velocity_forward_mm_s;
static float app_velocity_left_mm_s;
static float app_velocity_yaw_rad_s;
static uint32_t app_continuous_command_tick;
static uint32_t app_control_tick;
static uint32_t app_control_period_ms;
static uint32_t app_control_missed_deadlines;
static uint8_t app_control_started;

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
    /* 电机缺少反馈或 200 ms 内离线均不能判定为已停，避免误报完成。 */
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

static uint8_t AreChassisMotorsOnline(void)
{
    uint8_t can_id;
    for (can_id = 1U; can_id <= BSP_MOTOR_CHASSIS_COUNT; can_id++)
    {
        if (BspMotor_IsOnline(can_id, APP_CHASSIS_TASK_MOTOR_TIMEOUT_MS) == 0U)
            return 0U;
    }
    return 1U;
}

static float AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint8_t IsStoppedStable(uint32_t now)
{
    /* 任一周期不满足停车条件就重新计时，要求连续稳定而非瞬时过零。 */
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
    /* 模式切换清除停稳/保持计时并复位控制器，防止旧积分带入新命令。 */
    app_mode = mode;
    app_stop_tick = 0U;
    app_hold_tick = 0U;
    app_hold_started = 0U;
    BspChassis_ResetPid();
}

static void StopForFault(void)
{
    if (app_odom_ready != 0U)
        app_target_yaw_deg = BspChassisOdom_GetPose()->yaw_deg;
    app_motion_enabled = 0U;
    app_active_command = APP_CHASSIS_TASK_DONE_NONE;
    app_done_event = APP_CHASSIS_TASK_DONE_NONE;
    app_dkmotor_speed_rpm = 0.0f;
    app_velocity_forward_mm_s = 0.0f;
    app_velocity_left_mm_s = 0.0f;
    app_velocity_yaw_rad_s = 0.0f;
    SetMode(APP_CHASSIS_MODE_IDLE);
    (void)BspChassis_Stop();
}

static void MarkActiveCommandDone(void)
{
    /* 完成事件只有一个存储槽；通信任务应及时 Consume。 */
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
    /*
     * 将当前位置投影到起点->目标向量：起点为 0，目标为 1，越过目标可大于 1。
     * 投影进度用于速度曲线与越过终点判断，到点还要求横向误差在容差内。
     */
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
    float dx = app_target_x_mm - app_segment_start_x_mm;
    float dy = app_target_y_mm - app_segment_start_y_mm;
    float distance = sqrtf(dx * dx + dy * dy);
    float cross_error;
    if (BspChassisOdom_IsAt(app_target_x_mm,
                            app_target_y_mm,
                            BSP_CHASSIS_ODOM_POS_TOLERANCE_MM) != 0U)
    {
        return 1U;
    }

    if (distance < 1.0f) return 1U;
    cross_error = ((pose->x_mm - app_segment_start_x_mm) * -dy +
                   (pose->y_mm - app_segment_start_y_mm) * dx) / distance;
    return ((CalcSegmentProgress(pose) >= APP_CHASSIS_TASK_SEGMENT_DONE_PROGRESS) &&
            (AbsFloat(cross_error) <= BSP_CHASSIS_ODOM_POS_TOLERANCE_MM)) ? 1U : 0U;
}

static float CalcProfileMaxSpeed(const BspChassisOdomPose *pose)
{
    /* 根据沿程进度生成对称的起停速度上限；实际沿线速度还受位置 KP 限制。 */
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

    /*
     * Profile curve tuning / 速度曲线调参:
     * normal keeps the original sine curve; sharp uses an exponent below 1.0f
     * to reach higher speed earlier; smooth uses an exponent above 1.0f to
     * keep start/end slower.
     * NORMAL 保持原正弦曲线；SHARP 使用小于 1.0f 的指数，让速度更早升高；
     * SMOOTH 使用大于 1.0f 的指数，让起点和终点更慢。
     */
    scale = sinf(progress * app_pi);
    if (app_move_profile == APP_CHASSIS_TASK_PROFILE_SHARP)
    {
        scale = powf(scale, APP_CHASSIS_TASK_PROFILE_SHARP_EXP);
    }
    else if (app_move_profile == APP_CHASSIS_TASK_PROFILE_SMOOTH)
    {
        scale = powf(scale, APP_CHASSIS_TASK_PROFILE_SMOOTH_EXP);
    }

    if (scale < APP_CHASSIS_TASK_PROFILE_MIN_SCALE)
    {
        scale = APP_CHASSIS_TASK_PROFILE_MIN_SCALE;
    }

    return APP_CHASSIS_TASK_MOVE_SPEED_MM_S * scale;
}

static HAL_StatusTypeDef DriveLqr(const BspChassisOdomPose *pose,
                                  float gyro_z_deg_s,
                                  float target_x_mm,
                                  float target_y_mm,
                                  float target_yaw_deg,
                                  float vx_world_mm_s,
                                  float vy_world_mm_s,
                                  float wz_rad_s,
                                  uint8_t track_x,
                                  uint8_t track_y,
                                  uint8_t track_yaw,
                                  float max_speed_mm_s)
{
    const float rad_per_deg = app_pi / 180.0f;
    float cos_yaw;
    float sin_yaw;
    float forward_mm_s;
    float left_mm_s;
    AppLqrState state;
    AppLqrReference reference;
    AppLqrOutput output;

    if ((pose == NULL) || !isfinite(pose->x_mm) || !isfinite(pose->y_mm) ||
        !isfinite(pose->yaw_deg) || !isfinite(pose->vx_mm_s) ||
        !isfinite(pose->vy_mm_s) || !isfinite(gyro_z_deg_s) ||
        !isfinite(target_x_mm) || !isfinite(target_y_mm) ||
        !isfinite(target_yaw_deg) || !isfinite(vx_world_mm_s) ||
        !isfinite(vy_world_mm_s) || !isfinite(wz_rad_s)) return HAL_ERROR;
    state.x_m = pose->x_mm * 0.001f;
    state.y_m = pose->y_mm * 0.001f;
    state.yaw_rad = pose->yaw_deg * rad_per_deg;
    state.vx_m_s = pose->vx_mm_s * 0.001f;
    state.vy_m_s = pose->vy_mm_s * 0.001f;
    state.wz_rad_s = gyro_z_deg_s * rad_per_deg * (float)BSP_CHASSIS_GYRO_Z_DIR;
    reference.x_m = target_x_mm * 0.001f;
    reference.y_m = target_y_mm * 0.001f;
    reference.yaw_rad = target_yaw_deg * rad_per_deg;
    reference.vx_m_s = vx_world_mm_s * 0.001f;
    reference.vy_m_s = vy_world_mm_s * 0.001f;
    reference.wz_rad_s = wz_rad_s;
    reference.track_x = track_x;
    reference.track_y = track_y;
    reference.track_yaw = track_yaw;
    output = AppLqr_Calculate(&state, &reference, max_speed_mm_s * 0.001f);
    output = AppLqr_LimitOmni(output, state.yaw_rad,
                              APP_LQR_VELOCITY_LIMIT_M_S,
                              BSP_CHASSIS_ROTATION_RADIUS_MM * 0.001f,
                              BSP_CHASSIS_FORWARD_TO_LEFT_COMP,
                              BSP_CHASSIS_LEFT_TO_FORWARD_COMP);

    cos_yaw = cosf(state.yaw_rad);
    sin_yaw = sinf(state.yaw_rad);
    forward_mm_s = 1000.0f * (output.vx_m_s * cos_yaw + output.vy_m_s * sin_yaw);
    left_mm_s = 1000.0f * (-output.vx_m_s * sin_yaw + output.vy_m_s * cos_yaw);
    return BspChassis_SetBodySpeed(BspChassisOdom_MmSToMotorRpm(forward_mm_s),
                                   BspChassisOdom_MmSToMotorRpm(left_mm_s),
                                   BspChassisOdom_MmSToMotorRpm(BSP_CHASSIS_ROTATION_RADIUS_MM *
                                                                output.wz_rad_s),
                                   BSP_CHASSIS_ODOM_MAX_CURRENT);
}

static HAL_StatusTypeDef HoldTargetYaw(const BspChassisOdomPose *pose, float gyro_z_deg_s)
{
    return DriveLqr(pose, gyro_z_deg_s, pose->x_mm, pose->y_mm,
                    app_target_yaw_deg, 0.0f, 0.0f, 0.0f, 0U, 0U, 1U,
                    APP_CHASSIS_TASK_DKMOTOR_MAX_SPEED_MM_S);
}

static HAL_StatusTypeDef DriveDkMotor(const BspChassisOdomPose *pose, float gyro_z_deg_s)
{
    float speed_mm_s = BspChassisOdom_MotorRpmToMmS(app_dkmotor_speed_rpm);
    float move_rad = app_dkmotor_angle_deg * app_pi / 180.0f;
    float forward = speed_mm_s * cosf(move_rad);
    float left = speed_mm_s * sinf(move_rad);
    float yaw_rad = pose->yaw_deg * app_pi / 180.0f;
    if (app_dkmotor_head_lock == 0U)
    {
        if (IsYawAtTarget(app_target_yaw_deg, pose->yaw_deg) == 0U)
            forward = left = 0.0f;
        else
        {
            forward = speed_mm_s;
            left = 0.0f;
        }
    }
    return DriveLqr(pose, gyro_z_deg_s, pose->x_mm, pose->y_mm, app_target_yaw_deg,
                    forward * cosf(yaw_rad) - left * sinf(yaw_rad),
                    forward * sinf(yaw_rad) + left * cosf(yaw_rad),
                    0.0f, 0U, 0U, 1U, APP_CHASSIS_TASK_DKMOTOR_MAX_SPEED_MM_S);
}

static HAL_StatusTypeDef HoldReachedMove(uint32_t now,
                                         const BspChassisOdomPose *pose,
                                         float gyro_z_deg_s)
{
    /* 到点后保持 yaw；满足最小保持时间且停稳，或达到最大等待时间，才发布 done。 */
    HAL_StatusTypeDef status = HoldTargetYaw(pose, gyro_z_deg_s);

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
    return status;
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
    app_motion_enabled = 1U;
    app_dkmotor_speed_rpm = 0.0f;
    app_dkmotor_angle_deg = 0.0f;
    app_dkmotor_head_lock = 1U;
    app_move_profile = APP_CHASSIS_TASK_PROFILE_NORMAL;
    app_velocity_forward_mm_s = 0.0f;
    app_velocity_left_mm_s = 0.0f;
    app_velocity_yaw_rad_s = 0.0f;
    app_continuous_command_tick = 0U;
    app_control_tick = 0U;
    app_control_period_ms = 0U;
    app_control_missed_deadlines = 0U;
    app_control_started = 0U;
    (void)BspChassis_Stop();
}

HAL_StatusTypeDef AppChassisTask_SetMotionEnabled(uint8_t enabled)
{
    if ((enabled != 0U) &&
        ((app_odom_ready == 0U) || (AreChassisMotorsOnline() == 0U)))
        return HAL_BUSY;
    app_motion_enabled = (enabled != 0U) ? 1U : 0U;

    if (app_motion_enabled == 0U)
    {
        app_active_command = APP_CHASSIS_TASK_DONE_NONE;
        app_done_event = APP_CHASSIS_TASK_DONE_NONE;
        app_move_reached = 0U;
        app_dkmotor_speed_rpm = 0.0f;
        app_velocity_forward_mm_s = 0.0f;
        app_velocity_left_mm_s = 0.0f;
        app_velocity_yaw_rad_s = 0.0f;
        SetMode(APP_CHASSIS_MODE_IDLE);
        (void)BspChassis_Stop();
    }
    else
    {
        BspChassis_ResetPid();
    }

    return HAL_OK;
}

uint8_t AppChassisTask_IsMotionEnabled(void)
{
    return app_motion_enabled;
}

HAL_StatusTypeDef AppChassisTask_CommandJustStop(void)
{
    if ((app_motion_enabled == 0U) || (app_odom_ready == 0U))
    {
        return HAL_BUSY;
    }

    app_active_command = APP_CHASSIS_TASK_DONE_NONE;
    app_done_event = APP_CHASSIS_TASK_DONE_NONE;
    app_move_reached = 0U;
    app_dkmotor_speed_rpm = 0.0f;
    app_velocity_forward_mm_s = 0.0f;
    app_velocity_left_mm_s = 0.0f;
    app_velocity_yaw_rad_s = 0.0f;
    SetMode(APP_CHASSIS_MODE_IDLE);

    return HAL_OK;
}

HAL_StatusTypeDef AppChassisTask_CommandDistanceCm(float x_cm,
                                                  float y_cm,
                                                  uint8_t speed_profile)
{
    /* 输入是相对世界坐标 [cm]；在此转换为里程计使用的绝对目标 [mm]。 */
    const BspChassisOdomPose *pose;

    if ((app_motion_enabled == 0U) ||
        (app_odom_ready == 0U) ||
        (speed_profile > APP_CHASSIS_TASK_PROFILE_SMOOTH))
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
    app_move_profile = speed_profile;
    SetMode(APP_CHASSIS_MODE_MOVE);

    return HAL_OK;
}

HAL_StatusTypeDef AppChassisTask_CommandTurnDeg(float target_yaw_deg)
{
    /* TURN 使用绝对角度；最短旋转方向由 BspChassis_GetAngleErrorDeg() 决定。 */
    if ((app_motion_enabled == 0U) || (app_odom_ready == 0U))
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

HAL_StatusTypeDef AppChassisTask_CommandDkMotor(uint8_t speed_percent,
                                                float move_angle_deg,
                                                uint8_t head_lock)
{
    /* 百分比先映射到线速度 [mm/s]，再按轮径/减速比换算为电机轴 rpm。 */
    float speed_mm_s;

    if ((app_motion_enabled == 0U) || (app_odom_ready == 0U) || (speed_percent > 100U))
    {
        return HAL_BUSY;
    }

    app_done_event = APP_CHASSIS_TASK_DONE_NONE;
    app_active_command = APP_CHASSIS_TASK_DONE_NONE;
    app_move_reached = 0U;
    if ((app_mode == APP_CHASSIS_MODE_DKMOTOR) &&
        (app_dkmotor_head_lock == ((head_lock != 0U) ? 1U : 0U)) &&
        (app_dkmotor_angle_deg == BspChassis_WrapAngle360(move_angle_deg)) &&
        (app_dkmotor_speed_rpm == BspChassisOdom_MmSToMotorRpm(
            ((float)speed_percent * APP_CHASSIS_TASK_DKMOTOR_MAX_SPEED_MM_S) / 100.0f)))
    {
        app_continuous_command_tick = HAL_GetTick();
        return HAL_OK;
    }
    if ((app_mode != APP_CHASSIS_MODE_DKMOTOR) ||
        (app_dkmotor_head_lock != ((head_lock != 0U) ? 1U : 0U)))
        app_target_yaw_deg = ((head_lock != 0U) ?
                              BspChassisOdom_GetPose()->yaw_deg :
                              BspChassis_WrapAngle360(move_angle_deg));
    else if (head_lock == 0U)
        app_target_yaw_deg = BspChassis_WrapAngle360(move_angle_deg);
    app_dkmotor_head_lock = (head_lock != 0U) ? 1U : 0U;
    app_dkmotor_angle_deg = BspChassis_WrapAngle360(move_angle_deg);
    app_continuous_command_tick = HAL_GetTick();

    if (speed_percent == 0U)
    {
        app_dkmotor_speed_rpm = 0.0f;
        app_target_yaw_deg = BspChassisOdom_GetPose()->yaw_deg;
        SetMode(APP_CHASSIS_MODE_IDLE);
        if (BspChassis_Stop() != HAL_OK)
        {
            StopForFault();
            return HAL_ERROR;
        }
        return HAL_OK;
    }

    speed_mm_s = ((float)speed_percent * APP_CHASSIS_TASK_DKMOTOR_MAX_SPEED_MM_S) / 100.0f;
    app_dkmotor_speed_rpm = BspChassisOdom_MmSToMotorRpm(speed_mm_s);
    if (app_mode != APP_CHASSIS_MODE_DKMOTOR)
        SetMode(APP_CHASSIS_MODE_DKMOTOR);

    return HAL_OK;
}

HAL_StatusTypeDef AppChassisTask_CommandVelocity(float forward_mm_s,
                                                float left_mm_s,
                                                float yaw_rad_s)
{
    if ((app_motion_enabled == 0U) || (app_odom_ready == 0U) ||
        !isfinite(forward_mm_s) || !isfinite(left_mm_s) || !isfinite(yaw_rad_s) ||
        hypotf(forward_mm_s, left_mm_s) > APP_CHASSIS_TASK_DKMOTOR_MAX_SPEED_MM_S ||
        AbsFloat(yaw_rad_s) > APP_LQR_YAW_RATE_LIMIT_RAD_S)
        return HAL_ERROR;
    app_active_command = APP_CHASSIS_TASK_DONE_NONE;
    app_done_event = APP_CHASSIS_TASK_DONE_NONE;
    if ((app_mode == APP_CHASSIS_MODE_VELOCITY) &&
        (app_velocity_forward_mm_s == forward_mm_s) &&
        (app_velocity_left_mm_s == left_mm_s) &&
        (app_velocity_yaw_rad_s == yaw_rad_s))
    {
        app_continuous_command_tick = HAL_GetTick();
        return HAL_OK;
    }
    app_velocity_forward_mm_s = forward_mm_s;
    app_velocity_left_mm_s = left_mm_s;
    app_velocity_yaw_rad_s = yaw_rad_s;
    app_continuous_command_tick = HAL_GetTick();
    if ((AbsFloat(forward_mm_s) + AbsFloat(left_mm_s) + AbsFloat(yaw_rad_s)) < 0.001f)
    {
        app_target_yaw_deg = BspChassisOdom_GetPose()->yaw_deg;
        SetMode(APP_CHASSIS_MODE_IDLE);
    }
    else if (app_mode != APP_CHASSIS_MODE_VELOCITY)
        SetMode(APP_CHASSIS_MODE_VELOCITY);
    return HAL_OK;
}

uint32_t AppChassisTask_GetControlPeriodMs(void) { return app_control_period_ms; }
uint32_t AppChassisTask_GetMissedDeadlines(void) { return app_control_missed_deadlines; }

HAL_StatusTypeDef AppChassisTask_GetRequestDelta(float *dx_cm,
                                                 float *dy_cm,
                                                 float *dyaw_deg,
                                                 float *yaw_deg)
{
    /* 这是有副作用的采样接口：每次成功读取都会推进“上次请求”基准。 */
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
    /* read-and-clear；同一事件只能被一个消费者读取一次。 */
    AppChassisTaskDoneEvent event = app_done_event;

    app_done_event = APP_CHASSIS_TASK_DONE_NONE;
    return event;
}

void AppChassisTask_OnYawZero(float yaw_deg)
{
    /* 只替换姿态参考，不清零已经累计的平面位置。 */
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
    /* The scheduler never catches up with bursts after a missed deadline. */
    uint32_t now = HAL_GetTick();
    const BspChassisOdomPose *pose;
    HAL_StatusTypeDef drive_status = HAL_OK;
    uint32_t elapsed;

    if ((yaw_valid == 0U) || (gyro_valid == 0U) ||
        !isfinite(yaw_deg) || !isfinite(gyro_z_deg_s))
    {
        if (app_odom_ready != 0U) StopForFault();
        app_odom_ready = 0U;
        if (app_mode != APP_CHASSIS_MODE_WAIT_IMU)
            SetMode(APP_CHASSIS_MODE_WAIT_IMU);
        if ((app_control_started == 0U) ||
            ((now - app_control_tick) >= APP_LQR_PERIOD_MS))
        {
            app_control_tick = now;
            (void)BspChassis_Stop();
        }
        return;
    }

    yaw_deg = BspChassis_WrapAngle360(yaw_deg);
    if ((app_motion_enabled != 0U) && (app_odom_ready != 0U) &&
        (AreChassisMotorsOnline() == 0U))
    {
        StopForFault();
        return;
    }

    if (((app_mode == APP_CHASSIS_MODE_DKMOTOR) ||
         (app_mode == APP_CHASSIS_MODE_VELOCITY)) &&
        (AppControlSafety_IsFresh(now, app_continuous_command_tick,
                                  APP_CHASSIS_TASK_COMMAND_TIMEOUT_MS) == 0U))
    {
        StopForFault();
        return;
    }

    if (app_control_started != 0U)
    {
        elapsed = now - app_control_tick;
        if (elapsed < APP_LQR_PERIOD_MS) return;
        app_control_period_ms = elapsed;
        if (elapsed > (APP_LQR_PERIOD_MS + APP_LQR_PERIOD_MS / 2U))
            app_control_missed_deadlines++;
        if (elapsed > APP_CHASSIS_TASK_MAX_CONTROL_GAP_MS)
        {
            StopForFault();
            app_control_tick = now;
            return;
        }
    }
    else
    {
        app_control_started = 1U;
        app_control_period_ms = APP_LQR_PERIOD_MS;
    }
    app_control_tick = now;

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

    if (app_motion_enabled == 0U)
    {
        app_active_command = APP_CHASSIS_TASK_DONE_NONE;
        app_move_reached = 0U;
        if (app_mode != APP_CHASSIS_MODE_IDLE)
        {
            SetMode(APP_CHASSIS_MODE_IDLE);
        }
        (void)BspChassis_Stop();
        return;
    }

    switch (app_mode)
    {
    case APP_CHASSIS_MODE_MOVE:
        if (app_move_reached != 0U)
        {
            drive_status = HoldReachedMove(now, pose, gyro_z_deg_s);
        }
        else if (IsMoveTargetReached(pose) != 0U)
        {
            app_move_reached = 1U;
            BspChassis_ResetPid();
            drive_status = HoldReachedMove(now, pose, gyro_z_deg_s);
        }
        else
        {
            drive_status = DriveLqr(pose, gyro_z_deg_s, app_target_x_mm,
                                    app_target_y_mm, app_target_yaw_deg,
                                    0.0f, 0.0f, 0.0f, 1U, 1U, 1U,
                                    CalcProfileMaxSpeed(pose));
        }
        break;

    case APP_CHASSIS_MODE_TURN:
        if ((IsYawAtTarget(app_target_yaw_deg, yaw_deg) != 0U) &&
            (IsStoppedStable(now) != 0U))
        {
            MarkActiveCommandDone();
            SetMode(APP_CHASSIS_MODE_IDLE);
            drive_status = HoldTargetYaw(pose, gyro_z_deg_s);
        }
        else
        {
            drive_status = HoldTargetYaw(pose, gyro_z_deg_s);
        }
        break;

    case APP_CHASSIS_MODE_DKMOTOR:
        drive_status = DriveDkMotor(pose, gyro_z_deg_s);
        break;

    case APP_CHASSIS_MODE_VELOCITY:
    {
        float yaw_rad = pose->yaw_deg * app_pi / 180.0f;
        float cy = cosf(yaw_rad);
        float sy = sinf(yaw_rad);
        drive_status = DriveLqr(pose, gyro_z_deg_s, pose->x_mm, pose->y_mm,
                                pose->yaw_deg,
                                app_velocity_forward_mm_s * cy - app_velocity_left_mm_s * sy,
                                app_velocity_forward_mm_s * sy + app_velocity_left_mm_s * cy,
                                app_velocity_yaw_rad_s, 0U, 0U, 0U,
                                APP_CHASSIS_TASK_DKMOTOR_MAX_SPEED_MM_S);
        break;
    }

    case APP_CHASSIS_MODE_WAIT_IMU:
        drive_status = BspChassis_Stop();
        break;

    case APP_CHASSIS_MODE_IDLE:
    default:
        drive_status = HoldTargetYaw(pose, gyro_z_deg_s);
        break;
    }
    if (drive_status != HAL_OK) StopForFault();
}
