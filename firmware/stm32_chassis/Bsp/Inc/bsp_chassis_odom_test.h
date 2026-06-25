#ifndef BSP_CHASSIS_ODOM_TEST_H
#define BSP_CHASSIS_ODOM_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#ifndef BSP_CHASSIS_ODOM_TEST_ENABLE
/* 场地路径测试总开关；置 1 会自动驱动底盘，正常固件应置 0。 */
#define BSP_CHASSIS_ODOM_TEST_ENABLE 1U
#endif

#ifndef BSP_CHASSIS_ODOM_TEST_PRINT_ENABLE
/* 测试遥测输出开关；定位时开启，性能测试时可关闭。 */
#define BSP_CHASSIS_ODOM_TEST_PRINT_ENABLE 0U
#endif

#ifndef BSP_CHASSIS_ODOM_TEST_YAW_TOLERANCE_DEG
/* 航向达到判定容差 [deg]；过小会使路径阶段难以结束。 */
#define BSP_CHASSIS_ODOM_TEST_YAW_TOLERANCE_DEG 3.0f
#endif

#ifndef BSP_CHASSIS_ODOM_TEST_STOP_RPM
/* 四轮反馈绝对值均低于此值才视为接近停止 [rpm]。 */
#define BSP_CHASSIS_ODOM_TEST_STOP_RPM 35
#endif

#ifndef BSP_CHASSIS_ODOM_TEST_STOP_STABLE_MS
/* 低于停止阈值后需持续稳定的时间 [ms]，用于过滤瞬时过零。 */
#define BSP_CHASSIS_ODOM_TEST_STOP_STABLE_MS 250U
#endif

#ifndef BSP_CHASSIS_ODOM_TEST_STOP_MAX_WAIT_MS
/* 等待电机停止的最长时间 [ms]，超时后继续状态机以避免卡死。 */
#define BSP_CHASSIS_ODOM_TEST_STOP_MAX_WAIT_MS 1500U
#endif

#ifndef BSP_CHASSIS_ODOM_TEST_PROFILE_MIN_SCALE
/* 加减速曲线的最小速度比例；增大更不易停滞，但终点超调会加重。 */
#define BSP_CHASSIS_ODOM_TEST_PROFILE_MIN_SCALE 0.08f
#endif

#ifndef BSP_CHASSIS_ODOM_TEST_SEGMENT_DONE_PROGRESS
/* 沿线段完成比例，范围 0~1；越接近 1 路径覆盖越完整。 */
#define BSP_CHASSIS_ODOM_TEST_SEGMENT_DONE_PROGRESS 0.990f
#endif

#ifndef BSP_CHASSIS_ODOM_TEST_FIELD_LENGTH_MM
/* 可用场地长度 [mm]，必须按实际边界复核。 */
#define BSP_CHASSIS_ODOM_TEST_FIELD_LENGTH_MM 2150.0f
#endif

#ifndef BSP_CHASSIS_ODOM_TEST_FIELD_WIDTH_MM
/* 可用场地宽度 [mm]，必须按实际边界复核。 */
#define BSP_CHASSIS_ODOM_TEST_FIELD_WIDTH_MM 1540.0f
#endif

#ifndef BSP_CHASSIS_ODOM_TEST_LINE_MARGIN_MM
/* 车体外沿相对边界的额外安全余量 [mm]。 */
#define BSP_CHASSIS_ODOM_TEST_LINE_MARGIN_MM 20.0f
#endif

#ifndef BSP_CHASSIS_ODOM_TEST_BORDER_SPEED_MM_S
/* 沿场地边界运行的最大速度 [mm/s]。 */
#define BSP_CHASSIS_ODOM_TEST_BORDER_SPEED_MM_S 350.0f
#endif

#ifndef BSP_CHASSIS_ODOM_TEST_LINE_CROSS_KP
/* 横向偏离到纠偏速度的比例增益 [(mm/s)/mm]。 */
#define BSP_CHASSIS_ODOM_TEST_LINE_CROSS_KP 2.2f
#endif

#ifndef BSP_CHASSIS_ODOM_TEST_LINE_CROSS_MAX_MM_S
/* 横向纠偏速度绝对值上限 [mm/s]，避免大偏差时动作过猛。 */
#define BSP_CHASSIS_ODOM_TEST_LINE_CROSS_MAX_MM_S 180.0f
#endif

#ifndef BSP_CHASSIS_ODOM_TEST_LINE_CROSS_DEADBAND_MM
/* 横向误差死区 [mm]，增大可减少来回修正，但路径精度下降。 */
#define BSP_CHASSIS_ODOM_TEST_LINE_CROSS_DEADBAND_MM 8.0f
#endif

#ifndef BSP_CHASSIS_ODOM_TEST_MIN_SPEED_DISTANCE_MM
/* 距离终点小于该值时开始按剩余距离降低速度。 */
#define BSP_CHASSIS_ODOM_TEST_MIN_SPEED_DISTANCE_MM 220.0f
#endif

void BspChassisOdomTest_Init(void);
void BspChassisOdomTest_Task(uint8_t yaw_valid,
                             float yaw_deg,
                             uint8_t gyro_valid,
                             float gyro_z_deg_s);

#ifdef __cplusplus
}
#endif

#endif
