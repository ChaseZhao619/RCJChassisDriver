#ifndef BSP_CHASSIS_TEST_H
#define BSP_CHASSIS_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#ifndef BSP_CHASSIS_TEST_ENABLE
/* 置 1 后测试任务可能自动驱动底盘；正常业务固件必须置 0。 */
#define BSP_CHASSIS_TEST_ENABLE 1U
#endif

#ifndef BSP_CHASSIS_TEST_MOVE_RPM
/* 测试平移目标 [电机 rpm]；首次架空测试建议显著降低。 */
#define BSP_CHASSIS_TEST_MOVE_RPM 1500.0f
#endif

#ifndef BSP_CHASSIS_TEST_MAX_CURRENT
/* 测试期间单电机电流控制量上限；不应高于硬件允许值。 */
#define BSP_CHASSIS_TEST_MAX_CURRENT 8000
#endif

#ifndef BSP_CHASSIS_TEST_PRINT_ENABLE
/* 置 1 输出诊断信息；同步串口打印可能影响任务周期。 */
#define BSP_CHASSIS_TEST_PRINT_ENABLE 0U
#endif

void BspChassisTest_Init(void);
void BspChassisTest_Task(uint8_t yaw_valid, float yaw_deg);

#ifdef __cplusplus
}
#endif

#endif
