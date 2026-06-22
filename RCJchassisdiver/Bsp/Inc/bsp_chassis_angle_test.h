#ifndef BSP_CHASSIS_ANGLE_TEST_H
#define BSP_CHASSIS_ANGLE_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#ifndef BSP_CHASSIS_ANGLE_TEST_ENABLE
/* 置 1 启用串口交互角度测试；与其他底盘测试互斥。 */
#define BSP_CHASSIS_ANGLE_TEST_ENABLE 1U
#endif

#ifndef BSP_CHASSIS_ANGLE_TEST_MAX_CURRENT
/* 角度测试电流控制量上限；首次确认控制方向时应使用较小值。 */
#define BSP_CHASSIS_ANGLE_TEST_MAX_CURRENT 10000
#endif

#ifndef BSP_CHASSIS_ANGLE_TEST_PRINT_MS
/* 状态打印周期 [ms]；减小会增加串口阻塞和主循环负载。 */
#define BSP_CHASSIS_ANGLE_TEST_PRINT_MS 50U
#endif

void BspChassisAngleTest_Init(void);
void BspChassisAngleTest_Task(uint8_t yaw_valid, float yaw_deg);

#ifdef __cplusplus
}
#endif

#endif
