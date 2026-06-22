#ifndef BSP_SUCTION_MOTOR_TEST_H
#define BSP_SUCTION_MOTOR_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#ifndef BSP_SUCTION_MOTOR_TEST_ENABLE
/* 置 1 后自动执行 ESC 脉宽阶梯测试；安装叶轮时必须做好防护。 */
#define BSP_SUCTION_MOTOR_TEST_ENABLE 0U
#endif

#ifndef BSP_SUCTION_MOTOR_TEST_PRINT_USART
/* 测试日志串口；必须是 BspUsart_GetHandle() 已映射的编号。 */
#define BSP_SUCTION_MOTOR_TEST_PRINT_USART BSP_USART_6
#endif

#ifndef BSP_SUCTION_MOTOR_TEST_INIT_HOLD_MS
/* 上电解锁脉宽保持时间 [ms]，应满足具体 ESC 的初始化要求。 */
#define BSP_SUCTION_MOTOR_TEST_INIT_HOLD_MS 3000U
#endif

#ifndef BSP_SUCTION_MOTOR_TEST_STEP_HOLD_MS
/* 每级测试脉宽保持时间 [ms]；过短难以观察稳态电流和振动。 */
#define BSP_SUCTION_MOTOR_TEST_STEP_HOLD_MS 2000U
#endif

#ifndef BSP_SUCTION_MOTOR_TEST_MAX_PULSE_US
/* 测试允许的最高脉宽 [us]；首次运行应保守设置并逐步增加。 */
#define BSP_SUCTION_MOTOR_TEST_MAX_PULSE_US 1200U
#endif

void BspSuctionMotorTest_Init(void);
void BspSuctionMotorTest_Task(void);

#ifdef __cplusplus
}
#endif

#endif
