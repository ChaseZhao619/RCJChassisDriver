#ifndef BSP_CHASSIS_TEST_H
#define BSP_CHASSIS_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#ifndef BSP_CHASSIS_TEST_ENABLE
#define BSP_CHASSIS_TEST_ENABLE 1U
#endif

#ifndef BSP_CHASSIS_TEST_MOVE_RPM
#define BSP_CHASSIS_TEST_MOVE_RPM 1500.0f
#endif

#ifndef BSP_CHASSIS_TEST_MAX_CURRENT
#define BSP_CHASSIS_TEST_MAX_CURRENT 8000
#endif

#ifndef BSP_CHASSIS_TEST_PRINT_ENABLE
#define BSP_CHASSIS_TEST_PRINT_ENABLE 0U
#endif

void BspChassisTest_Init(void);
void BspChassisTest_Task(uint8_t yaw_valid, float yaw_deg);

#ifdef __cplusplus
}
#endif

#endif
