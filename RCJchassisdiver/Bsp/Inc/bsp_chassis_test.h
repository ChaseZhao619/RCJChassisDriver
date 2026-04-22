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

#ifndef BSP_CHASSIS_TEST_CURRENT
#define BSP_CHASSIS_TEST_CURRENT 500
#endif

#ifndef BSP_CHASSIS_TEST_MAX_CURRENT
#define BSP_CHASSIS_TEST_MAX_CURRENT 700
#endif

void BspChassisTest_Init(void);
void BspChassisTest_Task(void);

#ifdef __cplusplus
}
#endif

#endif
