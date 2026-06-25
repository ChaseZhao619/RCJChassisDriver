#ifndef BSP_SUCTION_DETECT_H
#define BSP_SUCTION_DETECT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#ifndef BSP_SUCTION_DETECT_ACTIVE_LEVEL
/* 检测到球时 GPIO 的电平。若逻辑完全反向，只修改此项；抖动应在上层做时间消抖。 */
#define BSP_SUCTION_DETECT_ACTIVE_LEVEL GPIO_PIN_RESET
#endif

HAL_StatusTypeDef BspSuctionDetect_Init(void);
uint8_t BspSuctionDetect_IsBallDetected(void);       /* 返回按有效电平换算后的逻辑值 0/1。 */
GPIO_PinState BspSuctionDetect_GetPinLevel(void);    /* 返回未经滤波的原始 GPIO 电平。 */

#ifdef __cplusplus
}
#endif

#endif
