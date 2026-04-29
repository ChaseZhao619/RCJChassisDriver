#ifndef BSP_SUCTION_DETECT_H
#define BSP_SUCTION_DETECT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#ifndef BSP_SUCTION_DETECT_ACTIVE_LEVEL
#define BSP_SUCTION_DETECT_ACTIVE_LEVEL GPIO_PIN_RESET
#endif

HAL_StatusTypeDef BspSuctionDetect_Init(void);
uint8_t BspSuctionDetect_IsBallDetected(void);
GPIO_PinState BspSuctionDetect_GetPinLevel(void);

#ifdef __cplusplus
}
#endif

#endif
