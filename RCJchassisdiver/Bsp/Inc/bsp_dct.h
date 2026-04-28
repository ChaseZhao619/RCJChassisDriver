#ifndef BSP_DCT_H
#define BSP_DCT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

HAL_StatusTypeDef BspDct_Init(void);
HAL_StatusTypeDef BspDct_SetEnabled(uint8_t enabled);
uint8_t BspDct_GetEnabled(void);

#ifdef __cplusplus
}
#endif

#endif
