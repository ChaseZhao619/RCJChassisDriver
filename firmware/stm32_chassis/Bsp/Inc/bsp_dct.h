#ifndef BSP_DCT_H
#define BSP_DCT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* 初始化数字控制输出并进入实现规定的安全默认状态。 */
HAL_StatusTypeDef BspDct_Init(void);
/* enabled 非 0 表示使能；实际有效电平由原理图和 bsp_dct.c 定义。 */
HAL_StatusTypeDef BspDct_SetEnabled(uint8_t enabled);
uint8_t BspDct_GetEnabled(void);

#ifdef __cplusplus
}
#endif

#endif
