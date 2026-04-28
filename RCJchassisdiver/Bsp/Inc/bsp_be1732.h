#ifndef BSP_BE1732_H
#define BSP_BE1732_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define BSP_BE1732_I2C_ADDR                 0x01U
#define BSP_BE1732_CHANNEL_COUNT            7U
#define BSP_BE1732_CMD_CHANNEL_1            1U
#define BSP_BE1732_CMD_STRONGEST_CHANNEL    8U
#define BSP_BE1732_CMD_STRONGEST_VALUE      9U
#define BSP_BE1732_CMD_WEAKEST_CHANNEL      10U
#define BSP_BE1732_CMD_WEAKEST_VALUE        11U
#define BSP_BE1732_CMD_AVERAGE_VALUE        12U
#define BSP_BE1732_CMD_NORMAL_MODE          13U
#define BSP_BE1732_CMD_MODULATED_MODE       14U
#define BSP_BE1732_CMD_ZERO_CAL             15U

#ifndef BSP_BE1732_I2C_TIMEOUT_MS
#define BSP_BE1732_I2C_TIMEOUT_MS           20U
#endif

#ifndef BSP_BE1732_I2C_READY_TRIALS
#define BSP_BE1732_I2C_READY_TRIALS         2U
#endif

typedef enum
{
    BSP_BE1732_MODE_NORMAL = 0,
    BSP_BE1732_MODE_MODULATED,
} BspBe1732Mode;

HAL_StatusTypeDef BspBe1732_Init(void);
HAL_StatusTypeDef BspBe1732_ReadCommand(uint8_t command, uint8_t *value);
HAL_StatusTypeDef BspBe1732_ReadChannelValue(uint8_t channel, uint8_t *value);
HAL_StatusTypeDef BspBe1732_ReadStrongestChannel(uint8_t *channel);
HAL_StatusTypeDef BspBe1732_SetMode(BspBe1732Mode mode);
BspBe1732Mode BspBe1732_GetMode(void);
uint32_t BspBe1732_GetLastI2cError(void);

#ifdef __cplusplus
}
#endif

#endif
