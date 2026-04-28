#include "bsp_dct.h"

static uint8_t dct_enabled;

HAL_StatusTypeDef BspDct_Init(void)
{
    return BspDct_SetEnabled(0U);
}

HAL_StatusTypeDef BspDct_SetEnabled(uint8_t enabled)
{
    dct_enabled = (enabled != 0U) ? 1U : 0U;
    HAL_GPIO_WritePin(JD1_GPIO_Port,
                      JD1_Pin,
                      (dct_enabled != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    return HAL_OK;
}

uint8_t BspDct_GetEnabled(void)
{
    return dct_enabled;
}
