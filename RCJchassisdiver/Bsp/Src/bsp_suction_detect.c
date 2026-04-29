#include "bsp_suction_detect.h"

HAL_StatusTypeDef BspSuctionDetect_Init(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    gpio_init.Pin = xqwd_Pin;
    gpio_init.Mode = GPIO_MODE_INPUT;
    gpio_init.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(xqwd_GPIO_Port, &gpio_init);

    return HAL_OK;
}

uint8_t BspSuctionDetect_IsBallDetected(void)
{
    return (BspSuctionDetect_GetPinLevel() == BSP_SUCTION_DETECT_ACTIVE_LEVEL) ? 1U : 0U;
}

GPIO_PinState BspSuctionDetect_GetPinLevel(void)
{
    return HAL_GPIO_ReadPin(xqwd_GPIO_Port, xqwd_Pin);
}
