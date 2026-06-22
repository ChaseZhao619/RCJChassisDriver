#include "bsp_suction_detect.h"

/*
 * 数字吸球检测输入。
 * 当前使用内部上拉，因此断线通常读为高电平；是否代表“无球”由
 * BSP_SUCTION_DETECT_ACTIVE_LEVEL 决定。本模块只做电平到逻辑值的转换，
 * 不做消抖。若任务周期为 T ms，建议上层连续 N 次有效后再确认，确认延迟约 N*T ms。
 */
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
    /* 统一返回 0/1，避免上层依赖 GPIO_PinState 的具体枚举值。 */
    return (BspSuctionDetect_GetPinLevel() == BSP_SUCTION_DETECT_ACTIVE_LEVEL) ? 1U : 0U;
}

GPIO_PinState BspSuctionDetect_GetPinLevel(void)
{
    /* 原始采样接口用于诊断极性、线路断开和抖动。 */
    return HAL_GPIO_ReadPin(xqwd_GPIO_Port, xqwd_Pin);
}
