#include "bsp_usart.h"

#include "usart.h"
#include <stdio.h>

UART_HandleTypeDef *BspUsart_GetHandle(uint8_t usart_id)
{
    switch (usart_id)
    {
    case BSP_USART_1:
        return &huart1;
    case BSP_USART_6:
        return &huart6;
    default:
        return NULL;
    }
}

HAL_StatusTypeDef BspUsart_Transmit(uint8_t usart_id, const uint8_t *data, uint16_t size, uint32_t timeout)
{
    UART_HandleTypeDef *huart = BspUsart_GetHandle(usart_id);

    if ((huart == NULL) || (data == NULL) || (size == 0U))
    {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(huart, (uint8_t *)data, size, timeout);
}

HAL_StatusTypeDef BspUsart_Receive(uint8_t usart_id, uint8_t *data, uint16_t size, uint32_t timeout)
{
    UART_HandleTypeDef *huart = BspUsart_GetHandle(usart_id);

    if ((huart == NULL) || (data == NULL) || (size == 0U))
    {
        return HAL_ERROR;
    }

    return HAL_UART_Receive(huart, data, size, timeout);
}

HAL_StatusTypeDef BspUsart_ReceiveIT(uint8_t usart_id, uint8_t *data, uint16_t size)
{
    UART_HandleTypeDef *huart = BspUsart_GetHandle(usart_id);

    if ((huart == NULL) || (data == NULL) || (size == 0U))
    {
        return HAL_ERROR;
    }

    return HAL_UART_Receive_IT(huart, data, size);
}

int VPrintf(uint8_t usart_id, const char *format, va_list args)
{
    char buffer[BSP_USART_PRINTF_BUFFER_SIZE];
    int length;

    if (format == NULL)
    {
        return -1;
    }

    length = vsnprintf(buffer, sizeof(buffer), format, args);
    if (length < 0)
    {
        return -2;
    }

    if ((uint32_t)length >= sizeof(buffer))
    {
        length = (int)sizeof(buffer) - 1;
    }

    if (BspUsart_Transmit(usart_id, (const uint8_t *)buffer, (uint16_t)length, HAL_MAX_DELAY) != HAL_OK)
    {
        return -3;
    }

    return length;
}

int Printf(uint8_t usart_id, const char *format, ...)
{
    va_list args;
    int length;

    va_start(args, format);
    length = VPrintf(usart_id, format, args);
    va_end(args);

    return length;
}
