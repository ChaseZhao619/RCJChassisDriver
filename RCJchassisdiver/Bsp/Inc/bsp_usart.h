#ifndef BSP_USART_H
#define BSP_USART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdarg.h>
#include <stdint.h>

#ifndef BSP_USART_PRINTF_BUFFER_SIZE
#define BSP_USART_PRINTF_BUFFER_SIZE 256U
#endif

typedef enum
{
    BSP_USART_1 = 1,
    BSP_USART_6 = 6
} BspUsartId;

UART_HandleTypeDef *BspUsart_GetHandle(uint8_t usart_id);
HAL_StatusTypeDef BspUsart_Transmit(uint8_t usart_id, const uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef BspUsart_Receive(uint8_t usart_id, uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef BspUsart_ReceiveIT(uint8_t usart_id, uint8_t *data, uint16_t size);

int Printf(uint8_t usart_id, const char *format, ...);
int VPrintf(uint8_t usart_id, const char *format, va_list args);

#ifdef __GNUC__
int Printf(uint8_t usart_id, const char *format, ...) __attribute__((format(printf, 2, 3)));
int VPrintf(uint8_t usart_id, const char *format, va_list args) __attribute__((format(printf, 2, 0)));
#endif

#ifdef __cplusplus
}
#endif

#endif
