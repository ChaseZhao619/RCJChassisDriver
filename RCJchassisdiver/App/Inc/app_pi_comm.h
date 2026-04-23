#ifndef APP_PI_COMM_H
#define APP_PI_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

void AppPiComm_Init(void);
void AppPiComm_Task(void);
void AppPiComm_OnUartRxCplt(UART_HandleTypeDef *huart);
void AppPiComm_OnUartError(UART_HandleTypeDef *huart);
uint16_t AppPiComm_Crc16Ccitt(const uint8_t *data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
