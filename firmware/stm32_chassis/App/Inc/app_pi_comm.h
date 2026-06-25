#ifndef APP_PI_COMM_H
#define APP_PI_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/*
 * 树莓派 USART6 ASCII 协议入口。
 * 帧格式、命令列表和 CRC 参数见 App/README.md。
 */
/* 清空接收状态并启动 1 字节中断接收；应在 UART 初始化完成后调用一次。 */
void AppPiComm_Init(void);
/* 从环形缓冲区解析完整行、执行命令并发送完成事件；应在主循环持续调用。 */
void AppPiComm_Task(void);
/* USART HAL 回调分发入口；仅处理 USART6，其他 UART 会被忽略。 */
void AppPiComm_OnUartRxCplt(UART_HandleTypeDef *huart);
/* UART 错误后重新挂接接收；不清空已进入软件环形缓冲区的数据。 */
void AppPiComm_OnUartError(UART_HandleTypeDef *huart);
/* CRC-16/CCITT-FALSE：poly=0x1021, init=0xFFFF, refin/refout=false, xorout=0。 */
uint16_t AppPiComm_Crc16Ccitt(const uint8_t *data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
