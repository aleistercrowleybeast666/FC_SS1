#ifndef __BSP_UART_DEBUG_H
#define __BSP_UART_DEBUG_H

#include <stdint.h>
#include "main.h"

void BspUartDebug_Init(void);

/* 发送接口 */
uint16_t BspUartDebug_Write(const uint8_t *data, uint16_t len);

/* 接收接口 */
uint16_t BspUartDebug_Read(uint8_t *data, uint16_t len);
uint16_t BspUartDebug_GetRxCount(void);

/* 发送环形缓冲区丢弃统计 */
uint16_t BspUartDebug_GetTxDiscarded(void);
void BspUartDebug_ResetTxDiscarded(void);

/* HAL回调转发 */
void BspUartDebug_TxCpltCallback(UART_HandleTypeDef *huart);
void BspUartDebug_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);
/* 处理错误 */
void BspUartDebug_ErrorCallback(UART_HandleTypeDef *huart);

/* 主动尝试启动发送（一般内部调用，也可外部补偿调用） */
void BspUartDebug_TryStartTx(void);


#endif /* BSP_UART_DEBUG_H */