#ifndef __DEBUG_CONFIG_H
#define __DEBUG_CONFIG_H

#include "main.h"
#include "usart.h"
#include "cmsis_gcc.h"
#include <stdint.h>

/* ===================== 串口选择 ===================== */
#define DEBUG_UART_HANDLE                 huart1

/* ===================== 缓冲区大小 ===================== */
/* TX软件环形缓冲区 */
#define DEBUG_TX_RINGBUF_SIZE             1024U
/* RX软件环形缓冲区 */
#define DEBUG_RX_RINGBUF_SIZE             1024U
/* DMA接收暂存缓冲区（半包/空闲中断搬运到RX环形缓冲区） */
#define DEBUG_RX_DMA_BUF_SIZE             128U

/* ===================== 日志格式 ===================== */
#define DEBUG_LOG_LINE_SIZE               192U
#define DEBUG_LOG_ENABLE                  1
#define DEBUG_LOG_AUTO_CRLF               1
#define DEBUG_LOG_PREFIX                  "[DBG] "

/* ===================== 临界区：FreeRTOS兼容 ===================== */
/*
 * 这里不直接用 taskENTER_CRITICAL()，
 * 因为本驱动既会在任务上下文调用，也会在ISR中调用。
 * 用PRIMASK保存/恢复更稳妥。
 */
static inline uint32_t Debug_IrqLock(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void Debug_IrqUnlock(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

#endif /* DEBUG_CONFIG_H */