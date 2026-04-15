#include "bsp_uart_debug.h"
#include "debug_config.h"
#include "common_ringbuf.h"
#include "usart.h"
#include <string.h>

/* ===================== 静态缓冲区 ===================== */
static uint8_t s_tx_ringbuf_mem[DEBUG_TX_RINGBUF_SIZE];
static uint8_t s_rx_ringbuf_mem[DEBUG_RX_RINGBUF_SIZE];
static uint8_t s_rx_dma_buf[DEBUG_RX_DMA_BUF_SIZE];

static ringbuf_t s_tx_rb;
static ringbuf_t s_rx_rb;

/* DMA发送状态 */
static volatile uint8_t  s_tx_dma_busy = 0U;
static volatile uint16_t s_tx_dma_len  = 0U;

/* ===================== 内部函数声明 ===================== */
static UART_HandleTypeDef *Debug_GetUartHandle(void);
static void Debug_StartRxDmaToIdle(void);

/* ===================== 内部函数实现 ===================== */
static UART_HandleTypeDef *Debug_GetUartHandle(void)
{
    return &DEBUG_UART_HANDLE;
}

static void Debug_StartRxDmaToIdle(void)
{
    UART_HandleTypeDef *huart = Debug_GetUartHandle();

    (void)HAL_UARTEx_ReceiveToIdle_DMA(huart, s_rx_dma_buf, DEBUG_RX_DMA_BUF_SIZE);

    if (huart->hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}

/* ===================== 对外接口 ===================== */
void BspUartDebug_Init(void)
{
    uint32_t primask;

    primask = Debug_IrqLock();

    RingBuf_Init(&s_tx_rb, s_tx_ringbuf_mem, DEBUG_TX_RINGBUF_SIZE);
    RingBuf_Init(&s_rx_rb, s_rx_ringbuf_mem, DEBUG_RX_RINGBUF_SIZE);

    s_tx_dma_busy = 0U;
    s_tx_dma_len  = 0U;

    Debug_IrqUnlock(primask);

    Debug_StartRxDmaToIdle();
}

uint16_t BspUartDebug_Write(const uint8_t *data, uint16_t len)
{
    uint16_t written;
    uint32_t primask;

    if ((data == NULL) || (len == 0U))
    {
        return 0U;
    }

    primask = Debug_IrqLock();
    written = RingBuf_Push(&s_tx_rb, data, len);
    Debug_IrqUnlock(primask);

    BspUartDebug_TryStartTx();
    return written;
}

void BspUartDebug_TryStartTx(void)
{
    UART_HandleTypeDef *huart;
    uint8_t *ptr;
    uint16_t len;
    uint32_t primask;

    huart = Debug_GetUartHandle();

    primask = Debug_IrqLock();

    if (s_tx_dma_busy != 0U)
    {
        Debug_IrqUnlock(primask);
        return;
    }

    len = RingBuf_GetLinearReadLen(&s_tx_rb);
    if (len == 0U)
    {
        Debug_IrqUnlock(primask);
        return;
    }

    ptr = RingBuf_GetLinearReadPtr(&s_tx_rb);
    s_tx_dma_busy = 1U;
    s_tx_dma_len  = len;

    Debug_IrqUnlock(primask);

    if (HAL_UART_Transmit_DMA(huart, ptr, len) != HAL_OK)
    {
        primask = Debug_IrqLock();
        s_tx_dma_busy = 0U;
        s_tx_dma_len  = 0U;
        Debug_IrqUnlock(primask);
    }
}

void BspUartDebug_TxCpltCallback(UART_HandleTypeDef *huart)
{
    uint32_t primask;
    uint16_t sent_len;

    if (huart != Debug_GetUartHandle())
    {
        return;
    }

    primask = Debug_IrqLock();

    sent_len = s_tx_dma_len;
    if (sent_len > 0U)
    {
        RingBuf_Skip(&s_tx_rb, sent_len);
    }

    s_tx_dma_len  = 0U;
    s_tx_dma_busy = 0U;

    Debug_IrqUnlock(primask);

    BspUartDebug_TryStartTx();
}

void BspUartDebug_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    uint32_t primask;

    if (huart != Debug_GetUartHandle())
    {
        return;
    }

    if (size > 0U)
    {
        primask = Debug_IrqLock();
        (void)RingBuf_Push(&s_rx_rb, s_rx_dma_buf, size);
        Debug_IrqUnlock(primask);
    }

    Debug_StartRxDmaToIdle();
}

void BspUartDebug_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != Debug_GetUartHandle())
    {
        return;
    }

    /*
     * debug串口主要问题一般出在接收侧：
     * 过载、噪声、帧错误后 DMA 接收可能停住。
     * 这里直接中止接收并重启 ReceiveToIdle DMA。
     */
    (void)HAL_UART_AbortReceive(huart);
    Debug_StartRxDmaToIdle();

    /*
     * 顺手补偿触发一次发送启动。
     * 正常情况下不会有副作用；
     * 若之前因为状态竞争导致未继续发送，这里能拉起来。
     */
    BspUartDebug_TryStartTx();
}

uint16_t BspUartDebug_Read(uint8_t *data, uint16_t len)
{
    uint16_t read_len;
    uint32_t primask;

    if ((data == NULL) || (len == 0U))
    {
        return 0U;
    }

    primask = Debug_IrqLock();
    read_len = RingBuf_Pop(&s_rx_rb, data, len);
    Debug_IrqUnlock(primask);

    return read_len;
}

uint16_t BspUartDebug_GetRxCount(void)
{
    uint16_t cnt;
    uint32_t primask;

    primask = Debug_IrqLock();
    cnt = RingBuf_GetUsed(&s_rx_rb);
    Debug_IrqUnlock(primask);

    return cnt;
}

uint16_t BspUartDebug_GetTxDiscarded(void)
{
    uint16_t discarded;
    uint32_t primask;

    primask = Debug_IrqLock();
    discarded = RingBuf_GetDiscarded(&s_tx_rb);
    Debug_IrqUnlock(primask);

    return discarded;
}

void BspUartDebug_ResetTxDiscarded(void)
{
    uint32_t primask;

    primask = Debug_IrqLock();
    RingBuf_ResetDiscarded(&s_tx_rb);
    Debug_IrqUnlock(primask);
}