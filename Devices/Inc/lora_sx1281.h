#ifndef __LORA_SX1281_H
#define __LORA_SX1281_H

#include <stdint.h>
#include "lora_config.h"

typedef enum
{
    LORA_RADIO_STATE_NOT_INIT = 0U,
    LORA_RADIO_STATE_READY    = 1U,
    LORA_RADIO_STATE_RX       = 2U,
    LORA_RADIO_STATE_TX       = 3U,
    LORA_RADIO_STATE_BUSY     = 4U
} LoraRadioState;

typedef enum
{
    LORA_TX_ENQUEUE_OK = 0U,
    LORA_TX_ENQUEUE_NOT_INIT,
    LORA_TX_ENQUEUE_BAD_PARAM,
    LORA_TX_ENQUEUE_QUEUE_FULL
} LoraTxEnqueueResult;

typedef enum
{
    LORA_RX_DEQUEUE_OK = 0U,
    LORA_RX_DEQUEUE_EMPTY,
    LORA_RX_DEQUEUE_BAD_PARAM
} LoraRxDequeueResult;

typedef enum
{
    LORA_BUSY_IDLE = 0U,
    LORA_BUSY_ACTIVE = 1U
} LoraBusyState;

typedef struct
{
    uint32_t tx_ok;
    uint32_t tx_dropped;
    uint32_t tx_timeout;
    uint32_t rx_ok;
    uint32_t rx_dropped;
    uint32_t rx_timeout;
    uint32_t rx_error;
    uint32_t rx_crc_error;
    LoraRadioState radio_state;
} LoraStats;

void Lora_Init(void);
void Lora_Process(void);
void Lora_StartRx(void);

LoraTxEnqueueResult Lora_TxEnqueue(const uint8_t *data, uint8_t len);
LoraRxDequeueResult Lora_RxDequeue(uint8_t *data, uint8_t *len, int8_t *rssi, int8_t *snr);

void Lora_GetStats(LoraStats *stats);
LoraBusyState Lora_IsBusy(void);

#endif
