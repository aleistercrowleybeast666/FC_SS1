#include "lora_sx1281.h"

#include <string.h>

#include "cmsis_gcc.h"
#include "sx1280.h"
#include "../../Middlewares/Third_Party/SX1280lib/radio.h"
#include "bsp_sx1281_port.h"

typedef struct
{
    uint8_t len;
    uint8_t data[LORA_MAX_PAYLOAD_LEN];
} LoraTxPacket;

typedef struct
{
    uint8_t len;
    int8_t rssi;
    int8_t snr;
    uint8_t data[LORA_MAX_PAYLOAD_LEN];
} LoraRxPacket;

static volatile uint8_t s_tx_done_flag = 0U;
static volatile uint8_t s_rx_done_flag = 0U;
static volatile uint8_t s_tx_timeout_flag = 0U;
static volatile uint8_t s_rx_timeout_flag = 0U;
static volatile uint8_t s_rx_error_flag = 0U;
static volatile IrqErrorCode_t s_rx_error_code = IRQ_HEADER_ERROR_CODE;

static volatile uint8_t s_inited = 0U;
static volatile uint8_t s_tx_busy = 0U;
static volatile uint8_t s_radio_in_rx = 0U;

static uint8_t s_rx_tmp_buf[LORA_MAX_PAYLOAD_LEN];
static PacketStatus_t s_pkt_status;
static ModulationParams_t s_mod_params;
static PacketParams_t s_pkt_params;
static LoraStats s_stats;

static LoraTxPacket s_tx_queue[LORA_TX_QUEUE_DEPTH];
static uint8_t s_tx_head = 0U;
static uint8_t s_tx_tail = 0U;
static uint8_t s_tx_count = 0U;

static LoraRxPacket s_rx_queue[LORA_RX_QUEUE_DEPTH];
static uint8_t s_rx_head = 0U;
static uint8_t s_rx_tail = 0U;
static uint8_t s_rx_count = 0U;

static void Lora_OnTxDone(void);
static void Lora_OnRxDone(void);
static void Lora_OnTxTimeout(void);
static void Lora_OnRxTimeout(void);
static void Lora_OnRxError(IrqErrorCode_t errCode);

static RadioCallbacks_t s_radio_callbacks =
{
    Lora_OnTxDone,
    Lora_OnRxDone,
    0,
    0,
    Lora_OnTxTimeout,
    Lora_OnRxTimeout,
    Lora_OnRxError,
    0,
    0
};

static uint32_t Lora_IrqLock(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void Lora_IrqUnlock(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static void Lora_SetRadioState(LoraRadioState state)
{
    uint32_t primask;

    primask = Lora_IrqLock();
    s_stats.radio_state = state;
    Lora_IrqUnlock(primask);
}

static void Lora_StatsIncrement(uint32_t *counter)
{
    uint32_t primask;

    if (counter == 0U)
    {
        return;
    }

    primask = Lora_IrqLock();
    (*counter)++;
    Lora_IrqUnlock(primask);
}

static void Lora_LoadDefaultConfig(void)
{
    memset(&s_mod_params, 0, sizeof(s_mod_params));
    memset(&s_pkt_params, 0, sizeof(s_pkt_params));
    memset(&s_pkt_status, 0, sizeof(s_pkt_status));

    s_mod_params.PacketType = PACKET_TYPE_LORA;
    s_mod_params.Params.LoRa.SpreadingFactor = LORA_CFG_SF;
    s_mod_params.Params.LoRa.Bandwidth = LORA_CFG_BW;
    s_mod_params.Params.LoRa.CodingRate = LORA_CFG_CR;

    s_pkt_params.PacketType = PACKET_TYPE_LORA;
    s_pkt_params.Params.LoRa.PreambleLength = LORA_CFG_PREAMBLE_LEN;
    s_pkt_params.Params.LoRa.HeaderType = LORA_CFG_HEADER_TYPE;
    s_pkt_params.Params.LoRa.CrcMode = LORA_CFG_CRC_MODE;
    s_pkt_params.Params.LoRa.InvertIQ = LORA_CFG_IQ_MODE;
    s_pkt_params.Params.LoRa.PayloadLength = LORA_MAX_PAYLOAD_LEN;
}

static void Lora_ClearRuntimeState(void)
{
    s_tx_done_flag = 0U;
    s_rx_done_flag = 0U;
    s_tx_timeout_flag = 0U;
    s_rx_timeout_flag = 0U;
    s_rx_error_flag = 0U;
    s_rx_error_code = IRQ_HEADER_ERROR_CODE;

    s_tx_busy = 0U;
    s_radio_in_rx = 0U;

    s_tx_head = 0U;
    s_tx_tail = 0U;
    s_tx_count = 0U;

    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_rx_count = 0U;

    memset(&s_stats, 0, sizeof(s_stats));
    Lora_SetRadioState(LORA_RADIO_STATE_NOT_INIT);
}

static uint8_t Lora_TxQueuePush(const uint8_t *data, uint8_t len)
{
    uint32_t primask;

    if ((data == 0U) || (len == 0U) || (len > LORA_MAX_PAYLOAD_LEN))
    {
        return 0U;
    }

    primask = Lora_IrqLock();

    if (s_tx_count >= LORA_TX_QUEUE_DEPTH)
    {
        Lora_IrqUnlock(primask);
        return 0U;
    }

    s_tx_queue[s_tx_head].len = len;
    memcpy(s_tx_queue[s_tx_head].data, data, len);

    s_tx_head++;
    if (s_tx_head >= LORA_TX_QUEUE_DEPTH)
    {
        s_tx_head = 0U;
    }

    s_tx_count++;
    Lora_IrqUnlock(primask);
    return 1U;
}

static uint8_t Lora_TxQueuePop(LoraTxPacket *pkt)
{
    uint32_t primask;

    if (pkt == 0U)
    {
        return 0U;
    }

    primask = Lora_IrqLock();

    if (s_tx_count == 0U)
    {
        Lora_IrqUnlock(primask);
        return 0U;
    }

    *pkt = s_tx_queue[s_tx_tail];

    s_tx_tail++;
    if (s_tx_tail >= LORA_TX_QUEUE_DEPTH)
    {
        s_tx_tail = 0U;
    }

    s_tx_count--;
    Lora_IrqUnlock(primask);
    return 1U;
}

static uint8_t Lora_RxQueuePush(const uint8_t *data, uint8_t len, int8_t rssi, int8_t snr)
{
    uint32_t primask;

    if ((data == 0U) || (len == 0U) || (len > LORA_MAX_PAYLOAD_LEN))
    {
        return 0U;
    }

    primask = Lora_IrqLock();

    if (s_rx_count >= LORA_RX_QUEUE_DEPTH)
    {
        Lora_IrqUnlock(primask);
        return 0U;
    }

    s_rx_queue[s_rx_head].len = len;
    s_rx_queue[s_rx_head].rssi = rssi;
    s_rx_queue[s_rx_head].snr = snr;
    memcpy(s_rx_queue[s_rx_head].data, data, len);

    s_rx_head++;
    if (s_rx_head >= LORA_RX_QUEUE_DEPTH)
    {
        s_rx_head = 0U;
    }

    s_rx_count++;
    Lora_IrqUnlock(primask);
    return 1U;
}

static uint8_t Lora_RxQueuePop(LoraRxPacket *pkt)
{
    uint32_t primask;

    if (pkt == 0U)
    {
        return 0U;
    }

    primask = Lora_IrqLock();

    if (s_rx_count == 0U)
    {
        Lora_IrqUnlock(primask);
        return 0U;
    }

    *pkt = s_rx_queue[s_rx_tail];

    s_rx_tail++;
    if (s_rx_tail >= LORA_RX_QUEUE_DEPTH)
    {
        s_rx_tail = 0U;
    }

    s_rx_count--;
    Lora_IrqUnlock(primask);
    return 1U;
}

static void Lora_TryStartNextTx(void)
{
    LoraTxPacket pkt;

    if ((s_inited == 0U) || (s_tx_busy != 0U))
    {
        return;
    }

    if (Lora_TxQueuePop(&pkt) == 0U)
    {
        return;
    }

    s_pkt_params.Params.LoRa.PayloadLength = pkt.len;
    Radio.SetPacketParams(&s_pkt_params);
    Radio.SetDioIrqParams(LORA_TX_IRQ_MASK, LORA_TX_IRQ_MASK, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
    Radio.SendPayload(pkt.data, pkt.len, (TickTime_t){ LORA_TX_TIMEOUT_STEP, LORA_TX_TIMEOUT_COUNT });

    s_tx_busy = 1U;
    s_radio_in_rx = 0U;
    Lora_SetRadioState(LORA_RADIO_STATE_TX);
}

void Lora_Init(void)
{
    BspSx1281_PortInit();
    Lora_LoadDefaultConfig();
    Lora_ClearRuntimeState();

    Radio.Init(&s_radio_callbacks);
    Radio.SetRegulatorMode(USE_LDO);
    Radio.SetStandby(STDBY_RC);
    Radio.SetPacketType(PACKET_TYPE_LORA);
    Radio.SetModulationParams(&s_mod_params);
    Radio.SetPacketParams(&s_pkt_params);
    Radio.SetRfFrequency(LORA_RF_FREQUENCY_HZ);
    Radio.SetBufferBaseAddresses(0x00, 0x00);
    Radio.SetTxParams(LORA_TX_OUTPUT_POWER_DBM, RADIO_RAMP_02_US);
    Radio.SetPollingMode();

    s_inited = 1U;
    Lora_SetRadioState(LORA_RADIO_STATE_READY);

    Lora_StartRx();
}

void Lora_StartRx(void)
{
    if (s_inited == 0U)
    {
        return;
    }

    s_pkt_params.Params.LoRa.PayloadLength = LORA_MAX_PAYLOAD_LEN;
    Radio.SetPacketParams(&s_pkt_params);
    Radio.SetDioIrqParams(LORA_RX_IRQ_MASK, LORA_RX_IRQ_MASK, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
    Radio.SetRx(RX_TX_CONTINUOUS);

    s_radio_in_rx = 1U;
    if (s_tx_busy != 0U)
    {
        Lora_SetRadioState(LORA_RADIO_STATE_TX);
    }
    else
    {
        Lora_SetRadioState(LORA_RADIO_STATE_RX);
    }
}

LoraTxEnqueueResult Lora_TxEnqueue(const uint8_t *data, uint8_t len)
{
    if (s_inited == 0U)
    {
        return LORA_TX_ENQUEUE_NOT_INIT;
    }

    if ((data == 0U) || (len == 0U) || (len > LORA_MAX_PAYLOAD_LEN))
    {
        return LORA_TX_ENQUEUE_BAD_PARAM;
    }

    if (Lora_TxQueuePush(data, len) == 0U)
    {
        Lora_StatsIncrement(&s_stats.tx_dropped);
        return LORA_TX_ENQUEUE_QUEUE_FULL;
    }

    return LORA_TX_ENQUEUE_OK;
}

void Lora_Process(void)
{
    uint8_t size = 0U;
    int8_t rssi;
    int8_t snr;

    if (s_inited == 0U)
    {
        return;
    }

    SX1280ProcessIrqs();

    if (s_tx_done_flag != 0U)
    {
        s_tx_done_flag = 0U;
        s_tx_busy = 0U;
        Lora_StatsIncrement(&s_stats.tx_ok);
        Lora_SetRadioState(LORA_RADIO_STATE_READY);
    }

    if (s_tx_timeout_flag != 0U)
    {
        s_tx_timeout_flag = 0U;
        s_tx_busy = 0U;
        Lora_StatsIncrement(&s_stats.tx_timeout);
        Lora_SetRadioState(LORA_RADIO_STATE_READY);
    }

    if (s_rx_done_flag != 0U)
    {
        s_rx_done_flag = 0U;

        if (Radio.GetPayload(s_rx_tmp_buf, &size, LORA_MAX_PAYLOAD_LEN) == 0U)
        {
            Radio.GetPacketStatus(&s_pkt_status);
            rssi = s_pkt_status.Params.LoRa.RssiPkt;
            snr = s_pkt_status.Params.LoRa.SnrPkt;

            if (Lora_RxQueuePush(s_rx_tmp_buf, size, rssi, snr) != 0U)
            {
                Lora_StatsIncrement(&s_stats.rx_ok);
            }
            else
            {
                Lora_StatsIncrement(&s_stats.rx_dropped);
            }
        }
        else
        {
            Lora_StatsIncrement(&s_stats.rx_dropped);
        }

        Lora_SetRadioState(LORA_RADIO_STATE_READY);
    }

    if (s_rx_timeout_flag != 0U)
    {
        s_rx_timeout_flag = 0U;
        Lora_StatsIncrement(&s_stats.rx_timeout);
        Lora_SetRadioState(LORA_RADIO_STATE_READY);
    }

    if (s_rx_error_flag != 0U)
    {
        s_rx_error_flag = 0U;
        Lora_StatsIncrement(&s_stats.rx_error);

        if (s_rx_error_code == IRQ_CRC_ERROR_CODE)
        {
            Lora_StatsIncrement(&s_stats.rx_crc_error);
        }

        Lora_SetRadioState(LORA_RADIO_STATE_READY);
    }

    if (s_tx_busy == 0U)
    {
        Lora_TryStartNextTx();
        if ((s_tx_busy == 0U) && (s_radio_in_rx == 0U))
        {
            Lora_StartRx();
        }
    }
}

LoraRxDequeueResult Lora_RxDequeue(uint8_t *data, uint8_t *len, int8_t *rssi, int8_t *snr)
{
    LoraRxPacket pkt;

    if ((data == 0U) || (len == 0U))
    {
        return LORA_RX_DEQUEUE_BAD_PARAM;
    }

    if (Lora_RxQueuePop(&pkt) == 0U)
    {
        return LORA_RX_DEQUEUE_EMPTY;
    }

    memcpy(data, pkt.data, pkt.len);
    *len = pkt.len;

    if (rssi != 0U)
    {
        *rssi = pkt.rssi;
    }

    if (snr != 0U)
    {
        *snr = pkt.snr;
    }

    return LORA_RX_DEQUEUE_OK;
}

void Lora_GetStats(LoraStats *stats)
{
    uint32_t primask;

    if (stats == 0U)
    {
        return;
    }

    primask = Lora_IrqLock();
    *stats = s_stats;
    Lora_IrqUnlock(primask);
}

LoraBusyState Lora_IsBusy(void)
{
    uint8_t busy;
    uint32_t primask;

    primask = Lora_IrqLock();
    busy = ((s_tx_busy != 0U) || (s_tx_count > 0U)) ? 1U : 0U;
    Lora_IrqUnlock(primask);

    if (busy != 0U)
    {
        return LORA_BUSY_ACTIVE;
    }

    return LORA_BUSY_IDLE;
}

static void Lora_OnTxDone(void)
{
    s_tx_done_flag = 1U;
    s_radio_in_rx = 0U;
}

static void Lora_OnRxDone(void)
{
    s_rx_done_flag = 1U;
    s_radio_in_rx = 0U;
}

static void Lora_OnTxTimeout(void)
{
    s_tx_timeout_flag = 1U;
    s_radio_in_rx = 0U;
}

static void Lora_OnRxTimeout(void)
{
    s_rx_timeout_flag = 1U;
    s_radio_in_rx = 0U;
}

static void Lora_OnRxError(IrqErrorCode_t errCode)
{
    s_rx_error_code = errCode;
    s_rx_error_flag = 1U;
    s_radio_in_rx = 0U;
}
