#include "lora_sx1280.h"

#include <string.h>
#include <stdio.h>

#include "sx1280.h"
#include "radio.h"
#include "debug_log.h"
#include "bsp_sx1280_port.h"
#include "lora_config.h"

static volatile uint8_t s_tx_done_flag = 0U;
static volatile uint8_t s_rx_done_flag = 0U;
static volatile uint8_t s_tx_timeout_flag = 0U;
static volatile uint8_t s_rx_timeout_flag = 0U;
static volatile uint8_t s_rx_error_flag = 0U;
static volatile IrqErrorCode_t s_rx_error_code = IRQ_HEADER_ERROR_CODE;

static uint8_t s_tx_busy = 0U;
static uint8_t s_rx_ready = 0U;
static uint8_t s_inited = 0U;

static uint8_t s_rx_buf[LORA_MAX_PAYLOAD_LEN];
static uint8_t s_rx_len = 0U;
static PacketStatus_t s_pkt_status;

static ModulationParams_t s_mod_params;
static PacketParams_t s_pkt_params;


static void Lora_OnTxDone(void);
static void Lora_OnRxDone(void);
static void Lora_OnTxTimeout(void);
static void Lora_OnRxTimeout(void);
static void Lora_OnRxError(IrqErrorCode_t errCode);

static RadioCallbacks_t s_radio_callbacks =
{
    Lora_OnTxDone,     /* txDone */
    Lora_OnRxDone,     /* rxDone */
    0,                 /* rxSyncWordDone */
    0,                 /* rxHeaderDone */
    Lora_OnTxTimeout,  /* txTimeout */
    Lora_OnRxTimeout,  /* rxTimeout */
    Lora_OnRxError,    /* rxError */
    0,                 /* rangingDone */
    0                  /* cadDone */
};

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
}

void Lora_Init(void)
{
    BspSx1280_PortInit();
    Lora_LoadDefaultConfig();

    Radio.Init(&s_radio_callbacks);
    Radio.SetRegulatorMode(USE_LDO);
    Radio.SetStandby(STDBY_RC);
    Radio.SetPacketType(PACKET_TYPE_LORA);
    Radio.SetModulationParams(&s_mod_params);
    Radio.SetPacketParams(&s_pkt_params);
    Radio.SetRfFrequency(LORA_RF_FREQUENCY_HZ);
    Radio.SetBufferBaseAddresses(0x00, 0x00);
    Radio.SetTxParams(LORA_TX_OUTPUT_POWER_DBM, RADIO_RAMP_02_US);

    /* 先用 polling 模式：EXTI 只负责置位，主循环里统一处理 IRQ */
    Radio.SetPollingMode();

    s_tx_busy = 0U;
    s_rx_ready = 0U;
    s_rx_len = 0U;
    s_inited = 1U;

    DebugLog_Print("SX1280 FW=0x%04X\r\n", Radio.GetFirmwareVersion());
    DebugLog_Print("LoRa init ok, freq=%lu Hz\r\n", (unsigned long)LORA_RF_FREQUENCY_HZ);

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
    s_tx_busy = 0U;
}

uint8_t Lora_Send(const uint8_t *data, uint8_t len)
{
    if ((s_inited == 0U) || (data == 0) || (len == 0U) || (len > LORA_MAX_PAYLOAD_LEN))
    {
        return 0U;
    }

    if (s_tx_busy != 0U)
    {
        return 0U;
    }

    s_pkt_params.Params.LoRa.PayloadLength = len;
    Radio.SetPacketParams(&s_pkt_params);
    Radio.SetDioIrqParams(LORA_TX_IRQ_MASK, LORA_TX_IRQ_MASK, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
    Radio.SendPayload((uint8_t *)data, len, (TickTime_t){ LORA_TX_TIMEOUT_STEP, LORA_TX_TIMEOUT_COUNT });

    s_tx_busy = 1U;
    return 1U;
}

void Lora_Process(void)
{
    uint8_t size = 0U;

    if (s_inited == 0U)
    {
        return;
    }

    SX1280ProcessIrqs();

    if (s_tx_done_flag != 0U)
    {
        s_tx_done_flag = 0U;
        s_tx_busy = 0U;
        DebugLog_Print("LoRa TX done\r\n");
        Lora_StartRx();
    }

    if (s_tx_timeout_flag != 0U)
    {
        s_tx_timeout_flag = 0U;
        s_tx_busy = 0U;
        DebugLog_Print("LoRa TX timeout\r\n");
        Lora_StartRx();
    }

    if (s_rx_done_flag != 0U)
    {
        s_rx_done_flag = 0U;

        if (Radio.GetPayload(s_rx_buf, &size, LORA_MAX_PAYLOAD_LEN) == 0U)
        {
            s_rx_len = size;
            s_rx_ready = 1U;
            Radio.GetPacketStatus(&s_pkt_status);

            DebugLog_Print("LoRa RX len=%u RSSI=%d SNR=%d\r\n",
                      s_rx_len,
                      s_pkt_status.Params.LoRa.RssiPkt,
                      s_pkt_status.Params.LoRa.SnrPkt);
        }
        else
        {
            DebugLog_Print("LoRa RX oversize\r\n");
        }

        Lora_StartRx();
    }

    if (s_rx_timeout_flag != 0U)
    {
        s_rx_timeout_flag = 0U;
        DebugLog_Print("LoRa RX timeout\r\n");
        Lora_StartRx();
    }

    if (s_rx_error_flag != 0U)
    {
        s_rx_error_flag = 0U;
        DebugLog_Print("LoRa RX error=%d\r\n", (int)s_rx_error_code);
        Lora_StartRx();
    }
}

uint8_t Lora_IsBusy(void)
{
    return s_tx_busy;
}

uint8_t Lora_GetLastPacket(uint8_t *data, uint8_t *len, int8_t *rssi, int8_t *snr)
{
    if ((s_rx_ready == 0U) || (data == 0) || (len == 0))
    {
        return 0U;
    }

    memcpy(data, s_rx_buf, s_rx_len);
    *len = s_rx_len;

    if (rssi != 0)
    {
        *rssi = s_pkt_status.Params.LoRa.RssiPkt;
    }

    if (snr != 0)
    {
        *snr = s_pkt_status.Params.LoRa.SnrPkt;
    }

    s_rx_ready = 0U;
    return 1U;
}

static void Lora_OnTxDone(void)
{
    s_tx_done_flag = 1U;
}

static void Lora_OnRxDone(void)
{
    s_rx_done_flag = 1U;
}

static void Lora_OnTxTimeout(void)
{
    s_tx_timeout_flag = 1U;
}

static void Lora_OnRxTimeout(void)
{
    s_rx_timeout_flag = 1U;
}

static void Lora_OnRxError(IrqErrorCode_t errCode)
{
    s_rx_error_code = errCode;
    s_rx_error_flag = 1U;
}