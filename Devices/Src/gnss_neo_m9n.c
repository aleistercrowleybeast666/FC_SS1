#include "gnss_neo_m9n.h"

#include "cmsis_gcc.h"
#include "common_ringbuf.h"
#include "sensor_config.h"
#include "usart.h"
#include <string.h>

#define GNSS_UBX_SYNC1              0xB5U
#define GNSS_UBX_SYNC2              0x62U
#define GNSS_UBX_NAV_CLASS          0x01U
#define GNSS_UBX_NAV_PVT_ID         0x07U
#define GNSS_UBX_NAV_PVT_LEN        92U
#define GNSS_UBX_MAX_PAYLOAD_LEN    GNSS_UBX_NAV_PVT_LEN
#define GNSS_UBX_CFG_CLASS          0x06U
#define GNSS_UBX_CFG_VALSET_ID      0x8AU
#define GNSS_UBX_TX_MAX_PAYLOAD_LEN 128U
#define GNSS_UBX_TX_FRAME_OVERHEAD  8U
#define GNSS_VALSET_HEADER_LEN      4U
#define GNSS_VALSET_TRANSACTION_NONE 0x00U
#define GNSS_CFG_ITEM_TYPE_L        1U
#define GNSS_CFG_ITEM_TYPE_U1       1U
#define GNSS_CFG_ITEM_TYPE_U2       2U
#define GNSS_CFG_ITEM_TYPE_U4       4U
#define GNSS_CFG_ITEM_TYPE_E1       1U
#define GNSS_UART_BAUDRATE_MIN      4800U
#define GNSS_CFG_UART1INPROT_UBX    0x10730001UL
#define GNSS_CFG_UART1INPROT_NMEA   0x10730002UL
#define GNSS_CFG_UART1INPROT_RTCM3X 0x10730004UL
#define GNSS_CFG_UART1OUTPROT_UBX   0x10740001UL
#define GNSS_CFG_UART1OUTPROT_NMEA  0x10740002UL
#define GNSS_CFG_UART1_BAUDRATE     0x40520001UL
#define GNSS_CFG_RATE_MEAS          0x30210001UL
#define GNSS_CFG_RATE_NAV           0x30210002UL
#define GNSS_CFG_RATE_TIMEREF       0x20210003UL
#define GNSS_CFG_MSGOUT_NAV_PVT_UART1 0x20910007UL
#define GNSS_CFG_NAVSPG_DYNMODEL    0x20110021UL
#define GNSS_CFG_SIGNAL_GPS_ENA     0x1031001FUL
#define GNSS_CFG_SIGNAL_GPS_L1CA_ENA 0x10310001UL
#define GNSS_CFG_SIGNAL_SBAS_ENA    0x10310020UL
#define GNSS_CFG_SIGNAL_SBAS_L1CA_ENA 0x10310005UL
#define GNSS_CFG_SIGNAL_GAL_ENA     0x10310021UL
#define GNSS_CFG_SIGNAL_GAL_E1_ENA  0x10310007UL
#define GNSS_CFG_SIGNAL_BDS_ENA     0x10310022UL
#define GNSS_CFG_SIGNAL_BDS_B1_ENA  0x1031000DUL
#define GNSS_CFG_SIGNAL_QZSS_ENA    0x10310024UL
#define GNSS_CFG_SIGNAL_QZSS_L1CA_ENA 0x10310012UL
#define GNSS_CFG_SIGNAL_GLO_ENA     0x10310025UL
#define GNSS_CFG_SIGNAL_GLO_L1_ENA  0x10310018UL

typedef enum
{
    GnssUbxStateSync1 = 0,
    GnssUbxStateSync2,
    GnssUbxStateClass,
    GnssUbxStateId,
    GnssUbxStateLen1,
    GnssUbxStateLen2,
    GnssUbxStatePayload,
    GnssUbxStateCkA,
    GnssUbxStateCkB
} GnssUbxParseState;

typedef struct
{
    GnssUbxParseState state;
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t payload_len;
    uint16_t payload_idx;
    uint8_t ck_a;
    uint8_t ck_b;
    uint8_t payload[GNSS_UBX_MAX_PAYLOAD_LEN];
} GnssUbxParser_t;

typedef struct
{
    uint32_t key;
    uint32_t value;
    uint8_t value_len;
} GnssCfgItem_t;

static uint8_t s_rx_dma_buf[GNSS_RX_DMA_BUF_SIZE];
static uint8_t s_rx_ringbuf_mem[GNSS_RX_RINGBUF_SIZE];
static ringbuf_t s_rx_rb;
static GnssUbxParser_t s_parser;
static SensorsGnssData_t s_data;
static uint8_t s_initialized = 0U;

static uint32_t Gnss_IrqLock(void);
static void Gnss_IrqUnlock(uint32_t primask);
static HAL_StatusTypeDef Gnss_StartRxDmaToIdle(void);
static void Gnss_ParserReset(void);
static uint8_t Gnss_RingPopByte(uint8_t *byte);
static void Gnss_ParseByte(uint8_t byte, uint32_t now_ms);
static void Gnss_ChecksumAdd(uint8_t byte);
static void Gnss_ParseNavPvt(uint32_t now_ms);
static uint32_t Gnss_ReadU32Le(const uint8_t *data);
static int32_t Gnss_ReadI32Le(const uint8_t *data);
static void Gnss_UpdateStatus(uint32_t now_ms);
static void Gnss_WriteU16Le(uint8_t *data, uint16_t value);
static void Gnss_WriteU32Le(uint8_t *data, uint32_t value);
static uint8_t Gnss_IsDynModelValid(uint8_t dyn_model);
static int GnssNeoM9n_SendValset(uint8_t layers, const GnssCfgItem_t *items, uint8_t count);

static uint32_t Gnss_IrqLock(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void Gnss_IrqUnlock(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static HAL_StatusTypeDef Gnss_StartRxDmaToIdle(void)
{
    HAL_StatusTypeDef status;

    status = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, s_rx_dma_buf, GNSS_RX_DMA_BUF_SIZE);

    if (huart2.hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    }

    return status;
}

static void Gnss_ParserReset(void)
{
    memset(&s_parser, 0, sizeof(s_parser));
    s_parser.state = GnssUbxStateSync1;
}

static uint8_t Gnss_RingPopByte(uint8_t *byte)
{
    uint8_t ok = 0U;
    uint32_t primask;

    primask = Gnss_IrqLock();
    if (RingBuf_Pop(&s_rx_rb, byte, 1U) == 1U)
    {
        ok = 1U;
    }
    Gnss_IrqUnlock(primask);

    return ok;
}

static void Gnss_ChecksumAdd(uint8_t byte)
{
    s_parser.ck_a = (uint8_t)(s_parser.ck_a + byte);
    s_parser.ck_b = (uint8_t)(s_parser.ck_b + s_parser.ck_a);
}

static uint32_t Gnss_ReadU32Le(const uint8_t *data)
{
    return ((uint32_t)data[0]) |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static int32_t Gnss_ReadI32Le(const uint8_t *data)
{
    return (int32_t)Gnss_ReadU32Le(data);
}

static void Gnss_WriteU16Le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static void Gnss_WriteU32Le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8) & 0xFFU);
    data[2] = (uint8_t)((value >> 16) & 0xFFU);
    data[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static uint8_t Gnss_IsDynModelValid(uint8_t dyn_model)
{
    return ((dyn_model == GNSS_DYNMODEL_PORTABLE) ||
            (dyn_model == GNSS_DYNMODEL_STATIONARY) ||
            (dyn_model == GNSS_DYNMODEL_PEDESTRIAN) ||
            (dyn_model == GNSS_DYNMODEL_AUTOMOTIVE) ||
            (dyn_model == GNSS_DYNMODEL_SEA) ||
            (dyn_model == GNSS_DYNMODEL_AIRBORNE_1G) ||
            (dyn_model == GNSS_DYNMODEL_AIRBORNE_2G) ||
            (dyn_model == GNSS_DYNMODEL_AIRBORNE_4G)) ? 1U : 0U;
}

static int GnssNeoM9n_SendValset(uint8_t layers, const GnssCfgItem_t *items, uint8_t count)
{
    uint8_t payload[GNSS_UBX_TX_MAX_PAYLOAD_LEN];
    uint16_t payload_len = GNSS_VALSET_HEADER_LEN;
    uint8_t i;

    if ((items == NULL) ||
        (count == 0U) ||
        ((layers & GNSS_CFG_LAYER_ALL) == 0U) ||
        ((layers & (uint8_t)(~GNSS_CFG_LAYER_ALL)) != 0U))
    {
        return -1;
    }

    payload[0] = 0x00U;
    payload[1] = layers;
    payload[2] = GNSS_VALSET_TRANSACTION_NONE;
    payload[3] = 0x00U;

    for (i = 0U; i < count; i++)
    {
        if ((items[i].value_len != GNSS_CFG_ITEM_TYPE_L) &&
            (items[i].value_len != GNSS_CFG_ITEM_TYPE_U2) &&
            (items[i].value_len != GNSS_CFG_ITEM_TYPE_U4))
        {
            return -1;
        }

        if ((uint16_t)(payload_len + 4U + items[i].value_len) > GNSS_UBX_TX_MAX_PAYLOAD_LEN)
        {
            return -1;
        }

        Gnss_WriteU32Le(&payload[payload_len], items[i].key);
        payload_len = (uint16_t)(payload_len + 4U);

        if (items[i].value_len == GNSS_CFG_ITEM_TYPE_U4)
        {
            Gnss_WriteU32Le(&payload[payload_len], items[i].value);
        }
        else if (items[i].value_len == GNSS_CFG_ITEM_TYPE_U2)
        {
            Gnss_WriteU16Le(&payload[payload_len], (uint16_t)items[i].value);
        }
        else
        {
            payload[payload_len] = (uint8_t)items[i].value;
        }

        payload_len = (uint16_t)(payload_len + items[i].value_len);
    }

    return GnssNeoM9n_SendUbx(GNSS_UBX_CFG_CLASS, GNSS_UBX_CFG_VALSET_ID, payload, payload_len);
}

static void Gnss_ParseNavPvt(uint32_t now_ms)
{
    const uint8_t *p = s_parser.payload;

    s_data.iTOW = Gnss_ReadU32Le(&p[0]);
    s_data.fixType = p[20];
    s_data.gnssFixOK = (uint8_t)(p[21] & 0x01U);
    s_data.numSV = p[23];
    s_data.lon = Gnss_ReadI32Le(&p[24]);
    s_data.lat = Gnss_ReadI32Le(&p[28]);
    s_data.height = Gnss_ReadI32Le(&p[32]);
    s_data.hMSL = Gnss_ReadI32Le(&p[36]);
    s_data.hAcc = Gnss_ReadU32Le(&p[40]);
    s_data.vAcc = Gnss_ReadU32Le(&p[44]);
    s_data.velN = Gnss_ReadI32Le(&p[48]);
    s_data.velE = Gnss_ReadI32Le(&p[52]);
    s_data.velD = Gnss_ReadI32Le(&p[56]);
    s_data.gSpeed = Gnss_ReadI32Le(&p[60]);
    s_data.headMot = Gnss_ReadI32Le(&p[64]);
    s_data.sAcc = Gnss_ReadU32Le(&p[68]);
    s_data.headAcc = Gnss_ReadU32Le(&p[72]);
    s_data.lastUpdate_ms = now_ms;

    Gnss_UpdateStatus(now_ms);
}

static void Gnss_UpdateStatus(uint32_t now_ms)
{
    uint32_t age_ms = now_ms - s_data.lastUpdate_ms;

    s_data.online = ((s_data.lastUpdate_ms != 0U) && (age_ms <= GNSS_TIMEOUT_MS)) ? 1U : 0U;
    s_data.hasValidFix = ((s_data.online != 0U) &&
                          (s_data.fixType >= 2U) &&
                          (s_data.gnssFixOK != 0U)) ? 1U : 0U;
    s_data.usableForNav = ((s_data.online != 0U) &&
                           (s_data.gnssFixOK != 0U) &&
                           (s_data.fixType == 3U) &&
                           (s_data.numSV >= GNSS_NAV_MIN_SV) &&
                           (s_data.hAcc <= GNSS_NAV_MAX_HACC_MM) &&
                           (s_data.vAcc <= GNSS_NAV_MAX_VACC_MM) &&
                           (age_ms <= GNSS_NAV_MAX_AGE_MS)) ? 1U : 0U;
}

static void Gnss_ParseByte(uint8_t byte, uint32_t now_ms)
{
    switch (s_parser.state)
    {
    case GnssUbxStateSync1:
        if (byte == GNSS_UBX_SYNC1)
        {
            s_parser.state = GnssUbxStateSync2;
        }
        break;

    case GnssUbxStateSync2:
        if (byte == GNSS_UBX_SYNC2)
        {
            s_parser.ck_a = 0U;
            s_parser.ck_b = 0U;
            s_parser.state = GnssUbxStateClass;
        }
        else
        {
            s_parser.state = GnssUbxStateSync1;
        }
        break;

    case GnssUbxStateClass:
        s_parser.msg_class = byte;
        Gnss_ChecksumAdd(byte);
        s_parser.state = GnssUbxStateId;
        break;

    case GnssUbxStateId:
        s_parser.msg_id = byte;
        Gnss_ChecksumAdd(byte);
        s_parser.state = GnssUbxStateLen1;
        break;

    case GnssUbxStateLen1:
        s_parser.payload_len = byte;
        Gnss_ChecksumAdd(byte);
        s_parser.state = GnssUbxStateLen2;
        break;

    case GnssUbxStateLen2:
        s_parser.payload_len |= ((uint16_t)byte << 8);
        Gnss_ChecksumAdd(byte);
        s_parser.payload_idx = 0U;

        if (s_parser.payload_len > GNSS_UBX_MAX_PAYLOAD_LEN)
        {
            Gnss_ParserReset();
        }
        else if (s_parser.payload_len == 0U)
        {
            s_parser.state = GnssUbxStateCkA;
        }
        else
        {
            s_parser.state = GnssUbxStatePayload;
        }
        break;

    case GnssUbxStatePayload:
        s_parser.payload[s_parser.payload_idx] = byte;
        s_parser.payload_idx++;
        Gnss_ChecksumAdd(byte);

        if (s_parser.payload_idx >= s_parser.payload_len)
        {
            s_parser.state = GnssUbxStateCkA;
        }
        break;

    case GnssUbxStateCkA:
        if (byte == s_parser.ck_a)
        {
            s_parser.state = GnssUbxStateCkB;
        }
        else
        {
            Gnss_ParserReset();
        }
        break;

    case GnssUbxStateCkB:
        if ((byte == s_parser.ck_b) &&
            (s_parser.msg_class == GNSS_UBX_NAV_CLASS) &&
            (s_parser.msg_id == GNSS_UBX_NAV_PVT_ID) &&
            (s_parser.payload_len == GNSS_UBX_NAV_PVT_LEN))
        {
            Gnss_ParseNavPvt(now_ms);
        }
        Gnss_ParserReset();
        break;

    default:
        Gnss_ParserReset();
        break;
    }
}

GnssNeoM9nInitResult GnssNeoM9n_Init(void)
{
    uint32_t primask;

    primask = Gnss_IrqLock();
    RingBuf_Init(&s_rx_rb, s_rx_ringbuf_mem, GNSS_RX_RINGBUF_SIZE);
    memset(&s_data, 0, sizeof(s_data));
    Gnss_ParserReset();
    s_initialized = 1U;
    Gnss_IrqUnlock(primask);

    if (Gnss_StartRxDmaToIdle() != HAL_OK)
    {
        s_initialized = 0U;
        return GnssNeoM9n_InitUartError;
    }

    /*
     * ================= NEO-M9N 可选 Flash 固化配置区 =================
     *
     * 默认不要自动解除注释。
     *
     * 当前中国地区测试推荐最终配置：
     * - 921600 baud
     * - UBX only 输出
     * - UBX-NAV-PVT 每个 navigation epoch 输出一次
     * - 25Hz 导航更新率
     * - Airborne <4g 动态模型
     * - GPS + BeiDou + Galileo
     * - 关闭 GLONASS / QZSS / SBAS
     *
     * 使用方法：
     * 1. 先确认 CubeMX USART2 波特率与当前 NEO-M9N 波特率一致。
     *    默认 NEO-M9N 通常是 38400，因此第一次固化配置时 CubeMX USART2 应保持 38400。
     *
     * 2. 解除下面这一行注释，烧录运行一次：
     *    GnssNeoM9n_ConfigPreset(GNSS_CFG_LAYER_RAM_FLASH,
     *                            GNSS_TARGET_BAUDRATE,
     *                            GNSS_TARGET_RATE_HZ);
     *
     * 3. 该函数会把 UBX 输出、NAV-PVT、25Hz、Airborne <4g、
     *    GPS+BDS+Galileo、921600 baud 写入 RAM+Flash。
     *
     * 4. 执行到波特率配置后，如果当前 STM32 USART2 仍是 38400，
     *    通信中断是正常现象。
     *
     * 5. 随后请重新注释掉该配置调用，把 CubeMX USART2 改为 921600，
     *    重新生成并烧录。
     *
     * 6. 正常飞行固件中，本配置调用必须保持注释；
     *    不要每次上电都写 Flash。
     *
     * 7. 如果后续需要修改 GNSS 配置，再按同样流程临时解除注释运行一次。
     */
    /*
     * GnssNeoM9n_ConfigPreset(GNSS_CFG_LAYER_RAM_FLASH,
     *                         GNSS_TARGET_BAUDRATE,
     *                         GNSS_TARGET_RATE_HZ);
     */

    /*
     * 正常飞行固件：
     * - 上述配置函数保持注释
     * - 只接收 UBX-NAV-PVT
     * - 使用 Sensors_GnssIsOnline / Sensors_GnssHasValidFix / Sensors_GnssIsUsableForNav 判断状态
     * ======================================================
     */
    return GnssNeoM9n_InitOk;
}

GnssNeoM9nUpdateResult GnssNeoM9n_Process(uint32_t now_ms)
{
    uint8_t byte;
    uint8_t processed = 0U;

    if (s_initialized == 0U)
    {
        return GnssNeoM9n_UpdateNoData;
    }

    while (Gnss_RingPopByte(&byte) != 0U)
    {
        Gnss_ParseByte(byte, now_ms);
        processed = 1U;
    }

    Gnss_UpdateStatus(now_ms);

    return (processed != 0U) ? GnssNeoM9n_UpdateOk : GnssNeoM9n_UpdateNoData;
}

uint8_t GnssNeoM9n_GetData(SensorsGnssData_t *out)
{
    if (out == NULL)
    {
        return 0U;
    }

    *out = s_data;
    return s_data.online;
}

int GnssNeoM9n_SendUbx(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len)
{
    uint8_t frame[GNSS_UBX_TX_MAX_PAYLOAD_LEN + GNSS_UBX_TX_FRAME_OVERHEAD];
    uint8_t ck_a = 0U;
    uint8_t ck_b = 0U;
    uint16_t frame_len;
    uint16_t i;

    if (((payload == NULL) && (len != 0U)) || (len > GNSS_UBX_TX_MAX_PAYLOAD_LEN))
    {
        return -1;
    }

    frame[0] = GNSS_UBX_SYNC1;
    frame[1] = GNSS_UBX_SYNC2;
    frame[2] = cls;
    frame[3] = id;
    Gnss_WriteU16Le(&frame[4], len);

    if (len != 0U)
    {
        memcpy(&frame[6], payload, len);
    }

    for (i = 2U; i < (uint16_t)(6U + len); i++)
    {
        ck_a = (uint8_t)(ck_a + frame[i]);
        ck_b = (uint8_t)(ck_b + ck_a);
    }

    frame[6U + len] = ck_a;
    frame[7U + len] = ck_b;
    frame_len = (uint16_t)(len + GNSS_UBX_TX_FRAME_OVERHEAD);

    return (HAL_UART_Transmit(&huart2, frame, frame_len, GNSS_CFG_TIMEOUT_MS) == HAL_OK) ? 0 : -1;
}

/*
 * 配置发送为同步阻塞接口，仅用于初始化阶段或手动调试阶段。
 * 不要在 UART/DMA/IDLE 回调中调用，也不要在正常飞行固件里每次上电写 Flash。
 */
int GnssNeoM9n_ConfigOutputUbxOnly(uint8_t layers)
{
    const GnssCfgItem_t items[] =
    {
        {GNSS_CFG_UART1INPROT_UBX, 1U, GNSS_CFG_ITEM_TYPE_L},
        {GNSS_CFG_UART1INPROT_NMEA, 1U, GNSS_CFG_ITEM_TYPE_L},
        {GNSS_CFG_UART1INPROT_RTCM3X, 1U, GNSS_CFG_ITEM_TYPE_L},
        {GNSS_CFG_UART1OUTPROT_UBX, 1U, GNSS_CFG_ITEM_TYPE_L},
        {GNSS_CFG_UART1OUTPROT_NMEA, 0U, GNSS_CFG_ITEM_TYPE_L}
    };

    /*
     * 这里的 UART1 是 u-blox 模块内部 UART1，不是 STM32 USART1。
     * 保留输入 NMEA/RTCM3X 是为了后续调试或差分输入。
     * 输出只开 UBX 是为了降低串口负载并简化 STM32 解析。
     * 如果写入 Flash，以后上电默认就不再输出 NMEA。
     */
    return GnssNeoM9n_SendValset(layers, items, (uint8_t)(sizeof(items) / sizeof(items[0])));
}

int GnssNeoM9n_ConfigNavPvtOutput(uint8_t layers, uint8_t rate)
{
    const GnssCfgItem_t items[] =
    {
        {GNSS_CFG_MSGOUT_NAV_PVT_UART1, rate, GNSS_CFG_ITEM_TYPE_U1}
    };

    /*
     * rate = 1 表示每个 navigation epoch 输出一次 UBX-NAV-PVT，rate 不是 Hz。
     * 如果导航频率是 25Hz 且 rate=1，则 PVT 输出就是 25Hz。
     * rate = 0 表示关闭 UBX-NAV-PVT 输出。
     */
    return GnssNeoM9n_SendValset(layers, items, (uint8_t)(sizeof(items) / sizeof(items[0])));
}

int GnssNeoM9n_ConfigNavRate(uint8_t layers, uint8_t hz)
{
    uint16_t meas_ms;
    const GnssCfgItem_t items_template[] =
    {
        {GNSS_CFG_RATE_MEAS, 0U, GNSS_CFG_ITEM_TYPE_U2},
        {GNSS_CFG_RATE_NAV, 1U, GNSS_CFG_ITEM_TYPE_U2},
        {GNSS_CFG_RATE_TIMEREF, 1U, GNSS_CFG_ITEM_TYPE_E1}
    };
    GnssCfgItem_t items[sizeof(items_template) / sizeof(items_template[0])];

    if ((hz == 0U) || (hz > GNSS_MAX_RATE_HZ) || ((1000U % hz) != 0U))
    {
        return -1;
    }

    meas_ms = (uint16_t)(1000U / hz);
    if (meas_ms < 40U)
    {
        return -1;
    }

    memcpy(items, items_template, sizeof(items));
    items[0].value = meas_ms;

    /*
     * 25Hz 是 NEO-M9N PVT 最大导航更新率，meas_ms = 40。
     * 25Hz 会增加串口数据量和解析频率，但本项目认为数据连续性优先。
     * 最终配置推荐配合 921600 baud 使用，并确保 ring buffer 留有余量。
     */
    return GnssNeoM9n_SendValset(layers, items, (uint8_t)(sizeof(items) / sizeof(items[0])));
}

int GnssNeoM9n_ConfigUartBaudrate(uint8_t layers, uint32_t baudrate)
{
    const GnssCfgItem_t items[] =
    {
        {GNSS_CFG_UART1_BAUDRATE, baudrate, GNSS_CFG_ITEM_TYPE_U4}
    };

    if ((baudrate < GNSS_UART_BAUDRATE_MIN) || (baudrate > GNSS_MAX_BAUDRATE))
    {
        return -1;
    }

    /*
     * 发送该配置后，NEO-M9N UART 会切换到新波特率，本函数不会修改 STM32 USART2。
     * 如果目标波特率与当前 CubeMX USART2 不一致，当前通信会中断，这是正常现象。
     * 用户需要手动修改 CubeMX USART2 波特率并重新烧录。
     * 波特率适合最终写入 Flash，否则模块断电后回到默认 38400，调试会很不方便。
     */
    return GnssNeoM9n_SendValset(layers, items, (uint8_t)(sizeof(items) / sizeof(items[0])));
}

int GnssNeoM9n_ConfigDynamicModel(uint8_t layers, uint8_t dyn_model)
{
    const GnssCfgItem_t items[] =
    {
        {GNSS_CFG_NAVSPG_DYNMODEL, dyn_model, GNSS_CFG_ITEM_TYPE_E1}
    };

    if (Gnss_IsDynModelValid(dyn_model) == 0U)
    {
        return -1;
    }

    /*
     * Airborne <4g 适合高动态场景，会让 GNSS 内部运动限制更宽松，
     * 更不容易因为高速/高动态导致解无效。
     * 代价是低动态或静止时报告的位置标准差可能更大。
     * 本项目认为“数据中断比精度稍差更不可接受”，因此默认建议 Airborne <4g。
     * 本函数只改动态模型，不修改 minCNO、minElev、pAcc、PDOP 等滤波门限。
     */
    return GnssNeoM9n_SendValset(layers, items, (uint8_t)(sizeof(items) / sizeof(items[0])));
}

int GnssNeoM9n_ConfigSignalsGpsBdsGal(uint8_t layers)
{
    int result;
    const GnssCfgItem_t items[] =
    {
        {GNSS_CFG_SIGNAL_GPS_ENA, 1U, GNSS_CFG_ITEM_TYPE_L},
        {GNSS_CFG_SIGNAL_GPS_L1CA_ENA, 1U, GNSS_CFG_ITEM_TYPE_L},
        {GNSS_CFG_SIGNAL_BDS_ENA, 1U, GNSS_CFG_ITEM_TYPE_L},
        {GNSS_CFG_SIGNAL_BDS_B1_ENA, 1U, GNSS_CFG_ITEM_TYPE_L},
        {GNSS_CFG_SIGNAL_GAL_ENA, 1U, GNSS_CFG_ITEM_TYPE_L},
        {GNSS_CFG_SIGNAL_GAL_E1_ENA, 1U, GNSS_CFG_ITEM_TYPE_L},
        {GNSS_CFG_SIGNAL_GLO_ENA, 0U, GNSS_CFG_ITEM_TYPE_L},
        {GNSS_CFG_SIGNAL_GLO_L1_ENA, 0U, GNSS_CFG_ITEM_TYPE_L},
        {GNSS_CFG_SIGNAL_QZSS_ENA, 0U, GNSS_CFG_ITEM_TYPE_L},
        {GNSS_CFG_SIGNAL_QZSS_L1CA_ENA, 0U, GNSS_CFG_ITEM_TYPE_L},
        {GNSS_CFG_SIGNAL_SBAS_ENA, 0U, GNSS_CFG_ITEM_TYPE_L},
        {GNSS_CFG_SIGNAL_SBAS_L1CA_ENA, 0U, GNSS_CFG_ITEM_TYPE_L}
    };

    /*
     * 中国地区测试配置：GPS + BeiDou + Galileo。
     * u-blox 这里不是“优先级配置”，而是“启用哪些星座/信号”。
     * BeiDou 在中国地区是重要主力星座，GPS 作为基础星座保留，
     * Galileo 用于增加可用卫星数和连续性。
     * GLONASS、QZSS、SBAS 默认关闭以减少变量。
     * 如果后续实测 numSV 不足或掉 fix，再考虑新增函数启用 GLONASS。
     * 修改 CFG-SIGNAL 组会触发 GNSS 子系统复位；调用后应等待 ACK，
     * 并至少延时 GNSS_SIGNAL_RESET_WAIT_MS，再继续发送其他 GNSS 配置命令。
     * 本函数不配置更严格的滤波门限，避免因为门限过严导致数据中断。
     */
    result = GnssNeoM9n_SendValset(layers, items, (uint8_t)(sizeof(items) / sizeof(items[0])));
    if (result == 0)
    {
        HAL_Delay(GNSS_SIGNAL_RESET_WAIT_MS);
    }

    return result;
}

int GnssNeoM9n_ConfigPreset(uint8_t layers, uint32_t baudrate, uint8_t nav_rate_hz)
{
    /*
     * 推荐顺序：
     * 1. 输出协议改为 UBX only。
     * 2. UBX-NAV-PVT 每个 navigation epoch 输出一次。
     * 3. 配置导航频率。
     * 4. 配置 Airborne <4g 动态模型。
     * 5. 配置中国地区 GPS+BDS+Galileo 星座策略。
     * 6. 最后配置波特率，因为执行后当前串口通信可能中断。
     *
     * layers = GNSS_CFG_LAYER_RAM_FLASH 时当前立即生效并写入 Flash，掉电保持。
     * layers = GNSS_CFG_LAYER_RAM 时只在当前运行中生效，断电丢失。
     * 最终固化配置建议只执行一次，不建议每次上电都写 Flash。
     */
    if (GnssNeoM9n_ConfigOutputUbxOnly(layers) != 0)
    {
        return -1;
    }

    if (GnssNeoM9n_ConfigNavPvtOutput(layers, 1U) != 0)
    {
        return -1;
    }

    if (GnssNeoM9n_ConfigNavRate(layers, nav_rate_hz) != 0)
    {
        return -1;
    }

    if (GnssNeoM9n_ConfigDynamicModel(layers, GNSS_TARGET_DYNMODEL) != 0)
    {
        return -1;
    }

    if (GnssNeoM9n_ConfigSignalsGpsBdsGal(layers) != 0)
    {
        return -1;
    }

    return GnssNeoM9n_ConfigUartBaudrate(layers, baudrate);
}

void GnssNeoM9n_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    uint32_t primask;

    if (huart != &huart2)
    {
        return;
    }

    if ((s_initialized != 0U) && (size > 0U))
    {
        if (size > GNSS_RX_DMA_BUF_SIZE)
        {
            size = GNSS_RX_DMA_BUF_SIZE;
        }

        primask = Gnss_IrqLock();
        (void)RingBuf_Push(&s_rx_rb, s_rx_dma_buf, size);
        Gnss_IrqUnlock(primask);
    }

    Gnss_StartRxDmaToIdle();
}

void GnssNeoM9n_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart2)
    {
        return;
    }

    (void)HAL_UART_AbortReceive(huart);
    Gnss_StartRxDmaToIdle();
}
