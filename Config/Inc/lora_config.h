#ifndef LORA_CONFIG_H
#define LORA_CONFIG_H

#include <stdint.h>
#include "sx1280.h"

/* 基本射频参数 */
#define LORA_RF_FREQUENCY_HZ            2473000000UL
#define LORA_TX_OUTPUT_POWER_DBM        12
#define LORA_MAX_PAYLOAD_LEN            64U

/* LoRa 调制参数 */
#define LORA_CFG_SF                     LORA_SF12
#define LORA_CFG_BW                     LORA_BW_1600
#define LORA_CFG_CR                     LORA_CR_LI_4_7

/* 包参数 */
#define LORA_CFG_PREAMBLE_LEN           12U
#define LORA_CFG_HEADER_TYPE            LORA_PACKET_VARIABLE_LENGTH
#define LORA_CFG_CRC_MODE              LORA_CRC_ON
#define LORA_CFG_IQ_MODE               LORA_IQ_NORMAL

/* 中断掩码 */
#define LORA_RX_IRQ_MASK                ( IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR )
#define LORA_TX_IRQ_MASK                ( IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT )

/* 收发超时 */
#define LORA_TX_TIMEOUT_STEP            RADIO_TICK_SIZE_1000_US
#define LORA_TX_TIMEOUT_COUNT           2000U
#define LORA_RX_CONTINUOUS_STEP         RADIO_TICK_SIZE_1000_US
#define LORA_RX_CONTINUOUS_COUNT        0xFFFFU

/* 拉距测试参数 */
#define LORA_TEST_AUTO_TX_ENABLE        1U
#define LORA_TEST_AUTO_TX_PERIOD_MS     1000U

#endif