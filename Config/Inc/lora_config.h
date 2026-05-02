#ifndef __LORA_CONFIG_H
#define __LORA_CONFIG_H

#include <stdint.h>
#include "sx1280.h"

/*
 * E28-2G4M12SX / SX1281 LoRa configuration.
 * ThirdParty/SX1280lib is still used because Semtech's SX1280/SX1281 command set
 * is compatible in this project.
 */
#define LORA_RF_FREQUENCY_HZ            2473000000UL
#define LORA_TX_OUTPUT_POWER_DBM        13
#define LORA_MAX_PAYLOAD_LEN            64U

#define LORA_TX_QUEUE_DEPTH             8U
#define LORA_RX_QUEUE_DEPTH             8U

#define LORA_PROFILE_RANGE_1K           0
#define LORA_PROFILE_2K                 1
#define LORA_PROFILE_4K                 2
#define LORA_PROFILE_6K                 3

/*
 * Experimental default:
 * 1000 m target, 5 Hz AIR_FLIGHT_STATE, 2 dBi + 5 dBi antenna pair.
 * If field test is unstable, change to LORA_PROFILE_2K or RANGE_1K.
 */
#define LORA_LINK_PROFILE               LORA_PROFILE_4K

#if (LORA_LINK_PROFILE == LORA_PROFILE_RANGE_1K)
#define LORA_CFG_SF                     LORA_SF11
#define LORA_CFG_BW                     LORA_BW_0200
#define LORA_CFG_CR                     LORA_CR_4_5
#define LORA_TX_TIMEOUT_COUNT           3000U

#elif (LORA_LINK_PROFILE == LORA_PROFILE_2K)
#define LORA_CFG_SF                     LORA_SF10
#define LORA_CFG_BW                     LORA_BW_0200
#define LORA_CFG_CR                     LORA_CR_4_5
#define LORA_TX_TIMEOUT_COUNT           2000U

#elif (LORA_LINK_PROFILE == LORA_PROFILE_4K)
#define LORA_CFG_SF                     LORA_SF9
#define LORA_CFG_BW                     LORA_BW_0200
#define LORA_CFG_CR                     LORA_CR_4_5
#define LORA_TX_TIMEOUT_COUNT           1500U

#elif (LORA_LINK_PROFILE == LORA_PROFILE_6K)
#define LORA_CFG_SF                     LORA_SF8
#define LORA_CFG_BW                     LORA_BW_0200
#define LORA_CFG_CR                     LORA_CR_4_5
#define LORA_TX_TIMEOUT_COUNT           1000U

#else
#error "Invalid LORA_LINK_PROFILE"
#endif

#define LORA_CFG_PREAMBLE_LEN           16U
#define LORA_CFG_HEADER_TYPE            LORA_PACKET_VARIABLE_LENGTH
#define LORA_CFG_CRC_MODE               LORA_CRC_ON
#define LORA_CFG_IQ_MODE                LORA_IQ_NORMAL

#define LORA_RX_IRQ_MASK                ( IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR )
#define LORA_TX_IRQ_MASK                ( IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT )

#define LORA_TX_TIMEOUT_STEP            RADIO_TICK_SIZE_1000_US

#endif
