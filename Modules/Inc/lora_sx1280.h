#ifndef __LORA_SX1280_H
#define __LORA_SX1280_H

#include <stdint.h>
#include "lora_config.h"

void Lora_Init(void);
void Lora_Process(void);
void Lora_StartRx(void);

uint8_t Lora_Send(const uint8_t *data, uint8_t len);
uint8_t Lora_IsBusy(void);

uint8_t Lora_GetLastPacket(uint8_t *data, uint8_t *len, int8_t *rssi, int8_t *snr);

#endif