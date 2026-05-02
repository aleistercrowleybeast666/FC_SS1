#ifndef __GNSS_NEO_M9N_H
#define __GNSS_NEO_M9N_H

#include "main.h"
#include "sensors.h"

typedef enum
{
    GnssNeoM9n_InitOk = 0,
    GnssNeoM9n_InitUartError
} GnssNeoM9nInitResult;

typedef enum
{
    GnssNeoM9n_UpdateOk = 0,
    GnssNeoM9n_UpdateNoData
} GnssNeoM9nUpdateResult;

GnssNeoM9nInitResult GnssNeoM9n_Init(void);
GnssNeoM9nUpdateResult GnssNeoM9n_Process(uint32_t now_ms);
uint8_t GnssNeoM9n_GetData(SensorsGnssData_t *out);
int GnssNeoM9n_SendUbx(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len);
//配置函数，仅初始化用
int GnssNeoM9n_ConfigOutputUbxOnly(uint8_t layers);
int GnssNeoM9n_ConfigNavPvtOutput(uint8_t layers, uint8_t rate);
int GnssNeoM9n_ConfigNavRate(uint8_t layers, uint8_t hz);
int GnssNeoM9n_ConfigUartBaudrate(uint8_t layers, uint32_t baudrate);
int GnssNeoM9n_ConfigDynamicModel(uint8_t layers, uint8_t dyn_model);
int GnssNeoM9n_ConfigSignalsGpsBdsGal(uint8_t layers);
int GnssNeoM9n_ConfigPreset(uint8_t layers, uint32_t baudrate, uint8_t nav_rate_hz);
//回调
void GnssNeoM9n_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);
void GnssNeoM9n_ErrorCallback(UART_HandleTypeDef *huart);

#endif /* __GNSS_NEO_M9N_H */
