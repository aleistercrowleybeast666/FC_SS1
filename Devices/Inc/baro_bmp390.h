#ifndef __BARO_BMP390_H
#define __BARO_BMP390_H

#include "sensors.h"
#include <stdint.h>

typedef enum
{
    BaroBmp390_InitOk = 0,
    BaroBmp390_InitBusError,
    BaroBmp390_InitChipIdError,
    BaroBmp390_InitCalibError
} BaroBmp390InitResult;

typedef enum
{
    BaroBmp390_UpdateOk = 0,
    BaroBmp390_UpdateBusError,
    BaroBmp390_UpdateNotReady
} BaroBmp390UpdateResult;

BaroBmp390InitResult BaroBmp390_Init(void);
BaroBmp390UpdateResult BaroBmp390_Update(uint32_t now_ms, SensorsBaroData_t *out);
uint8_t BaroBmp390_GetData(SensorsBaroData_t *out);

#endif /* __BARO_BMP390_H */
