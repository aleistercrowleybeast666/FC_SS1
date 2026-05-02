#ifndef __MAG_MMC5983MA_H
#define __MAG_MMC5983MA_H

#include "sensors.h"
#include <stdint.h>

typedef enum
{
    MagMmc5983ma_InitOk = 0,
    MagMmc5983ma_InitBusError,
    MagMmc5983ma_InitProductIdError
} MagMmc5983maInitResult;

typedef enum
{
    MagMmc5983ma_UpdateOk = 0,
    MagMmc5983ma_UpdateBusy,
    MagMmc5983ma_UpdateBusError,
    MagMmc5983ma_UpdateTimeout
} MagMmc5983maUpdateResult;

MagMmc5983maInitResult MagMmc5983ma_Init(void);
MagMmc5983maUpdateResult MagMmc5983ma_Update(uint32_t now_ms, SensorsMagData_t *out);
uint8_t MagMmc5983ma_GetData(SensorsMagData_t *out);

#endif /* __MAG_MMC5983MA_H */
