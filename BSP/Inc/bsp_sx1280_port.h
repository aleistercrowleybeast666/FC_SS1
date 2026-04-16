#ifndef __BSP_SX1280_PORT_H
#define __BSP_SX1280_PORT_H

#include <stdint.h>
#include "main.h"
#include "sx1280.h"

void BspSx1280_PortInit(void);
void BspSx1280_OnExti(uint16_t gpioPin);

#endif