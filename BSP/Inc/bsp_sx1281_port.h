#ifndef __BSP_SX1281_PORT_H
#define __BSP_SX1281_PORT_H

#include <stdint.h>
#include "main.h"
#include "sx1280.h"

void BspSx1281_PortInit(void);
void BspSx1281_OnExti(uint16_t gpioPin);

#endif
