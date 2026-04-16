#ifndef __HW_H__
#define __HW_H__

#include <stdint.h>
#include "main.h"
#include "spi.h"
#include "sx1280.h"

/* 与官方 sx1280-hal.c 对接 */
#define RADIO_NSS_PORT      SPI_RADIO_NSS_GPIO_Port
#define RADIO_NSS_PIN       SPI_RADIO_NSS_Pin

#define RADIO_BUSY_PORT     RADIO_BUSY_GPIO_Port
#define RADIO_BUSY_PIN      RADIO_BUSY_Pin

#define RADIO_nRESET_PORT   RADIO_RST_GPIO_Port
#define RADIO_nRESET_PIN    RADIO_RST_Pin

/* 官方 hal 层会调用这些函数 */
void GpioWrite(GPIO_TypeDef *port, uint16_t pin, uint32_t value);
uint8_t GpioRead(GPIO_TypeDef *port, uint16_t pin);
void GpioSetIrq(GPIO_TypeDef *port, uint16_t pin, uint32_t irqPriority, DioIrqHandler *irqHandler);

void SpiIn(uint8_t *txBuffer, uint16_t size);
void SpiInOut(uint8_t *txBuffer, uint8_t *rxBuffer, uint16_t size);

#endif