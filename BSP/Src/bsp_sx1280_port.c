#include "bsp_sx1280_port.h"

#include "hw.h"
#include "spi.h"

static DioIrqHandler *s_dio1_irq_handler = 0;
static DioIrqHandler *s_dio2_irq_handler = 0;
static DioIrqHandler *s_dio3_irq_handler = 0;

void BspSx1280_PortInit(void)
{
    s_dio1_irq_handler = 0;
    s_dio2_irq_handler = 0;
    s_dio3_irq_handler = 0;
}

void GpioWrite(GPIO_TypeDef *port, uint16_t pin, uint32_t value)
{
    HAL_GPIO_WritePin(port, pin, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint8_t GpioRead(GPIO_TypeDef *port, uint16_t pin)
{
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? 1U : 0U;
}

void GpioSetIrq(GPIO_TypeDef *port, uint16_t pin, uint32_t irqPriority, DioIrqHandler *irqHandler)
{
    (void)irqPriority;

    if ((port == RADIO_DIO1_GPIO_Port) && (pin == RADIO_DIO1_Pin))
    {
        s_dio1_irq_handler = irqHandler;
    }
    else if ((port == RADIO_DIO2_GPIO_Port) && (pin == RADIO_DIO2_Pin))
    {
        s_dio2_irq_handler = irqHandler;
    }
    else if ((port == RADIO_DIO3_GPIO_Port) && (pin == RADIO_DIO3_Pin))
    {
        s_dio3_irq_handler = irqHandler;
    }
}

void SpiIn(uint8_t *txBuffer, uint16_t size)
{
    HAL_SPI_Transmit(&hspi1, txBuffer, size, HAL_MAX_DELAY);
}

void SpiInOut(uint8_t *txBuffer, uint8_t *rxBuffer, uint16_t size)
{
    HAL_SPI_TransmitReceive(&hspi1, txBuffer, rxBuffer, size, HAL_MAX_DELAY);
}

void BspSx1280_OnExti(uint16_t gpioPin)
{
    if ((gpioPin == RADIO_DIO1_Pin) && (s_dio1_irq_handler != 0))
    {
        s_dio1_irq_handler();
    }
    else if ((gpioPin == RADIO_DIO2_Pin) && (s_dio2_irq_handler != 0))
    {
        s_dio2_irq_handler();
    }
    else if ((gpioPin == RADIO_DIO3_Pin) && (s_dio3_irq_handler != 0))
    {
        s_dio3_irq_handler();
    }
}
