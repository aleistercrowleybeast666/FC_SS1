#include "main.h"
#include "bsp_uart_debug.h"
#include "debug_config.h"
#include "bsp_sx1281_port.h"

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &DEBUG_UART_HANDLE)
    {
        BspUartDebug_TxCpltCallback(huart);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &DEBUG_UART_HANDLE)
    {
        BspUartDebug_RxEventCallback(huart, Size);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &DEBUG_UART_HANDLE)
    {
        BspUartDebug_ErrorCallback(huart);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == RADIO_DIO1_Pin)
    {
        BspSx1281_OnExti(GPIO_Pin);
    }
}