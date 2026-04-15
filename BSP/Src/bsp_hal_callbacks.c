#include "main.h"
#include "bsp_uart_debug.h"
#include "debug_config.h"


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
