#include "radio_function.h"

#include "cmsis_os.h"

void StartRADIO_TXRX_TASK(void *argument)
{
    (void)argument;

    for (;;)
    {
        Radio_Process();
        osDelay(1);
    }
}
