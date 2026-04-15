#include "main.h"
#include "cmsis_os.h"
#include "bsp_key.h"

void StartKEY_SCAN_TASK(void *argument)
{
    for(;;)
    {
        Key_Check();
        osDelay(5);
    }
}