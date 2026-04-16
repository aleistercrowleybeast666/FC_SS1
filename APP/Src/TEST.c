#include "main.h"
#include "cmsis_os.h"
#include "debug_log.h"
#include "lora_sx1280.h"
#include <stdio.h>

void StartTEST_TASK(void *argument)
{
  uint32_t last_tx_tick = 0U;
  uint32_t tx_seq = 0U;

  for(;;)
  {
    Lora_Process();

    if (osKernelGetTickCount() - last_tx_tick >= 1000U)
    {
      char tx_buf[40];
      int len;

      last_tx_tick = osKernelGetTickCount();
      len = snprintf(tx_buf, sizeof(tx_buf), "FC %lu", (unsigned long)tx_seq++);

      if (len > 0)
      {
        if (Lora_Send((const uint8_t *)tx_buf, (uint8_t)len) != 0U)
        {
          DebugLog_Print("LoRa TX req: %s", tx_buf);
        }
        else
        {
          DebugLog_Print("LoRa TX busy");
        }
      }
    }

    osDelay(1);
  }
}