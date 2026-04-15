#include "main.h"
#include "cmsis_os.h"

void StartTEST_TASK(void *argument)
{
  /* USER CODE BEGIN StartTEST_TASK */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTEST_TASK */
}