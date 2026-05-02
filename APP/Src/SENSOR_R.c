#include "main.h"
#include "cmsis_os.h"
#include "sensor_config.h"
#include "sensors.h"

void StartSENSOR_R_TASK(void *argument)
{
  for(;;)
  {
    Sensors_TaskUpdate();
    osDelay(SENSOR_TASK_PERIOD_MS);
  }

}
