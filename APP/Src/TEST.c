#include "main.h"
#include "cmsis_os.h"
#include "debug_log.h"
#include "imu_manager.h"
#include "imu_manager_config.h"
#include "lora_sx1281.h"
#include "sensors.h"

static void Test_SensorsOnline_Print(void);

static void Test_SensorsOnline_Print(void)
{
  DebugLog_Print("[DBG] SENSOR_ONLINE,gnss=%u,fix=%u,nav=%u,baro=%u,mag=%u,imu0=%u,imu1=%u,primary=%u,primaryOnline=%u,calState=%u,cal0=%u,cal1=%u",
                 Sensors_GnssIsOnline(),
                 Sensors_GnssHasValidFix(),
                 Sensors_GnssIsUsableForNav(),
                 Sensors_BaroIsOnline(),
                 Sensors_MagIsOnline(),
                 ImuManager_IsOnline(IMU_ID_BMI088),
                 ImuManager_IsOnline(IMU_ID_ICM42688P),
                 ImuManager_GetPrimaryId(),
                 ImuManager_IsPrimaryOnline(),
                 (uint32_t)ImuManager_GetCalibrationState(),
                 ImuManager_IsCalibrated(IMU_ID_BMI088),
                 ImuManager_IsCalibrated(IMU_ID_ICM42688P));
}

void StartTEST_TASK(void *argument)
{
  uint32_t last_print_ms = 0U;

  for(;;)
  {
    uint32_t now = HAL_GetTick();

    if ((now - last_print_ms) >= 1000U)
    {
      last_print_ms = now;
      Test_SensorsOnline_Print();
    }

    osDelay(1);
  }
}
