#include "main.h"
#include "cmsis_os.h"
#include "air_protocol.h"
#include "debug_log.h"
#include "radio_function.h"

#define TEST_LORA_TASK_DELAY_MS            10U
#define TEST_LORA_STATE_UPDATE_PERIOD_MS   50U
#define TEST_LORA_STATUS_REPEAT_COUNT      3U
#define TEST_LORA_STATUS_ARG_DEFAULT       0U

static void Test_LoraTaskRun(void);
static void Test_LoraFlightStateUpdate(void);
static void Test_LoraCommandProcess(void);
static void Test_LoraCommandApply(const RadioCommandEvent *event);
static void Test_LoraCommandPrint(const RadioCommandEvent *event);
static void Test_LoraStatusSend(uint8_t status_id);
static const char *Test_LoraCmdNameGet(uint8_t cmd_id);
static uint8_t Test_TimeElapsed(uint32_t now_ms, uint32_t last_ms, uint32_t period_ms);

/*
 * Original sensor-online debug output is disabled while TEST_TASK runs
 * the LoRa protocol test.
 *
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
 */

static uint8_t Test_TimeElapsed(uint32_t now_ms, uint32_t last_ms, uint32_t period_ms)
{
    return (((uint32_t)(now_ms - last_ms)) >= period_ms) ? 1U : 0U;
}

static void Test_LoraFlightStateUpdate(void)
{
    RadioFlightState state = { 0U };

    state.time_ms = HAL_GetTick();
    Radio_FlightStateUpdate(&state);
}

static void Test_LoraStatusSend(uint8_t status_id)
{
    RadioStatusEnqueueResult result;

    result = Radio_StatusEnqueue(status_id,
                                 HAL_GetTick(),
                                 TEST_LORA_STATUS_ARG_DEFAULT,
                                 TEST_LORA_STATUS_ARG_DEFAULT,
                                 TEST_LORA_STATUS_REPEAT_COUNT);
    if (result != RADIO_STATUS_ENQUEUE_OK)
    {
        DebugLog_Print("[DBG] AIR_STATUS enqueue failed,status=0x%02X,result=%u",
                       (unsigned int)status_id,
                       (unsigned int)result);
    }
}

static const char *Test_LoraCmdNameGet(uint8_t cmd_id)
{
    switch (cmd_id)
    {
    case AIR_CMD_START_MISSION:
        return "START_MISSION";

    case AIR_CMD_PING:
        return "PING";

    case AIR_CMD_LOCK:
        return "LOCK";

    case AIR_CMD_UNLOCK:
        return "UNLOCK";

    default:
        return "UNKNOWN";
    }
}

static void Test_LoraCommandPrint(const RadioCommandEvent *event)
{
    if (event == 0U)
    {
        return;
    }

    DebugLog_Print("[DBG] AIR_CMD,name=%s,seq=%u,cmd=0x%02X,token=0x%08lX,param0=%u,param1=%u,len=%u,ack=%u",
                   Test_LoraCmdNameGet(event->cmd_id),
                   (unsigned int)event->seq,
                   (unsigned int)event->cmd_id,
                   (unsigned long)event->token,
                   (unsigned int)event->param0,
                   (unsigned int)event->param1,
                   (unsigned int)event->frame_len,
                   (unsigned int)event->ack_result);
}

static void Test_LoraCommandApply(const RadioCommandEvent *event)
{
    if (event == 0U)
    {
        return;
    }

    if ((event->cmd_id == AIR_CMD_LOCK) &&
        ((event->ack_result == AIR_ACK_RESULT_OK) ||
         (event->ack_result == AIR_ACK_RESULT_ALREADY_LOCKED)))
    {
        Radio_LockStateSet(RADIO_LOCK_STATE_LOCKED);
        Test_LoraStatusSend(AIR_STATUS_LOCKED);
    }
    else if ((event->cmd_id == AIR_CMD_UNLOCK) &&
             ((event->ack_result == AIR_ACK_RESULT_OK) ||
              (event->ack_result == AIR_ACK_RESULT_ALREADY_UNLOCKED)))
    {
        Radio_LockStateSet(RADIO_LOCK_STATE_UNLOCKED);
        Test_LoraStatusSend(AIR_STATUS_UNLOCKED);
    }
}

static void Test_LoraCommandProcess(void)
{
    RadioCommandEvent event;

    while (Radio_CommandEventDequeue(&event) == RADIO_COMMAND_EVENT_DEQUEUE_OK)
    {
        Test_LoraCommandPrint(&event);
        Test_LoraCommandApply(&event);
    }
}

static void Test_LoraTaskRun(void)
{
    uint32_t last_state_update_ms;
    uint32_t now_ms;

    Test_LoraFlightStateUpdate();
    last_state_update_ms = HAL_GetTick();

    Radio_MissionStateSet(RADIO_MISSION_STATE_IDLE);
    Radio_LockStateSet(RADIO_LOCK_STATE_LOCKED);
    Radio_LoraFlagSet(LORA_ON);
    Test_LoraStatusSend(AIR_STATUS_BOOT);

    for (;;)
    {
        now_ms = HAL_GetTick();
        if (Test_TimeElapsed(now_ms, last_state_update_ms, TEST_LORA_STATE_UPDATE_PERIOD_MS) != 0U)
        {
            last_state_update_ms = now_ms;
            Test_LoraFlightStateUpdate();
        }

        Test_LoraCommandProcess();
        osDelay(TEST_LORA_TASK_DELAY_MS);
    }
}

void StartTEST_TASK(void *argument)
{
  (void)argument;

  /*
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
  */

  Test_LoraTaskRun();
}
