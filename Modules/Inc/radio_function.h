#ifndef __RADIO_FUNCTION_H
#define __RADIO_FUNCTION_H

#include <stdint.h>
#include "air_protocol.h"

typedef enum
{
    LORA_OFF = 0U,
    LORA_ON  = 1U
} RadioLoraFlag;

typedef struct
{
    uint32_t time_ms;

    int16_t ax_raw;
    int16_t ay_raw;
    int16_t az_raw;

    int16_t gx_raw;
    int16_t gy_raw;
    int16_t gz_raw;

    int16_t qw_q15;
    int16_t qx_q15;
    int16_t qy_q15;
    int16_t qz_q15;

    float vx;
    float vy;
    float vz;

    float x;
    float y;
    float z;
} RadioFlightState;

typedef enum
{
    RADIO_STATUS_ENQUEUE_OK = 0U,
    RADIO_STATUS_ENQUEUE_BAD_PARAM,
    RADIO_STATUS_ENQUEUE_QUEUE_FULL
} RadioStatusEnqueueResult;

typedef enum
{
    RADIO_COMMAND_GET_LATEST_OK = 0U,
    RADIO_COMMAND_GET_LATEST_BAD_PARAM
} RadioCommandGetLatestResult;

typedef enum
{
    RADIO_COMMAND_NONE = 0U,
    RADIO_COMMAND_START,
    RADIO_COMMAND_LOCK,
    RADIO_COMMAND_UNLOCK
} RadioCommandFlag;

typedef enum
{
    RADIO_LOCK_STATE_LOCKED = 0U,
    RADIO_LOCK_STATE_UNLOCKED
} RadioLockState;

typedef enum
{
    RADIO_MISSION_STATE_IDLE = 0U,
    RADIO_MISSION_STATE_STARTED
} RadioMissionState;

void Radio_Process(void);
void Radio_FlightStateUpdate(const RadioFlightState *state);

void Radio_LoraFlagSet(RadioLoraFlag flag);
RadioLoraFlag Radio_LoraFlagGet(void);

RadioCommandGetLatestResult Radio_CommandGetLatest(RadioCommandFlag *cmd,
                                                   uint8_t *seq,
                                                   uint32_t *counter);

void Radio_LockStateSet(RadioLockState state);
void Radio_MissionStateSet(RadioMissionState state);

RadioStatusEnqueueResult Radio_StatusEnqueue(uint8_t status_id,
                                             uint32_t time_ms,
                                             uint8_t arg0,
                                             uint8_t arg1,
                                             uint8_t repeat_count);

#endif
