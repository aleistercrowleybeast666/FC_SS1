#include "radio_function.h"

#include "cmsis_gcc.h"
#include "cmsis_os.h"
#include "lora_sx1281.h"

#define RADIO_FLIGHT_STATE_PERIOD_MS       200U
#define RADIO_STATUS_REPEAT_INTERVAL_MS    50U
#define RADIO_STATUS_QUEUE_DEPTH           8U
#define RADIO_STATUS_MAX_REPEAT_COUNT      3U

typedef struct
{
    uint8_t status_id;
    uint32_t time_ms;
    uint8_t arg0;
    uint8_t arg1;
    uint8_t repeat_left;
} RadioStatusPacket;

typedef struct
{
    uint8_t valid;
    uint8_t seq;
    uint8_t cmd_id;
    uint32_t token;
    uint8_t param0;
    uint8_t param1;
    AirAckResult result;
} RadioLastCmdAck;

static RadioStatusPacket s_status_queue[RADIO_STATUS_QUEUE_DEPTH];
static volatile RadioFlightState s_radio_flight_state;
static volatile RadioLoraFlag s_radio_lora_flag = LORA_OFF;
static volatile RadioCommandFlag s_radio_command_flag = RADIO_COMMAND_NONE;
static volatile uint8_t s_radio_command_seq = 0U;
static volatile uint32_t s_radio_command_update_counter = 0U;
static volatile RadioLockState s_radio_lock_state = RADIO_LOCK_STATE_LOCKED;
static volatile RadioMissionState s_radio_mission_state = RADIO_MISSION_STATE_IDLE;
static uint8_t s_status_head = 0U;
static uint8_t s_status_tail = 0U;
static uint8_t s_status_count = 0U;

static uint8_t s_flight_seq = 0U;
static uint8_t s_status_seq = 0U;
static uint8_t s_ack_seq = 0U;

static uint32_t s_last_flight_tick = 0U;
static uint32_t s_last_status_tick = 0U;
static uint8_t s_flight_timer_started = 0U;
static RadioLastCmdAck s_last_cmd_ack = { 0U, 0U, 0U, 0U, 0U, 0U, AIR_ACK_RESULT_OK };

static uint32_t Radio_IrqLock(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void Radio_IrqUnlock(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static uint8_t Radio_TimeElapsed(uint32_t now_tick, uint32_t last_tick, uint32_t period_ms)
{
    return (((uint32_t)(now_tick - last_tick)) >= period_ms) ? 1U : 0U;
}

static void Radio_FlightStateSnapshotGet(RadioFlightState *snapshot)
{
    uint32_t primask;

    if (snapshot == 0U)
    {
        return;
    }

    primask = Radio_IrqLock();

    snapshot->time_ms = s_radio_flight_state.time_ms;

    snapshot->ax_raw = s_radio_flight_state.ax_raw;
    snapshot->ay_raw = s_radio_flight_state.ay_raw;
    snapshot->az_raw = s_radio_flight_state.az_raw;

    snapshot->gx_raw = s_radio_flight_state.gx_raw;
    snapshot->gy_raw = s_radio_flight_state.gy_raw;
    snapshot->gz_raw = s_radio_flight_state.gz_raw;

    snapshot->qw_q15 = s_radio_flight_state.qw_q15;
    snapshot->qx_q15 = s_radio_flight_state.qx_q15;
    snapshot->qy_q15 = s_radio_flight_state.qy_q15;
    snapshot->qz_q15 = s_radio_flight_state.qz_q15;

    snapshot->vx = s_radio_flight_state.vx;
    snapshot->vy = s_radio_flight_state.vy;
    snapshot->vz = s_radio_flight_state.vz;

    snapshot->x = s_radio_flight_state.x;
    snapshot->y = s_radio_flight_state.y;
    snapshot->z = s_radio_flight_state.z;

    Radio_IrqUnlock(primask);
}

void Radio_FlightStateUpdate(const RadioFlightState *state)
{
    uint32_t primask;

    if (state == 0U)
    {
        return;
    }

    primask = Radio_IrqLock();
    s_radio_flight_state = *state;
    Radio_IrqUnlock(primask);
}

void Radio_LoraFlagSet(RadioLoraFlag flag)
{
    uint32_t primask;

    if ((flag != LORA_OFF) && (flag != LORA_ON))
    {
        return;
    }

    primask = Radio_IrqLock();
    s_radio_lora_flag = flag;
    Radio_IrqUnlock(primask);
}

RadioLoraFlag Radio_LoraFlagGet(void)
{
    RadioLoraFlag flag;
    uint32_t primask;

    primask = Radio_IrqLock();
    flag = s_radio_lora_flag;
    Radio_IrqUnlock(primask);

    return flag;
}

RadioCommandGetLatestResult Radio_CommandGetLatest(RadioCommandFlag *cmd,
                                                   uint8_t *seq,
                                                   uint32_t *counter)
{
    uint32_t primask;

    if ((cmd == 0U) || (seq == 0U) || (counter == 0U))
    {
        return RADIO_COMMAND_GET_LATEST_BAD_PARAM;
    }

    primask = Radio_IrqLock();
    *cmd = s_radio_command_flag;
    *seq = s_radio_command_seq;
    *counter = s_radio_command_update_counter;
    Radio_IrqUnlock(primask);

    return RADIO_COMMAND_GET_LATEST_OK;
}

void Radio_LockStateSet(RadioLockState state)
{
    uint32_t primask;

    if ((state != RADIO_LOCK_STATE_LOCKED) && (state != RADIO_LOCK_STATE_UNLOCKED))
    {
        return;
    }

    primask = Radio_IrqLock();
    s_radio_lock_state = state;
    Radio_IrqUnlock(primask);
}

void Radio_MissionStateSet(RadioMissionState state)
{
    uint32_t primask;

    if ((state != RADIO_MISSION_STATE_IDLE) && (state != RADIO_MISSION_STATE_STARTED))
    {
        return;
    }

    primask = Radio_IrqLock();
    s_radio_mission_state = state;
    Radio_IrqUnlock(primask);
}

static void Radio_AirPayloadFromFlightState(const RadioFlightState *radio_state,
                                            AirFlightStatePayload *air_state)
{
    if ((radio_state == 0U) || (air_state == 0U))
    {
        return;
    }

    air_state->time_ms = radio_state->time_ms;

    air_state->ax_raw = radio_state->ax_raw;
    air_state->ay_raw = radio_state->ay_raw;
    air_state->az_raw = radio_state->az_raw;

    air_state->gx_raw = radio_state->gx_raw;
    air_state->gy_raw = radio_state->gy_raw;
    air_state->gz_raw = radio_state->gz_raw;

    air_state->qw_q15 = radio_state->qw_q15;
    air_state->qx_q15 = radio_state->qx_q15;
    air_state->qy_q15 = radio_state->qy_q15;
    air_state->qz_q15 = radio_state->qz_q15;

    air_state->vx = radio_state->vx;
    air_state->vy = radio_state->vy;
    air_state->vz = radio_state->vz;

    air_state->x = radio_state->x;
    air_state->y = radio_state->y;
    air_state->z = radio_state->z;
}

static uint8_t Radio_FrameEnqueue(const uint8_t *frame, uint8_t frame_len)
{
    return (Lora_TxEnqueue(frame, frame_len) == LORA_TX_ENQUEUE_OK) ? 1U : 0U;
}

static AirAckResult Radio_CommandRecord(uint8_t seq, RadioCommandFlag command_flag)
{
    uint32_t primask;

    primask = Radio_IrqLock();
    s_radio_command_flag = command_flag;
    s_radio_command_seq = seq;
    s_radio_command_update_counter++;
    Radio_IrqUnlock(primask);

    return AIR_ACK_RESULT_OK;
}

static uint8_t Radio_CmdIsRepeated(const AirCmdPayload *cmd, AirAckResult *result)
{
    uint8_t repeated = 0U;
    uint32_t primask;

    if ((cmd == 0U) || (result == 0U))
    {
        return 0U;
    }

    primask = Radio_IrqLock();
    if ((s_last_cmd_ack.valid != 0U) &&
        (s_last_cmd_ack.seq == cmd->seq) &&
        (s_last_cmd_ack.cmd_id == cmd->cmd_id) &&
        (s_last_cmd_ack.token == cmd->token) &&
        (s_last_cmd_ack.param0 == cmd->param0) &&
        (s_last_cmd_ack.param1 == cmd->param1))
    {
        *result = s_last_cmd_ack.result;
        repeated = 1U;
    }
    Radio_IrqUnlock(primask);

    return repeated;
}

static void Radio_LastCmdAckSave(const AirCmdPayload *cmd, AirAckResult result)
{
    uint32_t primask;

    if (cmd == 0U)
    {
        return;
    }

    primask = Radio_IrqLock();
    s_last_cmd_ack.valid = 1U;
    s_last_cmd_ack.seq = cmd->seq;
    s_last_cmd_ack.cmd_id = cmd->cmd_id;
    s_last_cmd_ack.token = cmd->token;
    s_last_cmd_ack.param0 = cmd->param0;
    s_last_cmd_ack.param1 = cmd->param1;
    s_last_cmd_ack.result = result;
    Radio_IrqUnlock(primask);
}

static void Radio_AckSend(uint8_t ack_seq, uint8_t ack_cmd_id, AirAckResult result)
{
    uint8_t frame[AIR_ACK_LEN];
    uint8_t frame_len = 0U;

    if (Air_AckBuild(s_ack_seq,
                     ack_seq,
                     ack_cmd_id,
                     (uint8_t)result,
                     osKernelGetTickCount(),
                     frame,
                     sizeof(frame),
                     &frame_len) != AIR_BUILD_OK)
    {
        return;
    }

    if (Radio_FrameEnqueue(frame, frame_len) != 0U)
    {
        s_ack_seq++;
    }
}

static AirAckResult Radio_CmdDispatch(const AirCmdPayload *cmd)
{
    RadioLockState lock_state;
    RadioMissionState mission_state;
    uint32_t primask;

    if (cmd == 0U)
    {
        return AIR_ACK_RESULT_BAD_CMD;
    }

    switch (cmd->cmd_id)
    {
    case AIR_CMD_PING:
        return AIR_ACK_RESULT_OK;

    case AIR_CMD_START_MISSION:
        if (Air_CmdTokenIsValid(cmd->cmd_id, cmd->token) == 0U)
        {
            return AIR_ACK_RESULT_BAD_TOKEN;
        }

        primask = Radio_IrqLock();
        lock_state = s_radio_lock_state;
        mission_state = s_radio_mission_state;
        Radio_IrqUnlock(primask);

        if (mission_state != RADIO_MISSION_STATE_IDLE)
        {
            return AIR_ACK_RESULT_BAD_STATE;
        }

        if (lock_state != RADIO_LOCK_STATE_UNLOCKED)
        {
            return AIR_ACK_RESULT_LOCKED_REQUIRED;
        }

        return Radio_CommandRecord(cmd->seq, RADIO_COMMAND_START);

    case AIR_CMD_LOCK:
        if (Air_CmdTokenIsValid(cmd->cmd_id, cmd->token) == 0U)
        {
            return AIR_ACK_RESULT_BAD_TOKEN;
        }

        primask = Radio_IrqLock();
        lock_state = s_radio_lock_state;
        mission_state = s_radio_mission_state;
        Radio_IrqUnlock(primask);

        if (mission_state != RADIO_MISSION_STATE_IDLE)
        {
            return AIR_ACK_RESULT_BAD_STATE;
        }

        if (lock_state == RADIO_LOCK_STATE_LOCKED)
        {
            return AIR_ACK_RESULT_ALREADY_LOCKED;
        }

        return Radio_CommandRecord(cmd->seq, RADIO_COMMAND_LOCK);

    case AIR_CMD_UNLOCK:
        if (Air_CmdTokenIsValid(cmd->cmd_id, cmd->token) == 0U)
        {
            return AIR_ACK_RESULT_BAD_TOKEN;
        }

        primask = Radio_IrqLock();
        lock_state = s_radio_lock_state;
        mission_state = s_radio_mission_state;
        Radio_IrqUnlock(primask);

        if (mission_state != RADIO_MISSION_STATE_IDLE)
        {
            return AIR_ACK_RESULT_BAD_STATE;
        }

        if (lock_state == RADIO_LOCK_STATE_UNLOCKED)
        {
            return AIR_ACK_RESULT_ALREADY_UNLOCKED;
        }

        return Radio_CommandRecord(cmd->seq, RADIO_COMMAND_UNLOCK);

    default:
        return AIR_ACK_RESULT_BAD_CMD;
    }
}

static void Radio_AirFrameProcess(const uint8_t *frame, uint8_t frame_len)
{
    AirCmdPayload cmd;
    AirParseResult parse_result;
    uint8_t ack_cmd_id = 0U;

    if ((frame == 0U) || (frame_len < 2U))
    {
        return;
    }

    if (frame[0] != AIR_TYPE_CMD)
    {
        return;
    }

    if (frame_len >= 3U)
    {
        ack_cmd_id = frame[2];
    }

    if (frame_len != AIR_CMD_LEN)
    {
        Radio_AckSend(frame[1], ack_cmd_id, AIR_ACK_RESULT_BAD_LEN);
        return;
    }

    parse_result = Air_CmdParse(frame, frame_len, &cmd);
    if (parse_result == AIR_PARSE_OK)
    {
        AirAckResult result;

        if (Radio_CmdIsRepeated(&cmd, &result) == 0U)
        {
            result = Radio_CmdDispatch(&cmd);
            if (cmd.cmd_id != AIR_CMD_PING)
            {
                Radio_LastCmdAckSave(&cmd, result);
            }
        }

        Radio_AckSend(cmd.seq, cmd.cmd_id, result);
    }
    else if (parse_result == AIR_PARSE_BAD_LEN)
    {
        Radio_AckSend(frame[1], ack_cmd_id, AIR_ACK_RESULT_BAD_LEN);
    }
    else
    {
        Radio_AckSend(frame[1], ack_cmd_id, AIR_ACK_RESULT_BAD_CMD);
    }
}

static void Radio_RxProcess(void)
{
    uint8_t frame[LORA_MAX_PAYLOAD_LEN];
    uint8_t frame_len = 0U;
    int8_t rssi = 0;
    int8_t snr = 0;

    while (Lora_RxDequeue(frame, &frame_len, &rssi, &snr) == LORA_RX_DEQUEUE_OK)
    {
        (void)rssi;
        (void)snr;
        Radio_AirFrameProcess(frame, frame_len);
    }
}

static uint8_t Radio_StatusQueuePeek(RadioStatusPacket *packet)
{
    uint8_t has_packet = 0U;
    uint32_t primask;

    if (packet == 0U)
    {
        return 0U;
    }

    primask = Radio_IrqLock();
    if (s_status_count > 0U)
    {
        *packet = s_status_queue[s_status_tail];
        has_packet = 1U;
    }
    Radio_IrqUnlock(primask);

    return has_packet;
}

static void Radio_StatusQueueConsumeOne(void)
{
    uint32_t primask;

    primask = Radio_IrqLock();
    if (s_status_count > 0U)
    {
        if (s_status_queue[s_status_tail].repeat_left > 1U)
        {
            s_status_queue[s_status_tail].repeat_left--;
        }
        else
        {
            s_status_tail++;
            if (s_status_tail >= RADIO_STATUS_QUEUE_DEPTH)
            {
                s_status_tail = 0U;
            }
            s_status_count--;
        }
    }
    Radio_IrqUnlock(primask);
}

static void Radio_StatusProcess(uint32_t now_tick)
{
    RadioStatusPacket packet;
    uint8_t frame[AIR_STATUS_LEN];
    uint8_t frame_len = 0U;

    if (Radio_TimeElapsed(now_tick, s_last_status_tick, RADIO_STATUS_REPEAT_INTERVAL_MS) == 0U)
    {
        return;
    }

    if (Radio_StatusQueuePeek(&packet) == 0U)
    {
        return;
    }

    if (Air_StatusBuild(s_status_seq,
                        packet.status_id,
                        packet.time_ms,
                        packet.arg0,
                        packet.arg1,
                        frame,
                        sizeof(frame),
                        &frame_len) != AIR_BUILD_OK)
    {
        Radio_StatusQueueConsumeOne();
        s_last_status_tick = now_tick;
        return;
    }

    if (Radio_FrameEnqueue(frame, frame_len) != 0U)
    {
        s_status_seq++;
        Radio_StatusQueueConsumeOne();
    }

    s_last_status_tick = now_tick;
}

static void Radio_FlightStateProcess(uint32_t now_tick)
{
    RadioFlightState snapshot;
    AirFlightStatePayload air_state;
    uint8_t frame[AIR_FLIGHT_STATE_LEN];
    uint8_t frame_len = 0U;

    if (Radio_LoraFlagGet() != LORA_ON)
    {
        s_flight_timer_started = 0U;
        return;
    }

    if (s_flight_timer_started == 0U)
    {
        s_last_flight_tick = (uint32_t)(now_tick - RADIO_FLIGHT_STATE_PERIOD_MS);
        s_flight_timer_started = 1U;
    }

    if (Radio_TimeElapsed(now_tick, s_last_flight_tick, RADIO_FLIGHT_STATE_PERIOD_MS) == 0U)
    {
        return;
    }

    Radio_FlightStateSnapshotGet(&snapshot);
    Radio_AirPayloadFromFlightState(&snapshot, &air_state);

    if (Air_FlightStateBuild(s_flight_seq,
                             &air_state,
                             frame,
                             sizeof(frame),
                             &frame_len) != AIR_BUILD_OK)
    {
        s_last_flight_tick = now_tick;
        return;
    }

    if (Radio_FrameEnqueue(frame, frame_len) != 0U)
    {
        s_flight_seq++;
    }

    s_last_flight_tick = now_tick;
}

RadioStatusEnqueueResult Radio_StatusEnqueue(uint8_t status_id,
                                             uint32_t time_ms,
                                             uint8_t arg0,
                                             uint8_t arg1,
                                             uint8_t repeat_count)
{
    uint32_t primask;

    if ((Air_StatusIdIsValid(status_id) == 0U) ||
        (repeat_count == 0U) ||
        (repeat_count > RADIO_STATUS_MAX_REPEAT_COUNT))
    {
        return RADIO_STATUS_ENQUEUE_BAD_PARAM;
    }

    primask = Radio_IrqLock();

    if (s_status_count >= RADIO_STATUS_QUEUE_DEPTH)
    {
        Radio_IrqUnlock(primask);
        return RADIO_STATUS_ENQUEUE_QUEUE_FULL;
    }

    s_status_queue[s_status_head].status_id = status_id;
    s_status_queue[s_status_head].time_ms = time_ms;
    s_status_queue[s_status_head].arg0 = arg0;
    s_status_queue[s_status_head].arg1 = arg1;
    s_status_queue[s_status_head].repeat_left = repeat_count;

    s_status_head++;
    if (s_status_head >= RADIO_STATUS_QUEUE_DEPTH)
    {
        s_status_head = 0U;
    }
    s_status_count++;

    Radio_IrqUnlock(primask);

    return RADIO_STATUS_ENQUEUE_OK;
}

void Radio_Process(void)
{
    uint32_t now_tick;

    Lora_Process();
    Radio_RxProcess();

    now_tick = osKernelGetTickCount();
    Radio_StatusProcess(now_tick);
    Radio_FlightStateProcess(now_tick);
}
