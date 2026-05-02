#include "air_protocol.h"

#include <string.h>

static void Air_PutU16Le(uint8_t *buf, uint16_t value)
{
    buf[0] = (uint8_t)(value & 0xFFU);
    buf[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static void Air_PutU32Le(uint8_t *buf, uint32_t value)
{
    buf[0] = (uint8_t)(value & 0xFFU);
    buf[1] = (uint8_t)((value >> 8) & 0xFFU);
    buf[2] = (uint8_t)((value >> 16) & 0xFFU);
    buf[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static uint32_t Air_GetU32Le(const uint8_t *buf)
{
    return ((uint32_t)buf[0]) |
           ((uint32_t)buf[1] << 8) |
           ((uint32_t)buf[2] << 16) |
           ((uint32_t)buf[3] << 24);
}

static void Air_PutI16Le(uint8_t *buf, int16_t value)
{
    Air_PutU16Le(buf, (uint16_t)value);
}

static void Air_PutFloatLe(uint8_t *buf, float value)
{
    uint32_t raw = 0U;

    memcpy(&raw, &value, sizeof(raw));
    Air_PutU32Le(buf, raw);
}

uint8_t Air_GetExpectedFrameLength(uint8_t air_type)
{
    switch (air_type)
    {
    case AIR_TYPE_FLIGHT_STATE:
        return AIR_FLIGHT_STATE_LEN;

    case AIR_TYPE_STATUS:
        return AIR_STATUS_LEN;

    case AIR_TYPE_CMD:
        return AIR_CMD_LEN;

    case AIR_TYPE_ACK:
        return AIR_ACK_LEN;

    default:
        return 0U;
    }
}

AirParseResult Air_FrameValidate(const uint8_t *frame, uint8_t frame_len)
{
    uint8_t expected_len;

    if ((frame == 0U) || (frame_len < 2U))
    {
        return AIR_PARSE_BAD_PARAM;
    }

    expected_len = Air_GetExpectedFrameLength(frame[0]);
    if (expected_len == 0U)
    {
        return AIR_PARSE_BAD_TYPE;
    }

    if (frame_len != expected_len)
    {
        return AIR_PARSE_BAD_LEN;
    }

    return AIR_PARSE_OK;
}

AirBuildResult Air_FlightStateBuild(uint8_t seq,
                                    const AirFlightStatePayload *state,
                                    uint8_t *out_frame,
                                    uint8_t out_size,
                                    uint8_t *out_len)
{
    if ((state == 0U) || (out_frame == 0U) || (out_len == 0U))
    {
        return AIR_BUILD_BAD_PARAM;
    }

    if (out_size < AIR_FLIGHT_STATE_LEN)
    {
        *out_len = 0U;
        return AIR_BUILD_BAD_LEN;
    }

    out_frame[0] = AIR_TYPE_FLIGHT_STATE;
    out_frame[1] = seq;
    Air_PutU32Le(&out_frame[2], state->time_ms);

    Air_PutI16Le(&out_frame[6], state->ax_raw);
    Air_PutI16Le(&out_frame[8], state->ay_raw);
    Air_PutI16Le(&out_frame[10], state->az_raw);

    Air_PutI16Le(&out_frame[12], state->gx_raw);
    Air_PutI16Le(&out_frame[14], state->gy_raw);
    Air_PutI16Le(&out_frame[16], state->gz_raw);

    Air_PutI16Le(&out_frame[18], state->qw_q15);
    Air_PutI16Le(&out_frame[20], state->qx_q15);
    Air_PutI16Le(&out_frame[22], state->qy_q15);
    Air_PutI16Le(&out_frame[24], state->qz_q15);

    Air_PutFloatLe(&out_frame[26], state->vx);
    Air_PutFloatLe(&out_frame[30], state->vy);
    Air_PutFloatLe(&out_frame[34], state->vz);

    Air_PutFloatLe(&out_frame[38], state->x);
    Air_PutFloatLe(&out_frame[42], state->y);
    Air_PutFloatLe(&out_frame[46], state->z);

    *out_len = AIR_FLIGHT_STATE_LEN;
    return AIR_BUILD_OK;
}

AirBuildResult Air_StatusBuild(uint8_t seq,
                               uint8_t status_id,
                               uint32_t time_ms,
                               uint8_t arg0,
                               uint8_t arg1,
                               uint8_t *out_frame,
                               uint8_t out_size,
                               uint8_t *out_len)
{
    if ((out_frame == 0U) || (out_len == 0U))
    {
        return AIR_BUILD_BAD_PARAM;
    }

    if (Air_StatusIdIsValid(status_id) == 0U)
    {
        *out_len = 0U;
        return AIR_BUILD_BAD_TYPE;
    }

    if (out_size < AIR_STATUS_LEN)
    {
        *out_len = 0U;
        return AIR_BUILD_BAD_LEN;
    }

    out_frame[0] = AIR_TYPE_STATUS;
    out_frame[1] = seq;
    out_frame[2] = status_id;
    Air_PutU32Le(&out_frame[3], time_ms);
    out_frame[7] = arg0;
    out_frame[8] = arg1;

    *out_len = AIR_STATUS_LEN;
    return AIR_BUILD_OK;
}

AirBuildResult Air_AckBuild(uint8_t seq,
                            uint8_t ack_seq,
                            uint8_t ack_cmd_id,
                            uint8_t result,
                            uint32_t time_ms,
                            uint8_t *out_frame,
                            uint8_t out_size,
                            uint8_t *out_len)
{
    if ((out_frame == 0U) || (out_len == 0U))
    {
        return AIR_BUILD_BAD_PARAM;
    }

    if (out_size < AIR_ACK_LEN)
    {
        *out_len = 0U;
        return AIR_BUILD_BAD_LEN;
    }

    out_frame[0] = AIR_TYPE_ACK;
    out_frame[1] = seq;
    out_frame[2] = ack_seq;
    out_frame[3] = ack_cmd_id;
    out_frame[4] = result;
    Air_PutU32Le(&out_frame[5], time_ms);

    *out_len = AIR_ACK_LEN;
    return AIR_BUILD_OK;
}

AirParseResult Air_CmdParse(const uint8_t *frame, uint8_t frame_len, AirCmdPayload *cmd)
{
    AirParseResult validate_result;

    if ((frame == 0U) || (cmd == 0U))
    {
        return AIR_PARSE_BAD_PARAM;
    }

    validate_result = Air_FrameValidate(frame, frame_len);
    if (validate_result != AIR_PARSE_OK)
    {
        return validate_result;
    }

    if (frame[0] != AIR_TYPE_CMD)
    {
        return AIR_PARSE_BAD_TYPE;
    }

    cmd->seq = frame[1];
    cmd->cmd_id = frame[2];
    cmd->token = Air_GetU32Le(&frame[3]);
    cmd->param0 = frame[7];
    cmd->param1 = frame[8];

    return AIR_PARSE_OK;
}

uint8_t Air_CmdTokenIsValid(uint8_t cmd_id, uint32_t token)
{
    switch (cmd_id)
    {
    case AIR_CMD_START_MISSION:
        return (token == AIR_TOKEN_START_MISSION) ? 1U : 0U;

    case AIR_CMD_PING:
        return 1U;

    case AIR_CMD_LOCK:
        return (token == AIR_TOKEN_LOCK) ? 1U : 0U;

    case AIR_CMD_UNLOCK:
        return (token == AIR_TOKEN_UNLOCK) ? 1U : 0U;

    default:
        return 0U;
    }
}

uint8_t Air_StatusIdIsValid(uint8_t status_id)
{
    switch (status_id)
    {
    case AIR_STATUS_BOOT:
    case AIR_STATUS_SELFTEST_OK:
    case AIR_STATUS_MISSION_START:
    case AIR_STATUS_LAUNCH:
    case AIR_STATUS_PARACHUTE_DEPLOY:
    case AIR_STATUS_LANDING:
    case AIR_STATUS_LOCKED:
    case AIR_STATUS_UNLOCKED:
        return 1U;

    default:
        return 0U;
    }
}
