#ifndef AIR_PROTOCOL_H
#define AIR_PROTOCOL_H

#include <stdint.h>

#define AIR_MAX_FRAME_LEN                 50U

#define AIR_FLIGHT_STATE_LEN              50U
#define AIR_STATUS_LEN                    9U
#define AIR_CMD_LEN                       9U
#define AIR_ACK_LEN                       9U

#define AIR_TOKEN_START_MISSION           0xA55A3CC3UL
#define AIR_TOKEN_LOCK                    0xC33CA55AUL
#define AIR_TOKEN_UNLOCK                  0x55AA6996UL

typedef enum
{
    AIR_TYPE_FLIGHT_STATE = 0x10U,
    AIR_TYPE_STATUS       = 0x20U,
    AIR_TYPE_CMD          = 0x30U,
    AIR_TYPE_ACK          = 0x40U
} AirType;

typedef enum
{
    AIR_STATUS_BOOT             = 0x01U,
    AIR_STATUS_SELFTEST_OK      = 0x02U,
    AIR_STATUS_MISSION_START    = 0x03U,
    AIR_STATUS_LAUNCH           = 0x04U,
    AIR_STATUS_PARACHUTE_DEPLOY = 0x05U,
    AIR_STATUS_LANDING          = 0x06U,
    AIR_STATUS_LOCKED           = 0x07U,
    AIR_STATUS_UNLOCKED         = 0x08U
} AirStatusId;

typedef enum
{
    AIR_CMD_START_MISSION = 0x01U,
    AIR_CMD_PING          = 0x02U,
    AIR_CMD_LOCK          = 0x03U,
    AIR_CMD_UNLOCK        = 0x04U
} AirCmdId;

typedef enum
{
    AIR_ACK_RESULT_OK               = 0x00U,
    AIR_ACK_RESULT_BAD_LEN          = 0x01U,
    AIR_ACK_RESULT_BAD_CMD          = 0x02U,
    AIR_ACK_RESULT_BAD_TOKEN        = 0x03U,
    AIR_ACK_RESULT_BUSY             = 0x04U,
    AIR_ACK_RESULT_REJECTED         = 0x05U,
    AIR_ACK_RESULT_BAD_STATE        = 0x06U,
    AIR_ACK_RESULT_LOCKED_REQUIRED  = 0x07U,
    AIR_ACK_RESULT_ALREADY_LOCKED   = 0x08U,
    AIR_ACK_RESULT_ALREADY_UNLOCKED = 0x09U
} AirAckResult;

typedef enum
{
    AIR_BUILD_OK = 0U,
    AIR_BUILD_BAD_PARAM,
    AIR_BUILD_BAD_LEN,
    AIR_BUILD_BAD_TYPE
} AirBuildResult;

typedef enum
{
    AIR_PARSE_OK = 0U,
    AIR_PARSE_BAD_PARAM,
    AIR_PARSE_BAD_LEN,
    AIR_PARSE_BAD_TYPE
} AirParseResult;

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
} AirFlightStatePayload;

typedef struct
{
    uint8_t seq;
    uint8_t cmd_id;
    uint32_t token;
    uint8_t param0;
    uint8_t param1;
} AirCmdPayload;

uint8_t Air_GetExpectedFrameLength(uint8_t air_type);
AirParseResult Air_FrameValidate(const uint8_t *frame, uint8_t frame_len);

AirBuildResult Air_FlightStateBuild(uint8_t seq,
                                    const AirFlightStatePayload *state,
                                    uint8_t *out_frame,
                                    uint8_t out_size,
                                    uint8_t *out_len);

AirBuildResult Air_StatusBuild(uint8_t seq,
                               uint8_t status_id,
                               uint32_t time_ms,
                               uint8_t arg0,
                               uint8_t arg1,
                               uint8_t *out_frame,
                               uint8_t out_size,
                               uint8_t *out_len);

AirBuildResult Air_AckBuild(uint8_t seq,
                            uint8_t ack_seq,
                            uint8_t ack_cmd_id,
                            uint8_t result,
                            uint32_t time_ms,
                            uint8_t *out_frame,
                            uint8_t out_size,
                            uint8_t *out_len);

AirParseResult Air_CmdParse(const uint8_t *frame, uint8_t frame_len, AirCmdPayload *cmd);
uint8_t Air_CmdTokenIsValid(uint8_t cmd_id, uint32_t token);
uint8_t Air_StatusIdIsValid(uint8_t status_id);

#endif
