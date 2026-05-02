#ifndef __PWR_OUT_H
#define __PWR_OUT_H

typedef enum
{
    PWR_OUT_ID_1 = 1U,
    PWR_OUT_ID_2 = 2U
} PwrOutId;

typedef enum
{
    PWR_OUT_STATE_OFF = 0U,
    PWR_OUT_STATE_ON  = 1U
} PwrOutState;

typedef enum
{
    PWR_OUT_SET_OK = 0U,
    PWR_OUT_SET_BAD_ID,
    PWR_OUT_SET_BAD_STATE
} PwrOutSetResult;

typedef enum
{
    PWR_OUT_GET_OK = 0U,
    PWR_OUT_GET_BAD_ID,
    PWR_OUT_GET_BAD_PARAM
} PwrOutGetResult;

void PwrOut_Init(void);
PwrOutSetResult PwrOut_SetState(PwrOutId id, PwrOutState state);
PwrOutSetResult PwrOut_On(PwrOutId id);
PwrOutSetResult PwrOut_Off(PwrOutId id);
PwrOutSetResult PwrOut_Toggle(PwrOutId id);
PwrOutGetResult PwrOut_GetState(PwrOutId id, PwrOutState *state);

#endif /* __PWR_OUT_H */
