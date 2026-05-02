#include "PWR_out.h"

#include "PWR_out_config.h"

static GPIO_TypeDef *PwrOut_PortGet(PwrOutId id)
{
    switch (id)
    {
        case PWR_OUT_ID_1:
            return PWR_OUT_1_GPIO_Port;

        case PWR_OUT_ID_2:
            return PWR_OUT_2_GPIO_Port;

        default:
            return (GPIO_TypeDef *)0;
    }
}

static uint16_t PwrOut_PinGet(PwrOutId id)
{
    switch (id)
    {
        case PWR_OUT_ID_1:
            return PWR_OUT_1_Pin;

        case PWR_OUT_ID_2:
            return PWR_OUT_2_Pin;

        default:
            return 0U;
    }
}

void PwrOut_Init(void)
{
    (void)PwrOut_Off(PWR_OUT_ID_1);
    (void)PwrOut_Off(PWR_OUT_ID_2);
}

PwrOutSetResult PwrOut_SetState(PwrOutId id, PwrOutState state)
{
    GPIO_TypeDef *port = PwrOut_PortGet(id);
    uint16_t pin = PwrOut_PinGet(id);

    if ((port == (GPIO_TypeDef *)0) || (pin == 0U))
    {
        return PWR_OUT_SET_BAD_ID;
    }

    if (state == PWR_OUT_STATE_ON)
    {
        HAL_GPIO_WritePin(port, pin, PWR_OUT_ACTIVE_LEVEL);
    }
    else if (state == PWR_OUT_STATE_OFF)
    {
        HAL_GPIO_WritePin(port, pin, PWR_OUT_INACTIVE_LEVEL);
    }
    else
    {
        return PWR_OUT_SET_BAD_STATE;
    }

    return PWR_OUT_SET_OK;
}

PwrOutSetResult PwrOut_On(PwrOutId id)
{
    return PwrOut_SetState(id, PWR_OUT_STATE_ON);
}

PwrOutSetResult PwrOut_Off(PwrOutId id)
{
    return PwrOut_SetState(id, PWR_OUT_STATE_OFF);
}

PwrOutSetResult PwrOut_Toggle(PwrOutId id)
{
    GPIO_TypeDef *port = PwrOut_PortGet(id);
    uint16_t pin = PwrOut_PinGet(id);

    if ((port == (GPIO_TypeDef *)0) || (pin == 0U))
    {
        return PWR_OUT_SET_BAD_ID;
    }

    HAL_GPIO_TogglePin(port, pin);

    return PWR_OUT_SET_OK;
}

PwrOutGetResult PwrOut_GetState(PwrOutId id, PwrOutState *state)
{
    GPIO_TypeDef *port = PwrOut_PortGet(id);
    uint16_t pin = PwrOut_PinGet(id);
    GPIO_PinState pin_state;

    if (state == (PwrOutState *)0)
    {
        return PWR_OUT_GET_BAD_PARAM;
    }

    if ((port == (GPIO_TypeDef *)0) || (pin == 0U))
    {
        return PWR_OUT_GET_BAD_ID;
    }

    pin_state = HAL_GPIO_ReadPin(port, pin);
    *state = (pin_state == PWR_OUT_ACTIVE_LEVEL) ? PWR_OUT_STATE_ON : PWR_OUT_STATE_OFF;

    return PWR_OUT_GET_OK;
}
