#include "PWM_servo.h"

#include "PWM_servo_config.h"

static uint32_t PwmServo_ChannelGet(PwmServoId id)
{
    switch (id)
    {
        case PWM_SERVO_ID_3:
            return PWM_SERVO_3_CHANNEL;

        case PWM_SERVO_ID_4:
            return PWM_SERVO_4_CHANNEL;

        case PWM_SERVO_ID_5:
            return PWM_SERVO_5_CHANNEL;

        default:
            return 0U;
    }
}

static uint32_t PwmServo_CmpFromAngle(float angle)
{
    const float cmp_range = (float)(PWM_SERVO_MAX_DEG_CMP - PWM_SERVO_0_DEG_CMP);
    const float cmp = (cmp_range * angle / PWM_SERVO_MAX_ANGLE_DEG) + (float)PWM_SERVO_0_DEG_CMP;

    return (uint32_t)(cmp + 0.5f);
}

PwmServoInitResult PwmServo_Init(void)
{
    __HAL_TIM_SET_COMPARE(&PWM_SERVO_TIM, PWM_SERVO_3_CHANNEL, PwmServo_CmpFromAngle(0.0f));
    __HAL_TIM_SET_COMPARE(&PWM_SERVO_TIM, PWM_SERVO_4_CHANNEL, PwmServo_CmpFromAngle(0.0f));
    __HAL_TIM_SET_COMPARE(&PWM_SERVO_TIM, PWM_SERVO_5_CHANNEL, PwmServo_CmpFromAngle(0.0f));

    if (HAL_TIM_Base_Start(&PWM_SERVO_TIM) != HAL_OK)
    {
        return PWM_SERVO_INIT_HAL_ERROR;
    }

    if (HAL_TIM_PWM_Start(&PWM_SERVO_TIM, PWM_SERVO_3_CHANNEL) != HAL_OK)
    {
        return PWM_SERVO_INIT_HAL_ERROR;
    }

    if (HAL_TIM_PWM_Start(&PWM_SERVO_TIM, PWM_SERVO_4_CHANNEL) != HAL_OK)
    {
        return PWM_SERVO_INIT_HAL_ERROR;
    }

    if (HAL_TIM_PWM_Start(&PWM_SERVO_TIM, PWM_SERVO_5_CHANNEL) != HAL_OK)
    {
        return PWM_SERVO_INIT_HAL_ERROR;
    }

    return PWM_SERVO_INIT_OK;
}

PwmServoSetAngleResult PwmServo_SetAngle(PwmServoId id, float angle)
{
    uint32_t channel = PwmServo_ChannelGet(id);

    if (channel == 0U)
    {
        return PWM_SERVO_SET_ANGLE_BAD_ID;
    }

    if (!((angle >= 0.0f) && (angle <= PWM_SERVO_MAX_ANGLE_DEG)))
    {
        return PWM_SERVO_SET_ANGLE_BAD_ANGLE;
    }

    __HAL_TIM_SET_COMPARE(&PWM_SERVO_TIM, channel, PwmServo_CmpFromAngle(angle));

    return PWM_SERVO_SET_ANGLE_OK;
}
