#ifndef __PWM_SERVO_H
#define __PWM_SERVO_H

typedef enum
{
    PWM_SERVO_ID_3 = 3U,
    PWM_SERVO_ID_4 = 4U,
    PWM_SERVO_ID_5 = 5U
} PwmServoId;

typedef enum
{
    PWM_SERVO_INIT_OK = 0U,
    PWM_SERVO_INIT_HAL_ERROR
} PwmServoInitResult;

typedef enum
{
    PWM_SERVO_SET_ANGLE_OK = 0U,
    PWM_SERVO_SET_ANGLE_BAD_ID,
    PWM_SERVO_SET_ANGLE_BAD_ANGLE
} PwmServoSetAngleResult;

PwmServoInitResult PwmServo_Init(void);
PwmServoSetAngleResult PwmServo_SetAngle(PwmServoId id, float angle);

#endif /* __PWM_SERVO_H */
