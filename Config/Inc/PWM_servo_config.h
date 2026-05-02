#ifndef __PWM_SERVO_CONFIG_H
#define __PWM_SERVO_CONFIG_H

#include "tim.h"

/* SG90: 50 Hz, 0.5 ms - 2.5 ms pulse width. */
#define PWM_SERVO_TIM                 htim4

/* PCB设计问题，1、2号舵机不使用 */
#define PWM_SERVO_3_CHANNEL           TIM_CHANNEL_1
#define PWM_SERVO_4_CHANNEL           TIM_CHANNEL_2
#define PWM_SERVO_5_CHANNEL           TIM_CHANNEL_3

#define PWM_SERVO_MAX_ANGLE_DEG       180.0f

#define PWM_SERVO_0_DEG_CMP           25U
#define PWM_SERVO_MAX_DEG_CMP         125U

#endif /* __PWM_SERVO_CONFIG_H */
