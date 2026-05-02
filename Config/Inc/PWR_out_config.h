#ifndef __PWR_OUT_CONFIG_H
#define __PWR_OUT_CONFIG_H

#include "main.h"

#define PWR_OUT_1_GPIO_Port           P_CONTROL1_GPIO_Port
#define PWR_OUT_1_Pin                 P_CONTROL1_Pin

#define PWR_OUT_2_GPIO_Port           P_CONTROL2_GPIO_Port
#define PWR_OUT_2_Pin                 P_CONTROL2_Pin

#define PWR_OUT_ACTIVE_LEVEL          GPIO_PIN_SET
#define PWR_OUT_INACTIVE_LEVEL        GPIO_PIN_RESET

#endif /* __PWR_OUT_CONFIG_H */
