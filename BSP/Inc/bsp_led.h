#ifndef __BSP_LED_H
#define __BSP_LED_H

typedef enum 
{
    LED_ON,
    LED_OFF
}LEDState;

void LED_On(void);
void LED_Off(void);
void LED_Toggle(void);
LEDState LED_State(void);

#endif