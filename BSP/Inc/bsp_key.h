#ifndef __BSP_KEY_H
#define __BSP_KEY_H

typedef enum 
{
    KEY_PRESSED,
    KEY_UNPRESSED
}KeyState;

#define STABLE_COUNT 5

KeyState Key_Read(void);
void Key_Check(void);
KeyState Key_GetStable(void);

#endif