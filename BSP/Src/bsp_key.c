#include "main.h"
#include "bsp_key.h"

static KeyState key_state_stable = KEY_UNPRESSED;

KeyState Key_Read(void)
{
    if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin) == GPIO_PIN_SET)return KEY_PRESSED;
    else return KEY_UNPRESSED;
}

void Key_Check(void)
{
    static uint8_t count;
    static KeyState key_state = KEY_UNPRESSED;
    static KeyState key_state_last = KEY_UNPRESSED;

    key_state = Key_Read();
    
    if(key_state != key_state_last)count = 0;//按钮状态不同，开始计数
    else 
    {
        if(count < STABLE_COUNT)count ++;
        else key_state_stable = key_state;
    }

    key_state_last = key_state;
}

KeyState Key_GetStable(void)
{
    return key_state_stable;
}