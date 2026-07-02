#ifndef __KEY_H
#define __KEY_H

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#define Key_up_gpio GPIO_NUM_1
#define Key_down_gpio GPIO_NUM_3
#define Key_left_gpio GPIO_NUM_5
#define Key_right_gpio GPIO_NUM_32
#define Key_left_x_gpio GPIO_NUM_33
#define Key_right_x_gpio GPIO_NUM_34

typedef enum{
    KEY_NONE=0,
    KEY_UP,
    KEY_DOWN,
    KEY_LEFT,
    KEY_RIGHT,
    KEY_LEFT_X,
    KEY_RIGHT_X,
    KEY_RIGHT_X_LONG,
}Key_Type;

 

void Key_Init(void);

Key_Type Key_Get(void);

#endif 
