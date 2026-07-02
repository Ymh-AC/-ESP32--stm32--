#ifndef __LED_H
#define __LED_H

#include "driver/gpio.h"

#define LED_pin_1 GPIO_NUM_0
#define LED_pin_2 GPIO_NUM_2
#define LED_pin_3 GPIO_NUM_4
#define LED_pin_4 GPIO_NUM_34

typedef struct{
    gpio_num_t pin;
}LED_struct;


void LED_Init();
void led_turn_on(LED_struct *LED);
void led_turn_off(LED_struct *LED);
void led_toggle(LED_struct *LED);
#endif
