// led.c 文件
#include "led.h"
#include "driver/gpio.h"

void LED_Init(){
    gpio_config_t gpio_structure;
    gpio_structure.intr_type=GPIO_INTR_DISABLE;
    gpio_structure.mode=GPIO_MODE_OUTPUT;
    gpio_structure.pull_up_en=GPIO_PULLUP_DISABLE;
    gpio_structure.pull_down_en=GPIO_PULLDOWN_DISABLE;
    
    gpio_structure.pin_bit_mask=1ULL << GPIO_NUM_2;

    gpio_config(&gpio_structure);
}