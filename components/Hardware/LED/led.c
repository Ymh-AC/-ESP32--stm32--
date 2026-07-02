// led.c 文件
#include "led.h"


void LED_Init(){
    gpio_config_t gpio_structure={0};
    gpio_structure.intr_type=GPIO_INTR_DISABLE;
    gpio_structure.mode=GPIO_MODE_OUTPUT;
    gpio_structure.pull_up_en=GPIO_PULLUP_DISABLE;
    gpio_structure.pull_down_en=GPIO_PULLDOWN_DISABLE;
    
    //gpio_structure.pin_bit_mask=(1ULL << LED_pin_1)|(1ULL << LED_pin_2)|(1ULL << LED_pin_3);
    gpio_structure.pin_bit_mask=(1ULL << LED_pin_1)|(1ULL << LED_pin_2)|(1ULL << LED_pin_3)|(1ULL << LED_pin_4);
    gpio_config(&gpio_structure);
}

void led_turn_on(LED_struct *LED){
    gpio_set_level(LED->pin,1);
}

void led_turn_off(LED_struct *LED){
    gpio_set_level(LED->pin,0);
}

void led_toggle(LED_struct *LED){
    int level = gpio_get_level(LED->pin);
    gpio_set_level(LED->pin, !level);
}