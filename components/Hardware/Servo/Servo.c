#include <stdio.h>
#include "servo.h"
#include "driver/gpio.h"
#include "driver/ledc.h"


void Set_Angle(Servo_Struct *servo);

void Servo_Init(void){
    gpio_config_t Servo_Init={0};
    Servo_Init.mode=GPIO_MODE_OUTPUT;
    Servo_Init.pin_bit_mask=(1uLL<<Servo_GPIO1)|(1uLL<<Servo_GPIO2)|(1uLL<<Servo_GPIO3)|(1uLL<<Servo_GPIO4);
    Servo_Init.pull_down_en=GPIO_PULLDOWN_DISABLE;
    Servo_Init.pull_up_en= GPIO_PULLUP_DISABLE;
    Servo_Init.intr_type=GPIO_INTR_DISABLE;
    gpio_config(&Servo_Init);

    ledc_timer_config_t Servo_ledc={0};
    Servo_ledc.clk_cfg=LEDC_AUTO_CLK;
    Servo_ledc.duty_resolution=LEDC_TIMER_16_BIT;
    Servo_ledc.freq_hz=50;
    Servo_ledc.speed_mode=LEDC_LOW_SPEED_MODE;
    Servo_ledc.timer_num=Timer_num;
    ledc_timer_config(&Servo_ledc); 

    ledc_channel_config_t Servo_channel={0};
    Servo_channel.timer_sel=Timer_num;
    Servo_channel.speed_mode=LEDC_LOW_SPEED_MODE;
    Servo_channel.intr_type=LEDC_INTR_DISABLE;

    Servo_channel.duty=0;
    Servo_channel.gpio_num=Servo_GPIO1;
    Servo_channel.channel=CH1;
    ledc_channel_config(&Servo_channel);

    Servo_channel.duty=0;
    Servo_channel.gpio_num=Servo_GPIO2;
    Servo_channel.channel=CH2;
    ledc_channel_config(&Servo_channel);

    Servo_channel.duty=0;
    Servo_channel.gpio_num=Servo_GPIO3;
    Servo_channel.channel=CH3;
    ledc_channel_config(&Servo_channel);

    Servo_channel.duty=0;
    Servo_channel.gpio_num=Servo_GPIO4;
    Servo_channel.channel=CH4;
    ledc_channel_config(&Servo_channel);


}



//0°=1638，180°=8190
void Set_Angle(Servo_Struct *servo){
    if(servo->angle>140) servo->angle=140;
    else if(servo->angle<40) servo->angle=40;
    
    uint32_t duty =1638 + (servo->angle * (8191-1638)/180U);

    ledc_set_duty(LEDC_LOW_SPEED_MODE,servo->channel,duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE,servo->channel);
}