#include <stdio.h>
#include "driver/gpio.h"
#include "driver/ledc.h"

#define Timer_num   LEDC_TIMER_1 

#define Servo_GPIO1 GPIO_NUM_25
#define Servo_GPIO2 GPIO_NUM_26
#define Servo_GPIO3 GPIO_NUM_12
#define Servo_GPIO4 GPIO_NUM_27

#define CH1   LEDC_CHANNEL_0
#define CH2   LEDC_CHANNEL_1
#define CH3   LEDC_CHANNEL_2
#define CH4   LEDC_CHANNEL_3

void Set_Angel(uint8_t Channel, uint8_t angel);

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

    Set_Angel(0, 90);       // 右副翼
    Set_Angel(1, 90);       // 左副翼   
    Set_Angel(2, 90);       // 左主升降舵
    Set_Angel(3, 90);       // 主升降舵

}

//0°=1638，180°=8190
void Set_Angel(uint8_t Channel, uint8_t angel) {
    if(angel>140) angel=140;
    else if(angel<40) angel=40;
    
    uint32_t duty =1638 + (angel * (8191-1638)/180U);
    ledc_channel_t ch;
    switch (Channel)
    {
    case 0: ch=CH1; break;//左横滚
    case 1: ch=CH2; break;//右横滚
    case 2: ch=CH3; break;//俯仰
    case 3: ch=CH4; break;//俯仰     
    default:    
        return;
    }
    ledc_set_duty(LEDC_LOW_SPEED_MODE,ch,duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE,ch);
}