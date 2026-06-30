
#include "MOTOR.h"

#define ESC1_PIN    GPIO_NUM_13
#define ESC2_PIN    GPIO_NUM_14
#define ESC3_PIN    GPIO_NUM_18
#define ESC4_PIN    GPIO_NUM_19

#define CH_ESC1   LEDC_CHANNEL_4
#define CH_ESC2   LEDC_CHANNEL_5
#define CH_ESC3   LEDC_CHANNEL_6
#define CH_ESC4   LEDC_CHANNEL_7
#define Timer_num   LEDC_TIMER_0

#define MOTOR_MIN_PWM   410    // 1.0ms 反向最大
#define MOTOR_MID_PWM   614    // 1.5ms 油门中位(解锁)
#define MOTOR_MAX_PWM   819    // 2.0ms 正向满油

void Motor_Init(void)
{
    // 1. 先配置GPIO
    gpio_config_t gpio_structure;
    gpio_structure.pin_bit_mask=(1ULL<<ESC1_PIN)|(1ULL<<ESC2_PIN);
    gpio_structure.mode=GPIO_MODE_OUTPUT;
    gpio_structure.pull_up_en=GPIO_PULLUP_DISABLE;
    gpio_structure.pull_down_en=GPIO_PULLDOWN_DISABLE;
    gpio_structure.intr_type=GPIO_INTR_DISABLE;
    gpio_config(&gpio_structure);

    // 2. 配置LEDC定时器
    ledc_timer_config_t ledc_structure= {0};
    ledc_structure.speed_mode=LEDC_LOW_SPEED_MODE;
    ledc_structure.duty_resolution=LEDC_TIMER_13_BIT;
    ledc_structure.timer_num=Timer_num;
    ledc_structure.freq_hz=50;
    ledc_structure.clk_cfg=LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_structure);


    // 3. 再配置LEDC通道（上电直接输出中位）
    ledc_channel_config_t ch0={0};
    ch0.channel=CH_ESC1;
    ch0.duty=MOTOR_MID_PWM;
    ch0.gpio_num=ESC1_PIN;
    ch0.speed_mode=LEDC_LOW_SPEED_MODE;
    ch0.timer_sel = Timer_num;
    ledc_channel_config(&ch0);

    ledc_channel_config_t ch1={0};
    ch1.channel=CH_ESC2;
    ch1.duty=MOTOR_MID_PWM;
    ch1.gpio_num=ESC2_PIN;
    ch1.speed_mode=LEDC_LOW_SPEED_MODE;
    ch1.timer_sel = Timer_num;
    ledc_channel_config(&ch1);

    
}

// 增加限幅，防止数值越界
void Set_Motor(uint16_t duty)
{
    uint32_t temp = duty;
    if(temp<1750)temp=1750;
    else if(temp>2900)temp=2900;
    temp=(temp - 1750)*(MOTOR_MAX_PWM-MOTOR_MIN_PWM)/(2900-1750)+MOTOR_MIN_PWM; // 将输入的PWM值映射到实际的ESC控制范围
    ledc_set_duty(LEDC_LOW_SPEED_MODE,CH_ESC2,temp);
    ledc_update_duty(LEDC_LOW_SPEED_MODE,CH_ESC2);  
    ledc_set_duty(LEDC_LOW_SPEED_MODE,CH_ESC1,temp);
    ledc_update_duty(LEDC_LOW_SPEED_MODE,CH_ESC1);
    
}