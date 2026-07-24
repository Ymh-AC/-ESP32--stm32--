
#include "MOTOR.h"


void Motor_Init(void)
{
    // 1. 先配置GPIO
    gpio_config_t gpio_structure;
    gpio_structure.pin_bit_mask=(1ULL<<ESC1_PIN)|(1ULL<<ESC2_PIN)|(1ULL<<ESC3_PIN)|(1ULL<<ESC4_PIN);
    gpio_structure.mode=GPIO_MODE_OUTPUT;
    gpio_structure.pull_up_en=GPIO_PULLUP_DISABLE;
    gpio_structure.pull_down_en=GPIO_PULLDOWN_DISABLE;
    gpio_structure.intr_type=GPIO_INTR_DISABLE;
    gpio_config(&gpio_structure);

    // 2. 配置LEDC定时器
    ledc_timer_config_t ledc_structure= {0};
    ledc_structure.speed_mode=LEDC_LOW_SPEED_MODE;
    ledc_structure.duty_resolution=LEDC_TIMER_13_BIT;
    ledc_structure.timer_num=motor_timer_num;
    ledc_structure.freq_hz=50;
    ledc_structure.clk_cfg=LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_structure);


    // 3. 再配置LEDC通道（上电直接输出中位）
    ledc_channel_config_t ch0={0};
    ch0.channel=CH_ESC1;
    ch0.duty=MOTOR_MID_PWM;
    ch0.gpio_num=ESC1_PIN;
    ch0.speed_mode=LEDC_LOW_SPEED_MODE;
    ch0.timer_sel = motor_timer_num;
    ledc_channel_config(&ch0);

    ledc_channel_config_t ch1={0};
    ch1.channel=CH_ESC2;
    ch1.duty=MOTOR_MID_PWM;
    ch1.gpio_num=ESC2_PIN;
    ch1.speed_mode=LEDC_LOW_SPEED_MODE;
    ch1.timer_sel = motor_timer_num;
    ledc_channel_config(&ch1);

    ledc_channel_config_t ch2={0};
    ch2.channel=CH_ESC3;
    ch2.duty=MOTOR_MID_PWM;
    ch2.gpio_num=ESC3_PIN;
    ch2.speed_mode=LEDC_LOW_SPEED_MODE;
    ch2.timer_sel = motor_timer_num;
    ledc_channel_config(&ch2);

    ledc_channel_config_t ch3={0};
    ch3.channel=CH_ESC4;
    ch3.duty=MOTOR_MID_PWM;
    ch3.gpio_num=ESC4_PIN;
    ch3.speed_mode=LEDC_LOW_SPEED_MODE;
    ch3.timer_sel = motor_timer_num;
    ledc_channel_config(&ch3);
}

// 增加限幅，防止数值越界
void Set_Motor_PWM(Motor_Struct *motor)
{
    if(motor->duty > 1000)
    {
        motor->duty = 1000;
    }
    else if(motor->duty < 0)
    {
        motor->duty = 0;
    }
    motor->duty=(MOTOR_MAX_PWM-MOTOR_MIN_PWM)*(motor->duty-600)/400+MOTOR_MIN_PWM;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel,motor->duty); 
    
}

void start_Motor(Motor_Struct *motor)
{
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel);
}

void Motor_test(Motor_Struct *motor){
    ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel,motor->duty); 
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel); 
}