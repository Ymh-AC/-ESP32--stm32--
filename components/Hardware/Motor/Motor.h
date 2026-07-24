#ifndef __MOTOR_H
#define __MOTOR_H
#include <stdio.h>
#include "driver/gpio.h"
#include "driver/ledc.h"

#define ESC1_PIN    GPIO_NUM_13
#define ESC2_PIN    GPIO_NUM_14
#define ESC3_PIN    GPIO_NUM_18
#define ESC4_PIN    GPIO_NUM_19

#define CH_ESC1   LEDC_CHANNEL_4
#define CH_ESC2   LEDC_CHANNEL_5
#define CH_ESC3   LEDC_CHANNEL_6
#define CH_ESC4   LEDC_CHANNEL_7
#define motor_timer_num   LEDC_TIMER_0

#define MOTOR_MIN_PWM   500    // 1.0ms 反向最大 410 初始约百分之20
#define MOTOR_MID_PWM   614    // 1.5ms 油门中位(解锁)
#define MOTOR_MAX_PWM   819    // 2.0ms 正向满油

typedef struct
{
    ledc_channel_t channel;
    uint16_t duty;
}Motor_Struct;

void Motor_Init(void);
//最大1000，默认0
void Set_Motor_PWM(Motor_Struct *motor);

void start_Motor(Motor_Struct *motor);

void Motor_test(Motor_Struct *motor);

#endif