#ifndef  __SERVO_H
#define  __SERVO_H

#include <stdio.h>
#include "driver/gpio.h"
#include "driver/ledc.h"

#define Timer_num   LEDC_TIMER_1 

#define Servo_GPIO1 GPIO_NUM_25
#define Servo_GPIO2 GPIO_NUM_26
#define Servo_GPIO3 GPIO_NUM_12
#define Servo_GPIO4 GPIO_NUM_27

#define CH1 LEDC_CHANNEL_0
#define CH2 LEDC_CHANNEL_1
#define CH3 LEDC_CHANNEL_2
#define CH4 LEDC_CHANNEL_3

typedef struct
{ 
    ledc_channel_t channel;
    uint16_t angle;
}Servo_Struct;


void Servo_Init(void);

void Set_Angle(Servo_Struct *servo);//angel 0-180 


#endif