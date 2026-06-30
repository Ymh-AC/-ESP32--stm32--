#ifndef  __SERVO_H
#define  __SERVO_H


#include <stdio.h>
#include "driver/gpio.h"
#include "driver/ledc.h"

void Servo_Init(void);

void Set_Angel(uint8_t Channel, uint8_t angel);//angel 0-180 


#endif