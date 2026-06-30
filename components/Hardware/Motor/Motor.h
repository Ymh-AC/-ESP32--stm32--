#ifndef __MOTOR_H
#define __MOTOR_H
#include <stdio.h>
#include "driver/gpio.h"
#include "driver/ledc.h"

void Motor_Init(void);

void Set_Motor(uint16_t duty);

#endif