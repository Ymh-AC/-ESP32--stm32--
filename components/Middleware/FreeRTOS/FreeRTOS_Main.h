#ifndef  __FREERTOS_MAIN_H
#define  __FREERTOS_MAIN_H

#include <stdio.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"



void MOTOR_PWM(uint16_t throttle_pwm);

void Servo_pitch_PWM(uint16_t pitch_PWM); 

void Servo_roll_PWM(uint16_t roll_pwm);

void RC_Task(void *arg);

void Control_Task_IMU(void *arg);
void Control_Task_Man(void *arg);
void GPS_Task(void *arg);
#endif

