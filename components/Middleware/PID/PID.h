#ifndef __PID_H
#define __PID_H

#include <stdio.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"



typedef struct {
    float Kp;           
    float Ki;           
    float Kd;           
    
    float target;
    
    float error;        
    float integral;     
    float last_measurement; 
    
    float out_max;
    float integral_max;
    float integral_separation_err; 
} PID_Controller;


void PID_Init(PID_Controller *pid, float p, float i, float d, float out_max);

float PID_Compute(PID_Controller *pid, float current_angle, float dt);//PID 计算函数

#endif

