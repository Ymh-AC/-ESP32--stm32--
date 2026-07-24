#ifndef  __FREERTOS_MAIN_H
#define  __FREERTOS_MAIN_H

#include <stdio.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Servo.h"
#include "Motor.h"
#include "filter.h"
#include "DXLora.h"
#include "IMU.h"
#include "led.h"
#include "Config.h"
#include "Key.h"
#include "Printf_Debug.h"
#include "Receive_Data.h"
#include "Flight.h"



void power_Task(void *arg);
void LORA_Rx_Task(void *arg);
void flight_Task(void *arg);
void led_Task(void *arg);
void Key_Task(void *arg);


#endif

