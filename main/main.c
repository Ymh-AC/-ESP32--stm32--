// main.c 文件
#include <stdio.h>
#include <string.h>
#include <stdarg.h> 
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "DXLora.h"
#include "Motor.h"
#include "Servo.h"
#include "FreeRTOS_Main.h"
#include "IMU.h"
#include "Wireless.h"
#include "filter.h"
#include "GPS.h"
#include "Printf_Debug.h"
#include "Receive_Data.h"

volatile uint16_t TxData[4];//发送值
volatile uint16_t RxData[4];//接收值

volatile uint16_t Last_RxData[4];//上传接受值
#define ONENET_UPLOAD_TOPIC "$sys/0n7kH7J14x/ESP32/thing/property/post/json"

extern int16_t gyro_offset_x ;
extern int16_t gyro_offset_y ;
extern int16_t gyro_offset_z ;


 // 
IMU_DATA IMU_Data;
GPS_t gps_data;
// 滤波器
pt2Filter_t gyro_filter_pitch;
pt2Filter_t gyro_filter_roll;
// 飞控主循环频率定义
const float DT = 0.01f;     // 10ms = 100Hz
const int LOOP_HZ = 200;

void app_main(void)
{
    Servo_Init();
    Motor_Init();
    GPS_Init(9600);
    DXLora_Init(115200);

    esp_err_t init_err = bmi270_full_init(&IMU_Data);
    if (init_err != ESP_OK) {
        debug_printf("BMI270 initialization failed! Error: 0x%x\r\n", init_err);
    } else {
        debug_printf("BMI270 initialized successfully!\r\n");
    }
    // 1. 陀螺仪零偏校准
    BMI270_Calibrate(&IMU_Data);
    vTaskDelay(pdMS_TO_TICKS(500));
    // 2. 姿态倾斜零点校准（解决机身不平）
    BMI270_Attitude_Calibrate(&IMU_Data);

    pt2FilterInit(&gyro_filter_pitch, 20.0f, LOOP_HZ);
    pt2FilterInit(&gyro_filter_roll,  20.0f, LOOP_HZ);
    
    Start_Task();
    
    while(1){
        
    }
}

void rx_control(uint8_t *Flag)	
{
    
    // while (buf_count >= sizeof(RC_Data_t)) {
        
    //     if (buff[0] == 0xAA) {
            
    //         RC_Data_t *p = (RC_Data_t *)buff;    
            
    //         uint8_t sum = (p->throttle ^ p->pitch ^ p->roll ^ p->yaw^ p->shutdown^p->fix_hight) & 0xFF;      
            
    //         if (sum == p->checksum) {  
                   
    //             RxData[0] = p->throttle;      
    //             RxData[1] = p->pitch;      
    //             RxData[2] = p->roll;      
    //             RxData[3] = p->yaw;    
    //             RxData_KEY[0] = p->shutdown;
    //             RxData_KEY[1] = p->fix_hight;
                
    //             buf_count -= sizeof(RC_Data_t);
    //             if (buf_count > 0) {
    //                 memmove((void *)buff, (void *)(buff + sizeof(RC_Data_t)), buf_count);
    //             }
    //             continue; 
    //         }
    //     }
    //     buf_count--;
    //     if (buf_count > 0) {
    //         memmove((void *)buff, (void *)(buff + 1), buf_count);
    //     }
    // }
}