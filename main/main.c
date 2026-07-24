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

TaskHandle_t xPowerTaskHandle = NULL;

// 飞控主循环频率定义
extern const float flight_dt;     // 4ms = 250Hz
extern int flight_LOOP_HZ;

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
    for(int i = 0; i < 750; i++) // 750 * 4ms = 3.0s
    {
        BMI270_Get_Angle_Plus(&IMU_Data); // 以正确的 4ms dt 持续解算
        vTaskDelay(pdMS_TO_TICKS(4));     // 严格保持 250Hz 的物理时序
    }
    // 2. 姿态倾斜零点校准（解决机身不平）
    BMI270_Attitude_Calibrate(&IMU_Data);

    pt2FilterInit(&gyro_filter_pitch, 20.0f, flight_LOOP_HZ);
    pt2FilterInit(&gyro_filter_roll,  20.0f, flight_LOOP_HZ);
    while(1){
        //创建电源任务
        //xTaskCreate(power_Task,"power_Task",1024,NULL,5,&xPowerTaskHandle);
        //创建飞行任务
        xTaskCreate(flight_Task,"flight_Task",2048,NULL,5,NULL);
        //创建LED任务
        xTaskCreate(led_Task,"led_Task",1024,NULL,2,NULL);
        //创捷通信任务
        xTaskCreate(LORA_Rx_Task,"LORA_Rx_Task",2048,NULL,6,NULL);
        //创建按键任务
        //xTaskCreate(Key_Task,"Key_Task",1024,NULL,2,NULL);
        vTaskDelay(portMAX_DELAY);
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