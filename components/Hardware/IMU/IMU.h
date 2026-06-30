#ifndef __IMU_H
#define __IMU_H


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

typedef struct
{
    // 原始整型数据
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t rawTemp;

    // 物理量（g / dps）
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;

    // 欧拉角
    float yaw;
    float roll;
    float pitch;
    float temp;

    // 四元数
    float q0;
    float q1;
    float q2;
    float q3;
}IMU_DATA;

/*===========常用函数===========*/
esp_err_t bmi270_full_init(IMU_DATA *_this);
esp_err_t bmi270_read_data(IMU_DATA *this);
void BMI270_Get_Angle_Plus(IMU_DATA* _this);
void BMI270_Calibrate(IMU_DATA *_this);
void BMI270_Attitude_Calibrate(IMU_DATA *_this);
/*===========不常用函数========*/
esp_err_t BMI270_Init(void);
esp_err_t bmi270_check_chip_id(void);
esp_err_t bmi270_init_device(IMU_DATA *_this);
esp_err_t BMI270_Acc_config(void);
esp_err_t BMI270_Gyr_config(void);
esp_err_t bmi270_enable_sensors(void);


#endif

