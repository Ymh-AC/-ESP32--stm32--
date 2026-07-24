#ifndef __PID_H
#define __PID_H

#include <stdio.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "FreeRTOS_Main.h"


extern const float flight_dt;
#define PID_PERIOD flight_dt


//可以换为double
//kp,ki,kd需要在初始化时确定，目标值和测量值需要在计算时传递
typedef struct {
    double Kp;       //比例 响应速度
    double Ki;       //积分 稳态误差 一般不使用
    double Kd;       //微分 过调震荡    
    
    double desire;   //目标角度
    double err;      //误差值   
    double measure;  //测量值
    double last_err; //上一次误差值   

    double integral; //积分累计

    double output;   //输出结果
} PID_Struct;

/*
*单次PID
*/
void PID_Calc(PID_Struct *pid);

/*
*串级PID
*/
void PID_Calc_Chain(PID_Struct *out_pid,PID_Struct *in_pid);



#endif

