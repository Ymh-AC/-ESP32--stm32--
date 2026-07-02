#ifndef __CONFIG_H
#define __CONFIG_H

#include <stdio.h>

//连接状态
typedef enum{
    REMOTE_CONNECT=0,//正常连接
    REMOTE_DISCONNECT,//失联
}Remote_State;

//飞机状态
typedef enum{
    IDLE=0,//空闲
    NORMAL,//正常
    FIX_HEIGHT,//定高
    FAIL,//故障
}Flight_State;


// 定义结构体
typedef struct {
    uint8_t  header;    // 0xAA
    uint16_t thr;  
    uint16_t pit;  
    uint16_t rol;  
    uint16_t yaw;  
    uint8_t shutdown;//1:关机
    uint8_t fix_hight;//1：定高度
    uint8_t  checksum;   // 简单校验
}__attribute__((packed))Remote_Data;//遥控器数据

#endif