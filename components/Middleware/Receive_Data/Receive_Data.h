#ifndef __RECEIVE_DATA_H
#define __RECEIVE_DATA_H

#include "DxLora.h"
#include "Config.h"
#include "String.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "printf_debug.h"
#define MAX_RETRY_COUNT 10

/*接收遥控器的数据
*/
uint8_t Receive_Data(uint8_t *Flag);

/*
* @brief 处理连接状态
* @param res 上一次接收数据的结果 0表示接收到数据 1表示未接收到数据
*/
void Process_connect_state(uint8_t res);

/*
* @brief 处理飞行状态
*/
void Process_flight_state(void);

#endif 

