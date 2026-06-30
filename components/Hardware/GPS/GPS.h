#ifndef __GPS_H
#define __GPS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define      BUFF_MAX      526
typedef struct GPS{
	uint8_t  Frame_Buffer[BUFF_MAX];
	uint16_t   buff_length;
	uint8_t  buff_flag;
	char UTCTime[10];     // UTC 时间 hhmmss.ss
	char GPS_Status[10];  // 定位状态 A/V
	char GPS_lat[16];     // 纬度原始串 DDMM.MMMM
	char GPS_ulat[10];    // 纬度方向 N/S
 	char GPS_lon[16];     // 经度原始串 DDDMM.MMMM
	char GPS_ulon[10];    // 经度方向 E/W
	char GPS_spd[12];     // 地面速度
	char GPS_cog[12];     // 航向角
	char GPS_date[12];    // 日期

	uint8_t is_valid;         // 定位状态: 1=已定位, 0=未搜到星
    double latitude_decimal;  // 十进制纬度 (度)
    double longitude_decimal; // 十进制经度 (度)
    float  speed_m_s;         // 速度 (米/秒)
    float  course_angle;      // 航向角 (0-360度)
}GPS_t;

void GPS_Init(uint32_t baud);
void GPS_Read_Data(GPS_t* buffer);

#endif

