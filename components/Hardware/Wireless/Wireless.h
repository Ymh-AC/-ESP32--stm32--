#ifndef WIRELESS_H
#define WIRELESS_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// WiFi 配置
#define WIFI_SSID       "Xiaomi_13_Ultra"
#define WIFI_PASSWD     "12345678"

// OneNET MQTT 配置
#define MQTT_BROKER     "mqtt://mqtts.heclouds.com:1883"
#define PRODUCT_ID      "0n7kH7J14x"
#define DEVICE_ID       "ESP32"
#define DEVICE_TOKEN    "version=2018-10-31&res=products%2F0n7kH7J14x%2Fdevices%2FESP32&et=1910271117&method=md5&sign=fZcZKPLX57SHvphp%2Fhauhg%3D%3D"

// OneNET 主题
#define POST_TOPIC      "$sys/" PRODUCT_ID "/" DEVICE_ID "/thing/property/post"
#define POST_REPLY_TOPIC "$sys/" PRODUCT_ID "/" DEVICE_ID "/thing/property/post/reply"
#define SET_TOPIC       "$sys/" PRODUCT_ID "/" DEVICE_ID "/thing/property/set"

// 姿态数据缓冲区
extern char g_json_data[256];

void Wireless_Init(void);
void mqtt_app_start(void);
void Build_Attitude_JSON(char *buf, size_t buf_len, double roll, double pitch);

#endif /* WIRELESS_H */