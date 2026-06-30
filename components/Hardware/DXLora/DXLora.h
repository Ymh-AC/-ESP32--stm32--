#ifndef DXLORA_H
#define DXLORA_H

#include <stdint.h>

void DXLora_Init(uint32_t baud);
void USART1_SEND(uint8_t *data, uint16_t len);

extern volatile uint8_t buff[64];
extern volatile uint8_t buf_count;
extern volatile uint8_t rx_flag;

void DXLora_Init(uint32_t baud);
void LORA_SEND(uint8_t* data, uint16_t length);
void LORA_Rx_Task(void *pvParameters);
#endif