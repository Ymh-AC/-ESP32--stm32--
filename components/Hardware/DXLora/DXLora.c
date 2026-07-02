#include <string.h>
#include <stdio.h>
#include "DXLora.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "Receive_Data.h"

#define LORA_NUM        UART_NUM_1

#define LORA_TX         GPIO_NUM_17
#define LORA_RX         GPIO_NUM_16
#define LORA_M0         GPIO_NUM_4
#define LORA_M1         GPIO_NUM_5
#define LORA_AUX        GPIO_NUM_34

#define RX_BUF_LEN      64

volatile uint8_t buff[RX_BUF_LEN];
volatile uint8_t buf_count = 0;

extern void rx_control(uint8_t *Flag);

static QueueHandle_t uart_queue = NULL;

void DXLora_Init(uint32_t baud)
{
    // 配置 M0, M1 输出
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << LORA_M0) | (1ULL << LORA_M1),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_cfg);
    gpio_set_level(LORA_M0, 0);
    gpio_set_level(LORA_M1, 0);

    // 配置 AUX 输入
    io_cfg.pin_bit_mask = (1ULL << LORA_AUX);
    io_cfg.mode = GPIO_MODE_INPUT;
    gpio_config(&io_cfg);


    uart_config_t uart_cfg = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB
    };
    uart_param_config(LORA_NUM, &uart_cfg);
    uart_set_pin(LORA_NUM, LORA_TX, LORA_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // 安装驱动，创建事件队列（队列长度 20）
    uart_driver_install(LORA_NUM, 1024, 1024, 20, &uart_queue, 0);

    // 设置接收阈值为 10 字节，超时 1 字符时间
    uart_set_rx_full_threshold(LORA_NUM, 10);
    uart_set_rx_timeout(LORA_NUM, 10);
    uart_enable_rx_intr(LORA_NUM);
}

void LORA_SEND(uint8_t *data, uint16_t length)
{
    uart_write_bytes(LORA_NUM, (const char *)data, length);
}

uint8_t LORA_Task(void)
{
    uint8_t Flag = 1;//表示是否接收到数据 0接收到 1为收到
    uart_event_t event;
    int len;

    while (1) {
        if (xQueueReceive(uart_queue, &event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    len = uart_read_bytes(LORA_NUM, (uint8_t *)&buff[buf_count], RX_BUF_LEN - buf_count, 0);
                    if (len > 0) {  
                        
                        buf_count += len;
                        Receive_Data(&Flag);
                    }
                    break;
                case UART_FIFO_OVF:
                case UART_BUFFER_FULL:
                    uart_flush_input(LORA_NUM);
                    buf_count = 0;
                    break;
                default:
                    break;
            }
        }

        size_t buffered = 0;
        uart_get_buffered_data_len(LORA_NUM, &buffered);
        if (buffered == 0 && buf_count > 0) {
            Receive_Data(&Flag);
        }
    }
    return Flag;
}