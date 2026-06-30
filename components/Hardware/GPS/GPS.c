#include"GPS.h"

#define GPS_UART_NUM UART_NUM_2

#define GPS_TX_PIN GPIO_NUM_23
#define GPS_RX_PIN GPIO_NUM_15

static uint8_t tmp_rx_buf[BUFF_MAX];

void Data_Transfor(GPS_t* buffer);

void GPS_Init(uint32_t baud){
    // GPS模块初始化代码
    // 例如，配置UART接口、设置波特率等
    uart_config_t gps_uart_config = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(GPS_UART_NUM, &gps_uart_config);
    uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(GPS_UART_NUM, 1024 * 2, 0, 0, NULL, 0);
}

void GPS_Read_Data(GPS_t* buffer){
    int len = uart_read_bytes(GPS_UART_NUM, tmp_rx_buf, BUFF_MAX-1,pdMS_TO_TICKS(20));
    if(len <= 0)return;
    for(int i=0;i<len;i++){
        
        buffer->Frame_Buffer[buffer->buff_length++] = tmp_rx_buf[i];
        if(tmp_rx_buf[i] == '\n'){
            buffer->Frame_Buffer[buffer->buff_length] = '\0';
            buffer->buff_flag = 1;

            char *p_buf  = (char *)buffer->Frame_Buffer;
            char *p_comma, *p_next;
            
            if(strstr(p_buf, "GNRMC") != NULL){
                p_comma = strstr(p_buf, ",");
                if(p_comma == NULL) goto parse_end;
                for(uint8_t idx = 0;idx < 10; idx++){
                    p_comma++;
                    p_next = strstr(p_comma, ",");
                    if(p_next == NULL) break;
                    switch(idx)
                        {
                            case 0: // 时间
                                memcpy(buffer->UTCTime, p_comma, p_next - p_comma);
                                buffer->UTCTime[p_next - p_comma] = '\0';
                                break;
                            case 1: // 定位状态
                                memcpy(buffer->GPS_Status, p_comma, p_next - p_comma);
                                buffer->GPS_Status[p_next - p_comma] = '\0';
                                break;
                            case 2: // 纬度
                                memcpy(buffer->GPS_lat, p_comma, p_next - p_comma);
                                buffer->GPS_lat[p_next - p_comma] = '\0';
                                break;
                            case 3: // 南北
                                memcpy(buffer->GPS_ulat, p_comma, p_next - p_comma);
                                buffer->GPS_ulat[p_next - p_comma] = '\0';
                                break;
                            case 4: // 经度
                                memcpy(buffer->GPS_lon, p_comma, p_next - p_comma);
                                buffer->GPS_lon[p_next - p_comma] = '\0';
                                break;
                            case 5: // 东西
                                memcpy(buffer->GPS_ulon, p_comma, p_next - p_comma);
                                buffer->GPS_ulon[p_next - p_comma] = '\0';
                                break;
                            case 6: // 速度
                                memcpy(buffer->GPS_spd, p_comma, p_next - p_comma);
                                buffer->GPS_spd[p_next - p_comma] = '\0';
                                break;
                            case 7: // 航向
                                memcpy(buffer->GPS_cog, p_comma, p_next - p_comma);
                                buffer->GPS_cog[p_next - p_comma] = '\0';
                                break;
                            case 8: // 日期
                                memcpy(buffer->GPS_date, p_comma, p_next - p_comma);
                                buffer->GPS_date[p_next - p_comma] = '\0';
                                break;
                            default: break;
                        }
                    p_comma = p_next;
                }
                buffer->buff_flag = 1;
                Data_Transfor(buffer);
            }
            
parse_end:
            buffer->buff_length = 0;
        }
    }
    
}

void Data_Transfor(GPS_t* buffer) {
    // 1. 检查数据有效性 ('A' 代表已定位，'V' 代表未定位)
    if (buffer->GPS_Status[0] != 'A') {
        buffer->is_valid = 0; 
        return; // 未定位时，直接退出，不耗费算力
    }
    
    buffer->is_valid = 1;

    // 2. 将字符串提取为 double 浮点数
    double raw_lat = atof(buffer->GPS_lat);
    double raw_lon = atof(buffer->GPS_lon);

    // 3. 纬度转换 (DDMM.MMMMM -> DD.DDDDDD)
    int lat_degrees = (int)(raw_lat / 100);             // 提取度 (DD)
    double lat_minutes = raw_lat - (lat_degrees * 100); // 提取分 (MM.MMMMM)
    buffer->latitude_decimal = lat_degrees + (lat_minutes / 60.0);
    
    // 如果是南半球(S)，纬度取负
    if (buffer->GPS_ulat[0] == 'S') {
        buffer->latitude_decimal = -buffer->latitude_decimal;
    }

    // 4. 经度转换 (DDDMM.MMMMM -> DD.DDDDDD)
    int lon_degrees = (int)(raw_lon / 100);             // 提取度 (DDD)
    double lon_minutes = raw_lon - (lon_degrees * 100); // 提取分
    buffer->longitude_decimal = lon_degrees + (lon_minutes / 60.0);

    // 如果是西半球(W)，经度取负
    if (buffer->GPS_ulon[0] == 'W') {
        buffer->longitude_decimal = -buffer->longitude_decimal;
    }
    
    // 5. 将地面速度 (节) 转换为米/秒 (1 节 = 0.514444 m/s)
    buffer->speed_m_s = atof(buffer->GPS_spd) * 0.514444f;
    
    // 6. 转换航向角
    buffer->course_angle = atof(buffer->GPS_cog);
}