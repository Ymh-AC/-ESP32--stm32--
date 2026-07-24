#include"FreeRTOS_Main.h"

extern volatile uint16_t TxData[4];
extern volatile uint16_t RxData[4];

extern IMU_DATA IMU_Data;

//内存管理=>c语言结构体一般放在堆里面，不会自动回收=>所以循环使用同一个结构体

//LED结构体
LED_struct LED_1={.pin=LED_pin_1};
LED_struct LED_2={.pin=LED_pin_2};
LED_struct LED_3={.pin=LED_pin_3};
//LED_struct LED_4={.pin=LED_pin_4};

//当前连接遥控状态
Remote_State remote_state=REMOTE_CONNECT;
//当前飞行状态
Flight_State flight_state=IDLE;
//遥控数据
Remote_Data remote_data={.thr=0,.yaw=500,.pit=500,.rol=500,.fix_hight=0,.shutdown=0};


extern TaskHandle_t xPowerTaskHandle;

//飞控频率
const float flight_dt = 0.004f;     // 4ms = 250Hz
const int flight_LOOP_HZ = 250;


void power_Task(void *arg){
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(1000); // 1Hz 1000ms 精准周期
    xLastWakeTime = xTaskGetTickCount(); 
    while(1){

        uint32_t res = ulTaskNotifyTake(pdTRUE, xPeriod);
        if(res != 0){
            //执行关机操作
            
        }
        else{
            //没有收到关机通知，继续正常操作
            
        }
    }
}

void flight_Task(void *arg){
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(4); // 250Hz 4ms 精准周期
    xLastWakeTime = xTaskGetTickCount(); 
    while(1){
        //读取IMU数据
        BMI270_Get_Angle_Plus(&IMU_Data);
        //debug_printf(":%.2f,%.2f,%.2f\r\n", IMU_Data.pitch, IMU_Data.roll, IMU_Data.yaw);
        //
        //计算PID控制输出
        flight_pid_process();
        
        //根据PID结果对电机进行控制
        flight_control_motor();

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

void LORA_Rx_Task(void *arg){
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(10); // 100Hz 10ms 精准周期
    xLastWakeTime = xTaskGetTickCount(); 
    
    while(1){
        //接收数据
        uint8_t res=LORA_Task();
        
        //处理连接状态
        Process_connect_state(res);
        //处理关机逻辑
        if(remote_data.shutdown==1){
            xTaskNotifyGive(xPowerTaskHandle);// 通知电源任务执行关机操作
        }
        //处理飞机状态
        Process_flight_state();

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}


void Key_Task(void *arg){
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(20); // 50Hz 10ms 精准周期
    xLastWakeTime = xTaskGetTickCount();    
    while(1){
        Key_Type key=Key_Get();
        if(key!=KEY_NONE){
            debug_printf("key=%d\n",key);
        }
        else{
            debug_printf("key=none\n");
        }
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

void led_Task(void *arg){
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(100); // 10hz 100ms 精准周期
    xLastWakeTime = xTaskGetTickCount(); 
    uint8_t count=0;
    while(1){
        count++;
        //判断遥控器连接状态
        if(remote_state==REMOTE_CONNECT)led_turn_on(&LED_1);//连接上LED亮
        else led_turn_off(&LED_1);

        //飞行状态
        if(flight_state==IDLE){if(count%5==0)led_toggle(&LED_2);}//慢闪
        else if(flight_state==NORMAL){if(count%2==0)led_toggle(&LED_1);}//快闪
        else if(flight_state==FIX_HEIGHT){led_turn_on(&LED_2);}//led2亮
        else if(flight_state==FAIL){led_turn_off(&LED_2);}//led2灭

        if(count==10)count=0;
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}
