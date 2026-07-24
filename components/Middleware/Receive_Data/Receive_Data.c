#include "Receive_Data.h"
//连接状态
extern Remote_State remote_state;
//飞行状态
extern Flight_State flight_state;
//遥控数据
extern Remote_Data remote_data;
//接收数据
extern volatile uint8_t buff[64];	
extern volatile uint8_t buf_count;	

//油门解锁状态
Thr_State thr_state=FREE;   

//重试次数
uint8_t retry_count = 0;

//MAX状态进入时间
uint32_t max_enter_time = 0;
//MIN 状态进入时间
uint32_t min_enter_time = 0;


uint8_t Receive_Data(uint8_t *Flag){
    
    while (buf_count >= sizeof(Remote_Data)) {
        
        if (buff[0] == 0xAA) {
            
            Remote_Data *p = (Remote_Data *)buff;    
            
            uint8_t sum = (p->thr ^ p->pit ^ p->rol ^ p->yaw^ p->shutdown^p->fix_hight) & 0xFF;      
            
            if (sum == p->checksum) {  
                *Flag = 0;   
                remote_data.thr = p->thr;      
                remote_data.pit = p->pit;      
                remote_data.rol = p->rol;      
                remote_data.yaw = p->yaw;    
                remote_data.shutdown = p->shutdown;
                remote_data.fix_hight = p->fix_hight;
                
                buf_count -= sizeof(Remote_Data);
                if (buf_count > 0) {
                    memmove((void *)buff, (void *)(buff + sizeof(Remote_Data)), buf_count);
                }
                continue; 
                
            }
        }
        buf_count--;
        if (buf_count > 0) {
            memmove((void *)buff, (void *)(buff + 1), buf_count);
        }
    }
    //debug_printf(":%d,%d,%d,%d\r\n",remote_data.pit,remote_data.rol,remote_data.yaw,remote_data.thr);
    return *Flag;
}


void Process_connect_state(uint8_t res){
    if(res == 0){
        remote_state = REMOTE_CONNECT;
        retry_count = 0;
    }
    else if(res == 1){
        retry_count++;
        if(retry_count >= MAX_RETRY_COUNT){
            remote_state = REMOTE_DISCONNECT;
            retry_count = 0;
        }
        
    }
}

/*
* @brief 处理解锁逻辑
* @return 0表示解锁成功 1表示解锁失败
*/
static uint8_t process_unlock(void){
    switch (thr_state)
    {
    case FREE:
        if(remote_data.thr >= 900){
            //进入MAX状态
            thr_state=MAX;
            max_enter_time=xTaskGetTickCount();
        }
        break;    
    case MAX:
        if(remote_data.thr < 900){
            //判断是否超过1秒
            if((xTaskGetTickCount()-max_enter_time)>=pdMS_TO_TICKS(1000)){
            //进入LEAVE_MAX状态
                thr_state=LEAVE_MAX;
            }
            else{
                //未超过1秒，继续等待
                thr_state=FREE;
            }
        }
        break;
    case LEAVE_MAX:
        if(remote_data.thr <= 100){
            //进入MIN状态
            thr_state=MIN;
            min_enter_time=xTaskGetTickCount();
        }
        break;
    case MIN:
        if(xTaskGetTickCount()-min_enter_time<=pdMS_TO_TICKS(1000)){
            if(remote_data.thr > 100){
                //进入UNLOCK状态
                thr_state=FREE;
            }
        }
        else{
            //超过1秒，解锁完成
            thr_state=UNLOCK;
        }
        break;    
    case UNLOCK:
        /* code */
        break;
    default:
        break;
    }
    if(thr_state==UNLOCK){
        return 0; //解锁成功
    }
    else{
        return 1; //解锁失败
    }
    return 0; //解锁成功
} 

void Process_flight_state(void){
    //使用状态机
    //1.轮询 判断当前状态
    switch(flight_state){
        case IDLE:
            //空闲状态
            if(process_unlock()==0){
                flight_state=NORMAL;
                thr_state=FREE;//解锁成功后，重置油门状态
            }
            break;
        case NORMAL:
            //正常状态
            if(remote_data.fix_hight==1){
                flight_state=FIX_HEIGHT;
                remote_data.fix_hight=0;
            }
            if(remote_state==REMOTE_DISCONNECT){
                flight_state=FAIL;
            }
            break;
        case FIX_HEIGHT:
            //定高状态
            if(remote_data.fix_hight==1){
                flight_state=NORMAL;
                remote_data.fix_hight=0;
            }
            if(remote_state==REMOTE_DISCONNECT){
                flight_state=FAIL;
            }
            break;
        case FAIL:
            //故障状态 缓慢降落 
            flight_state=IDLE;
            vTaskDelay(pdMS_TO_TICKS(1)); //延时1ms
            break;
        default:
            break;
    }
}
