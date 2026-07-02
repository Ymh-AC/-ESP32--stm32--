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


//重试次数
uint8_t retry_count = 0;

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


static uint8_t process_unlock(void){

} 

void Process_flight_state(void){
    //使用状态机
    //1.轮询 判断当前状态
    switch(flight_state){
        case IDLE:
            //空闲状态
            if(remote_data.fix_hight==1){
                flight_state=FIX_HEIGHT;
            }
            else{
                flight_state=NORMAL;
            }
            break;
        case NORMAL:
            //正常状态
            if(remote_data.fix_hight==1){
                flight_state=FIX_HEIGHT;
            }
            else{
                flight_state=NORMAL;
            }
            break;
        case FIX_HEIGHT:
            //定高状态
            if(remote_data.fix_hight==0){
                flight_state=NORMAL;
            }
            else{
                flight_state=FIX_HEIGHT;
            }
            break;
        case FAIL:
            //故障状态
            flight_state=FAIL;
            break;
        default:
            flight_state=IDLE;
    }
}
