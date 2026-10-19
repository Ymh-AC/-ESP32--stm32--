#include "Flight.h"

//俯仰角PID结构体  
PID_Struct pitch_pid={.Kp=0.00,.Ki=0.00,.Kd=0.00};
//Y轴角速度结构体  俯仰角内环
PID_Struct  gryo_y_pid={.Kp=1.00,.Ki=0.00,.Kd=0.00};
//横滚角PID结构体 
PID_Struct roll_pid={.Kp=0.00,.Ki=0.00,.Kd=0.00};
//Y轴角速度结构体  横滚角内环
PID_Struct  gryo_x_pid={.Kp=1.00,.Ki=0.00,.Kd=0.00};
extern Remote_Data remote_data;
extern IMU_DATA IMU_Data;

extern float GYR_RANGE_DATA;

extern float bmi270_gyroScale; 
extern float bmi270_accelScale;

extern int16_t gyro_offset_y;
extern int16_t gyro_offset_x;

extern Flight_State flight_state;

//电机结构体
Motor_Struct left_top_motor={.channel=CH_ESC1,.duty=0};
Motor_Struct left_bottom_motor={.channel=CH_ESC4,.duty=0};
Motor_Struct right_top_motor={.channel=CH_ESC2,.duty=0};
Motor_Struct right_bottom_motor={.channel=CH_ESC3,.duty=0};

//舵机结构体
Servo_Struct    roll_left_servo={.channel=CH1,.angle=90};
Servo_Struct    roll_right_servo={.channel=CH2,.angle=90};
Servo_Struct    pitch_left_servo={.channel=CH3,.angle=90};   
Servo_Struct    pitch_right_servo={.channel=CH4,.angle=90};

void flight_pid_process(void){

    //1，赋值目标值和测量值
    //debug_printf(":%.2f,%.2f\r\n", IMU_Data.pitch, gyro_y_physical);

    //remote_data.pit(0-1000) 500为中心点 控制范围±10°
    pitch_pid.desire=(remote_data.pit-500)/50;   

    //内环测量值
    pitch_pid.measure=IMU_Data.pitch;

    //外环测量值
    gryo_y_pid.measure = (IMU_Data.gyro_y - gyro_offset_y) / GYR_RANGE_DATA;
    //gryo_y_pid.measure = (IMU_Data.gyro_y) / GYR_RANGE_DATA;
    
    //2，进行PID的计算
    PID_Calc_Chain(&pitch_pid,&gryo_y_pid);
    //debug_printf(":%.2f,%.2f\r\n", pitch_pid.err,gryo_y_pid.output);
    //debug_printf(":%.2f\r\n", pitch_pid.measure);
    
    // ========== 2. 横滚Roll串级PID（机身左右侧倾）==========
    // 遥控器横滚通道 中点500 → ±10°横滚角范围
    roll_pid.desire = (remote_data.rol - 500) / 50.0f;
    roll_pid.measure = IMU_Data.roll;                    // 外环：横滚角度
    gryo_x_pid.measure = (IMU_Data.gyro_x - gyro_offset_x) / GYR_RANGE_DATA;  //内环：X轴横滚角速度
    PID_Calc_Chain(&roll_pid, &gryo_x_pid);
}

void flight_control_motor(void){
    switch (flight_state)
    {
    case IDLE:
        //一旦进入加锁状态
        left_top_motor.duty=0;
        left_bottom_motor.duty=0;
        right_top_motor.duty=0;
        right_bottom_motor.duty=0;
        break;
    case NORMAL:
        //俯仰角向前飞 => 负误差 => 需要向后的反馈 => 前两个电机快
        //前面两个电机 减去 负误差
        left_top_motor.duty     =remote_data.thr - gryo_y_pid.output;
        left_bottom_motor.duty  =remote_data.thr + gryo_y_pid.output;
        right_top_motor.duty    =remote_data.thr - gryo_y_pid.output;
        right_bottom_motor.duty =remote_data.thr + gryo_y_pid.output;

        roll_left_servo.angle     =90- gryo_x_pid.output;
        roll_right_servo.angle    =90+ gryo_x_pid.output;
        pitch_left_servo.angle    =90+ gryo_y_pid.output;
        pitch_right_servo.angle   =90+ gryo_y_pid.output;

        break;
    case FIX_HEIGHT:
        
        break;
    case FAIL:
        
        break;
        
    default:
        break;
    }
        

    Set_Motor_PWM(&left_top_motor);
    Set_Motor_PWM(&left_bottom_motor);
    Set_Motor_PWM(&right_top_motor);
    Set_Motor_PWM(&right_bottom_motor);

    Set_Angle(&roll_left_servo);
    Set_Angle(&roll_right_servo);
    Set_Angle(&pitch_left_servo);
    Set_Angle(&pitch_right_servo);
    //debug_printf(":%d,%d,%d,%d\r\n", left_top_motor.duty,left_bottom_motor.duty,right_top_motor.duty,right_bottom_motor.duty);
    //debug_printf(":%.2f\r\n",gryo_y_pid.output);
}
