#include"FreeRTOS_Main.h"
#include"Servo.h"
#include"Motor.h"
#include "filter.h"
#include "PID.h"
#include "IMU.h"
#include "GPS.h"
#include "filter.h"
#define MOTOR_MIN_PWM   410    // 1.0ms, 停转/解锁脉宽
#define MOTOR_MAX_PWM   800    // 2.0ms, 全油门脉宽

extern volatile uint16_t TxData[4];
extern volatile uint16_t RxData[4];

extern IMU_DATA IMU_Data;
extern GPS_t gps_data;
const float CTRL_DT = 0.005f;// 飞控运行周期 200Hz

volatile float target_pitch = 0.0f;     // 遥控器给飞控设定的目标俯仰角
volatile float target_roll  = 0.0f;      // 遥控器给飞控设定的目标横滚角
volatile uint16_t throttle_output = 0;   // 传递给电机的油门输出值
// PID 控制器
PID_Controller pid_pitch; 
PID_Controller pid_roll; 

extern pt2Filter_t gyro_filter_pitch;
extern pt2Filter_t gyro_filter_roll;

#define pitch_offset_left 5.0f//舵机偏移，根据实际情况调整
#define roll_offset_right 5.0f

#define pitch_kp 0.9f
#define pitch_ki 0.01f   
#define pitch_kd 0.04f

#define roll_kp 0.8f
#define roll_ki 0.01f
#define roll_kd 0.04f  

#define pitch_out 45.0f // 俯仰自稳限制在最大补偿 +-40度
#define roll_out 35.0f  // 横滚自稳限制在最大补偿 +-35度

#define PITCH_MID  90   // 升降舵中位
#define ROLL_MID   90   // 副翼中位

#define PITCH_MIN_LIMIT  PITCH_MID-pitch_out    // 允许的最小角度
#define PITCH_MAX_LIMIT  PITCH_MID+pitch_out    // 允许的最大角度
#define ROLL_MIN_LIMIT   ROLL_MID-roll_out      // 允许的最小角度
#define ROLL_MAX_LIMIT   ROLL_MID+roll_out      // 允许的最大角度

#define GPS_DT 1000 // GPS数据读取周期 1000ms

extern float bmi270_gyroScale;         // 角速度量程缩放因子
extern float bmi270_accelScale;                // 加速度计量程缩放因子(16g/65536)
/*Min_pwm 185~220 Max_PWM 980左右，峰值1300
*/
void MOTOR_PWM(uint16_t throttle_pwm){
    Set_Motor(throttle_pwm);
}

//RXData[1]MIN下降  340       mid 1600       MAX  2800~2900
//0°=1638，180°=8190
void Servo_pitch_PWM(uint16_t pitch_PWM){
    if(pitch_PWM<340)pitch_PWM=340;
    if(pitch_PWM>2900)pitch_PWM=2900;
    uint8_t angle = (pitch_PWM - 340) * 120 / (2900 - 340);
    Set_Angel(2,angle);
    Set_Angel(3,angle);
}

//RXData[3]MIN  325       mid 1591       MAX  2800~2900
void Servo_roll_PWM(uint16_t roll_pwm){
    if(roll_pwm<325)roll_pwm=325;
    if(roll_pwm>2900)roll_pwm=2900;
    uint8_t angle = (roll_pwm - 325) * 120 / (2900 - 325);
    Set_Angel(0,angle);
    Set_Angel(1,angle);
}
void Control_PID_Init(void) {
    PID_Init(&pid_pitch, pitch_kp, pitch_ki, pitch_kd, pitch_out); // 俯仰自稳限制在最大补偿 +-70度
    PID_Init(&pid_roll,  roll_kp, roll_ki, roll_kd, roll_out); // 横滚自稳
}
void Control_Task_IMU(void *arg)
{
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(5); // 200Hz 5ms 精准周期
    Control_PID_Init();
    xLastWakeTime = xTaskGetTickCount(); 
    while(1){
        MOTOR_PWM(RxData[0]);
        BMI270_Get_Angle_Plus(&IMU_Data);
        
        float filtered_pitch = pt2FilterApply(&gyro_filter_pitch, IMU_Data.pitch);
        float filtered_roll  = pt2FilterApply(&gyro_filter_roll,  IMU_Data.roll);

        pid_pitch.target = target_pitch;
        pid_roll.target  = target_roll;

        float pitch_output = PID_Compute(&pid_pitch, filtered_pitch, CTRL_DT);
        float roll_output  = PID_Compute(&pid_roll,  filtered_roll,  CTRL_DT);

        // 4. 将输出转换为舵机角度 (中位90°)
        float angle_pitch      = PITCH_MID + (int)pitch_output;
        float angle_roll  = ROLL_MID + (int)roll_output;

        if (angle_pitch < PITCH_MIN_LIMIT) angle_pitch = PITCH_MIN_LIMIT;
        if (angle_pitch > PITCH_MAX_LIMIT)  angle_pitch = PITCH_MAX_LIMIT;
        if (angle_roll < ROLL_MIN_LIMIT)  angle_roll = ROLL_MIN_LIMIT;
        if (angle_roll > ROLL_MAX_LIMIT)   angle_roll = ROLL_MAX_LIMIT;

        Set_Angel(0, angle_roll);       // 左副翼
        Set_Angel(1, angle_roll+roll_offset_right);       // 右副翼
        Set_Angel(2, 180-(angle_pitch+pitch_offset_left));   // 左主升降舵
        Set_Angel(3, angle_pitch);   // 右主升降舵    
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}
void Control_Task_Man(void *arg){
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(5); // 200Hz 5ms 精准周期
    xLastWakeTime = xTaskGetTickCount(); 
    while(1){
        float angle_pitch = PITCH_MID + target_pitch;
        float angle_roll  = ROLL_MID + target_roll;

        Set_Angel(0, angle_roll);       // 主升降舵 左副翼
        Set_Angel(1, angle_roll+roll_offset_right);       // 右副翼
        Set_Angel(2, 180-(angle_pitch+pitch_offset_left));      // 双垂尾/升降联动
        Set_Angel(3, angle_pitch);      // 双垂尾/升降联动      
        
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}
void RC_Task(void *arg)
{
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(10); //100hz
    xLastWakeTime = xTaskGetTickCount(); //基准时间
    while(1)
    {
        float temp_pitch = 0.0f;
        float temp_roll  = 0.0f;
        if (RxData[1] >= 1600) {
            temp_pitch = ((float)RxData[1] - 1600) / (2900.0f - 1600) * pitch_out;
        } else {
            temp_pitch = ((float)RxData[1] - 1600) / (1600 - 340.0f) * pitch_out;
        }
        if (RxData[2] >= 1540) {
            temp_roll = ((float)RxData[2] - 1540.0f) / (2900.0f - 1540.0f) * roll_out;
        } else {
            temp_roll = ((float)RxData[2] - 1540.0f) / (1540.0f - 300.0f) * roll_out;
        }
        if (temp_pitch < -pitch_out) temp_pitch = -pitch_out;
        if (temp_pitch > pitch_out)  temp_pitch = pitch_out;
        if (temp_roll < -roll_out)  temp_roll = -roll_out;
        if (temp_roll > roll_out)   temp_roll = roll_out;

        target_pitch = temp_pitch;
        target_roll  = -temp_roll;

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
            
    }
}

void GPS_Task(void *arg){
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(GPS_DT); //每秒一次
    xLastWakeTime = xTaskGetTickCount(); //基准时间
    while(1){
        GPS_Read_Data(&gps_data);
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        printf("GPS: Time=%s, Status=%s, Lat=%s %s, Lon=%s %s, Spd=%s, COG=%s, Date=%s\r\n",
            gps_data.UTCTime, gps_data.GPS_Status, gps_data.GPS_lat, gps_data.GPS_ulat,
            gps_data.GPS_lon, gps_data.GPS_ulon, gps_data.GPS_spd, gps_data.GPS_cog,
            gps_data.GPS_date);
        }
}
