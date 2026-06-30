#include <stdio.h>
#include <math.h>

typedef struct {
    float Kp;           
    float Ki;           
    float Kd;           
    
    float target;       // 目标角度
    
    float error;     //误差   
    float integral;  //积分   
    
    // 改用上一次的实际测量值，消除微分突变
    float last_measurement; 
    
    float out_max;      // 输出限幅
    float integral_max; // 积分限幅
    
    // 新增：积分分离阈值 (比如设为15.0度，误差大于15度不积分)
    float integral_separation_err; 
} PID_Controller;

// 初始化 PID 参数
void PID_Init(PID_Controller *pid, float p, float i, float d, float out_max) {
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;
    pid->error = 0;
    pid->integral = 0;
    pid->last_measurement = 0; // 初始化为0
    
    pid->out_max = out_max;     //最大输出
    pid->integral_max = out_max * 0.5f; // 积分限幅设为最大输出的50%比较合理
    pid->integral_separation_err = 15.0f; // 默认误差超过15度时停止积分
}

// PID 计算函数 (每次循环调用)
// current_angle: 比如 MPU6050 结算出的当前俯仰角 (Pitch)
// dt: 两次调用的时间间隔，比如 0.02f (50Hz)
float PID_Compute(PID_Controller *pid, float current_angle, float dt) {
    // 1. 计算当前误差
    pid->error = pid->target - current_angle;
    
    // 2. 计算积分 (带有积分分离与抗饱和)
    // 只有当误差在可控范围内时，才进行积分，防止大机动或受干扰时积分过冲
    if(fabs(pid->error) < pid->integral_separation_err) {
        pid->integral += pid->error * dt;
        
        // 积分限幅
        if(pid->integral > pid->integral_max) pid->integral = pid->integral_max;
        else if(pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    } else {
        // 如果误差过大，可以适当衰减积分，或者保持不变
        // 这里选择保持不变，避免突变
    }
    
    // 3. 计算微分 (使用测量值的微分，消除由于target突变引起的"微分突变")
    // 注意这里是 (last - current)，因为目标是消除变化
    float derivative = (pid->last_measurement - current_angle) / dt;
    pid->last_measurement = current_angle; // 更新历史测量值
    
    // 4. 计算总输出
    float output = (pid->Kp * pid->error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);
    
    // 5. 输出限幅 (必须有，比如舵机PWM一般限制在 1000~2000 的某个偏差范围，如 -500 到 +500)
    if(output > pid->out_max) output = pid->out_max;
    else if(output < -pid->out_max) output = -pid->out_max;
    
    return output;
}

