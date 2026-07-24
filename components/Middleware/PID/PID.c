#include <stdio.h>
#include <math.h>
#include "PID.h"



/*
*单次PID
*/
void PID_Calc(PID_Struct *pid){
    //计算误差值
    pid->err=pid->measure-pid->desire;
    //计算积分误差
    pid->integral +=pid->err;
    //计算微分误差
    if(pid->last_err==0){
        pid->last_err=pid->err;
    }
    double der =pid->err-pid->last_err;
    //计算输出
    pid->output= (pid->Kp*pid->err) + (pid->Ki*pid->integral*PID_PERIOD) + (pid->Kd*der/PID_PERIOD);
    //保存上次误差
    pid->last_err=pid->err;
}

/*
*串级PID
*/
void PID_Calc_Chain(PID_Struct *out_pid,PID_Struct *in_pid){
    //先计算外环
    PID_Calc(out_pid);
    //将外环的输出值作为内环的目标值
    in_pid->desire=out_pid->output;
    //计算内环
    PID_Calc(in_pid);
}

