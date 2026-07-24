#ifndef __FLIGHT_H
#define __FLIGHT_H

#include "Receive_Data.h"
#include "IMU.h"
#include "PID.h"
#include "Motor.h"
#include "Servo.h"
/*根据欧拉角计算pid的值
*/
void flight_pid_process(void);

void flight_control_motor(void);

#endif 
