#include "zf_common_headfile.h"

PID PID_out;
PID PID_leg;
PID PID_balance;
PID PID_gyro;
PID PID_dir;
PID PID_roll;
PID PID_pitch;
PID PID_L;
PID PID_R;


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     位置式pid
// 参数说明     pid结构体地址,当前值
// 返回参数     float
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
float PosionPID_realize(PID *pid, float actual_val)
{
    /*计算目标值与实际值的误差*/
    pid->Error = pid->target_val - actual_val;   
    if(PID_pitch.Error>200) PID_pitch.Error = -5;
    if(PID_pitch.Error<-200) PID_pitch.Error = 5; 
    /*积分项*/
    pid->integral += pid->Error;
    if(pid->integral>=pid->integralmax)pid->integral=pid->integralmax;
    if(pid->integral<=-pid->integralmax)pid->integral=pid->integralmax;
    /*PID算法实现*/
    pid->output_val = pid->Kp * pid->Error +
                      pid->Ki * pid->integral +
                      pid->Kd *(pid->Error -pid->LastError);
    
   
//    printf("%f\n",pid->Error -pid->LastError);
//    tft180_show_float(20, 110, pid->Error -pid->LastError, 4, 4);
    /*误差传递*/
    pid-> LastError = pid->Error;
    /*返回当前实际值*/
    if(pid->output_val>=pid->max)pid->output_val=pid->max;
    if(pid->output_val<=pid->min)pid->output_val=pid->min;



    return pid->output_val;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     增量式pid
// 参数说明     pid结构体地址,当前值
// 返回参数     float
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
float addPID_realize(PID *pid, float actual_val)
{
    /*计算目标值与实际值的误差*/
    pid->Error = pid->target_val - actual_val;
    if(fabs(pid->Error)<=pid->qing)
    {
        pid->output_val=0;
    }

    /*PID算法实现，照搬公式*/
    pid->output_val += pid->Kp * (pid->Error - pid-> LastError) +
                      pid->Ki * pid->Error +
                      pid->Kd *(pid->Error -2*pid->LastError+pid->PrevError);

    /*误差传递*/
    pid-> PrevError = pid->LastError;
    pid-> LastError = pid->Error;
    /*返回当前实际值*/
    if(pid->output_val>=pid->max)pid->output_val=pid->max;
    if(pid->output_val<=pid->min)pid->output_val=pid->min;
    return pid->output_val;
}
