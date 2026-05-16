/*
 * motor_control.c
 *
 *  Created on: 2025年1月16日
 *      Author: 白嘉豪
 */
#include "zf_common_headfile.h"
#include "PID.h"

motor Motor;

int flagz=0,flag_stop=0;
int flagrx=0;
float Middle_angle=18.5;
float stab_roll;
int Lowpass_imu660rc_gyro_z;
int last_imu660rc_gyro_z;
float car_speed;
int Lroll,Rroll;
int dat=0;
float angle_n;

void leg_PID_param_init(void)//
{
    PID_leg.target_val=0;                       //目标速度
    PID_leg.output_val=0.0;                     //目标倾角
    PID_leg.Error=0.0;
    PID_leg.LastError=0.0;
    PID_leg.integral=0.0;
    PID_leg.integralmax=20;
    PID_leg.Kp = 0.00;//0.03
    PID_leg.Ki = 0.000;//0.00
    PID_leg.Kd = 0.0;
    PID_leg.max = 8;//12
    PID_leg.min = -8;//-12
    PID_leg.qing = -1;
}

void out_PID_param_init(void)
{
    PID_out.target_val=PID_leg.target_val;                       //目标速度
    PID_out.output_val=0.0;                     //目标倾角
    PID_out.Error=0.0;
    PID_out.LastError=0.0;
    PID_out.integral=0.0;
    PID_out.integralmax=6;
    PID_out.Kp = 0.0;//0.03
    PID_out.Ki = 0.000;//0.0
    PID_out.Kd = 0.00;
    PID_out.max = 10000000;//10
    PID_out.min = -10000000;//-10
}

void balance_PID_param_init(void)//
{
    PID_balance.target_val=7;      //目标倾角
    PID_balance.output_val=0.0;                 //目标角速度
    PID_balance.Error=0.0;
    PID_balance.LastError=0.0;
    PID_balance.integral=0.0;
    PID_balance.integralmax=420;
    PID_balance.Kp = 0.0;//8.0
    PID_balance.Ki = 0.0;
    PID_balance.Kd = 0.0;
    PID_balance.max = 10000.0;//270
    PID_balance.min = -10000.0;//-270
}

void gyro_PID_param_init(void)//
{
    PID_gyro.target_val=0;                   //目标角速度
    PID_gyro.output_val=0.0;                 //目标转速
    PID_gyro.Error=0.0;
    PID_gyro.LastError=0.0;
    PID_gyro.integral=0.0;
    PID_gyro.Kp = 0.0;//0.004,0.007
    PID_gyro.Ki = 0.0000;
    PID_gyro.Kd = 0.0000;
    PID_gyro.max = 600.0;
    PID_gyro.min = -600.0;
}

void dir_PID_param_init(void)//
{
    PID_dir.target_val=0;                   //目标水平方向角速度
    PID_dir.output_val=0.0;                 //目标差速
    PID_dir.Error=0.0;
    PID_dir.LastError=0.0;
    PID_dir.integral=0.0;
    PID_dir.Kp = 0.0;//0.16
    PID_dir.Ki = 0.0;
    PID_dir.Kd = 0.0;//0
    PID_dir.max = 30.0;
    PID_dir.min = -30.0;
}

void roll_PID_param_init(void)
{
    PID_roll.target_val=0;                   //目标滚转角
    PID_roll.output_val=0.0;                 //目标滚转姿态补偿角
    PID_roll.Error=0.0;
    PID_roll.LastError=0.0;
    PID_roll.integral=0.0;
    PID_roll.Kp = 0.0;//
    PID_roll.Ki = 0.0012;//0.0029
    PID_roll.Kd = 0.07;//
    PID_roll.max = 50.0;
    PID_roll.min = -50.0;
    PID_roll.qing = -1;
}

void pitch_PID_param_init(void)
{
    PID_pitch.target_val=0;                   //目标滚转角
    PID_pitch.output_val=0.0;                 //目标滚转姿态补偿角
    PID_pitch.Error=0.0;
    PID_pitch.LastError=0.0;
    PID_pitch.integral=0.0;
    PID_pitch.Kp = 0;//
    PID_pitch.Ki = 0.0;//
    PID_pitch.Kd = 0;//
    PID_pitch.max = 2000.0;
    PID_pitch.min = -2000.0;
    PID_pitch.qing = -1;
}

void L_PID_param_init(void)
{
    PID_L.target_val=0;                         //目标转速
    PID_L.output_val=0.0;                       //占空比
    PID_L.Error=0.0;
    PID_L.LastError=0.0;
    PID_L.integral=0.0;
    PID_L.integralmax=20;    
    PID_L.Kp = 0.007;
    PID_L.Ki = 0.0004;
    PID_L.Kd = 0.0;
    PID_L.max= 25;
    PID_L.min= -25;
}

void R_PID_param_init(void)//
{
    PID_R.target_val=0;
    PID_R.output_val=0.0;
    PID_R.Error=0.0;
    PID_R.LastError=0.0;
    PID_R.integral=0.0;
    PID_R.integralmax=10;
    PID_R.Kp = 0.019;
    PID_R.Ki = 0.00005;
    PID_R.Kd = 0.0;
    PID_R.max= 500;
    PID_R.min= -500;
}

void Lowpass(void)
{
    Lowpass_imu660rc_gyro_z=(int)(0.2*(float)imu660rc_gyro_z+0.8*(float)last_imu660rc_gyro_z);
    last_imu660rc_gyro_z=Lowpass_imu660rc_gyro_z;
}

void motor_Loop(void)
{
    int r,l;
    l=(int)(PID_R.target_val-PID_dir.output_val+Lroll);
    r=(int)(PID_R.target_val+PID_dir.output_val+Rroll);
    if(l>=50)l=50;
    if(l<=-50)l=-50;
    if(r>=50)r=50;
    if(r<=-50)r=-50;
    motor_set_pwm(l,r);
}

















