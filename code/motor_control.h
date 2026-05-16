#ifndef CODE_MOTOR_CONTROL_H_
#define CODE_MOTOR_CONTROL_H_

typedef struct{
        int16 target_speed;
        int16 targrt_speedL;
        int16 target_speedR;
        int16 receive_speed;
        int16 receive_speedL;
        int16 receive_speedR;
        int16 pwm_L;
        int16 pwm_R;
}motor;

extern  motor Motor;
extern int flagz,flag_stop;
extern int flagrx;
extern float Middle_angle;
extern float stab_roll;//滚转姿态补偿角
extern int Lowpass_imu660rc_gyro_z;
extern float car_speed;
extern int Lroll,Rroll;
extern int dat;
extern float angle_n;

void    leg_PID_param_init(void);

void    out_PID_param_init(void);

void    dir_PID_param_init(void);

void    pitch_PID_param_init(void);

void    L_PID_param_init(void);

void    R_PID_param_init(void);

void    balance_PID_param_init(void);

void    gyro_PID_param_init(void);

void    roll_PID_param_init(void);

void    Lowpass(void);

void    motor_Loop(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷设置占空比
// 参数说明     void
// 返回参数     void
// 备注信息     串口发送
//-------------------------------------------------------------------------------------------------------------------
#define     motor_set_pwm(L,R)     (small_driver_set_duty(-L * (PWM_DUTY_MAX / 100), R * (PWM_DUTY_MAX / 100)))



#endif /* CODE_MOTOR_CONTROL_H_ */
