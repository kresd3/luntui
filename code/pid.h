#ifndef CODE_PID_H_
#define CODE_PID_H_


typedef struct
{
    float target_val;                       //目标值
    float Error;                            /*第 k 次偏差 */
    float LastError;                        /* Error[-1],第 k-1 次偏差 */
    float PrevError;                        /* Error[-2],第 k-2 次偏差 */
    float Kp,Ki,Kd;                         //比例、积分、微分系数
    float integral;                         //积分值
    float integralmax;                      //积分限幅
    float output_val;                       //输出值
    float max;                              //最大值
    float min;                              //最小值
    float qing;
}PID;

extern PID PID_out;
extern PID PID_leg;
extern PID PID_L;
extern PID PID_R;
extern PID PID_balance;
extern PID PID_gyro;
extern PID PID_dir;
extern PID PID_pitch;
extern PID PID_roll;


float   PosionPID_realize(PID *pid, float actual_val);                          //位置式pid

float   addPID_realize(PID *pid, float actual_val);                             //增量式pid


#endif /* CODE_PID_H_ */




























