#ifndef CODE_LEG_CONTROL_H_
#define CODE_LEG_CONTROL_H_

#define LeftFront                   (TCPWM_CH12_P05_3)                      //左前舵机引脚ATOM3_CH0_P15_6
#define LeftRear                    (TCPWM_CH11_P05_2)                      //左后舵机引脚ATOM1_CH1_P15_7
#define RightFront                  (TCPWM_CH10_P05_1)                      //右前舵机引脚ATOM0_CH3_P21_5
#define RightRear                   (TCPWM_CH09_P05_0)                      //右后舵机引脚ATOM0_CH1_P33_9

#define L1                          (61)                                   //腿长(mm)
#define L2                          (91)
#define L3                          (91)
#define L4                          (61)
#define L5                          (37)
#define L6                          (107)

typedef struct{                                                            //角度坐标参数结构体
    float alphaLeft, betaLeft;
    float alphaRight, betaRight;
    float XLeft,YLeft;
    float XRight, YRight;
}IKparam;

extern IKparam IKParam;

extern float E_H,bend_HL,bend_HR;                            //左右腿高度差
extern float B_H,B_X;                            //基本高度

void    inverseKinematics(void);                                                          //舵机循环函数，运动学逆解算

void    leg_position_set(float X , float yleft , float yright);         //设置左右轮坐标

void    leg_control_init(void);                                                           //初始化舵机

#endif /* CODE_LEG_CONTROL_H_ */


























