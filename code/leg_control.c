#include "zf_common_headfile.h"

IKparam IKParam;
float E_H,last_E_H,bend_HL=0,bend_HR=0;//左右轮腿差
float B_H=45,B_X=18.45;//基本高度
int16_t alphaLeftToAngle,betaLeftToAngle,alphaRightToAngle,betaRightToAngle;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     舵机循环函数，运动学逆解算
// 参数说明     void
// 返回参数     void
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void inverseKinematics(){
  float alpha1,alpha2,beta1,beta2;
  uint16_t servoLeftFront,servoLeftRear,servoRightFront,servoRightRear;

  float aLeft = 2 * IKParam.XLeft * L1;
  float bLeft = 2 * IKParam.YLeft * L1;
  float cLeft = IKParam.XLeft * IKParam.XLeft + IKParam.YLeft * IKParam.YLeft + L1 * L1 - L2 * L2;
  float dLeft = 2 * L4 * (IKParam.XLeft - L5);
  float eLeft = 2 * L4 * IKParam.YLeft;
  float fLeft = ((IKParam.XLeft - L5) * (IKParam.XLeft - L5) + L4 * L4 + IKParam.YLeft * IKParam.YLeft - L3 * L3);

  alpha1 = 2 * atan((bLeft + sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft))) / (aLeft + cLeft));
  alpha2 = 2 * atan((bLeft - sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft))) / (aLeft + cLeft));
  beta1 = 2 * atan((eLeft + sqrt((dLeft * dLeft) + eLeft * eLeft - (fLeft * fLeft))) / (dLeft + fLeft));
  beta2 = 2 * atan((eLeft - sqrt((dLeft * dLeft) + eLeft * eLeft - (fLeft * fLeft))) / (dLeft + fLeft));

  alpha1 = (alpha1 >= 0)?alpha1:(alpha1 + 2 * PI);
  alpha2 = (alpha2 >= 0)?alpha2:(alpha2 + 2 * PI);

  if(alpha1 >= PI/4) IKParam.alphaLeft = alpha1;
  else IKParam.alphaLeft = alpha2;
  if(beta1 >= 0 && beta1 <= PI/4) IKParam.betaLeft = beta1;
  else IKParam.betaLeft = beta2;

  float aRight = 2 * IKParam.XRight * L1;
  float bRight = 2 * IKParam.YRight * L1;
  float cRight = IKParam.XRight * IKParam.XRight + IKParam.YRight * IKParam.YRight + L1 * L1 - L2 * L2;
  float dRight = 2 * L4 * (IKParam.XRight - L5);
  float eRight = 2 * L4 * IKParam.YRight;
  float fRight = ((IKParam.XRight - L5) * (IKParam.XRight - L5) + L4 * L4 + IKParam.YRight * IKParam.YRight - L3 * L3);

  IKParam.alphaRight = 2 * atan((bRight + sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
  IKParam.betaRight = 2 * atan((eRight - sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));

  alpha1 = 2 * atan((bRight + sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
  alpha2 = 2 * atan((bRight - sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
  beta1 = 2 * atan((eRight + sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));
  beta2 = 2 * atan((eRight - sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));

  alpha1 = (alpha1 >= 0)?alpha1:(alpha1 + 2 * PI);
  alpha2 = (alpha2 >= 0)?alpha2:(alpha2 + 2 * PI);

  if(alpha1 >= PI/4) IKParam.alphaRight = alpha1;
  else IKParam.alphaRight = alpha2;
  if(beta1 >= 0 && beta1 <= PI/4) IKParam.betaRight = beta1;
  else IKParam.betaRight = beta2;

  alphaLeftToAngle = (int)((IKParam.alphaLeft / 6.28) * 360);//弧度转角度
  betaLeftToAngle = (int)((IKParam.betaLeft / 6.28) * 360);

  alphaRightToAngle = (int)((IKParam.alphaRight / 6.28) * 360);
  betaRightToAngle = (int)((IKParam.betaRight / 6.28) * 360);

  servoLeftFront = (int)(4625 - betaLeftToAngle*33.33);
  servoLeftRear = (int)((180-alphaLeftToAngle)*33.33+4520);//4400
  servoRightFront = (int)((180 - alphaRightToAngle)*33.33+4520);
  servoRightRear = (int)(4625 -  betaRightToAngle*33.33);//4625
  if(servoLeftFront>=10000)servoLeftFront=10000;
  if(servoLeftRear>=10000)servoLeftRear=10000;
  if(servoRightFront>=10000)servoRightFront=10000;
  if(servoRightRear>=10000)servoRightRear=10000;
  pwm_set_duty(RightRear, servoRightFront);            //后744（0）260（90）beta
  pwm_set_duty(RightFront , servoRightRear );            //前744（180）1220（90）alpha....servoRightRear
  pwm_set_duty(LeftRear , servoLeftFront );            //后744（0）260（90）beta....servoLeftFront
  pwm_set_duty(LeftFront  , servoLeftRear  );            //前744（180）1220（90）alpha....servoLeftRear

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置左右轮坐标
// 参数说明     void
// 返回参数     void
// 备注信息
//------------------------------------------------------------------------------------------------------------------
void leg_position_set(float X , float yleft  ,float yright)
{
    if(yleft<=25)yleft=25;
    if(yright<=25)yright=25;
    if(yleft>=150)yleft=150;
    if(yright>=150)yright=150;
    IKParam.XLeft = X;
    IKParam.YLeft = yleft;
    IKParam.XRight = 37-X;
    IKParam.YRight = yright;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置左右轮坐标
// 参数说明     void
// 返回参数     void
// 备注信息
//------------------------------------------------------------------------------------------------------------------
void leg_control_init()
{
    pwm_init(LeftFront , 300, 0);
    pwm_init(LeftRear  , 300, 0);
    pwm_init(RightFront, 300, 0);
    pwm_init(RightRear , 300, 0);
    leg_position_set( 18.5 , 25 , 25 );//18.5,55,55
}

























