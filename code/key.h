#ifndef __Key_H__
#define __Key_H__

#include "zf_common_headfile.h"

#define KEY1                    (P20_0)                 //按键1引脚，对应确认键
#define KEY2                    (P20_1)                 //按键2引脚，对应返回键
#define KEY3                    (P20_2)                 //按键3引脚，对应上移/加数键
#define KEY4                    (P20_3)                 //按键4引脚，对应下移/减数键

typedef struct KeyNum                                      //按键检测参数结构体
{
	uint8(*Get_GPIO)(gpio_pin_enum pin);                   //GPIO电平读取函数
	int flag;                                               //按键状态标志，0为松开，1为按下
	uint8_t Key_buf[3];                                     //按键三次采样缓存，用于消抖
}KeyNum;

extern KeyNum TheKey[];                                    //按键对象数组
extern int Num_Key;                                        //当前按键编号，0表示无按键
extern int Key_flag_1;
extern int Key_flag_2;
extern int Key_flag_3;
extern int Key_flag_4;
void Get_KeyNum(void);                                     //扫描按键并更新 Num_Key

#endif
