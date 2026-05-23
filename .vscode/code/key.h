#ifndef __Key_H__
#define __Key_H__

#include "zf_common_headfile.h"

#define KEY1                    (P20_0)
#define KEY2                    (P20_1)
#define KEY3                    (P20_2)
#define KEY4                    (P20_3)

typedef struct KeyNum
{
	uint8(*Get_GPIO)(gpio_pin_enum pin);
	int flag;
	uint8_t Key_buf[3];
}KeyNum;

extern KeyNum TheKey[];
extern int Num_Key;
extern int Key_flag_1;
extern int Key_flag_2;
extern int Key_flag_3;
extern int Key_flag_4;
void Get_KeyNum(void);

#endif

