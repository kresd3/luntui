#include "Key.h"

typedef uint8(*KeyFunc)(gpio_pin_enum pin);

void KeyTriggerFunc(KeyFunc Func,gpio_pin_enum GPIO_Num,uint8_t buf[],int *flag);

KeyNum TheKey[]= //创建按键及初始化
{
	{gpio_get_level,0,{1,1,1}},
	{gpio_get_level,0,{1,1,1}},
	{gpio_get_level,0,{1,1,1}},
	{gpio_get_level,0,{1,1,1}}
};
int Num_Key= 0;

//////////////////////////////////////////////////////////////////////////////////////

void Get_KeyNum(void) //功能实现
{
  KeyTriggerFunc(TheKey[0].Get_GPIO, KEY1,TheKey[0].Key_buf,&TheKey[0].flag);
  KeyTriggerFunc(TheKey[1].Get_GPIO, KEY2,TheKey[1].Key_buf,&TheKey[1].flag);
  KeyTriggerFunc(TheKey[2].Get_GPIO, KEY3,TheKey[2].Key_buf,&TheKey[2].flag);
  KeyTriggerFunc(TheKey[3].Get_GPIO, KEY4,TheKey[3].Key_buf,&TheKey[3].flag);
}

void KeyTriggerFunc(KeyFunc Func,gpio_pin_enum GPIO_Num,uint8_t buf[],int *flag)//通过滤波实现按键消抖
{
	buf[0] = buf[1];
	buf[1] = buf[2];
	buf[2] = Func(GPIO_Num);
	
	if((buf[0] == buf[1]) && (buf[1] == buf[2]))
	{
		if( *flag == 0 && buf[2] == 0 ) //下降沿
		{
		        if(GPIO_Num == KEY1) Num_Key = 3;
			if(GPIO_Num == KEY2) Num_Key = 4;
			if(GPIO_Num == KEY3) Num_Key = 2;
			if(GPIO_Num == KEY4) Num_Key = 1;
			*flag = 1;
		}
		else if( *flag == 1 && buf[2] == 1 )//上升沿
		{
			*flag = 0;
		}
	}
}
