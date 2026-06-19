#include "Key.h"

typedef uint8(*KeyFunc)(gpio_pin_enum pin);

void KeyTriggerFunc(KeyFunc Func,gpio_pin_enum GPIO_Num,uint8_t buf[],int *flag);

KeyNum TheKey[]= //创建按键对象，并初始化按键状态缓存
{
	{gpio_get_level,0,{1,1,1}},
	{gpio_get_level,0,{1,1,1}},
	{gpio_get_level,0,{1,1,1}},
	{gpio_get_level,0,{1,1,1}}
};
int Num_Key= 0; //按键编号，1~4分别对应菜单操作，0表示无按键

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     扫描四个按键，并更新 Num_Key
// 参数说明     void
// 返回参数     void
// 备注信息     需要周期调用，内部通过三次采样滤波完成按键消抖
//-------------------------------------------------------------------------------------------------------------------
void Get_KeyNum(void)
{
  KeyTriggerFunc(TheKey[0].Get_GPIO, KEY1,TheKey[0].Key_buf,&TheKey[0].flag);
  KeyTriggerFunc(TheKey[1].Get_GPIO, KEY2,TheKey[1].Key_buf,&TheKey[1].flag);
  KeyTriggerFunc(TheKey[2].Get_GPIO, KEY3,TheKey[2].Key_buf,&TheKey[2].flag);
  KeyTriggerFunc(TheKey[3].Get_GPIO, KEY4,TheKey[3].Key_buf,&TheKey[3].flag);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     单个按键触发检测函数
// 参数说明     Func        GPIO电平读取函数
// 参数说明     GPIO_Num    按键引脚
// 参数说明     buf         三次采样缓存
// 参数说明     flag        按键按下状态标志
// 返回参数     void
// 备注信息     按键低电平有效，检测到稳定下降沿时更新 Num_Key
//-------------------------------------------------------------------------------------------------------------------
void KeyTriggerFunc(KeyFunc Func,gpio_pin_enum GPIO_Num,uint8_t buf[],int *flag)
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
