#ifndef __MENU_H__
#define __MENU_H__

#include "zf_common_headfile.h"
#include "small_driver_uart_control.h"
#include "Key.h"

#define x_x 0  //菜单初始坐标
#define y_y 16  //y的初始坐标与分辨率一致
#define RES 16     //分辨率
#define String_Display   tft180_show_string      //字符串显示函数
#define Sign_Display   tft180_show_char     //按键显示函数
#define ChangeSign_Dispaly    tft180_show_char //按键选择显示函数 
#define Num_Display    tft180_show_float   //数字显示函数 

typedef struct menu_create
{	
	int layer; //级数
	int last_num; //上一个页面保存的行数，二级菜单也可认为是页面数	
	int flag;
	char **p;
        int Line_Num ;		//行数
}menu_create;

typedef struct menuNum_create
{
	int layer;
	int layer_num;
	int Line;
	float *p;
}menuNum_create;

extern menu_create menu[];
extern int layer;
extern float a;
extern float task;

void menu_low(int max,int *p,int now_page);//按键向下移动
void menu_add(int max,int *p,int now_page);//按键向上移动
int menu_Confirm(int *p);//确认按键
void menu_display(int now_page,int max ,char  *p[], int line_start);//一级菜单显示函数
int menu_Func(int now_page,char *pthis[],int line_num,int *p);//功能实现函数
void menu_Display(void);


#endif
