#ifndef __MENU_H__
#define __MENU_H__

#include "zf_common_headfile.h"
#include "small_driver_uart_control.h"
#include "Key.h"

#define x_x 0                                   //菜单显示起始 X 坐标
#define y_y 16                                  //菜单行距
#define RES 16                                  //屏幕行高分辨率
#define String_Display   tft180_show_string     //字符串显示函数
#define Sign_Display   tft180_show_char         //选中标志显示函数
#define ChangeSign_Dispaly    tft180_show_char  //参数修改标志显示函数
#define Num_Display    tft180_show_float        //数字显示函数

typedef struct menu_create                         //菜单页面参数结构体
{	
	int layer;                                    //菜单层级
	int last_num;                                 //上一级菜单保存的行号，二级菜单中也可作为页面编号
	int flag;                                     //当前页面显示标志
	char **p;                                     //菜单字符串数组
        int Line_Num ;		                         //当前页面行数
}menu_create;

typedef struct menuNum_create                      //菜单参数绑定结构体
{
	int layer;                                    //参数所在菜单层级
	int layer_num;                                //参数所在页面编号
	int Line;                                     //参数所在行号
	float *p;                                     //参数变量地址
}menuNum_create;

extern menu_create menu[];                         //菜单页面数组
extern int layer;                                  //当前菜单层级
extern float a;
extern float task;                                 //任务选择参数

void menu_low(int max,int *p,int now_page);                         //菜单下移或参数减小
void menu_add(int max,int *p,int now_page);                         //菜单上移或参数增加
int menu_Confirm(int *p);                                           //确认按键处理
void menu_display(int now_page,int max ,char  *p[], int line_start); //菜单显示函数
int menu_Func(int now_page,char *pthis[],int line_num,int *p);       //菜单功能执行函数
void menu_Display(void);                                            //菜单总调度函数


#endif
