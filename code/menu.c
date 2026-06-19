#include "menu.h"

void menu_low(int max,int *p,int now_page);                         //按键向下移动
void menu_add(int max,int *p,int now_page);                         //按键向上移动
int menu_Confirm(int *p);                                           //确认按键处理
void menu_display(int now_page,int max ,char  *p[], int line_start); //菜单显示函数
int menu_return(void);
int menu_Func(int now_page,char *pthis[],int line_num,int *p);


int layer = 0;            //当前菜单层级，0为一级菜单，1为二级菜单
int line = 1;             //当前选中行号
int ChangeNum_flag = 0;   //参数修改标志，0为选择菜单，1为修改数值

uint8 data_buffer[32];
float data_len;
float a = 0,navigation_start = 0,navigation_end = 0,navigation_rec = 0,pid_flag,task;
float turn_start,turn_stop,turn_p,reset;

/////////////////////////////////////
//菜单显示内容

////////////////////////////////////////////
 char *menu1_string[]={"task","turn"};
 char *menu2_string[]={"task","nag_r","nag_e","nag_s","PID"};
 char *menu3_string[]={"T_s","T_p","reset"};
/////////////////////////////////////////////
menu_create menu[]=
{
    {0,1,1,menu1_string,2},
    {1,1,0,menu2_string,5},
    {1,2,0,menu3_string,3}
};

menuNum_create menu_num[]=
{
    {1,1,1,&task},    
    {1,1,2,&navigation_rec},
    {1,1,3,&navigation_end},
    {1,1,4,&navigation_start},
    {1,1,5,&pid_flag},

    {1,2,1,&turn_start},
    {1,2,2,&turn_p},
    {1,2,3,&reset}
};
/////////////////////////////////////////////

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     菜单功能执行函数
// 参数说明     now_page    当前页面编号
// 参数说明     pthis       当前页面显示字符串数组
// 参数说明     line_num    当前页面总行数
// 参数说明     p           当前选中行号地址
// 返回参数     int         返回当前选中行号
// 备注信息     在菜单循环中处理按键、显示刷新以及导航/转向/PID参数触发
//-------------------------------------------------------------------------------------------------------------------
int  menu_Func(int now_page,char *pthis[],int line_num,int *p)
{
        menu_display(now_page,line_num,pthis,*p);
	while(1)
	{
                   tft180_show_int(  80,  16*8,  N.Run_index, 4);
                   tft180_show_int(  80,  16*9,  (int)angle_n, 4);
                
		menu_low(line_num,p,now_page);
		menu_add(line_num,p,now_page);
		if( menu_Confirm(p) == 1) return *p;
		if( menu_return() == 1) return *p;
                
                if(pid_flag>=1)
                {
                      PID_gyro.Kp = 0.012;//0.004,0.007
                      PID_gyro.Kd = 0.0005;
                      
                      pid_flag=0;
                      
                      PID_balance.Kp = 240.0;//8.0
                      PID_balance.Ki = 0;
                      PID_balance.Kd = 1.2;
                      
                      PID_leg.Kp = 0.04;//0.03
                      
                      PID_dir.Kp = -0.008;//0.16
                      
                      PID_pitch.Kp = 250;//
                      PID_pitch.Ki = 0.0;//
                      PID_pitch.Kd = 0.5;//
                      
                }
                
                if(navigation_rec>=1) 
                {
                  N.Nag_SystemRun_Index=1;//1读取
                  navigation_rec=0;
                }
                if(navigation_end>=1 && N.Nag_SystemRun_Index == 1) 
                {
                  N.End_f=1;//End_f请勿重复赋值
                  navigation_end=0;
                }

                if(navigation_start>=1)
                {
                  N.Nag_SystemRun_Index=2;//2复现
                  navigation_start = 0;
                }
                if(N.Nag_SystemRun_Index == 2) NagFlashRead();//复现时必须调用，直接读取 Flash 路径数据

                if(turn_start>=1)
                {
                  turn_start=0;
                  Nag_Turn_Start((int)turn_p);
                }
                
                if(reset>=1)
                {
                  reset=0;
                  count=0;
                }
                
				
	        menu_display(now_page,line_num,pthis,*p);		
	}
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     菜单下移或参数减小
// 参数说明     max         当前页面最大行号
// 参数说明     p           当前选中行号地址
// 参数说明     now_page    当前页面编号
// 返回参数     void
// 备注信息     Num_Key 为 1 时触发；修改模式下对当前参数减 1
//-------------------------------------------------------------------------------------------------------------------
void menu_low(int max,int *p,int now_page)
{
	if(Num_Key == 1)
	{		
                tft180_clear();		
		Num_Key = 0;
		if(ChangeNum_flag == 0)
		{
		  (*p)++;
		  if(*p == max+1) *p = 1;
		}
		if(ChangeNum_flag == 1)//减数
		{
		   for(int a = 0;a < (sizeof(menu_num)/sizeof(menu_num[0]));a++)
            {
	          if((menu_num[a].layer == layer)&&(menu_num[a].layer_num == now_page)&&(menu_num[a].Line == *p))
	          {
		          (*menu_num[a].p)-=1;
				  break;
	          }
            }
		}
	}
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     菜单上移或参数增加
// 参数说明     max         当前页面最大行号
// 参数说明     p           当前选中行号地址
// 参数说明     now_page    当前页面编号
// 返回参数     void
// 备注信息     Num_Key 为 2 时触发；修改模式下对当前参数加 1
//-------------------------------------------------------------------------------------------------------------------
void menu_add(int max,int *p,int now_page)
{
	if(Num_Key == 2)
	{		
                tft180_clear(); 
		Num_Key = 0;
		if(ChangeNum_flag == 0)
		{
		  (*p)--;
		  if(*p == 0) *p = max;
		}
		if(ChangeNum_flag == 1)//加数
		{
		   for(int a = 0;a < (sizeof(menu_num)/sizeof(menu_num[0]));a++)
           {
	          if((menu_num[a].layer == layer)&&(menu_num[a].layer_num == now_page)&&(menu_num[a].Line == *p))
	          {
		          (*menu_num[a].p)+=1;
				  break;
	          }
            }
		}
	}
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     确认按键处理
// 参数说明     p           当前选中行号地址
// 返回参数     int         1表示页面或模式发生变化，0表示无动作
// 备注信息     Num_Key 为 3 时触发，进入下一级菜单或进入参数修改模式
//-------------------------------------------------------------------------------------------------------------------
int menu_Confirm(int *p)
{
	if(Num_Key == 3)
	{
            tft180_clear();
            Num_Key = 0;
            layer++;
            
            if(layer >= 2) 
            {
                    layer = 1;
                    ChangeNum_flag = 1;
                    return 1;
            }
            
            for(int a = 0;a < (sizeof(menu)/sizeof(menu[0]));a++)//初始化菜单标志
            {
                    if(menu[a].flag!=0) 
                    {
                      menu[a].flag = 0;
                      break;
                    }
            }
            
            for(int a = 0;a < (sizeof(menu)/sizeof(menu[0]));a++)//识别并选择对应页面
            {
                    if(menu[a].layer == layer)
                    {
                            if(menu[a].last_num == *p)
                            {
                                    menu[a].flag = 1;
                                    line = 1;
                                    return 1;
                            }
                    }
            }
	}
	return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     返回按键处理
// 参数说明     void
// 返回参数     int         1表示页面或模式发生变化，0表示无动作
// 备注信息     Num_Key 为 4 时触发，退出参数修改模式或返回上一级菜单
//-------------------------------------------------------------------------------------------------------------------
int menu_return(void)
{
	if(Num_Key == 4)
	{
          tft180_clear();  
          Num_Key = 0;		
          if(ChangeNum_flag == 1)
          {
                  ChangeNum_flag = 0;
                  return 1;
          }
          layer--;
          if(layer < 0) layer = 0;


          
          for(int a = 0;a < (sizeof(menu)/sizeof(menu[0]));a++)//初始化菜单标志
          {
                    if(menu[a].flag!=0) 
                    {
                      menu[a].flag = 0;
                      break;
                    }
          }
          
          for(int a = 0;a < (sizeof(menu)/sizeof(menu[0]));a++)//识别并选择对应页面
          {
                    if(menu[a].layer == layer)
                    {    
                            menu[a].flag = 1;				
                            return 1;			
                    }
          } 
	}
	return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     菜单显示函数
// 参数说明     now_page    当前页面编号
// 参数说明     max         当前页面最大行号
// 参数说明     p           当前页面显示字符串数组
// 参数说明     line_start  当前选中行号
// 返回参数     void
// 备注信息     普通选择模式显示 O，参数修改模式显示 X，并显示对应参数值
//-------------------------------------------------------------------------------------------------------------------
void menu_display(int now_page,int max ,char  *p[], int line_start)
{
      for(int line = 1;line <= max;line++)
      {
              String_Display(x_x,y_y*line,p[line-1]);
      }
      
      if(ChangeNum_flag == 0)
      {
         Sign_Display(16*7,RES*line_start,'O');
      }
      else 
      {
              ChangeSign_Dispaly(110,RES*line_start,'X');
      }
      
      for(int a = 0;a < (sizeof(menu_num)/sizeof(menu_num[0]));a++)
      {
         if((menu_num[a].layer == layer)&&(menu_num[a].layer_num == now_page))
         {
              Num_Display(x_x+48,menu_num[a].Line*y_y,*menu_num[a].p,3,3);
         }
      }	
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     菜单总调度函数
// 参数说明     void
// 返回参数     void
// 备注信息     菜单创建完成后周期调用，根据当前 flag 进入对应页面
//-------------------------------------------------------------------------------------------------------------------
void menu_Display(void)
{     
      for(int a = 0;a < (sizeof(menu)/sizeof(menu[0]));a++)
      {
            if(menu[a].flag == 1)
            {
                int  b = menu_Func(menu[a].last_num,menu[a].p,menu[a].Line_Num,&line);
            }
      }
}
