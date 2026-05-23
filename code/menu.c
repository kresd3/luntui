#include "menu.h"

void menu_low(int max,int *p,int now_page);//按键向下移动
void menu_add(int max,int *p,int now_page);//按键向上移动
int menu_Confirm(int *p);//确认按键
void menu_display(int now_page,int max ,char  *p[], int line_start);//一级菜单显示函数
int menu_return(void);
int menu_Func(int now_page,char *pthis[],int line_num,int *p);


int layer = 0;
int line = 1;
int ChangeNum_flag = 0;

uint8 data_buffer[32];
float data_len;
float a = 0,navigation_start = 0,navigation_end = 0,navigation_rec = 0,pid_flag,task;
float turn_start,turn_stop,turn_p,reset;

/////////////////////////////////////
//添加显示内容

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

//功能实现函数（显示内容，按键的最大值，菜单的初始位置（为1就行），用于记录行数的数的地址）
int  menu_Func(int now_page,char *pthis[],int line_num,int *p)//按键的值的范围应为1-max
{
        menu_display(now_page,line_num,pthis,*p);
	while(1)
	{
//                if(imu660rc_yaw <=180)printf("%d,%f,%f\r\n",0,a,imu660rc_yaw);
//                else if(imu660rc_yaw > 180) printf("%d,%f,%f\r\n",0,a,imu660rc_yaw-360);
                   tft180_show_int(  80,  16*8,  N.Run_index, 4);
                   tft180_show_int(  80,  16*9,  (int)angle_n, 4);
//                   printf("%d,%f,%f\r\n",0,car_speed,angle_n);
                
//                data_len = ble6a20_read_buffer(data_buffer, 32);                            // 查看是否有消息 默认缓冲区是BLE6A20_BUFFER_SIZE 总共 64 字节
//                if(data_len != 0)                                                           // 收到了消息 读取函数会返回实际读取到的数据个数
//                {
//                  a+=10;
//                }
                
		menu_low(line_num,p,now_page);
		menu_add(line_num,p,now_page);
		if( menu_Confirm(p) == 1) return *p;
		if( menu_return() == 1) return *p;
                
                if(pid_flag>=1)
                {
                      PID_gyro.Kp = 0.012;//0.004,0.007
                      PID_gyro.Kd = 0.0005;
                      
                      PID_balance.Kp = 200.0;//8.0
                      PID_balance.Ki = 0;
                      PID_balance.Kd = 1;
                      
                      PID_leg.Kp = 0.02;//0.03
                      
                      PID_dir.Kp = -0.008;//0.16
                      
                      PID_pitch.Kp = 160;//
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
                if(N.Nag_SystemRun_Index == 2) NagFlashRead();//移植的时候这个必须要。直接复制粘贴过去就行

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


/////////////////////////////////////////////////////////////////


void menu_low(int max,int *p,int now_page)//按键向下移动
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
		if(ChangeNum_flag == 1)//加数
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

void menu_add(int max,int *p,int now_page)// 按键向上移动
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
		if(ChangeNum_flag == 1)//减数
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

int menu_Confirm(int *p)//确认按键
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
            
            for(int a = 0;a < (sizeof(menu)/sizeof(menu[0]));a++)//初始化
            {
                    if(menu[a].flag!=0) 
                    {
                      menu[a].flag = 0;
                      break;
                    }
            }
            
            for(int a = 0;a < (sizeof(menu)/sizeof(menu[0]));a++)//识别并选择相应页面 
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

int menu_return(void)// 返回按键
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


          
          for(int a = 0;a < (sizeof(menu)/sizeof(menu[0]));a++)//初始化
          {
                    if(menu[a].flag!=0) 
                    {
                      menu[a].flag = 0;
                      break;
                    }
          }
          
          for(int a = 0;a < (sizeof(menu)/sizeof(menu[0]));a++)//识别并选择相应页面 
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

void menu_display(int now_page,int max ,char  *p[], int line_start)//菜单显示函数
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

void menu_Display(void)//创建完后调用这个函数 
{     
      for(int a = 0;a < (sizeof(menu)/sizeof(menu[0]));a++)
      {
            if(menu[a].flag == 1)
            {
                int  b = menu_Func(menu[a].last_num,menu[a].p,menu[a].Line_Num,&line);
            }
      }
}
