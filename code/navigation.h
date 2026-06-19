/*
 * nagivation.h
 *
 *  Created on: 2024年10月16日
 *      Author: Monst
 */

#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_


//*********************用户设置区域****************************//
#define MaxSize 500    //flash存储的最大页面

#define Read_MaxSize 10000//最大读取设置，1w个应该是够了

//参数范围 <0 - 47>
#define Nag_End_Page 1      //flash中止页面
#define Nag_Start_Page 45   //flah起始页面

#define Turn_Flash_Page          2
#define Turn_ItemSize            2       //每条自转事件占3个数据：开始计值、结束计值、方向
#define Turn_DataStart           0       //自转事件组从flash_union_buffer[0]开始保存
#define Turn_MaxSize             (MaxSize / Turn_ItemSize) //最大自转事件数量

#define Nag_Turn_Left            1       //左转标志
#define Nag_Turn_Right           2       //右转标志
#define Nag_Turn_Gyro_Target     -7000.0f  //复现自转时的目标水平角速度



#define Nag_Set_mileage 2100 //里程计//5cm
#define Nag_Prev 200    //前瞻
#define Nag_Yaw angle_n //陀螺仪读取出来的偏航角

#define L_Mileage -motor_value.receive_left_speed_data   //左轮编码器
#define R_Mileage motor_value.receive_right_speed_data //右轮编码器

#define Nag_Turn_State_Path      0       // 普通路径复现
#define Nag_Turn_State_Brake     1       // 反向制动
#define Nag_Turn_State_Turn      2       // 正式自转

#define Nag_Turn_Brake_PreCount  23       // 提前几个路径计数开始制动
#define Nag_Turn_Brake_Speed     -100     // 反向制动速度
#define Nag_Run_Speed            500     // 正常路径速度

//********************************************************//
typedef struct{
       uint16 start_index;              //自转开始路径计值
       int8 direction;                  //自转方向
}NagTurn;

typedef struct{
       float Final_Out; //最终输出
       float Mileage_All;   //里程计数
       float Angle_Run; //读取的偏航角
       bool Nag_Stop_f; //惯导中止flag
       uint8 Flash_read_f;//惯导读取flag
       uint16 size; //惯导数组索引通用计数
       uint16 Run_index;
       uint16 Save_count;
       uint16 Save_index;//保存的flag

       uint16 Turn_Save_index;          // 自转事件保存数量
       uint16 Turn_Run_index;           // 自转事件复现索引
       uint8 Turn_Run_State;            // 自转复现状态

       float Turn_Target_Angle;         // 自转目标累计角度
       float Turn_Angle_All;            // 自转已经累计角度
       float Turn_Last_Yaw;             // 上一次角度


       uint8 Save_state;
       uint8 End_f;//中止flag
       //与flash相关的
       uint8 Flash_page_index;//flash页面索引
       uint8 Flash_Save_Page_Index;//flash保存页码索引
       uint8 Nag_SystemRun_Index;   //惯导执行索引
       //暂时未开发部分
       int Prev_mile[Nag_Prev]; //前瞻
}Nag;

extern Nag N;   //整个变量的结构体，方便开发和移植
extern int stay;
extern float error,count;
extern NagTurn Turn_read[Turn_MaxSize];
extern int32 Nav_read[Read_MaxSize];//按5cm算的话,1000可以跑50m
void Nag_Run(); //偏航角复现总函数
void Run_Nag_GPS();//偏航角复现

void NagFlashRead();   //Flash读取到目标数组。
void Run_Nag_Save();    //偏航角读取函数
void Nag_Read();    //偏航角读取总函数

void Nag_Turn_Start(int8 direction);
float Nag_Angle_Diff(float target, float current);
//****************************//
void Init_Nag();    //这个是参数初始化与flash的缓冲区初始化，请放到函数开始。
void Nag_System();  //这个是惯性导航最后的包装函数，请放到中断中。
#endif /* _NAVIGATION_H_ */
