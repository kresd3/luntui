/*
 * navigation.c
 *
 *  Created on: 2024年10月16日
 *      Author: Monst
 *
 *
 */

#include "zf_common_headfile.h"
#include "navigation.h"

int32 Nav_read[Read_MaxSize];//按5cm算的话,1000可以跑50m
NagTurn Turn_read[Turn_MaxSize];        //自转事件数组
Nag N;

int stay;
float error,count;
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取偏航角的线程函数
// 参数说明     读取偏航角的线程函数，通过切换N.End_f来切换线程
// 返回参数     void
// 使用示例     用户无需调用
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Nag_Read()
{
        switch(N.End_f)
        {
            case 0:Run_Nag_Save();  //默认执行函数
                break;
            case 1:;
                    flash_Nag_Write();  //写入最后一页，保证falsh存储满
                    N.End_f++;
                    break;
            case 2://Buzzer_check(500);   //蜂鸣器确认执行
                    N.End_f++;  //结束线程
                    break;
        }
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     用于生成偏差计算
// 参数说明     N.Final_Out为最终生成的偏差大小
// 返回参数     void
// 使用示例     用户无需调用
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Nag_Run()
{
    Run_Nag_GPS();  //偏航角读取复现
    if(N.Nag_Stop_f) //防止旋转
    {
        N.Final_Out=0;
        PID_leg.target_val=0;  
        PID_dir.Kp=0;
        return;
    }
  N.Final_Out=imu660rc_yaw-N.Angle_Run;
    
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     偏航角存入
// 参数说明     将读取的YAW存储到flash中存储
// 返回参数     void
// 使用示例     用户无需调用
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Run_Nag_Save()
{
    N.Mileage_All += (int)((abs((int)L_Mileage) + abs((int)R_Mileage)) / 2.0f);//历程计读取，左右编码器，使用浮点数的话误差能保留下来

    if(N.size > MaxSize)//当大于这页有的flash大小的时候，写入一次，防止重复写入
    {
        flash_Nag_Write();
        N.size=0;   //索引重置为0从下一个缓冲区开始读取
        N.Flash_page_index--;   //flash页面索引减小
        zf_assert(N.Flash_page_index > Nag_End_Page);//防止越界报错
    }

    if(N.Mileage_All >= Nag_Set_mileage)    //大于你的设定值的时候
    {
       int32 Save=(int32)(Nag_Yaw*100); //读取的偏航角放大100倍，避免使用Float类型来存储
       flash_union_buffer[N.size++].int32_type = Save;  //将偏航角写入缓冲区

       N.Save_index++;


       if(N.Mileage_All > 0) N.Mileage_All -= Nag_Set_mileage;//重置历程计数字//保存到flash
       else N.Mileage_All += Nag_Set_mileage;//倒车
    }

}
// 偏航角复现
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     偏航角复现
// 参数说明     读取flash中存储的YAW
// 返回参数     void
// 使用示例     用户无需调用
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Run_Nag_GPS()
{
    uint16 prospect = 0;

    if(N.Turn_Run_State == Nag_Turn_State_Brake)
    {
        PID_leg.target_val = -Nag_Turn_Brake_Speed;
        N.Mileage_All += (int)((abs((int)L_Mileage) + abs((int)R_Mileage)) / 2.0f);
        prospect = N.Run_index;

        if(prospect > N.Save_index - 2)         prospect = N.Save_index - 2;

        N.Angle_Run = Nav_read[prospect] / 100.0f; 

        if(N.Mileage_All >= Nag_Set_mileage)
        {
            N.Mileage_All -= Nag_Set_mileage;

            if(N.Run_index < Turn_read[N.Turn_Run_index].start_index)       N.Run_index++;

            if(N.Run_index >= Turn_read[N.Turn_Run_index].start_index)
            {
                uint16 start_index = Turn_read[N.Turn_Run_index].start_index;

                if(start_index + 1 >= N.Save_index)
                {
                    N.Nag_Stop_f++;
                    return;
                }

                float start_angle = Nav_read[start_index] / 100.0f;
                float next_angle = Nav_read[start_index + 1] / 100.0f;
                float diff = Nag_Angle_Diff(next_angle, start_angle);

                N.Turn_Target_Angle = 720.0f + fabs(diff);
                N.Turn_Angle_All = 0;
                N.Turn_Last_Yaw = angle_n;

                N.Turn_Run_State = Nag_Turn_State_Turn;
                N.Mileage_All = 0;

                PID_leg.target_val = 0;
                PID_dir.target_val = 0;
            }
        }

        return;
    }

    if(N.Turn_Run_State == Nag_Turn_State_Turn)
    {
        float delta = Nag_Angle_Diff(angle_n, N.Turn_Last_Yaw);

        N.Turn_Last_Yaw = angle_n;
        N.Turn_Angle_All += fabs(delta);

        PID_leg.target_val = 0;

        if(Turn_read[N.Turn_Run_index].direction == Nag_Turn_Right)       PID_dir.target_val = -Nag_Turn_Gyro_Target;
        else if(Turn_read[N.Turn_Run_index].direction == Nag_Turn_Left)   PID_dir.target_val = Nag_Turn_Gyro_Target;

        if(N.Turn_Angle_All >= N.Turn_Target_Angle)
        {
            N.Turn_Run_State = Nag_Turn_State_Path;
            N.Turn_Run_index++;

            PID_dir.target_val = 0;
            PID_leg.target_val = Nag_Run_Speed;
            N.Mileage_All = 0;

            prospect = N.Run_index;

            if(prospect > N.Save_index - 2)           prospect = N.Save_index - 2;

            N.Angle_Run = Nav_read[prospect] / 100.0f;
        }

        return;
    }

    N.Mileage_All += (int)((abs((int)L_Mileage) + abs((int)R_Mileage)) / 2.0f);

    if(N.Turn_Run_State == Nag_Turn_State_Path
       && N.Turn_Run_index < N.Turn_Save_index)
    {
        uint16 turn_start = Turn_read[N.Turn_Run_index].start_index;
        uint16 brake_start = 0;

        if(turn_start > Nag_Turn_Brake_PreCount)          brake_start = turn_start - Nag_Turn_Brake_PreCount;
        else         brake_start = turn_start;

        if(N.Run_index >= brake_start)
        {
            N.Turn_Run_State = Nag_Turn_State_Brake;
            PID_leg.target_val = -Nag_Turn_Brake_Speed;
            return;
        }
    }

    if(N.Mileage_All >= Nag_Set_mileage)
    {
        if(N.Run_index > N.Save_index - 2)
        {
            N.Nag_Stop_f++;
            return;
        }

        N.Run_index++;
        prospect = N.Run_index;

        if(prospect > N.Save_index - 2)     prospect = N.Save_index - 2;

        N.Angle_Run = Nav_read[prospect] / 100.0f;
        N.Mileage_All -= Nag_Set_mileage;
    }
}




//-------------------------------------------------------------------------------------------------------------------
// 函数简介     惯导参数初始化
// 返回参数     void
// 使用示例     放入程序执行开始
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Init_Nag()
{
    memset(&N, 0, sizeof(N));
    memset(Turn_read, 0, sizeof(Turn_read));
    N.Flash_page_index = Nag_Start_Page;
    flash_buffer_clear();
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     惯性导航执行函数
// 参数说明     index           索引
// 参数说明     type            类型值
// 返回参数     void
// 使用示例     放入中断中
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Nag_System(){
    //卫保护
    if(!N.Nag_SystemRun_Index || N.Nag_Stop_f )  return;

    switch(N.Nag_SystemRun_Index)
    {
       case 1 : Nag_Read();    //1是读取
            break;
      case 3: Nag_Run();
            break;
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     一次性读取程序，只读取一次！
// 参数说明     index           索引
// 参数说明     type            类型值
// 返回参数     void
// 使用示例     放入主函数直接调用，demo中有示例。
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void NagFlashRead(){
  if(N.Save_state) return;
  flash_Nag_Read();
  uint8 page_trun=0;
  
  for(int index=0;index <= N.Save_index;index++)
  {
    if(index >= N.Save_index)
    {
        N.Save_state=1;
        break;
    }
    int temp_index=index-(500*page_trun);
    if(temp_index >=MaxSize)    //当大于设定的flsh大小的时候
    {
        N.Flash_page_index--;   //页面减少
        page_trun++;
        flash_Nag_Read(); //重新读取
    }
     Nav_read[index]= flash_union_buffer[index-(500*page_trun)].int32_type;
  }

  flash_Turn_Read();

  N.Nag_SystemRun_Index++;
  PID_leg.target_val =  300;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     记录自转开始点flash_Turn_Read();
// 参数说明     direction       自转方向，Nag_Turn_Left 为左转，Nag_Turn_Right 为右转
// 返回参数     void
// 使用示例     Nag_Turn_Start(Nag_Turn_Left);
// 备注信息     仅在惯导记录模式下有效，只记录开始路径计值和方向，不重置路径计值
//-------------------------------------------------------------------------------------------------------------------

float Nag_Angle_Diff(float target, float current)
{
    float diff = target - current;

    while(diff > 180.0f) diff -= 360.0f;
    while(diff < -180.0f) diff += 360.0f;

    return diff;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     记录自转开始点flash_Turn_Read();
// 参数说明     direction       自转方向，Nag_Turn_Left 为左转，Nag_Turn_Right 为右转
// 返回参数     void
// 使用示例     Nag_Turn_Start(Nag_Turn_Left);
// 备注信息     仅在惯导记录模式下有效，只记录开始路径计值和方向，不重置路径计值
//-------------------------------------------------------------------------------------------------------------------
void Nag_Turn_Start(int8 direction)
{
    if(N.Nag_SystemRun_Index != 1) return;
    if(N.Turn_Save_index >= Turn_MaxSize) return;
    if(direction != Nag_Turn_Left && direction != Nag_Turn_Right) return;

    Turn_read[N.Turn_Save_index].start_index = N.Save_index;
    Turn_read[N.Turn_Save_index].direction = direction;

    N.Turn_Save_index++;

    flash_Turn_Write();
}


