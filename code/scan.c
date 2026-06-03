#include "zf_common_headfile.h"

int jumpaaaa=0,jumpaaaa_flag=0;

int CX,CY,DX,DY,AX,AY,BX,BY;
int A_finish_flag,B_finish_flag,C_finish_flag,D_finish_flag;
float kl,kr;
int Rline_zhaodian[MT9V03X_H/2],Lline_zhaodian[MT9V03X_H/2],Mline_zhaodian[MT9V03X_H/2];
int Rline[MT9V03X_H/2],Lline[MT9V03X_H/2],Mline[MT9V03X_H/2];
int White_Column[MT9V03X_W/2],zchBailie;
int l_empty=0,r_empty=0,last_r_empty=0,last_l_empty=0;
float error_dir,last_error_dir;
int element_type;
int motor_type;
int Alin[8],Blin[8],Clin[8],Dlin[8];
int A_kur=0,B_kur=0;
int empty_flag=0;
int DBQ_time1,DBQ_time2,DBQ_time3;
int jump_time1,jump_time2;
int jump_count=0;
int PoDao_time1,PoDao_time2;
uint32 time_ms=0;
int danbianqiao_flag=0,jump_flag=0;
int cccc=0;
int zhongxian,qianzhan_jin,qianzhan_yuan;
int distence;
int aa;

float dbq_yaw_ref = 0;
int canshu=1;
int jichu_sudu = 950;//810,1070
float tuibu_xianfu = 2.5;
int qianzhan = 45;//36
int jiansu_time = 820;//400    250             300
int dbqiao_time = 1800;//2000   900            900
int dbqiao_tuigao = 50;//59    35
int jump_qianzhan = 23;//20
int dbq_tbxianfu = 6;//7       7
float roll_kp = 0.0037;//0.0029

void linyu()//遍历拐点邻域
{
    A_kur=0;
    B_kur=0;
    for(int i=0;i<=2;i++)
    {
        Alin[i]=image_sobel[AY+2][AX-i];
        Blin[i]=image_sobel[BY+2][BX+i];
        Clin[i]=image_sobel[CY-i][CX-2];
        Dlin[i]=image_sobel[DY-i][DX+2];
    }
    for(int i=3;i<=6;i++)
    {
        Alin[i]=image_sobel[AY+4-i][AX-2];
        Blin[i]=image_sobel[BY+4-i][BX+2];
        Clin[i]=image_sobel[CY-2][CX-4+i];
        Dlin[i]=image_sobel[DY-2][DX+4-i];
    }
    Alin[7]=image_sobel[AY-2][AX-1];
    Blin[7]=image_sobel[BY-2][BX+1];
    Clin[7]=image_sobel[CY-1][CX+2];
    Dlin[7]=image_sobel[DY-1][DX-2];
    for(int i=0;i<=7;i++)
    {
        if(Alin[i]==0)A_kur++;//计算拐点峰度
        if(Blin[i]==0)B_kur++;
    }
}

int Jump_judgment()
{
//    int aa=Mline[35],bb=Mline[40],cc=Mline[45];
//    while(image_sobel[35][aa]==255&&aa<=92)
//    {
//        aa++;
//    }
//    while(image_sobel[40][bb]==255&&bb<=92)
//    {
//        bb++;
//    }
//    while(image_sobel[45][cc]==255&&cc<=92)
//    {
//        cc++;
//    }
//
//    int a=Mline[35],b=Mline[40],c=Mline[45];
//    while(image_sobel[35][a]==255&&a>=2)
//    {
//        a--;
//    }
//    while(image_sobel[40][b]==255&&b>=2)
//    {
//        b--;
//    }
//    while(image_sobel[45][c]==255&&c>=2)
//    {
//        c--;
//    }
//
//    int white=0;
//    for(int j=jump_qianzhan;j<=jump_qianzhan+1;j++)
//    {
//        for(int i=38;i<=56;i++)
//        {
//            if(image_sobel[j][i]==255)
//            {
//                white++;
//            }
//        }
//    }
//    if(white<=5&&time_ms>=600&&jump_flag==0&&abs((c-b)-(b-a))<=2&&a>10&&aa<84&&abs((cc-bb)-(bb-aa))<=2&&C_finish_flag&&D_finish_flag&&abs(DX-CX)<3&&image_sobel[20][CX]==255&&zchBailie<50)return 1;//
////    if(system_getval_ms()>=5000&&system_getval_ms()<=5300)return 1;
//    else return 0;

      if(jump_count>=1 || jump_flag!=0) return 0;
      if(dl1b_distance_mm<=90) return 1;
      else return 0;
}
void Jump_control()
{
    jump_time2=time_ms;

    jump_flag=1;
    PID_leg.Kp = 0.015;//0.015
    if(jump_time2-jump_time1<=100)
    {
        B_H=120;
    }
    if(jump_time2-jump_time1>100&&jump_time2-jump_time1<=200)
    {
        B_H=25;
        PID_leg.target_val=0;
    }
    if(jump_time2-jump_time1>250&&jump_time2-jump_time1<=270)
    {
        B_H=60;
        PID_leg.target_val=-200;//-1000
    }
    if(jump_time2-jump_time1>320&&jump_time2-jump_time1<=420)
    {
        B_H = 60-((jump_time2-jump_time1)-320)*0.15;
    }
    if(jump_time2-jump_time1>420)
    {
        PID_leg.target_val=200;
        B_X = 18.48;
        B_H = 45;
        PID_leg.Kp = 0.02;//0.03
        jump_flag=2;
        empty_flag=0;
        jumpaaaa_flag = 0;
    }
}


//int PoDao_judgement()
//{
//    int aa=Mline[35],bb=Mline[40],cc=Mline[45];
//    while(image_sobel[35][aa]==255&&aa<=92)
//    {
//        aa++;
//    }
//    while(image_sobel[40][bb]==255&&bb<=92)
//    {
//        bb++;
//    }
//    while(image_sobel[45][cc]==255&&cc<=92)
//    {
//        cc++;
//    }
//
//    int a=Mline[35],b=Mline[40],c=Mline[45];
//    while(image_sobel[35][a]==255&&a>=2)
//    {
//        a--;
//    }
//    while(image_sobel[40][b]==255&&b>=2)
//    {
//        b--;
//    }
//    while(image_sobel[45][c]==255&&c>=2)
//    {
//        c--;
//    }
//
//    if(dl1b_distance_mm<1200&&dl1b_distance_mm>800&&zchBailie>49&&time_ms>=5000&&abs((c-b)-(b-a))<=2&&abs((cc-bb)-(bb-aa))<=2&&a>17&&aa<77//
//            &&((A_finish_flag&&AY<25)||!A_finish_flag)
//            &&((B_finish_flag&&BY<25)||!B_finish_flag)
//            &&((C_finish_flag&&CY<25)||!C_finish_flag)
//            &&((D_finish_flag&&DY<25)||!D_finish_flag))return 1;
//    else return 0;
//}
//void PoDao_control()
//{
//    PoDao_time2 = system_getval_ms();
//
//    PID_leg.target_val= 0;
//    gpio_set_level(P22_0, 1);
//
//    PID_balance.Kp = 8.0;
//    PID_gyro.Kp = 0.007;
//    PID_leg.Kp = 0.02;
//    PID_leg.max = 5;//10
//    PID_leg.min = -5;//-10
//    PID_pitch.Kp = 10.0;
//    PID_leg.target_val=500;
//
//
//    if(PoDao_time2-PoDao_time1>1000)
//    {
//        B_H = 30;
//        PID_leg.Kp = 0.03;//0.03
////        PID_leg.target_val=750;//750
////        PID_out.target_val=750;//750
//        empty_flag=0;
//    }
//
//}
//
//
//
int DanBianQiao_judgment()
{

    int xx=Mline[DY+4],yy=Mline[DY+10],zz=Mline[DY+16];
    while(image_sobel[DY+4][xx]==255&&xx<=92)
    {
        xx++;
    }
    while(image_sobel[DY+10][yy]==255&&yy<=92)
    {
        yy++;
    }
    while(image_sobel[DY+16][zz]==255&&zz<=92)
    {
        zz++;
    }

    int x=Mline[CY+4],y=Mline[CY+10],z=Mline[CY+16];
    while(image_sobel[CY+4][x]==255&&x>=2)
    {
        x--;
    }
    while(image_sobel[CY+10][y]==255&&y>=2)
    {
        y--;
    }
    while(image_sobel[CY+16][z]==255&&z>=2)
    {
        z--;
    }

    int a=AX-1,aa=0,b=BX+1,bb=0;
    while(image_sobel[AY+1][a]==0&&a>=AX-11)
    {
        a--;
        aa++;
    }
    while(image_sobel[BY+1][b]==0&&b<=BX+11)
    {
        b++;
        bb++;
    }
    int d=CX,dd=DX;
    while(image_sobel[CY+3][d]==255&&d>=3)
    {
        d--;
    }
    while(image_sobel[DY+3][dd]==255&&dd<=90)
    {
        dd++;
    }

    int h=Mline[CY+2],hh=0,k=Mline[DY+2],kk=0;
    while(image_sobel[CY+2][h]==0&&h<=90)
    {
        h++;
        hh++;
    }
    while(image_sobel[DY+2][k]==0&&k>=3)
    {
        k--;
        kk++;
    }



    if(
            ((A_finish_flag&&C_finish_flag&&D_finish_flag&&image_sobel[(int)((CY+AY)/2)][(int)((CX+AX)/2)+2]==255&&abs((z-y)-(y-x))<=2&&abs((zz-yy)-(yy-xx))<=2&&x>10&&xx<84&&AY<CY&&abs(AX-CX)<5&&abs(AX-DX)<5)||//&&d-AX<((AY-20)*1.3+40)/2&&d-AX>4&&aa>=8,&&abs(CX-Mline[CY+3])<4
            (B_finish_flag&&D_finish_flag&&C_finish_flag&&image_sobel[(int)((DY+BY)/2)][(int)((DX+BX)/2)-2]==255&&abs((z-y)-(y-x))<=2&&abs((zz-yy)-(yy-xx))<=2&&x>10&&xx<84&&BY<DY&&abs(BX-DX)<5&&abs(BX-CX)<5))&&//&&BX-dd<((BY-20)*1.3+40)/2&&BX-dd>4&&bb>=8,&&abs(DX-Mline[DY+3])<4
            zchBailie>20&&zchBailie<55&&
            danbianqiao_flag==0&&
            time_ms>1100
       )
        return 1;
    else
    return 0;
}
void DanBianQiao_control()
{
    if(aa==0)
    {
      DBQ_time2=time_ms;
      aa=1;
    }
    qianzhan_jin = 33;//40
    qianzhan_yuan = 25;//33
    if(danbianqiao_flag==1&&time_ms-DBQ_time2<jiansu_time)
    {
    }
    if(danbianqiao_flag==1&&time_ms-DBQ_time2>=jiansu_time)//400
      {
        danbianqiao_flag=2;
        DBQ_time2=time_ms;
      }

    int a=Mline[35],b=Mline[45],c=47;
    while(image_sobel[35][a]==255&&a<=91)
    {
        a++;
    }
    while(image_sobel[45][b]==255&&b<=91)
    {
        b++;
    }
    while(image_sobel[55][c]==255&&c<=91)
    {
        c++;
    }
    int aa=Mline[35],bb=Mline[45],cc=47;
    while(image_sobel[35][aa]==255&&aa>=3)
    {
        aa--;
    }
    while(image_sobel[45][bb]==255&&bb>=3)
    {
        bb--;
    }
    while(image_sobel[55][cc]==255&&cc>=3)
    {
        cc--;
    }
    int d=47,e=47;
    while(image_sobel[58][d]==255&&c<=91)
    {
        d++;
    }
    while(image_sobel[58][e]==255&&cc>=3)
    {
        e--;
    }
    if(danbianqiao_flag==2&&((time_ms-DBQ_time2>=dbqiao_time+3200)||
//            (abs((c-b)-(b-a))<=3&&abs((cc-bb)-(bb-aa))<=3&&fabs(E_H)<=2&&abs(d-e)>=70&&DBQ_time2-DBQ_time1>=1500)
            (((!A_finish_flag)||(A_finish_flag&&AY<40))&&((!B_finish_flag)||(B_finish_flag&&BY<40))&&time_ms-DBQ_time2>=(dbqiao_time+3200)&&abs(d-e)>=60))
            )//abs((c-b)-(b-a))<=8&&abs((cc-bb)-(bb-aa))<=8&&fabs(E_H)<=8&&abs(d-e)>=60
    {
        danbianqiao_flag=3;
        DBQ_time3=time_ms;
    }
    if(danbianqiao_flag==3&&DBQ_time2-DBQ_time3>=0)
    {
      //  PID_leg.Kp = 0.03;
       PID_out.Kp = -90.0;
     //   PID_balance.Kp = 8.3;
        PID_leg.max = 12;//
        PID_leg.min = -12;//
        empty_flag=0;
    }
}


void element_judgment()
{
    if(jump_flag==2 && dl1b_distance_mm>200)
    {
        jump_flag=0;
    }

    if(empty_flag==0)
    {
         if( DanBianQiao_judgment() )
         {
              element_type = DanBianQiao;
              danbianqiao_flag=1;
              DBQ_time1=time_ms;
              empty_flag=1;
              PID_out.Kp=0;
              dbq_yaw_ref=angle_n;
            }
        else if( Jump_judgment() )
        {
            element_type = Jump;
            jump_flag=1;
            jump_count++;
            empty_flag=1;
            PID_pitch.Error=0;;
            jump_time1=time_ms;
        }
        else
            {
                element_type=Wu;
            }

    }


}

void element_perform()
{
    switch(element_type)
    {
 //       case DanBianQiao : DanBianQiao_control();break;
//
 //       case Jump : Jump_control();break;
//
//        case PoDao : PoDao_control();break;

        case Wu :
        {
//                if(time_ms>podao_time1&&time_ms<podao_time2)
//                {
//                    PID_leg.target_val = podao_speed;
//                }
                zhongxian = 49;//47
                qianzhan_jin = 33;//33
                qianzhan_yuan = qianzhan - (int)(0.003*car_speed);//
        }break;
    }
}

float track()
{
    int sumjin=0,sumyuan=0;
    float error=0;

        for(int i=5;i>0;i--)
        {
            sumjin+=Mline[qianzhan_jin-i];//33
            sumyuan+=Mline[qianzhan_yuan-i];
        }
        error = (sumjin*0.1+sumyuan*0.1)-zhongxian;
        if(error>50)error = 50;
        if(error<-50)error = -50;

    return (sumjin*0.0+sumyuan*0.2)-zhongxian ;//47
}

void zhaodian()//寻找图像拐点
{
    int ii=0;
    Mline_zhaodian[55]=47;
    Rline_zhaodian[0]=94;
    Lline_zhaodian[0]=0;

    A_finish_flag = 0;
    B_finish_flag = 0;
    C_finish_flag = 0;
    D_finish_flag = 0;

    for(int i=55;i>1;i--)
    {
        if(i==55)
        {
        Rline_zhaodian[i]=Mline_zhaodian[55];
        Lline_zhaodian[i]=Mline_zhaodian[55];
        }else if(image_sobel[i][Mline_zhaodian[i+1]]==255)
        {
            Rline_zhaodian[i]=Mline_zhaodian[i+1];
            Lline_zhaodian[i]=Mline_zhaodian[i+1];
        }
        else
        {
            if(image_sobel[i][Mline_zhaodian[i+1]+(int)(((i-20)*1.3+40)/4.0)]==255)
            {
                Rline_zhaodian[i]=(int)(Mline_zhaodian[i+1]+((i-20)*1.3+40)/4.0);
                Lline_zhaodian[i]=(int)(Mline_zhaodian[i+1]+((i-20)*1.3+40)/4.0);
            }
            else if(image_sobel[i][Mline_zhaodian[i+1]-(int)(((i-20)*1.3+40)/4.0)]==255)
            {
                Rline_zhaodian[i]=(int)(Mline_zhaodian[i+1]-((i-20)*1.3+40)/4.0);
                Lline_zhaodian[i]=(int)(Mline_zhaodian[i+1]-((i-20)*1.3+40)/4.0);
            }
            else
            {
                ii=i;
                Rline_zhaodian[i]=47;
                Lline_zhaodian[i]=47;
                break;
            }
        }
        while(!(image_sobel[i][Rline_zhaodian[i]-1]==255&&image_sobel[i][Rline_zhaodian[i]]==0)&&Rline_zhaodian[i]<94)
        {
            Rline_zhaodian[i]++;
        }
        while(!(image_sobel[i][Lline_zhaodian[i]+1]==255&&image_sobel[i][Lline_zhaodian[i]]==0)&&Lline_zhaodian[i]>0)
        {
            Lline_zhaodian[i]--;
        }

        if(Rline_zhaodian[i]<=0)Rline_zhaodian[i]=0;
        if(Rline_zhaodian[i]>=94)Rline_zhaodian[i]=94;
        if(Lline_zhaodian[i]<=0)Lline_zhaodian[i]=0;
        if(Lline_zhaodian[i]>=94)Lline_zhaodian[i]=94;

        Mline_zhaodian[i]=(Rline_zhaodian[i]+Lline_zhaodian[i])/2;
    }

    for(int i=55;i>ii;i--)
    {

        if(i<54&&i>8){
        if(i>8&&i<50)
        {
            if(!A_finish_flag)
            {
                if(Lline_zhaodian[i]-2*Lline_zhaodian[i-2]+Lline_zhaodian[i-4]<=-3&&abs(Lline_zhaodian[i]-Lline_zhaodian[i-2])<=3)
                {
                    AX = Lline_zhaodian[i-2];
                    AY = i-2;
                    A_finish_flag = 1;
                }
            }
            if(!B_finish_flag)
            {
                if(Rline_zhaodian[i]-2*Rline_zhaodian[i-2]+Rline_zhaodian[i-4]>=3&&abs(Rline_zhaodian[i]-Rline_zhaodian[i-2])<=3)
                {
                    BX = Rline_zhaodian[i-2];
                    BY = i-2;
                    B_finish_flag = 1;
                }
            }
        }
        if(i<55)
        {
            if(!C_finish_flag)
            {
                if(Lline_zhaodian[i]-2*Lline_zhaodian[i-2]+Lline_zhaodian[i-4]<=-10&&abs(Lline_zhaodian[i-2]-Lline_zhaodian[i-4])<=3)
                {
                    CX = Lline_zhaodian[i-2];
                    CY = i-2;
                    C_finish_flag = 1;
                }
            }
            if(!D_finish_flag)
            {
                if(Rline_zhaodian[i]-2*Rline_zhaodian[i-2]+Rline_zhaodian[i-4]>=10&&abs(Rline_zhaodian[i-2]-Rline_zhaodian[i-4])<=3)
                {
                    DX = Rline_zhaodian[i-2];
                    DY = i-2;
                    D_finish_flag = 1;
                }
            }
        }
        }
    }

    linyu();

    for(int i=23;i<=71;i+=2)
    {
        White_Column[i] = 0;
        for (int j = 59; j >= 0; j--)
        {
            if(image_sobel[j][i] == 0)
                break;
            else
                White_Column[i]++;
        }
    }
    zchBailie = 0;
    for(int i=23;i<=71;i++)
    {
        if (zchBailie < White_Column[i])//找最长的那一列
        {
            zchBailie = White_Column[i];//【0】是白列长度
        }
    }
}


void scan()
{
    Mline[49]=47;
    Rline[0]=94;
    Lline[0]=0;

    l_empty=0;r_empty=0;
    for(int i=49;i>1;i--)
    {
        if(i==49)
        {
        Rline[i]=Mline[49];
        Lline[i]=Mline[49];
        }else if(image_sobel[i][Mline[i+1]]==255)
        {
            Rline[i]=Mline[i+1];
            Lline[i]=Mline[i+1];
        }
        else
        {
            if(image_sobel[i][Mline[i+1]+(int)(((i-20)*1.3+40)/4.0)]==255)
            {
                Rline[i]=(int)(Mline[i+1]+((i-20)*1.3+40)/4.0);
                Lline[i]=(int)(Mline[i+1]+((i-20)*1.3+40)/4.0);
            }
            else if(image_sobel[i][Mline[i+1]-(int)(((i-20)*1.3+40)/4.0)]==255)
            {
                Rline[i]=(int)(Mline[i+1]-((i-20)*1.3+40)/4.0);
                Lline[i]=(int)(Mline[i+1]-((i-20)*1.3+40)/4.0);
            }
            else
            {
                Rline[i]=47;
                Lline[i]=47;
            }
        }
        while(image_sobel[i][Rline[i]]==255&&Rline[i]<94)
        {
            Rline[i]++;
        }
        while(image_sobel[i][Lline[i]]==255&&Lline[i]>0)
        {
            Lline[i]--;
        }

        if(Rline[i]<=0)Rline[i]=0;
        if(Rline[i]>=94)Rline[i]=94;
        if(Lline[i]<=0)Lline[i]=0;
        if(Lline[i]>=94)Lline[i]=94;

                if(Rline[i]>=91)r_empty++;
                if(Lline[i]<=2)l_empty++;

         Mline[i]=(int)(((Rline[i]+Lline[i])/2)+(r_empty-l_empty)*0.8);//0.8

    }
    error_dir=track();
    last_error_dir=error_dir;
}



























