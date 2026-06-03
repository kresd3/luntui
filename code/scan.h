#ifndef CODE_SCAN_H_
#define CODE_SCAN_H_

extern int jumpaaaa,jumpaaaa_flag;

extern int CX,CY,DX,DY,AX,AY,BX,BY;
extern int A_finish_flag,B_finish_flag,C_finish_flag,D_finish_flag;
extern int A_kur,B_kur;
extern int Alin[8],Blin[8],Clin[8],Dlin[8];

extern int Rline_zhaodian[MT9V03X_H/2],Lline_zhaodian[MT9V03X_H/2],Mline_zhaodian[MT9V03X_H/2];
extern int Rline[MT9V03X_H/2],Lline[MT9V03X_H/2],Mline[MT9V03X_H/2];
extern int White_Column[MT9V03X_W/2],zchBailie;
extern int l_empty,r_empty,last_r_empty,last_l_empty;
extern float error_dir,last_error_dir;
extern uint32 time_ms;
extern int element_type;
extern int motor_type;
extern int danbianqiao_flag,jump_flag,jump_time1;

extern int canshu;
extern int jichu_sudu;
extern float tuibu_xianfu;
extern int qianzhan;
extern int jiansu_time;
extern int dbqiao_time;
extern int dbqiao_tuigao;
extern int jump_qianzhan;
extern int dbq_tbxianfu;
extern float roll_kp;
extern int podao_time1;
extern int podao_time2;
extern int podao_speed;
extern float dbq_yaw_ref;

typedef enum
{
    Wu                              = 0,                                    // 无特殊元素
    PoDao                           = 1,                                    // 坡道
    DanBianQiao                     = 2,                                    // 单边桥
    Jump                            = 3,                                    // 台阶
}element_type_enum;

typedef enum
{
    TiaoShi                          = 0,                                    // 调试模式
    BiSai                           = 1,                                    // 比赛模式
}motor_type_enum;


void zhaodian(void);
void linyu(void);
void element_judgment(void);
void element_perform(void);
void scan(void);
void Jump_control(void);

#endif /* CODE_SCAN_H_ */
