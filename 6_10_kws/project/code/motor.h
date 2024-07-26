#ifndef _MOTOR_H
#define _MOTOR_H

#include "headfile.h"
float Speed_now(int16 dat);
float Distance(float dat);
#define encoder_period  (float)0.01

#define MOTOR_PWM_MAX 8000  //�޷�
#define MOTOR_PWM_MIN -8000 //�޷�
#define wheel_angle 47.5   //ת��ĽǶ�
#define wheel_diam 6.3

#define SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))    //����Ƕ�����Ϊ0 - 180��90��Ϊ��ֵ//200-333-1000
#define SERVO_MOTOR_FREQ                (50 )    
#define servo_pin_1   PWM1_MODULE2_CHB_D17       //������������ p14
#define servo_pin_2   PWM4_MODULE0_CHA_B24       //������������ p12
#define servo_pin_3   PWM4_MODULE2_CHA_C30       //������������ p10
//#define servo_pin_4   PWM4_MODULE3_CHA_C31       //����������

#define magnet_pin1 B9 
#define magnet_pin2 C26 
#define magnet_pin3 B10 
#define magnet_pin4 C27 
#define magnet_pin5 B29 
typedef struct Encoder 
{
int16 ed_1;
int16 ed_2;
int16 ed_3;
int16 ed_4;
}Encoder;

typedef struct IncrementalPID    //����ʽPID
{

    float REF;//�ο���
    float feedBack;//����

    float preError;//ǰ�����
    float prePreError;//ǰǰ�����

    float PID_Out;

} IncrementalPID;
//pp->U_D = kd_t * (error - pp->preError) * 0.5 + 0.5 * pp->U_D;  ��ʵ������Ǽ�����һ����ͨ�˲�
//���ٻ��ֵ�PID���Ƶ��ŵ㣨����ͨPID��ȣ���
//     �� ʵ�����ñ�������������ƫ��û�����������Сƫ�������������ԣ��Ӷ���ȫ�����˻��ֱ�������
//     �� ����С�˳����������Ժ����׵�ʹϵͳ�ȶ��������˵�����Ʒ��
//     �� ��Ӧ����ǿ��һЩ�ó���PID���Ʋ�����Ĺ��̿��Բ��ô����㷨
//     �� �����������ף�����������໥Ӱ��С
//�� ����ַ���ıȽϣ�
//     �� ���ߺ����ƣ������ڷ�ʽ��ͬ�����ַ���Ի�������á����ء����ƣ������ٻ������Ǹ������Ĵ�С�ı�������ٶȣ������Կ��ơ���������ߵ���Ʒ�ʴ�Ϊ��ߣ���һ�����͵�PID����

typedef struct Speed_Info
{   float distance;
    float lastdistance;
    float aimdistance;
    float nowSpeed;
    float varLT[3];                //������ֵ
    float varRT[3];                //������ֵ
    float varLB[3];                //������ֵ
    float varRB[3];                //������ֵ
    float encodernow[3];              //����������

    float nowSpeedLT;              //����ʵ���ٶ�
    float nowSpeedRT;              //����ʵ���ٶ�
    float nowSpeedLB;              //����ʵ���ٶ�
    float nowSpeedRB;              //����ʵ���ٶ�
    float aimSpeed;	            //Ŀ���ٶ�
} Speed_Info;


extern int servo_duty;

void motor_crt(void);
void init_all(void);
void move_left(void);
void move_right(void);
void move_ahead(void);
void move_behind(void);
void move_stop(void);
void move_leftad(void);
void move_leftbh(void);
void move_rightad(void);
void move_rightbh(void);
void move_turnright(void);
void move_turn(int a);
void motor_init();
void move_set(int32 a,int32 b,int32 c,int32 d);
void servo_control_se(int *flag,int *cnt);
void move_set_all(int a);
void SpeedInfo_Init();
void Encoder_clear_all();
//void getEncoder_all_1();
void get_site_data();
float pwm_protect(float a);
void pwm_protect_pro(int a[]);
void speed_protect_pro(float *speed_val,int max);
void speed_protect(float *speed_val);
void speed_protect_1(float *speed_val);
//void getEncoder_all();

void speed_protect_pro_2(float *speed_val_x,float *speed_val_y,int max);

#endif