#ifndef _MOTOR_H
#define _MOTOR_H

#include "headfile.h"
float Speed_now(int16 dat);
float Distance(float dat);
#define encoder_period  (float)0.01

#define MOTOR_PWM_MAX 8000  //限幅
#define MOTOR_PWM_MIN -8000 //限幅
#define wheel_angle 47.5   //转轴的角度
#define wheel_diam 6.3

#define SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))    //舵机角度设置为0 - 180，90度为中值//200-333-1000
#define SERVO_MOTOR_FREQ                (50 )    
#define servo_pin_1   PWM1_MODULE2_CHB_D17       //定义舵机引脚下 p14
#define servo_pin_2   PWM4_MODULE0_CHA_B24       //定义舵机引脚上 p12
#define servo_pin_3   PWM4_MODULE2_CHA_C30       //定义舵机引脚中 p10
//#define servo_pin_4   PWM4_MODULE3_CHA_C31       //定义舵机引脚

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

typedef struct IncrementalPID    //增量式PID
{

    float REF;//参考量
    float feedBack;//反馈

    float preError;//前次误差
    float prePreError;//前前次误差

    float PID_Out;

} IncrementalPID;
//pp->U_D = kd_t * (error - pp->preError) * 0.5 + 0.5 * pp->U_D;  其实这差不多就是加上了一个低通滤波
//变速积分的PID控制的优点（与普通PID相比）：
//     — 实现了用比例作用消除大偏差，用积分作用消除小偏差的理想调节特性，从而完全消除了积分饱和现象
//     — 大大减小了超调量，可以很容易地使系统稳定，改善了调节特品质
//     — 适应能力强，一些用常规PID控制不理想的过程可以采用此种算法
//     — 参数整定容易，各参数间的相互影响小
//★ 与积分分离的比较：
//     — 二者很类似，但调节方式不同。积分分离对积分项采用“开关”控制，而变速积分则是根据误差的大小改变积分项速度，属线性控制。因而，后者调节品质大为提高，是一种新型的PID控制

typedef struct Speed_Info
{   float distance;
    float lastdistance;
    float aimdistance;
    float nowSpeed;
    float varLT[3];                //编码器值
    float varRT[3];                //编码器值
    float varLB[3];                //编码器值
    float varRB[3];                //编码器值
    float encodernow[3];              //编码器总量

    float nowSpeedLT;              //左轮实际速度
    float nowSpeedRT;              //右轮实际速度
    float nowSpeedLB;              //左轮实际速度
    float nowSpeedRB;              //右轮实际速度
    float aimSpeed;	            //目标速度
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