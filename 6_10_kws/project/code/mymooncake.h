#ifndef _MYMOONCAKE_H
#define _MYMOONCAKE_H
#include "headfile.h"
  
#define x_max 700  //单位cm
#define y_max 500

#define fruit 1
#define vegetable 2
#define grain 3

#define put_limit_length 10
#define put_limit_width 10


typedef struct site{
  float x;
  float y;
}site;

void get_class();




int UpdateTGT(target newTGT);
int RectIntersect(target TGT1,target TGT2);

void kws_run();
void run_back_se();
void run_back_servo_test();
void send_nowTGT_aimST();
void send_TGT_Upper_computer();
void receive_unknownTGT(uint8 uart_receive[]);
void get_error(uint8 uart_receive[]);
void updata_STcar2();
void get_TGT();
float calculate_distance(float x,float y,float aimx,float aimy);
float calculate_angle(float x1,float y1,float aimx,float aimy);
void clear_class(int a[]);
void send_carsite(site *ST);
void receive_carsite(site *ST);
void judge_send_TGTinfo();
void send_uart4receive();//car1接受到的摄像头数据直接发给car2

void send_nowTGT_ori(site *ST,int flag);
void send_TGT();
void servo_only();
void run_triangle();
void run_straight();
void run_xy();
void run_xy_pro();
void run_back();
void run_xy_2();

void img_intervene(int *flag);
void fast_debug();

float abs_float(float m);

void run_TMD();

#endif