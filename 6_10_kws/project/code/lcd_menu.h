#ifndef _LCD_MENU_H
#define _LCD_MENU_H

#include "headfile.h"

//�����˵�ҳ��Ĳ���
typedef struct MENU_PARM
{
uint8_t ExitMark;//�˳��˵�(0-���˳���1-�˳�)
int cursor;//���λ��
int param;//ҳ�������Ҫִ�еĶ���
}MENU_PARM;
//�����˵�ѡ��Ĳ���
typedef struct MENU_FORM
{
int8 *name;//�˵�����
void (*function)(void);//�˵�����
float *parm;//�˵�Ҫ���ԵĲ���
}MENU_FORM;

typedef struct
{
    uint16 x;
    uint16 y;
}Site_t;

void servo_change(void);
void uart_test();



void magnet_pin_test();
void car2_test();
void send_ST_test();
void receive_unknownTGT_test();
void take_photo();
void servo_test_all();
void LB_test();
void LT_test();
void RB_test();
void RT_test();
void motor_test_all();
void img_test();
void uart4_class_test();
void uart4_TGT_test();
void encoder_test();
void motor_test();
void goback();
void runTMDgo(void);
void menu_progress(MENU_PARM *parm,MENU_FORM *mf);
void menu_display(MENU_PARM *pr,MENU_FORM *mmp);//ǰһ��������ҳ��ģ���һ���ǵ�������
void sidelight(void);
void second_pid();
void pid_change(void);
void pid_change2(void);
void gofirst(void);
void second_main_page(void);
void Menu_PrmtInit(MENU_PARM *prmt);
void savedata();
void read_data();
void goto_pid();
void MainMenu_Set(void);
void run_test();
void run_test2();
void run_test3();
void run_test4();
void servo_test();
#endif