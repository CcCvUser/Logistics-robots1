#ifndef _LCD_MENU_H
#define _LCD_MENU_H

#include "headfile.h"

//整个菜单页面的参数
typedef struct MENU_PARM
{
uint8_t ExitMark;//退出菜单(0-不退出，1-退出)
int cursor;//光标位置
int param;//页面多少想要执行的东西
}MENU_PARM;
//单个菜单选项的参数
typedef struct MENU_FORM
{
int8 *name;//菜单名字
void (*function)(void);//菜单函数
float *parm;//菜单要调试的参数
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
void menu_display(MENU_PARM *pr,MENU_FORM *mmp);//前一个是整个页面的，后一个是单个参数
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