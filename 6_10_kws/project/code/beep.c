/*
 * beep.c
 *
 *  Created on: 2022��7��28��
 *      Author: zh'n
 */
#include "beep.h"
#include "headfile.h"
int buzzerTime ;
void BEEP_init(){
  //tft180_show_string(0, 0,"BEEP_init"); 

    gpio_init(BEEP_ENABLE, GPO, 0, SPEED_100MHZ | DSE_R0 | PULLUP_47K | PULL_EN);
  //tft180_show_string(0, 16,"BEEP_init_finished"); 
}
void BEEP_on()
{
//�����������ƽ�����ߵ�ƽ����
    gpio_set_level(BEEP_ENABLE, 1);
}
//�ر�
void BEEP_off()
{
//�����������ƽ�����͵�ƽ���ر�
    gpio_set_level(BEEP_ENABLE, 0);
}


void BEEP_deal(){
  if (buzzerTime > 0){
    buzzerTime-=40;
    if (buzzerTime <= 0){
        buzzerTime=0;
        BEEP_off();
    }
  }
}

