/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2019,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		headfile
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		IAR 8.3 or MDK 5.28
 * @Target core		NXP RT1064DVL6A
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2019-04-30
 ********************************************************************************************************************/
 
#ifndef _headfile_h
#define _headfile_h

#include <math.h>
#include <stdint.h>
#include "fsl_common.h"

#include "fsl_debug_console.h"
#include "fsl_iomuxc.h"
#include "fsl_pit.h"

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_cache.h"
#include "zf_common_headfile.h"
//#include "zf_vector.h"

//------�ļ�ϵͳ���ͷ�ļ�
#include "ff.h"
#include "diskio.h"
#include "fsl_sd_disk.h"

//#include "SEEKFREE_PRINTF.h"

#include "run_kws_model_demo.h"

typedef enum
{
    KEY_U=0,  //��
    KEY_D,  //��

    KEY_L,  //��
    KEY_R,  //��s

    KEY_B,  //ѡ��

    KEY_RUN,  //��ʼ
    KEY_LCD_DISPLAY,   //ֹͣ

    KEY_MAX,
} KEY_e;

struct target{
  float x;
  float y;
  //int id;
  int sigma;
  int main_class;
  int second_class;
  int real_class;
  uint8 ori;
};
typedef struct target target;
//#include "SEEKFREE_IIC.h"


//����ͷ�ļ�
#include "pid.h"
#include "Attitude_Calculation.h"
#include "key.h"
#include "lcd_menu.h"
#include "motor.h"
#include "mymooncake.h"
#include "bmx055.h"
#include "UARTset.h"
#include "deal_img.h"
#include "lcd_show.h"
#include "VCAN_VisualScope.h"
#include "beep.h"
#include "zap.h"
#include "LinkQueue.h" 


#endif

