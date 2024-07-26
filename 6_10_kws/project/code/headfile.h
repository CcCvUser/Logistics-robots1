/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2019,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		headfile
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
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

//------文件系统相关头文件
#include "ff.h"
#include "diskio.h"
#include "fsl_sd_disk.h"

//#include "SEEKFREE_PRINTF.h"

#include "run_kws_model_demo.h"

typedef enum
{
    KEY_U=0,  //上
    KEY_D,  //下

    KEY_L,  //左
    KEY_R,  //右s

    KEY_B,  //选择

    KEY_RUN,  //开始
    KEY_LCD_DISPLAY,   //停止

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


//自用头文件
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

