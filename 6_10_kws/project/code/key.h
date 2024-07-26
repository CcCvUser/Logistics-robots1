#ifndef _KEY_H
#define _KEY_H
#include "zf_common_font.h"



#define ZAP_BOARD_version2


#ifdef STUDY_BOARD

#define KEYU    C27  //上
#define KEYD    C31  //下
#define KEYL    C4  //左
#define KEYR    C26  //右
//定义拨码开关引脚
#define KEYEnter C26

#define SW1     D4  
#define SW2     D27  
#endif


#ifdef ZAP_BOARD_version1

#define KEYU    B10 
#define KEYD    B9  
#define KEYL    C27  
#define KEYR    C26 
#define KEYEnter C26

#define SW1     D4  
#define SW2     D27   
#endif


#ifdef ZAP_BOARD_version2

#define KEYU    C4 
#define KEYD    D16  
#define KEYL    D26  
#define KEYR    D27  
#define KEYEnter D4

#define SW1     C21  
#define SW2     C20   
#endif


#define KS key_status
#define KF key_flag
typedef struct KEY_STATUS{
//开关状态变量
uint8 keyU_status;
uint8 keyD_status;
uint8 keyL_status;
uint8 keyR_status;
uint8 KEYEnter_status;
//拨码开关状态变量
uint8 sw1_status;             
uint8 sw2_status;
//上一次开关状态变量
uint8 keyU_last_status;
uint8 keyD_last_status;
uint8 keyL_last_status;
uint8 keyR_last_status;
uint8 KEYEnter_last_status;
}KEY_STATUS;



//开关标志位
typedef struct KEY_FLAG{
uint8 keyU_flag;
uint8 keyD_flag;
uint8 keyL_flag;
uint8 keyR_flag;
uint8 KEYEnter_flag;
}KEY_FLAG;


void init_key();
void key_scan();
#endif