/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library ����RT1064DVL6A ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
* 
* ���ļ��� RT1064DVL6A ��Դ���һ����
* 
* RT1064DVL6A ��Դ�� ��������
* �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
* 
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
* 
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
* 
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ���֤Э�� �����������Ϊ���İ汾
* �������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ���֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
* 
* �ļ�����          isr
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 8.32.4 or MDK 5.33
* ����ƽ̨          RT1064DVL6A
* ��������          https://seekfree.taobao.com/
* 
* �޸ļ�¼
* ����              ����                ��ע
* 2022-09-21        SeekFree            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "zf_common_debug.h"
#include "isr.h"

#include "headfile.h"
   
int cnt=0;   
extern uint8_t IsGyroOffsetReset;
extern KEY_STATUS KS;
extern KEY_FLAG KF;
extern float actual_speed;
extern int key_scan_flag;//��������ɨ��ı�־λ
extern Speed_Info SI;
extern EulerAngleTypedef angle;
extern int move_can;//������������ȡֵ����λ���ٶȴ���pid�ı�־λ
extern int small_move_can;
extern int servo_can;
extern int move_back_can;
extern int img_can;
extern site ST_car1;
extern int img_flag;
extern int turn_flag;//����ת��pid�ı�־λ
extern int send_carST_flag;
extern int intervene_can;
extern int uart_CNM_flag;
uint8 xxx;

int cnt2;
        /*cnt2++;
      if(cnt2>8){
      cnt2=0;
      }
        tft180_show_uint(80,0*16,cnt2,4);
    }*/
void CSI_IRQHandler(void)
{
    CSI_DriverIRQHandler();    // ����SDK�Դ����жϺ��� ���������������������õĻص�����
    
    __DSB();                    // ����ͬ������
}

void PIT_IRQHandler(void)
{
    if(pit_flag_get(PIT_CH0))
    {
        pit_flag_clear(PIT_CH0);
        if(0==IsGyroOffsetReset)
        {           
            //�Ѿ��ĳ�2ms�ж���//if( 0 == (++time_2ms_flag % 2) )       //�൱��2ms�ж�  BMXҪ2ms��һ��
            {            
                getBMXData();
                //time_2ms_flag = 0;
            }
        }
    }
    
    if(pit_flag_get(PIT_CH1))
    {
        pit_flag_clear(PIT_CH1);
    }
    
    if(pit_flag_get(PIT_CH2))
    {
        pit_flag_clear(PIT_CH2);
        BEEP_deal();
        cnt2++;
        if(cnt2>8){
           cnt2=0;
        }
        if(uart_CNM_flag){
          send_carsite(&ST_car1);
        }
        //tft180_show_uint(100,0*16,cnt2,4);
    }        
    
    
    if(pit_flag_get(PIT_CH3))
    {
        pit_flag_clear(PIT_CH3);
        
        get_site_data();

        if (move_can) {
          if(small_move_can==1) control_path_small(); 
          else control_path(); //��ֱ�� 
        }

        else if (move_back_can){
          if(small_move_can){
            control_path_small();
          }
          else{
            if(move_back_can==1){
              control_path_on_back(500);
            }
            else if(move_back_can==2){
              control_path_on_back(500);
            }
            else if(move_back_can==3){
              control_path_on_back(500);          
            }
            else if(move_back_can==4){
              control_path_on_back(500);          
            }
            else if(move_back_can==5){
              control_path_on_back(500);          
            }
          }
       }
       
        if (servo_can){
            cnt++;
            servo_control_se(&servo_can,&cnt);//������ͷ�
        }
        
        if (key_scan_flag) key_scan();

    }

    __DSB();
}

/*void LPUART1_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART1))
    {
        // �����ж�
        
    }
        
    LPUART_ClearStatusFlags(LPUART1, kLPUART_RxOverrunFlag);    // ������ɾ��
}*/

void LPUART2_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART2))
    {
        // �����ж�
        
    }
        
    LPUART_ClearStatusFlags(LPUART2, kLPUART_RxOverrunFlag);    // ������ɾ��
}

/*void LPUART3_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART3))
    {
        // �����ж�
        
    }
        
    LPUART_ClearStatusFlags(LPUART3, kLPUART_RxOverrunFlag);    // ������ɾ��
}*/

/*void LPUART4_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART4))
    {
        // �����ж� 
        if(NULL != flexio_camera_uart_handler)
        {
            flexio_camera_uart_handler();
        }
        
        gps_uart_callback();
    }
        
    LPUART_ClearStatusFlags(LPUART4, kLPUART_RxOverrunFlag);    // ������ɾ��
}*/

/*void LPUART5_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART5))
    {
        // �����ж�
        if(NULL != camera_uart_handler)
        {
            camera_uart_handler();
        }
    }
        
    LPUART_ClearStatusFlags(LPUART5, kLPUART_RxOverrunFlag);    // ������ɾ��
}*/

void LPUART6_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART6))
    {
        // �����ж�
        
    }
        
    LPUART_ClearStatusFlags(LPUART6, kLPUART_RxOverrunFlag);    // ������ɾ��
}


void LPUART8_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART8))
    {
        // �����ж�
        wireless_module_uart_handler();
        
    }
    else if(LPUART_GetStatusFlags(LPUART8) & kLPUART_IdleLineFlag)
    {
        extern void uart_dma_callback();
        uart_dma_callback();
    }
        
    LPUART_ClearStatusFlags(LPUART8, kLPUART_RxOverrunFlag);    // ������ɾ��
}


void GPIO1_Combined_0_15_IRQHandler(void)
{
    if(exti_flag_get(B0))
    {
        exti_flag_clear(B0);// ����жϱ�־λ
    }
    
}


void GPIO1_Combined_16_31_IRQHandler(void)
{
    if(exti_flag_get(B16))
    {
        exti_flag_clear(B16); // ����жϱ�־λ
    }

    
}

void GPIO2_Combined_0_15_IRQHandler(void)
{
    // ��ɾ����IF���
    if(NULL != flexio_camera_vsync_handler)
    {
        flexio_camera_vsync_handler();
    }
    
    
    if(exti_flag_get(C0))
    {
        exti_flag_clear(C0);// ����жϱ�־λ
    }

}


void GPIO2_Combined_16_31_IRQHandler(void)
{
    if(exti_flag_get(C16))
    {
        exti_flag_clear(C16); // ����жϱ�־λ
    }
    
    
}




void GPIO3_Combined_0_15_IRQHandler(void)
{

    if(exti_flag_get(D4))
    {
        exti_flag_clear(D4);// ����жϱ�־λ
    }
}








/*
�жϺ������ƣ��������ö�Ӧ���ܵ��жϺ���
Sample usage:��ǰ���������ڶ�ʱ���ж�
void PIT_IRQHandler(void)
{
    //��������־λ
    __DSB();
}
�ǵý����жϺ������־λ
CTI0_ERROR_IRQHandler
CTI1_ERROR_IRQHandler
CORE_IRQHandler
FLEXRAM_IRQHandler
KPP_IRQHandler
TSC_DIG_IRQHandler
GPR_IRQ_IRQHandler
LCDIF_IRQHandler
CSI_IRQHandler
PXP_IRQHandler
WDOG2_IRQHandler
SNVS_HP_WRAPPER_IRQHandler
SNVS_HP_WRAPPER_TZ_IRQHandler
SNVS_LP_WRAPPER_IRQHandler
CSU_IRQHandler
DCP_IRQHandler
DCP_VMI_IRQHandler
Reserved68_IRQHandler
TRNG_IRQHandler
SJC_IRQHandler
BEE_IRQHandler
PMU_EVENT_IRQHandler
Reserved78_IRQHandler
TEMP_LOW_HIGH_IRQHandler
TEMP_PANIC_IRQHandler
USB_PHY1_IRQHandler
USB_PHY2_IRQHandler
ADC1_IRQHandler
ADC2_IRQHandler
DCDC_IRQHandler
Reserved86_IRQHandler
Reserved87_IRQHandler
GPIO1_INT0_IRQHandler
GPIO1_INT1_IRQHandler
GPIO1_INT2_IRQHandler
GPIO1_INT3_IRQHandler
GPIO1_INT4_IRQHandler
GPIO1_INT5_IRQHandler
GPIO1_INT6_IRQHandler
GPIO1_INT7_IRQHandler
GPIO1_Combined_0_15_IRQHandler
GPIO1_Combined_16_31_IRQHandler
GPIO2_Combined_0_15_IRQHandler
GPIO2_Combined_16_31_IRQHandler
GPIO3_Combined_0_15_IRQHandler
GPIO3_Combined_16_31_IRQHandler
GPIO4_Combined_0_15_IRQHandler
GPIO4_Combined_16_31_IRQHandler
GPIO5_Combined_0_15_IRQHandler
GPIO5_Combined_16_31_IRQHandler
WDOG1_IRQHandler
RTWDOG_IRQHandler
EWM_IRQHandler
CCM_1_IRQHandler
CCM_2_IRQHandler
GPC_IRQHandler
SRC_IRQHandler
Reserved115_IRQHandler
GPT1_IRQHandler
GPT2_IRQHandler
PWM1_0_IRQHandler
PWM1_1_IRQHandler
PWM1_2_IRQHandler
PWM1_3_IRQHandler
PWM1_FAULT_IRQHandler
SEMC_IRQHandler
USB_OTG2_IRQHandler
USB_OTG1_IRQHandler
XBAR1_IRQ_0_1_IRQHandler
XBAR1_IRQ_2_3_IRQHandler
ADC_ETC_IRQ0_IRQHandler
ADC_ETC_IRQ1_IRQHandler
ADC_ETC_IRQ2_IRQHandler
ADC_ETC_ERROR_IRQ_IRQHandler
PIT_IRQHandler
ACMP1_IRQHandler
ACMP2_IRQHandler
ACMP3_IRQHandler
ACMP4_IRQHandler
Reserved143_IRQHandler
Reserved144_IRQHandler
ENC1_IRQHandler
ENC2_IRQHandler
ENC3_IRQHandler
ENC4_IRQHandler
TMR1_IRQHandler
TMR2_IRQHandler
TMR3_IRQHandler
TMR4_IRQHandler
PWM2_0_IRQHandler
PWM2_1_IRQHandler
PWM2_2_IRQHandler
PWM2_3_IRQHandler
PWM2_FAULT_IRQHandler
PWM3_0_IRQHandler
PWM3_1_IRQHandler
PWM3_2_IRQHandler
PWM3_3_IRQHandler
PWM3_FAULT_IRQHandler
PWM4_0_IRQHandler
PWM4_1_IRQHandler
PWM4_2_IRQHandler
PWM4_3_IRQHandler
PWM4_FAULT_IRQHandler
Reserved171_IRQHandler
GPIO6_7_8_9_IRQHandler*/



