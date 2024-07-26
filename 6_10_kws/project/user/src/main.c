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
* �ļ�����          main
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

#include "headfile.h"
extern KEY_STATUS KS;//�����¼��Ľṹ��
extern KEY_FLAG KF;
extern uint8_t nowData;
extern uint8_t nowData_U1;
extern PID Speed_pid[4];
extern PID Angle_pid;
extern Speed_Info SI;
extern uint8   uart1_receive[25];
extern target TGT[44];

extern int key_scan_flag;//��������ɨ��ı�־λ
extern int move_can;//������������ȡֵ����λ���ٶȴ���pid�ı�־λ
extern int menu_flag;//�����˵��ı�־λ
extern int ExitMark;//�뿪�˵��ı�־λ
extern int turn_flag;//����ת��pid�ı�־λ

// ���µĹ��̻��߹����ƶ���λ�����ִ�����²���
// ��һ�� �ر��������д򿪵��ļ�
// �ڶ��� project->clean  �ȴ��·�����������

// �������ǿ�Դ����ֲ�ÿչ���

extern LinkQueue temp_TGT_Q;


void init_all()
{
  tft180_set_dir(TFT180_CROSSWISE_180);
  tft180_init();
  tft180_set_color( RGB565_BLUE, RGB565_WHITE);
  
  //interrupt_global_enable(0);

  BEEP_init();
  tft180_show_string(0, 0,"BEEP_init_finished");
  motor_init();
  tft180_show_string(0, 16,"motor_init_finished");
  uart4_init();
  uart1_init();
  uart5_init();
  uart3_init();
  uart8_init();
  tft180_show_string(0, 2*16,"uart_init_finished");
 
  init_key();
  tft180_show_string(0, 3*16,"key_init_finished");
  init_pid();
  SpeedInfo_Init();
  flash_init();
  tft180_show_string(0, 4*16,"flash_init_finished");
  while(0==BMX055_init());
  GyroOffset_init();//��������
  tft180_show_string(0, 5*16,"BMX055_init_finished");
  pit_ms_init(PIT_CH2, 40);               //��ʼ��pitͨ��3 ����    �ĳ�2ms�ж�
  pit_ms_init(PIT_CH3, 10); 
  pit_ms_init(PIT_CH0, 5);              //��ʼ��pitͨ��0 ����
  tft180_show_string(0, 6*16,"pit_init_finished");
  NVIC_SetPriority(PIT_IRQn, 0);
//NVIC_EnableIRQ(PIT_IRQn); 
  memset(TGT,0,sizeof(TGT));
  InitQueue(&temp_TGT_Q);
  timer_init(GPT_TIM_1, TIMER_US);
  CreateTable();
}


 
extern int nowTGT;

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);  // ����ɾ��
    //debug_init();                   // ���Զ˿ڳ�ʼ��
    system_delay_ms(600); 
    // �˴���д�û����� ���������ʼ�������
    //fast_debug();
    init_all();   
    key_scan_flag=1;
    Encoder_clear_all();
    menu_flag=1;
    BEEP(200); 
    ExitMark=0; 
    /*uint8 sbcnm[4]={3,6,18,66};
    while(1){
      uint8 uart_send;
      static int cnt=0;
      if(cnt==4){
        cnt=0;
      }
      char send[11];
      send[0]=0xfe;
      send[1]=0xfe;
      send[2]=0xfe;
      send[3]=0xfe;


        send[4]=((int)(5*20*10))/100;
        send[5]=((int)(5*20*10))%100;
        send[6]=((int)(5*20*10))/100;
        send[7]=((int)(5*20*10))%100;
        send[8]=sbcnm[cnt];

      send[9]=0xff;
      send[10]=0;
      if(send[4]==0) send[4]=0xfa;
      if(send[5]==0) send[5]=0xfa;
      if(send[6]==0) send[6]=0xfa;
      if(send[7]==0) send[7]=0xfa;
      if(send[8]==0) {
        BEEP(2000);
        tft180_set_color(RGB565_BLUE, RGB565_RED);
      }
      uart_putstr(UART_1,send);
      uart_putstr(UART_1,send);
      system_delay_ms(1000);
      uart_send=0Xfd;  //�����ֽڷ���
      uart_putchar(UART_1,uart_send);
      system_delay_ms(5000);
      cnt++;
    }*/
    if (menu_flag) 
      MainMenu_Set();
    key_scan_flag=0;    
    while(1)
    {
        // �˴���д��Ҫѭ��ִ�еĴ���
       //run_TMD(); 
        
        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}




