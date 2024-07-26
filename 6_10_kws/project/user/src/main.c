/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library 即（RT1064DVL6A 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* 
* 本文件是 RT1064DVL6A 开源库的一部分
* 
* RT1064DVL6A 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 8.32.4 or MDK 5.33
* 适用平台          RT1064DVL6A
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2022-09-21        SeekFree            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"

#include "headfile.h"
extern KEY_STATUS KS;//按键事件的结构体
extern KEY_FLAG KF;
extern uint8_t nowData;
extern uint8_t nowData_U1;
extern PID Speed_pid[4];
extern PID Angle_pid;
extern Speed_Info SI;
extern uint8   uart1_receive[25];
extern target TGT[44];

extern int key_scan_flag;//开启按键扫描的标志位
extern int move_can;//开启，编码器取值并且位置速度串级pid的标志位
extern int menu_flag;//开启菜单的标志位
extern int ExitMark;//离开菜单的标志位
extern int turn_flag;//开启转向pid的标志位

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库移植用空工程

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
  GyroOffset_init();//陀螺仪零
  tft180_show_string(0, 5*16,"BMX055_init_finished");
  pit_ms_init(PIT_CH2, 40);               //初始化pit通道3 周期    改成2ms中断
  pit_ms_init(PIT_CH3, 10); 
  pit_ms_init(PIT_CH0, 5);              //初始化pit通道0 周期
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
    clock_init(SYSTEM_CLOCK_600M);  // 不可删除
    //debug_init();                   // 调试端口初始化
    system_delay_ms(600); 
    // 此处编写用户代码 例如外设初始化代码等
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
      uart_send=0Xfd;  //串口字节发送
      uart_putchar(UART_1,uart_send);
      system_delay_ms(5000);
      cnt++;
    }*/
    if (menu_flag) 
      MainMenu_Set();
    key_scan_flag=0;    
    while(1)
    {
        // 此处编写需要循环执行的代码
       //run_TMD(); 
        
        // 此处编写需要循环执行的代码
    }
}




