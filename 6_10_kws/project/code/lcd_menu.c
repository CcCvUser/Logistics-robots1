#include "lcd_menu.h"
#include "mymooncake.h"


extern target TGT[44];
extern int nowTGT;
extern int total_TGT;
extern site ST_car1;
extern site ST_car2;

extern uint8   uart4_receive[65];
extern  uint8 nowData; 

extern KEY_STATUS KS;
extern KEY_FLAG KF;

extern Speed_Info SI;
extern PID Location_pid;
extern PID Location_pid_x;
extern PID Location_pid_y;
extern PID Speed_pid[4];
extern PID Angle_pid;
extern PID SMALL_pid;

extern EulerAngleTypedef angle;
extern int move_can;
extern int servo_can;

extern char* kws_table[9];
extern int kws_label[3];

float test_servo_duty1[3]={500,500,500};
float test_servo_duty2[3]={500,500,500};
float test_servo_duty3[5]={5000,5000,5000,5000,5000};

extern int LAB_Threshold[3][2];

uint8_t ExitMenu_flag = 0;
int ExitMark=0;
int key_scan_flag=0;
uint8 status;
int menu_flag = 1 ;
 
//-------------------------------------设置储存器
#define EXAMPLE_FLASH_SECTOR        (127)

//定义所在扇区的页编号
#define EXAMPLE_FLASH_SECTOR_PAGE   FLASH_PAGE_3
#define FLASH_SAVE_NUM  19//宏定义需要存储参数的个数    一个页最多为256字节，一个参数需要占4个字节，因此最多这里为256/4=64
uint32 write_buf[FLASH_SAVE_NUM];
uint32 read_buf[FLASH_SAVE_NUM];


void menu_progress(MENU_PARM *parm,MENU_FORM *mf)
{

  tft180_full(RGB565_WHITE);
  do{
    menu_display(parm,mf);//前一个是整个页面的，后一个是单个参数
    
    if(KF.keyU_flag)
    {
      KF.keyU_flag=0;
      parm->cursor--;
      if(parm->cursor<0) parm->cursor=parm->param-1;//光标上移
    }
    
    if (KF.keyD_flag)
    {
      KF.keyD_flag=0;
      parm->cursor++;
      if (parm->cursor>=parm->param) parm->cursor=0; //光标下移
    }
  
    if (KF.keyR_flag||KF.KEYEnter_flag)
    {
      KF.keyR_flag=0;
      KF.KEYEnter_flag=0;
      mf[parm->cursor].function(); //进入下一级菜单或者是执行菜单的函数
    }
    ExitMark=0;
    if (KF.keyL_flag)
    {
      KF.keyL_flag=0;
      ExitMark=1;//结束当前菜单    
    }
   
  }while(ExitMark == 0 && ExitMenu_flag == 0);//如果跑的话，还是把菜单关了跑
  tft180_full(RGB565_WHITE);
}

void menu_display(MENU_PARM *pr,MENU_FORM *mmp)//前一个是整个页面的，后一个是单个参数
{
    tft180_show_float(90,0*16,AD_L_Yaw_angle,3,1);
  //tft180_show_float(0, site.y*16, num_t,4,2);
  uint8_t i=0;
  Site_t site;
  for (i = 0; i < pr->param; i++)           
  {
    if (pr->cursor == i)          //按下去的那个字体白色，背景蓝色
    {
      /* 反白显示当前光标选中菜单项 */
      site.x = 0;    site.y = i;
      /*显示菜单的名字*/
      tft180_set_color(RGB565_WHITE,RGB565_BLUE);
      tft180_show_string(site.x, site.y*16,(char*)mmp[i].name);
      /* 若此菜单项有需要调的参数，则显示该参数 */
      if(mmp[i].parm != NULL)
      {
        site.x = 90;
        float num_t = (*(mmp[i].parm));  //用一个副本存值
        tft180_set_color(RGB565_RED, RGB565_WHITE);        
        tft180_show_float(site.x, site.y*16, num_t,4,2);
      }
    }
    else                       //其他没按下的字体是蓝色,背景白色
    {
      /* 正常显示其余菜单项 */
      site.x = 0;    site.y = i;
      tft180_set_color( RGB565_BLUE, RGB565_WHITE);
      tft180_show_string(site.x, site.y*16,(char*)mmp[i].name);
      /* 若此菜单项有需要调的参数，则显示该参数 */
      if(mmp[i].parm != NULL)
      {
        site.x = 90;
        tft180_set_color( RGB565_BLUE, RGB565_WHITE);
        float num_t = (*(mmp[i].parm));  //用一个副本存值
        tft180_show_float(site.x, site.y*16, num_t,4,2);
      }
    }
  }
}
//------------------------------------------------------------------------
//一级菜单 
MENU_PARM Main_menu_form={0,0,8};//最大值为8

MENU_FORM Main_menu[]=                 
{  
  {"first page:",sidelight,NULL},        //彩蛋
  {"1.pidchange",goto_pid,NULL},         //调节pid
  {"2.img test",img_test,NULL},          //开始比赛
  {"3.servo test",servo_test,NULL},            //接受坐标
  {"4.uart test",uart_test,NULL},         //测试uart
  {"5.kws_test",take_photo,NULL},
  {"6.runxy_test",run_test4,NULL},          //测试电机方向
  {"next page",second_main_page,NULL} ,  //去第二面
};    
//一级菜单第二面
MENU_PARM Main_menu2_form={0,0,8};

MENU_FORM Main_menu2[]=   
{
  {"magnet test",magnet_pin_test,NULL},               //去第一面，学长的代码menu套menu，其实是新开了一个第一页菜单的循环，退出麻烦
  {"show binary img",img_test,NULL},     //显示总钻风二值化后图像，对我们组其实没用
  {"run stright",run_test,NULL},
  {"run back",run_test2,NULL},
  {"run triangle",run_test3,NULL},                //只跑坐标不识别
  {"servo test",servo_test_all,NULL}, //调整舵机角度
  {"encoder test",encoder_test,NULL},
  {"motor test",motor_test,NULL},
};

MENU_PARM uart_menu_form={0,0,5};

MENU_FORM uart_menu[]=   
{
  {"car2 test",car2_test,NULL},
  {"unknownTGTtest",receive_unknownTGT_test,NULL},
  {"send_ST_test",send_ST_test,NULL},
  {"uart4 testclass",uart4_class_test,NULL},
  {"uart4 testgetTGT",uart4_TGT_test,NULL},
};


MENU_PARM motor_menu_form={0,0,6};

MENU_FORM motor_menu[]=   
{
  {"go back",goback,NULL},               
  {"LB test",LB_test,NULL},    
  {"LT test",LT_test,NULL},
  {"RB test",RB_test,NULL},
  {"RT test",RT_test,NULL},
  {"test all",motor_test_all,NULL},
};
//---------------------------------------------------------------------------------
//二级菜单PID 1
MENU_PARM PID_menu_form={0,0,8};
MENU_FORM PID_menu[]=                 
{  
  {"1.lo P x",pid_change,&Location_pid_x.Kp},
  {"2.lo I x",pid_change,&Location_pid_x.Ki},
  {"3.lo D x",pid_change,&Location_pid_x.Kd},
  {"4.lo P y",pid_change,&Location_pid_y.Kp},
  {"5.lo I y",pid_change,&Location_pid_y.Ki},
  {"6.lo D y",pid_change,&Location_pid_y.Kd},
    {"7.speed p",pid_change,&Speed_pid[0].Kp},
  {"to second",second_pid,NULL},
};
//二级菜单PID 2
MENU_PARM PID_menu2_form={0,0,8};

MENU_FORM PID_menu2[]=                 
{  
    {"1.speed i",pid_change2,&Speed_pid[0].Ki},     
    {"2.speed d",pid_change2,&Speed_pid[0].Kd},   
    {"3.ag P",pid_change2,&Angle_pid.Kp},
  {"4.ag I",pid_change2,&Angle_pid.Ki},
  {"5.ag D",pid_change2,&Angle_pid.Kd},
  {"6.lo P",pid_change2,&Location_pid.Kp},
  {"7.lo I",pid_change2,&Location_pid.Ki},
  {"8.lo D",pid_change2,&Location_pid.Kd},
};

MENU_PARM servo_menu_form={0,0,8};

MENU_FORM servo_menu[]=                 
{  
  {"servo1 0",servo_change,&test_servo_duty1[0]},     
  {"servo1 1",servo_change,&test_servo_duty1[1]},
  {"servo1 2",servo_change,&test_servo_duty1[2]},
  {"servo2 0",servo_change,&test_servo_duty2[0]},
  {"servo2 1",servo_change,&test_servo_duty2[1]},
  {"servo2 3",servo_change,&test_servo_duty2[2]},
  {"servo3 0",servo_change,&test_servo_duty3[0]},
  {"servo3 1",servo_change,&test_servo_duty3[1]},
};


//--------------------------------------------------------------
//彩蛋
void sidelight(void){
  tft180_full(RGB565_WHITE);
  tft180_set_color(RGB565_BLUE, RGB565_WHITE);
  tft180_show_string(0,0, "zyh very handsome");
  system_delay_ms(1000);
}
//进入pid调试界面
void goto_pid(){
  tft180_full(RGB565_WHITE);
  menu_progress(&PID_menu_form,PID_menu);
}

void second_pid(){
  tft180_full(RGB565_WHITE);
  menu_progress(&PID_menu2_form,PID_menu2);
}


void goback(){
  tft180_full(RGB565_WHITE);
  ExitMark=1;
}
//改变pid的值
void pid_change(void)
{ 
  tft180_set_color(RGB565_WHITE, RGB565_WHITE);
  tft180_show_string(70,0, "success"); 
  tft180_set_color(RGB565_BLUE, RGB565_WHITE); 
  Site_t site;
 int flagback=1;
    do{
      site.x=90;site.y=PID_menu_form.cursor;
      float num_t = (*(PID_menu[PID_menu_form.cursor].parm));
      tft180_set_color(RGB565_WHITE, RGB565_RED);      
      tft180_show_float(site.x, site.y*16,num_t,4,2);  
      if (KF.keyU_flag&&!KS.sw1_status&&!KS.sw2_status)//00
          {KF.keyU_flag=0;
            *PID_menu[PID_menu_form.cursor].parm+=0.1;                            //数值加减
           }
      if (KF.keyD_flag&&!KS.sw1_status&&!KS.sw2_status)
          {KF.keyD_flag=0;
            *PID_menu[PID_menu_form.cursor].parm-=0.1;                            
           }
      
      
      
      if (KF.keyU_flag&&KS.sw1_status&&!KS.sw2_status)//10
          {KF.keyU_flag=0;
            *PID_menu[PID_menu_form.cursor].parm+=10;                            
           }
      if (KF.keyD_flag&&KS.sw1_status&&!KS.sw2_status)
          {KF.keyD_flag=0;
            *PID_menu[PID_menu_form.cursor].parm-=10;                            
           }
      if (KF.keyU_flag&&!KS.sw1_status&&KS.sw2_status)//01
          {KF.keyU_flag=0;
            *PID_menu[PID_menu_form.cursor].parm+=1;                            
           }
      if (KF.keyD_flag&&!KS.sw1_status&&KS.sw2_status)
          {KF.keyD_flag=0;
            *PID_menu[PID_menu_form.cursor].parm-=1;                            
           } 
      if (KF.keyU_flag&&KS.sw1_status&&KS.sw2_status)//11
          {KF.keyU_flag=0;
            *PID_menu[PID_menu_form.cursor].parm+=100;                            
           }
      if (KF.keyD_flag&&KS.sw1_status&&KS.sw2_status)
          {KF.keyD_flag=0;
            *PID_menu[PID_menu_form.cursor].parm-=100;                            
           } 
      if (KF.keyR_flag||KF.KEYEnter_flag)           //保存数据
          {KF.keyR_flag=0;
          KF.KEYEnter_flag=0;
          savedata();
          Speed_pid[1].Kd=Speed_pid[0].Kd;
          Speed_pid[1].Ki=Speed_pid[0].Ki;
          Speed_pid[1].Kp=Speed_pid[0].Kp; 
          Speed_pid[2].Kd=Speed_pid[0].Kd;
          Speed_pid[2].Ki=Speed_pid[0].Ki;
          Speed_pid[2].Kp=Speed_pid[0].Kp; 
          Speed_pid[3].Kd=Speed_pid[0].Kd;
          Speed_pid[3].Ki=Speed_pid[0].Ki;
          Speed_pid[3].Kp=Speed_pid[0].Kp;   
          tft180_set_color(RGB565_BLUE, RGB565_WHITE); 
          tft180_show_string(70,0, "success"); 

          }
          
      if (KF.keyL_flag)
          {KF.keyL_flag=0;
          flagback=0;                             
      }
    }while(flagback);
}

void pid_change2(void)
{
  tft180_set_color(RGB565_WHITE, RGB565_WHITE);
  tft180_show_string(70,0, "success"); 
  tft180_set_color(RGB565_BLUE, RGB565_WHITE);  
  Site_t site;
 int flagback=1;
    do{
      site.x=90;site.y=PID_menu2_form.cursor;
      float num_t = (*(PID_menu2[PID_menu2_form.cursor].parm));
      tft180_set_color(RGB565_WHITE, RGB565_RED);      
      tft180_show_float(site.x, site.y*16,num_t,3,2);   
      if (KF.keyU_flag&&!KS.sw1_status&&!KS.sw2_status)//00
          {KF.keyU_flag=0;
            *PID_menu2[PID_menu2_form.cursor].parm+=0.1;                            //数值加减
           }
      if (KF.keyD_flag&&!KS.sw1_status&&!KS.sw2_status)
          {KF.keyD_flag=0;
            *PID_menu2[PID_menu2_form.cursor].parm-=0.1;                            
           }
      
      
      
      if (KF.keyU_flag&&KS.sw1_status&&!KS.sw2_status)//10
          {KF.keyU_flag=0;
            *PID_menu2[PID_menu2_form.cursor].parm+=10;                            
           }
      if (KF.keyD_flag&&KS.sw1_status&&!KS.sw2_status)
          {KF.keyD_flag=0;
            *PID_menu2[PID_menu2_form.cursor].parm-=10;                            
           }
      if (KF.keyU_flag&&!KS.sw1_status&&KS.sw2_status)//01
          {KF.keyU_flag=0;
            *PID_menu2[PID_menu2_form.cursor].parm+=1;                            
           }
      if (KF.keyD_flag&&!KS.sw1_status&&KS.sw2_status)
          {KF.keyD_flag=0;
            *PID_menu2[PID_menu2_form.cursor].parm-=1;                            
           } 
      if (KF.keyU_flag&&KS.sw1_status&&KS.sw2_status)//11
          {KF.keyU_flag=0;
            *PID_menu2[PID_menu2_form.cursor].parm+=100;                            
           }
      if (KF.keyD_flag&&KS.sw1_status&&KS.sw2_status)
          {KF.keyD_flag=0;
            *PID_menu2[PID_menu2_form.cursor].parm-=100;                            
           } 
      if (KF.keyR_flag||KF.KEYEnter_flag)           //保存数据
          {KF.keyR_flag=0;
          KF.KEYEnter_flag=0;
          savedata();
          tft180_set_color(RGB565_BLUE, RGB565_WHITE); 
          tft180_show_string(70,0, "success"); 
          }
          
      if (KF.keyL_flag)
          {KF.keyL_flag=0;
          flagback=0;                             
      }
    }while(flagback);
}

void servo_change(void)
{ 
  tft180_set_color(RGB565_WHITE, RGB565_WHITE);
  tft180_show_string(70,0, "success"); 
  tft180_set_color(RGB565_BLUE, RGB565_WHITE); 
  Site_t site;
 int flagback=1;
    gpio_set_level(magnet_pin1,1);
    do{
      site.x=90;site.y=servo_menu_form.cursor;
      float num_t = (*(servo_menu[servo_menu_form.cursor].parm));
      tft180_set_color(RGB565_WHITE, RGB565_RED);      
      tft180_show_float(site.x, site.y*16,num_t,4,2);  
      if (KF.keyU_flag&&!KS.sw1_status&&!KS.sw2_status)//00
          {KF.keyU_flag=0;
            *servo_menu[servo_menu_form.cursor].parm+=1000;                            //数值加减
           }
      if (KF.keyD_flag&&!KS.sw1_status&&!KS.sw2_status)
          {KF.keyD_flag=0;
            *servo_menu[servo_menu_form.cursor].parm-=1000;                            
           }
      if (KF.keyU_flag&&KS.sw1_status&&!KS.sw2_status)//10
          {KF.keyU_flag=0;
            *servo_menu[servo_menu_form.cursor].parm+=10;                            
           }
      if (KF.keyD_flag&&KS.sw1_status&&!KS.sw2_status)
          {KF.keyD_flag=0;
            *servo_menu[servo_menu_form.cursor].parm-=10;                            
           }
      if (KF.keyU_flag&&!KS.sw1_status&&KS.sw2_status)//01
          {KF.keyU_flag=0;
            *servo_menu[servo_menu_form.cursor].parm+=1;                            
           }
      if (KF.keyD_flag&&!KS.sw1_status&&KS.sw2_status)
          {KF.keyD_flag=0;
            *servo_menu[servo_menu_form.cursor].parm-=1;                            
           } 
      if (KF.keyU_flag&&KS.sw1_status&&KS.sw2_status)//11
          {KF.keyU_flag=0;
            *servo_menu[servo_menu_form.cursor].parm+=100;                            
           }
      if (KF.keyD_flag&&KS.sw1_status&&KS.sw2_status)
          {KF.keyD_flag=0;
            *servo_menu[servo_menu_form.cursor].parm-=100;                            
           } 
      if (KF.keyR_flag)           //保存数据
          {KF.keyR_flag=0;  
          tft180_set_color(RGB565_BLUE, RGB565_WHITE); 
          tft180_show_string(70,0, "success"); 

          }
          
      if (KF.keyL_flag)
          {KF.keyL_flag=0;
          flagback=0;

      }
      if(servo_menu_form.cursor<=2)
        pwm_set_duty(servo_pin_1, (int)*servo_menu[servo_menu_form.cursor].parm);
      if(servo_menu_form.cursor<=4&&servo_menu_form.cursor>2)
        pwm_set_duty(servo_pin_2, (int)*servo_menu[servo_menu_form.cursor].parm);
      if(servo_menu_form.cursor==5)
        pwm_set_duty(servo_pin_3, (int)*servo_menu[servo_menu_form.cursor].parm);      
    }while(flagback);
 
}

void uart_test()
{
  tft180_full(RGB565_WHITE);
  menu_progress(&uart_menu_form,uart_menu);
}

void motor_test()
{
  tft180_full(RGB565_WHITE);
  menu_progress(&motor_menu_form,motor_menu);
}

void servo_test(){
  tft180_full(RGB565_WHITE); 
  menu_progress(&servo_menu_form,servo_menu);
}

void runTMDgo(void)
{       
  tft180_full(RGB565_WHITE);
  ExitMenu_flag=1;
}
//去第二面
void second_main_page(void)
{
  tft180_full(RGB565_WHITE);
  menu_progress(&Main_menu2_form,Main_menu2);
}  
//写入数据
void savedata(){
  if(flash_check(EXAMPLE_FLASH_SECTOR,EXAMPLE_FLASH_SECTOR_PAGE))//校验当前 扇区所在页是否有数据，如果有数据则擦除整个扇区
    {
        status = flash_erase_page(EXAMPLE_FLASH_SECTOR,EXAMPLE_FLASH_SECTOR_PAGE);//擦除扇区，如果扇区已经有数据则必须擦除扇区之后才能再次写入新的数据
        if(status)  while(1);//擦除失败
    }
    flash_union_buffer[0].float_type=Location_pid.Kp ; 
    flash_union_buffer[1].float_type=Location_pid.Ki ;
    flash_union_buffer[2].float_type=Location_pid.Kd ;
    flash_union_buffer[3].float_type=Speed_pid[0].Kp ;
    flash_union_buffer[4].float_type=Speed_pid[0].Ki ;
    flash_union_buffer[5].float_type=Speed_pid[0].Kd;
    flash_union_buffer[6].float_type=Angle_pid.Kp ;
    flash_union_buffer[7].float_type=Angle_pid.Ki ;
    flash_union_buffer[8].float_type=Angle_pid.Kd;
    flash_union_buffer[9].float_type=SMALL_pid.Kp ;
    flash_union_buffer[10].float_type=SMALL_pid.Ki ;
    flash_union_buffer[11].float_type=SMALL_pid.Kd ;
    flash_union_buffer[12].float_type=Threshold ;
    flash_union_buffer[13].float_type=Location_pid_x.Kp ; 
    flash_union_buffer[14].float_type=Location_pid_x.Ki ;
    flash_union_buffer[15].float_type=Location_pid_x.Kd ;
    flash_union_buffer[16].float_type=Location_pid_y.Kp ; 
    flash_union_buffer[17].float_type=Location_pid_y.Ki ;
    flash_union_buffer[18].float_type=Location_pid_y.Kd ;
    /*flash_union_buffer[19].int16_type=LAB_Threshold[0][0] ;
    flash_union_buffer[20].int16_type=LAB_Threshold[0][1];
    flash_union_buffer[21].int16_type=LAB_Threshold[1][0] ;
    flash_union_buffer[22].int16_type=LAB_Threshold[1][1] ;
    flash_union_buffer[23].int16_type=LAB_Threshold[2][0] ;
    flash_union_buffer[24].int16_type=LAB_Threshold[2][1] ; */  
    status = flash_write_page_from_buffer(EXAMPLE_FLASH_SECTOR, EXAMPLE_FLASH_SECTOR_PAGE);
    if(status)  while(1);//写入失败
   
}

void read_data(){//分别读取对应数据
   flash_read_page_to_buffer(EXAMPLE_FLASH_SECTOR, EXAMPLE_FLASH_SECTOR_PAGE);
    Location_pid.Kp = flash_union_buffer[0].float_type;
    Location_pid.Ki = flash_union_buffer[1].float_type;
    Location_pid.Kd = flash_union_buffer[2].float_type;
    Speed_pid[0].Kp =flash_union_buffer[3].float_type;
    Speed_pid[0].Ki =flash_union_buffer[4].float_type;
    Speed_pid[0].Kd =flash_union_buffer[5].float_type;
    Angle_pid.Kp =flash_union_buffer[6].float_type;
    Angle_pid.Ki =flash_union_buffer[7].float_type;
    Angle_pid.Kd =flash_union_buffer[8].float_type;;
    SMALL_pid.Kp =flash_union_buffer[9].float_type;
    SMALL_pid.Ki =flash_union_buffer[10].float_type;
    SMALL_pid.Kd =flash_union_buffer[11].float_type;
    Threshold =flash_union_buffer[12].float_type;
    Location_pid_x.Kp = flash_union_buffer[13].float_type;
    Location_pid_x.Ki = flash_union_buffer[14].float_type;
    Location_pid_x.Kd = flash_union_buffer[15].float_type;
    Location_pid_y.Kp = flash_union_buffer[16].float_type;
    Location_pid_y.Ki = flash_union_buffer[17].float_type;
    Location_pid_y.Kd = flash_union_buffer[18].float_type;
    /*LAB_Threshold[0][0] =flash_union_buffer[19].int16_type;
    LAB_Threshold[0][1] =flash_union_buffer[20].int16_type;
    LAB_Threshold[1][0] =flash_union_buffer[21].int16_type;
    LAB_Threshold[1][1] =flash_union_buffer[22].int16_type;
    LAB_Threshold[2][0] =flash_union_buffer[23].int16_type;
    LAB_Threshold[2][1] =flash_union_buffer[24].int16_type;*/
    
    //Location_pid_x=Location_pid;
    //Location_pid_y=Location_pid;
    
    //Location_pid_y.Kd=15;
}

//----------------------------------------------------------------------------------------
//主菜单设置
void MainMenu_Set(void){  
  //savedata();
  SpeedInfo_Init();
  Encoder_clear_all();
  read_data();
  ExitMenu_flag = 0;
  while(ExitMenu_flag == 0){
    tft180_full(RGB565_WHITE);
    menu_progress(&Main_menu_form,Main_menu);
    move_can=0;
  }
}
//用来测试的函数
extern uint8 uart_send_1;
extern int turn_flag;
extern uint8_t nowData_U1;


extern int get_class_flag;
void uart4_class_test(){
  tft180_full(RGB565_WHITE);  
  while(1){
    if (KF.keyL_flag){
      KF.keyL_flag=0;
      break;    
    }   
    system_delay_ms(600); 
    
    get_class_flag=1;
    uint8 uart_send='\x06';
    uart_putchar(UART_4,uart_send);
    while(get_class_flag==1){
      if (KF.keyL_flag){
        KF.keyL_flag=0;
        return;    
      }   
    }
    show_class();
  }
}


extern int get_TGT_flag;
void uart4_TGT_test(){
  while(1){
    if (KF.keyL_flag)
    {
      KF.keyL_flag=0;
      break;    
    }  
    tft180_full(RGB565_WHITE);
  get_TGT_flag=1;
  uint8 uart_send='\x05';       
        //串口字节发送
  uart_putchar(UART_4,uart_send);
  while(get_TGT_flag==1){
    if (KF.keyL_flag)
    {
      KF.keyL_flag=0;
      return;    
    }   
  }
    ACO();
    show_TGT_site();
    system_delay_ms(3000); 
    tft180_full(RGB565_WHITE);
  }
}

extern int    ACO_flag;
extern int update_path_flag;
void receive_unknownTGT_test(){
  //static int cnt;
  tft180_full(RGB565_WHITE);
  ST_car1.x=0*20;
  ST_car1.y=0*20;
  total_TGT=2;
  //ACO();  
  //nowTGT=10;
  TGT[1].x=7*20;
  TGT[1].y=4*20;
  TGT[2].x=19*20;
  TGT[2].y=13*20;
  //TGT[3].x=4*20;
  //TGT[3].y=5*20;
  ACO();
  //tft180_draw_line((int)TGT[1].x/5,(int)TGT[1].y/5,(int)TGT[2].x/5,(int)TGT[2].y/5,RGB565_RED);
  while(1){
    if (KF.keyL_flag){
      KF.keyL_flag=0;
      break;
    }    
    show_path();
  }
  system_delay_ms(1000); 
  nowTGT=0;
  while(1){
    //update_path_flag=1;
    nowTGT=1;
    tft180_full(RGB565_WHITE);
    send_carsite(&ST_car1);
    if (KF.keyL_flag){
      KF.keyL_flag=0;
      break;
    }
    if (KF.keyR_flag){
      KF.keyR_flag=0;
      nowTGT+=3;
    }    
    if(update_path_flag) {
      update_path();
      /*tft180_show_string(0,0, "ACObegin"); 
      ACO();
      tft180_show_string(0,1*16, "ACOfinish"); */
      update_path_flag=0;
    }
    tft180_show_uint(50,0*16,nowTGT,4);
    tft180_show_uint(50,1*16,ACO_flag,4);
    show_path();
    //system_delay_ms(1000);
  }
}


void car2_test(){
  TGT[1].x=10*20;
  TGT[1].y=11*20;
  TGT[2].x=29*20;
  TGT[2].y=20*20;
  TGT[3].x=4*20;
  TGT[3].y=5*20;  
  TGT[4].x=8*20;
  TGT[4].y=9*20;
  TGT[5].x=21*20;
  TGT[5].y=10*20;
  TGT[6].x=3*20;
  TGT[6].y=10*20;
  TGT[0].main_class=1;
  TGT[1].main_class=1;
  TGT[2].main_class=3;
  TGT[3].main_class=2;  
  TGT[4].main_class=2; 
  TGT[5].main_class=1;
  TGT[6].main_class=3;
 TGT[0].second_class=1;
  TGT[1].second_class=1;
  TGT[2].second_class=3;
  TGT[3].second_class=2;  
  TGT[4].second_class=2; 
  TGT[5].second_class=1;
  TGT[6].second_class=3;
 TGT[0].real_class=4;
  TGT[1].real_class=4;
  TGT[2].real_class=3;
  TGT[3].real_class=2;  
  TGT[4].real_class=2; 
  TGT[5].real_class=4;
  TGT[6].real_class=3;
  total_TGT=6;
  nowTGT=1;
  tft180_full(RGB565_WHITE);
  while(1){
    tft180_show_uint(80,0*16,nowTGT-1,4);
    if (KF.keyL_flag)
    {
      
      KF.keyL_flag=0;
  char uart_send_str[5];
  uart_send_str[0]=0xf0;
  uart_send_str[1]=0xfe;
  uart_send_str[2]=1;
  uart_send_str[3]=0xff;
  uart_send_str[4]=0;
  uart_putstr(UART_3,uart_send_str);
      break;
    }
    if(KF.keyR_flag){
      KF.keyR_flag=0;
      judge_send_TGTinfo();
      nowTGT++;
    }
  }
}

extern int uart_CNM_flag;
void send_ST_test(){
  //uint8 cntx=1;
  //uint8 cnty=1;
  /*uint8 uart_SBCNM[50];
memset(uart_SBCNM,1,sizeof(uart_SBCNM));
uart_SBCNM[49]=0;
uart_SBCNM[48]=0xff;
uart_SBCNM[0]=0xfe;*/
  move_can=1;
  uart_CNM_flag=1;
  while(1){
    tft180_show_uint(0,0*16,1,4);
    tft180_show_uint(80,0*16,2,4);
  //uart_putstr(UART_1,uart_SBCNM);
    //system_delay_ms(40);
  ST_car1.x=19.83+ST_car1.x;
  ST_car1.y=17.65+ST_car1.y;
  if(ST_car1.x>650) ST_car1.x=19.83;
  if(ST_car1.y>450) ST_car1.y=17.65;  
    //if(cntx>250) cntx=0;
    //if(cnty>250) cnty=0;
    if (KF.keyL_flag)
    {
      KF.keyL_flag=0;
      break;
    }  
    tft180_full(RGB565_WHITE);
    //uint8 uart_send[6]={0xfe,cntx,cntx+1,cnty,cnty+1,0xff};
        //串口字节发送
    send_carsite(&ST_car1);
    //uart_putstr(UART_4,uart_send);
    //tft180_show_uint(0,0*16,cntx,4);
    //tft180_show_uint(80,0*16,cnty,4);
    show_attitude_site();
    system_delay_ms(20); 
  }
  //tft180_full(RGB565_WHITE);
  //move_can=1;
}

void take_photo(){
  
  uint8 uart_send='\x01';       //串口字节发送
  uart_putchar(UART_4,uart_send);
  tft180_full(RGB565_WHITE);
  tft180_show_string(0,0, "success"); 
  system_delay_ms(1000);
}

void encoder_test(){
  tft180_full(RGB565_WHITE);
  while(1){
    if (KF.keyL_flag){
      KF.keyL_flag=0;
      break;    
    }      
    show_Speed_Info();
  }
}


void LB_test(){
  tft180_full(RGB565_WHITE);
  
  tft180_set_color(RGB565_BLUE, RGB565_WHITE); 
  tft180_show_string(0,4,"LB_test");   
  int a=5000;
  while(1){
    system_delay_ms(200);
    move_set(a,0,0,0);
    a-=100;
    if (KF.keyL_flag||a<-5000){
      KF.keyL_flag=0; 
      move_set(0,0,0,0);
      break;  
    }
  }  
}

void LT_test(){
  tft180_full(RGB565_WHITE);
  
  tft180_set_color(RGB565_BLUE, RGB565_WHITE); 
  tft180_show_string(0,4,"LT_test");   

  int a=5000;
  while(1){
    system_delay_ms(200);
    move_set(0,0,a,0);
    a-=100;
    if (KF.keyL_flag||a<-5000){
      KF.keyL_flag=0; 
      move_set(0,0,0,0);
      break;  
    }
  }  
}
void RB_test(){
  tft180_full(RGB565_WHITE);
  
  tft180_set_color(RGB565_BLUE, RGB565_WHITE); 
  tft180_show_string(0,4,"RB_test");   
  int a=5000;
  while(1){
    system_delay_ms(200);
    move_set(0,a,0,0);
    a-=100;
    if (KF.keyL_flag||a<-5000){
      KF.keyL_flag=0; 
      move_set(0,0,0,0);
      break;  
    }
  }  
}
void RT_test(){
  tft180_full(RGB565_WHITE);
  
  tft180_set_color(RGB565_BLUE, RGB565_WHITE); 
  tft180_show_string(0,4,"RT_test");   
  int a=5000;
  while(1){
    system_delay_ms(200);
    move_set(0,0,0,a);
    
    a-=100;
    if (KF.keyL_flag||a<-5000){
      KF.keyL_flag=0; 
      move_set(0,0,0,0);
      break;  
    }
  }  
}

void motor_test_all(){
  tft180_full(RGB565_WHITE);
  tft180_set_color(RGB565_BLUE, RGB565_WHITE); 
  tft180_show_string(0,4,"motor_test_all");
  int a=10000;
  int flag=0;
  while(1){
    system_delay_ms(200);
    move_set(a,a,a,a);
    if(flag<0){
      a-=500;
    }
    else if(flag>0){
      a+=500;
    }
    if (KF.keyL_flag){
      KF.keyL_flag=0; 
      move_set(0,0,0,0);
      break;  
    }
    if (KF.keyR_flag){
      KF.keyR_flag=0; 
      flag=(flag+1)*(-2);
    }
    if(a<-9500){
      flag=0;
    }
  } 

}

        
extern int intervene_flag;
extern uint8 Binary_map[IMG_H][IMG_W];//先行后列
extern uint8 Blur_image[IMG_H][IMG_W];
extern uint8 Border_image[IMG_H][IMG_W];
extern uint8 Binary_map_Inflation[IMG_H][IMG_W];
extern uint8 Binary_map_corrode[IMG_H][IMG_W];
extern uint8 pathMap[IMG_H][IMG_W];
void img_test(){
  tft180_clear();
  int flag=1;
  int temp_Threshold=40;
  if(img_ID==0)  {
    while(1){
      if (KF.keyL_flag){
        KF.keyL_flag=0; 
        return;  
      }
      if (KF.keyU_flag){
        KF.keyU_flag=0; 
        temp_Threshold+=10;  
      }
      if (KF.keyD_flag){
        KF.keyD_flag=0; 
        temp_Threshold-=10;  
      }     
      if(mt9v03x_finish_flag){
        Threshold=temp_Threshold;
        tft180_show_uint(60,6*16,temp_Threshold,4);
      //没写指针后悔了，memcpy解决算了
      //memcpy(Border_image,Blur_image,sizeof(Blur_image));
      memset(Binary_map,0,sizeof(Binary_map));
      memset(Blur_image,0,sizeof(Blur_image));  
      memset(Border_image,0,sizeof(Border_image)); 
      memset(Binary_map_Inflation,0,sizeof(Binary_map_Inflation));   
      memset(Binary_map_corrode,0,sizeof(Binary_map_corrode));
      //memset(blob_info,0,sizeof(blob_info));  
      //memset(pathMap,0,sizeof(pathMap));
      smooth_extract(mt9v03x_image,Blur_image);
      get_border(Blur_image,Border_image);
      Binary_deal(temp_Threshold,Border_image,Binary_map);
      corrode(Binary_map,Binary_map_corrode);
      DoubleInflation(Binary_map_corrode,Binary_map_Inflation);
        //smooth_extract();
        //get_border();
        //Binary_deal();
        //corrode();
        //DoubleInflation();
          //tft180_displayimage03x((const uint8 *)mt9v03x_image, 160, 128);   // 需要注意 直接显示 188*120 分辨率是显示不下的 会直接进入断言报错
        tft180_show_gray_image(0, 0, Blur_image[0], IMG_W, IMG_H, IMG_W/3, IMG_H/3, 0);
        tft180_show_gray_image(0, IMG_H/3, Border_image[0], IMG_W, IMG_H, IMG_W/3, IMG_H/3, 0);
        tft180_show_gray_image(0, 2*IMG_H/3, Binary_map[0], IMG_W, IMG_H, IMG_W/3, IMG_H/3, 0);
        tft180_show_gray_image(IMG_W/2, 0, Binary_map_corrode[0], IMG_W, IMG_H, IMG_W/3, IMG_H/3, 0);
        tft180_show_gray_image(IMG_W/2, IMG_H/3, Binary_map_Inflation[0], IMG_W, IMG_H, IMG_W/3, IMG_H/3, 0);
        mt9v03x_finish_flag=0;
      }
      if (KF.keyR_flag){
        KF.keyR_flag=0;
        break;  
      }  
    }
    

    while(1){
      if (KF.keyL_flag){
        KF.keyL_flag=0; 
        break;  
      }
      if (KF.keyU_flag){
        KF.keyU_flag=0; 
        temp_Threshold+=10;  
      }
      if (KF.keyD_flag){
        KF.keyD_flag=0; 
        temp_Threshold-=10;  
      }
      if (KF.keyR_flag){
        KF.keyR_flag=0; 
        flag++;  
      }
      //intervene_flag=flag;
      Threshold=temp_Threshold;
      tft180_show_uint(50,6*16,temp_Threshold,4);
      tft180_show_uint(90,6*16,flag,1);
      //show_img();
      if(mt9v03x_finish_flag){
        
        //tft180_show_gray_image(0, 0, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W/2, MT9V03X_H/2, temp_Threshold);
        //tft180_show_gray_image(0, MT9V03X_H/2, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W/2, MT9V03X_H/2, 0);
        
        tft180_show_uint(60,6*16,temp_Threshold,4);
        intervene_flag=0;
        get_img_info(&flag,temp_Threshold);
        //get_line(temp_Threshold);
        tft180_show_gray_image(0, 0, Binary_map[0], IMG_W, IMG_H, IMG_W/2, IMG_H/2, 0);
        //tft180_show_gray_image(0, MT9V03X_H/2, pathMap[0], MT9V03X_W, MT9V03X_H, MT9V03X_W/2, MT9V03X_H/2, 3);
        show_img();
        
        
        //tft180_show_gray_image(0, 0, Binary_map_Inflation[0], MT9V03X_W, MT9V03X_H, MT9V03X_W/2, MT9V03X_H/2, 0);
        //tft180_displayimage03x((const uint8 *)mt9v03x_image, 160, 128); 
        mt9v03x_finish_flag=0;
        
      }
    }
  }
  else if(img_ID==1){
    int cnt=0;
    while(1){
      if(KS.sw1_status==KS.sw2_status)  {
        if (KF.keyL_flag){
          KF.keyL_flag=0; 
          LAB_Threshold[cnt%3][0]-=10; 
        }
        if (KF.keyR_flag){
          KF.keyR_flag=0; 
          LAB_Threshold[cnt%3][0]+=10; 
        }
        if (KF.keyU_flag){
          KF.keyU_flag=0; 
          LAB_Threshold[cnt%3][1]+=10;  
        }
        if (KF.keyD_flag){
          KF.keyD_flag=0; 
          LAB_Threshold[cnt%3][1]-=10;  
        }
        if (KF.KEYEnter_flag){
          KF.KEYEnter_flag=0; 
          cnt+=1;  
          savedata();
        }
        
        if(cnt%3==0){
          tft180_show_string(50,4*16,"L_Th");
        }
        else if(cnt%3==1){
          tft180_show_string(50,4*16,"A_Th");
        }
        else{
          tft180_show_string(50,4*16,"B_Th");
        }
        if(scc8660_finish_flag){      
          memset(Binary_map,0,sizeof(Binary_map));
          memset(Binary_map_Inflation,0,sizeof(Binary_map_Inflation));   
          memset(Binary_map_corrode,0,sizeof(Binary_map_corrode));
          memset(blob_info,0,sizeof(blob_info));  
          memset(pathMap,0,sizeof(pathMap));
          
          RGB565_LAB_Binary_deal(scc8660_image,Binary_map);
          corrode(Binary_map,Binary_map_corrode);
          DoubleInflation(Binary_map_corrode,Binary_map_Inflation);
          
          tft180_show_rgb565_image(0, 0, scc8660_image[0], SCC8660_W, SCC8660_H, SCC8660_W / 3, SCC8660_H / 3, 0);
          tft180_show_gray_image(0, IMG_H/3, Binary_map[0], IMG_W, IMG_H, IMG_W/3, IMG_H/3, 0);
          tft180_show_gray_image(0, 2*IMG_H/3, Binary_map_corrode[0], IMG_W, IMG_H, IMG_W/3, IMG_H/3, 0);
          tft180_show_gray_image(IMG_W/2, 0, Binary_map_Inflation[0], IMG_W, IMG_H, IMG_W/3, IMG_H/3, 0);
          
          tft180_show_string(0,5*16,"L_Th");
          tft180_show_int(50,5*16,LAB_Threshold[0][0],4);
          tft180_show_int(100,5*16,LAB_Threshold[0][1],4);
          
          tft180_show_string(0,6*16,"A_Th");
          tft180_show_int(50,6*16,LAB_Threshold[1][0],4);
          tft180_show_int(100,6*16,LAB_Threshold[1][1],4);
          
          tft180_show_string(0,7*16,"B_Th");
          tft180_show_int(50,7*16,LAB_Threshold[2][0],4);
          tft180_show_int(100,7*16,LAB_Threshold[2][1],4);
          scc8660_finish_flag=0;
        }
      }
      else{
      if (KF.keyL_flag){
        KF.keyL_flag=0; 
        break;  
      }
      if (KF.keyR_flag){
        KF.keyR_flag=0; 
        flag++;  
      }
      if (KF.keyU_flag){
        KF.keyU_flag=0; 
        temp_Threshold+=10;  
      }
      if (KF.keyD_flag){
        KF.keyD_flag=0; 
        temp_Threshold-=10;  
      }
      if(scc8660_finish_flag){ 
        get_img_info(&flag,temp_Threshold);
        show_img();
        scc8660_finish_flag=0;
      }
      
      
      }
    }
  
  }  
}
#define USE_WIFI  false
void kws_test(){
  if(KS.sw1_status==KS.sw2_status){
#if USE_WIFI
    while (1)
    {
        //PC端通过TCP发送数据，串口会自动接收，并将audio_data_get_finish置1
        if(audio_data_get_finish)
        {
            //"back", "carry", "front", "left", "leftturn", "null", "right", "rightturn", "stop"
            
            //语音识别
            audio_predict();
            audio_data_get_finish = 0;
        }
    }
#elif !USE_WIFI
    while(1)
    {
        kws_label[0]=audio_predict_use_data();
        if(kws_label[0]>=0) tft180_show_string(0,3*16,kws_table[kws_label[0]]);
        system_delay_ms(300);
    }
#endif
  }
  else{
    kws_run();
  
  
  
  }
}

void magnet_pin_test(){
  tft180_clear();
  while(1){
        if (KF.KEYEnter_flag){
          KF.KEYEnter_flag=0; 
          tft180_show_string(0,4,"magnet_pin2");
          gpio_set_level(magnet_pin2,1); 
        }
        if (KF.keyR_flag){
          tft180_show_string(0,4,"magnet_pin3");
          KF.keyR_flag=0; 
          gpio_set_level(magnet_pin3,1); 
        }
        if (KF.keyL_flag){
          KF.keyL_flag=0; 
          tft180_show_string(0,4,"magnet_pin4");
          gpio_set_level(magnet_pin4,1); 
        }
        if (KF.keyD_flag){
          KF.keyD_flag=0;
          tft180_clear();
          gpio_set_level(magnet_pin4,0);
          gpio_set_level(magnet_pin3,0);
          gpio_set_level(magnet_pin2,0);
          gpio_set_level(magnet_pin5,0);
        }
        if (KF.keyU_flag){
          KF.keyU_flag=0; 
          tft180_show_string(0,4,"magnet_pin5");
          gpio_set_level(magnet_pin5,1); 
          //break;
        }
  }
}

extern int servo_duty1[8];
extern int servo_duty2[8];
extern int servo_duty3[5];

void servo_test_all(){
  tft180_set_color(RGB565_BLUE, RGB565_WHITE); 
  tft180_clear();
    
  tft180_show_string(0,4,"servo_test");
    if (KF.keyL_flag){
      KF.keyL_flag=0; 
      return;  
    }  
  //run_back_servo_test();
    while(1){
      servo_only();
    }

}
int motor_test_flag=0;
void run_test(){
//tft180_show_string(0, 1*16 ,"to_continue");      
        pwm_set_duty(servo_pin_1, servo_duty1[0]); 
        pwm_set_duty(servo_pin_2, servo_duty2[0]);
  tft180_full(RGB565_WHITE);  
  /*while(1){
    if(motor_test_flag==0){
      tft180_show_string(0, 0*16 ,"LBRB");

    }
    else{
      tft180_show_string(0, 0*16 ,"LTRT");
    }
    if (KF.keyR_flag){
      KF.keyR_flag=0; 
      break;  
    }  
    if (KF.keyR_flag){
      KF.keyR_flag=0; 
      return;  
    }  
    if (KF.keyD_flag){
      KF.keyD_flag=0; 
      motor_test_flag=1;  
    }
    if (KF.keyU_flag){
      KF.keyU_flag=0; 
      motor_test_flag=0;  
    }
  }*/
  
  tft180_set_color(RGB565_BLUE, RGB565_WHITE); 
  tft180_clear();
  TGT[1].y=1*20;
  TGT[1].x=1*20;
  Speed_pid[1].Kd=Speed_pid[0].Kd;
  Speed_pid[1].Ki=Speed_pid[0].Ki;
  Speed_pid[1].Kp=Speed_pid[0].Kp; 
  Speed_pid[2].Kd=Speed_pid[0].Kd;
  Speed_pid[2].Ki=Speed_pid[0].Ki;
  Speed_pid[2].Kp=Speed_pid[0].Kp; 
  Speed_pid[3].Kd=Speed_pid[0].Kd;
  Speed_pid[3].Ki=Speed_pid[0].Ki;
  Speed_pid[3].Kp=Speed_pid[0].Kp; 
  tft180_full(RGB565_WHITE);
  while(1){
    tft180_show_uint(0,6*16,(int)TGT[1].x/20,4);
    tft180_show_uint(60,6*16,(int)TGT[1].y/20,4);
    if (KF.keyR_flag){
      KF.keyR_flag=0; 
      TGT[1].x=TGT[1].x+40;  
    }  
    if (KF.keyU_flag){
      KF.keyU_flag=0; 
      TGT[1].y=TGT[1].y+40;  
    }  
    if (KF.keyL_flag){
      KF.keyL_flag=0; 
      break;
    } 
  }
  run_straight();
}

void run_test2(){
  Speed_pid[1].Kd=Speed_pid[0].Kd;
  Speed_pid[1].Ki=Speed_pid[0].Ki;
  Speed_pid[1].Kp=Speed_pid[0].Kp; 
  Speed_pid[2].Kd=Speed_pid[0].Kd;
  Speed_pid[2].Ki=Speed_pid[0].Ki;
  Speed_pid[2].Kp=Speed_pid[0].Kp; 
  Speed_pid[3].Kd=Speed_pid[0].Kd;
  Speed_pid[3].Ki=Speed_pid[0].Ki;
  Speed_pid[3].Kp=Speed_pid[0].Kp; 
  tft180_full(RGB565_WHITE);
  ST_car1.x=25*20;
  ST_car1.y=6*20;
  total_TGT=0;
  nowTGT=0;
  run_back();
}

void run_test3(){
  
  
  Speed_pid[1].Kd=Speed_pid[0].Kd;
  Speed_pid[1].Ki=Speed_pid[0].Ki;
  Speed_pid[1].Kp=Speed_pid[0].Kp; 
  Speed_pid[2].Kd=Speed_pid[0].Kd;
  Speed_pid[2].Ki=Speed_pid[0].Ki;
  Speed_pid[2].Kp=Speed_pid[0].Kp; 
  Speed_pid[3].Kd=Speed_pid[0].Kd;
  Speed_pid[3].Ki=Speed_pid[0].Ki;
  Speed_pid[3].Kp=Speed_pid[0].Kp; 
  tft180_full(RGB565_WHITE);
  run_triangle();
}


extern int8 virtual_flag;
extern int8 classify_flag;
void run_test4(){
  Speed_pid[1].Kd=Speed_pid[0].Kd;
  Speed_pid[1].Ki=Speed_pid[0].Ki;
  Speed_pid[1].Kp=Speed_pid[0].Kp; 
  Speed_pid[2].Kd=Speed_pid[0].Kd;
  Speed_pid[2].Ki=Speed_pid[0].Ki;
  Speed_pid[2].Kp=Speed_pid[0].Kp; 
  Speed_pid[3].Kd=Speed_pid[0].Kd;
  Speed_pid[3].Ki=Speed_pid[0].Ki;
  Speed_pid[3].Kp=Speed_pid[0].Kp; 
  tft180_full(RGB565_WHITE);
  while(1){
    tft180_show_string(30, 0*16 ,"with_virtual");
    tft180_show_string(30, 7*16 ,"without_virtual");      
    pwm_set_duty(servo_pin_1, servo_duty1[0]); 
    pwm_set_duty(servo_pin_2, servo_duty2[0]);
    if (KF.KEYEnter_flag){
      KF.KEYEnter_flag=0; 
    } 
    if (KF.keyR_flag){
      KF.keyR_flag=0;
    } 
    if (KF.keyU_flag){
      KF.keyU_flag=0; 
      virtual_flag=1;
      break;  
    }  
    if (KF.keyD_flag){
      KF.keyD_flag=0; 
      virtual_flag=0;
      break;  
    }  
    if (KF.keyL_flag){
      KF.keyL_flag=0; 
      return;  
    }  
  }
  while(1){
      tft180_show_string(30, 0*16 ,"vegetable");
      tft180_show_string(30, 3*16 ,"fruit");      
      tft180_show_string(30, 7*16 ,"grain");    
    if (KF.keyU_flag){
      KF.keyU_flag=0; 
      classify_flag=vegetable;
      break;  
    }  
    if (KF.KEYEnter_flag){
      KF.KEYEnter_flag=0; 
      classify_flag=fruit;
      break;  
    }  
    if (KF.keyD_flag){
      KF.keyD_flag=0; 
      classify_flag=grain;
      break;  
    }  
    if (KF.keyR_flag){
      KF.keyR_flag=0;
      classify_flag=0;
      break;
    }  
    if (KF.keyL_flag){
      KF.keyL_flag=0; 
      return;  
    }  
  }
  tft180_full(RGB565_WHITE);
  run_xy_pro();
}



/*************
void para_change()
{
  lcd_clear(WHITE);
  menu_progress(&para_menu_form,para_menu);
}
void para_test()
{
  Site_t site;
 int flagback=1;
    do{
      site.x=90;site.y=para_menu_form.cursor;
      float num_t = (*(para_menu[para_menu_form.cursor].parm));
      lcd_showfloat(site.x, site.y, RED, WHITE,num_t,3,2);  
      if (KF.keyU_flag&&!KS.sw1_status&&!KS.sw2_status)
          {KF.keyU_flag=0;
            *para_menu[para_menu_form.cursor].parm+=1;                            //数值加减
           }
      if (KF.keyD_flag&&!KS.sw1_status&&!KS.sw2_status)
          {KF.keyD_flag=0;
            *para_menu[para_menu_form.cursor].parm-=1;                            
           }
      if (KF.keyL_flag&&!KS.sw1_status&&!KS.sw2_status)
          {KF.keyL_flag=0;
            *para_menu[para_menu_form.cursor].parm+=10;                            
           }
      if (KF.keyR_flag&&!KS.sw1_status&&!KS.sw2_status)
          {KF.keyR_flag=0;
            *para_menu[para_menu_form.cursor].parm-=10;                            
           }
      if (KF.keyU_flag&&KS.sw1_status&&!KS.sw2_status)
          {KF.keyU_flag=0;
            *para_menu[para_menu_form.cursor].parm+=0.1;                            
           }
      if (KF.keyD_flag&&KS.sw1_status&&!KS.sw2_status)
          {KF.keyD_flag=0;
            *para_menu[para_menu_form.cursor].parm-=0.1;                            
           }
      
      if (KF.keyL_flag&&KS.sw1_status&&KS.sw2_status)           //保存数据
          {KF.keyL_flag=0;
          savedata();
          lcd_showstr(70,0, BLUE, WHITE, "success");
          }
          
      if (KF.keyR_flag&&KS.sw1_status&&KS.sw2_status)
          {KF.keyR_flag=0;
          flagback=0;                             
                    }
    }while(flagback);
menu_progress(&para_menu_form,para_menu);
}

********/




/*总钻风相关
//显示二值化搜线之后的样子-----------------------------------------------------------------
extern uint8 Binary_map[MT9V03X_CSI_H][MT9V03X_CSI_W];
int img_flag=0;
extern uint8 Line_map[MT9V03X_CSI_H][MT9V03X_CSI_W];
extern float first_point[2];//第一个点
extern float last_point[2];//最后一个点
extern float line_point[188][2];//所有的点
extern int first_see_flag;
extern uint8 smooth_map[MT9V03X_CSI_H][MT9V03X_CSI_W];
extern uint8 horizontal_map[MT9V03X_CSI_H][MT9V03X_CSI_W];
extern float kl1,kl2; 
uint32 use_time_1;
void show_binary_img()
{
  SMALL_pid.target_val=0;//平移
  SMALL_pid.target_val_1=95;//前后，也就是distance
  SMALL_pid.target_val_2=0;//如果斜率不对就自己转圈圈，然后再前后平移
 // line_flag=1;
  systick_delay_ms(500);
  img_flag=1;
  
  systick_delay_ms(500);
  lcd_clear(WHITE);
 // lcd_showstr(0, 3, BLUE, WHITE,"point num:");
   while(img_flag)
    {  
     // //使用时间
 
      //get_line();
      ;//small_flag=1;
      //use_time = systick_getval_ms();
     
        if(mt9v03x_csi_finish_flag)
        {      
			mt9v03x_csi_finish_flag = 0;
            systick_start();
            convolution_process();
            use_time_1 = systick_getval_us();
            //systick_start();
           // smooth_extract();
            //use_time =systick_getval_us();
            //使用缩放显示函数，根据原始图像大小 以及设置需要显示的大小自动进行缩放或者放大显示
            lcd_displayimage032_zoom(Binary_map[0], MT9V03X_CSI_W, MT9V03X_CSI_H, 160, 128);      
        }
        
  //      show_point();
        if (KF.keyR_flag&&KS.sw1_status&&KS.sw2_status)
          {KF.keyR_flag=0;
          img_flag=0;                             
                    }
    }
   img_flag=1;
   while(img_flag)
    {     
        if(mt9v03x_csi_finish_flag)
        {      
            mt9v03x_csi_finish_flag = 0;
            systick_start();
            get_line();
            use_time_1 = systick_getval_us();//使用缩放显示函数，根据原始图像大小 以及设置需要显示的大小自动进行缩放或者放大显示
            lcd_displayimage032_zoom(Line_map[0], MT9V03X_CSI_W, MT9V03X_CSI_H, 160, 128);      
        }
        use_time_1 = systick_getval_ms();
        show_point();
        if (KF.keyR_flag&&KS.sw1_status&&KS.sw2_status)
          {KF.keyR_flag=0;
          img_flag=0;                             
                    }
    }
   img_flag=1;
   while(img_flag)
    {     
        if(mt9v03x_csi_finish_flag)
        {      
			mt9v03x_csi_finish_flag = 0;
                        Binary_deal();
            //使用缩放显示函数，根据原始图像大小 以及设置需要显示的大小自动进行缩放或者放大显示
            lcd_displayimage032_zoom(Binary_map[0], MT9V03X_CSI_W, MT9V03X_CSI_H, 160, 128);      
        }
  //      show_point();
        if (KF.keyR_flag&&KS.sw1_status&&KS.sw2_status)
          {KF.keyR_flag=0;
          img_flag=0;                             
                    }
    }
   img_flag=1;
   while(img_flag)
    {     
        if(mt9v03x_csi_finish_flag)
        {      
            mt9v03x_csi_finish_flag = 0;
            		//使用缩放显示函数，根据原始图像大小 以及设置需要显示的大小自动进行缩放或者放大显示
            lcd_displayimage032_zoom(mt9v03x_csi_image[0], MT9V03X_CSI_W, MT9V03X_CSI_H, 160, 128);      
        }
  //      show_point();
        if (KF.keyR_flag&&KS.sw1_status&&KS.sw2_status)
          {KF.keyR_flag=0;
          img_flag=0;                             
                    }
    }
   show_binary_img();
   menu_progress(&Main_menu_form,Main_menu);
}*/
