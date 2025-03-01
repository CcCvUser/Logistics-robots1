#include "motor.h"
#include "math.h"
volatile Speed_Info speed_info;
int32 motor1,motor2,motor3,motor4;//左后m1，右后m2，左前m3，右前m4
//volatile IncrementalPID motorPID; 
extern int car_flag;
extern target TGT[44];
extern int nowTGT;
extern site ST_car1;
extern site ST_car2; 
extern int servo_flag;

extern PID Location_pid;
extern PID Speed_pid[4];

int move_can=0;//能跑的标志
int small_move_can=0;
int servo_can=0;
int img_can=0;


float x_now=0,y_now=0;//当前坐标
uint8 test_site[2];

int move_can_1;
int runState=1;
extern EulerAngleTypedef angle;
#define TARGET_SPEED_MAX 50   //cm/s


int dis_x=0;
int dis_y=0;

float rectify_table_x[10];
float rectify_table_y[10];

/***************
舵机方案1
****************/
/***************
舵机方案1
****************/
#define PER_TIME 150 
#define TIME_1 65   //摆动单位时间
#define TIME_2 85   //吸取单位时间
#define TIME_3 125   //摆动单位时间/210
#define TIME_4 165 //摆动单位时间/
#define TIME_5 200
#define TIME_6 225


#define TIME_BACK 150   //摆动单位时间
/*#define servo_pin_1 PWM4_MODULE2_CHA_C30      //定义舵机引脚
#define servo_pin_2 PWM1_MODULE2_CHA_D16  
#define servo_pin_transfer_1 PWM4_MODULE3_CHA_C31  
#define servo_pin_transfer_2 PWM4_MODULE2_CHA_C30 */

int servo_duty1[9]={1130,710,1160,1160,1130,1130,710,1000,1000};
int servo_duty2[9]={820,1170,400,400,240,820,1170,240,240};
//int servo_duty1[9]={1130,710,1010,1120,1130,1130,710,1000,1000};
//int servo_duty2[9]={820,1170,400,320,240,820,1170,240,240};
int servo_duty3[5]={300,555,801,1054,1035};
//int servo_duty4[4]={320,520,720,930,1130};
int servo_duty3_back[4]={600,310,510,270};

int servo_transfer_duty1[2]={0,0};
int servo_transfer_duty2[2]={0,0};


int servo2_duty=5000;


void motor_crt(void)

{
      if(motor1>0)
      {
        gpio_set_level(D0,1);
        pwm_set_duty(PWM2_MODULE3_CHA_D2, motor1);
      }
      else
      {
        gpio_set_level(D0,0);
        pwm_set_duty(PWM2_MODULE3_CHA_D2, -motor1);
      }
      
      if(motor2>0)
      {
        gpio_set_level(D1,1);
        pwm_set_duty(PWM2_MODULE3_CHB_D3, motor2);
       
      }
      else
      {
        gpio_set_level(D1,0);
        pwm_set_duty(PWM2_MODULE3_CHB_D3, -motor2);
      }
      
      if(motor3>0)
      {
        gpio_set_level(D14,0);
        pwm_set_duty(PWM1_MODULE0_CHA_D12, motor3);
      }
      else
      {
        gpio_set_level(D14,1);
        pwm_set_duty(PWM1_MODULE0_CHA_D12, -motor3);
      }
      
      if(motor4>0)
      {
        gpio_set_level(D15,0);
        pwm_set_duty(PWM1_MODULE0_CHB_D13, motor4);
      }
      else
      {
        gpio_set_level(D15,1);
        pwm_set_duty(PWM1_MODULE0_CHB_D13, -motor4);
      }
}

void motor_init()   //读编码器的时候只能读一个相，另一相是0
{       
    //tft180_show_string(0, 0,"motor_init"); 

   

    move_can=0;
    move_can_1=0;
   //初始化 QTIMER_1 A相使用QTIMER1_TIMER0_C0 B相使用QTIMER1_TIMER1_C1
    encoder_quad_init(QTIMER1_ENCODER1,QTIMER1_ENCODER1_CH1_C0,QTIMER1_ENCODER1_CH2_C1);    
    //初始化 QTIMER_1 A相使用QTIMER1_TIMER2_C2 B相使用QTIMER1_TIMER3_C24
    encoder_quad_init(QTIMER1_ENCODER2,QTIMER1_ENCODER2_CH1_C2,QTIMER1_ENCODER2_CH2_C24);   
    encoder_quad_init(QTIMER2_ENCODER1,QTIMER2_ENCODER1_CH1_C3,QTIMER2_ENCODER1_CH2_C25);
    encoder_quad_init(QTIMER3_ENCODER2,QTIMER3_ENCODER2_CH1_B18,QTIMER3_ENCODER2_CH2_B19);

    pwm_init(PWM1_MODULE0_CHA_D12,17*1000,0);
    pwm_init(PWM1_MODULE0_CHB_D13,17*1000,0);
    pwm_init(PWM2_MODULE3_CHA_D2,17*1000,0);
    pwm_init(PWM2_MODULE3_CHB_D3,17*1000,0);    
 
    gpio_init(D14, GPO ,0,GPO_PUSH_PULL);
    gpio_init(D15, GPO ,0,GPO_PUSH_PULL);
    gpio_init(D0, GPO ,0,GPO_PUSH_PULL);
    gpio_init(D1, GPO ,0,GPO_PUSH_PULL); 

    gpio_init(magnet_pin1, GPO ,0,FAST_GPO_PUSH_PULL);
    gpio_init(magnet_pin2, GPO ,0,FAST_GPO_PUSH_PULL);
    gpio_init(magnet_pin3, GPO ,0,FAST_GPO_PUSH_PULL);
    gpio_init(magnet_pin4, GPO ,0,FAST_GPO_PUSH_PULL); 
    gpio_init(magnet_pin5, GPO ,0,FAST_GPO_PUSH_PULL);//GPIO_PIN_CONFIG


    
    pwm_init(servo_pin_1,50,700);
    pwm_init(servo_pin_2,50,260);
    pwm_init(servo_pin_3,50,servo_duty3[0]);//250-1220
    //pwm_init(servo_pin_4,50,servo_duty4[0]);
    SpeedInfo_Init();
  //tft180_show_string(0, 16,"motor_init_finished"); 
}
void move_left(void){
      motor1=10000;
      motor2=-10000;
      motor3=-10000;
      motor4=10000;
      motor_crt();
      //向左平移

}
void move_right(void){
      motor1=-10000;
      motor2=10000;
      motor3=10000;
      motor4=-10000;
      motor_crt();
      //向右平移
}

void move_ahead(void){     
      motor1=10000;
      motor2=10000;
      motor3=10000;
      motor4=10000;
      motor_crt();
      //向前
}
void move_behind(void){
      motor1=-6000;
      motor2=-6000;
      motor3=-6000;
      motor4=-6000;
      motor_crt();
      //向后
}
void move_stop(void){
      motor1=0;
      motor2=0;
      motor3=0;
      motor4=0;
      motor_crt();
      //停住
}
void move_leftad(void){
      motor1=10000;
      motor4=10000;
      motor_crt();
      system_delay_ms(1000);//左前
}
void move_leftbh(void){
      motor1=-10000;
      motor4=-10000;
      motor_crt();
      system_delay_ms(1000);//左后
}
void move_rightad(void){
      motor1=0;
      motor4=0;
      motor2=10000;
      motor3=10000;
      motor_crt();
      system_delay_ms(1000);//右前
}
void move_rightbh(void){
      motor1=0;
      motor4=0;
      motor2=-10000;
      motor3=-10000;
      motor_crt();
      system_delay_ms(1000);//右后
}
void move_turnright(void){
     motor1=10000;
      motor2=-10000;
      motor3=10000;
      motor4=-10000;
      motor_crt();
      system_delay_ms(1000);//原地顺时针转圈圈
 }


void move_turn(int a)
{//如果a>0就向逆时针转
      motor1=-a;
      motor2=a;
      motor3=-a;
      motor4=a;
      motor_crt(); 

}

void move_set(int32 a,int32 b,int32 c,int32 d){
motor1=a;motor2=b;motor3=c;motor4=d;
motor_crt();
}
void move_set_all(int a){
motor1=a;motor2=a;motor3=a;motor4=a;
motor_crt();
}




void SpeedInfo_Init()
{   SI.distance=0;
    SI.lastdistance=0;
    SI.aimdistance=0;
    SI.encodernow[0]=0;
    SI.encodernow[1]=0;
    SI.encodernow[2]=0;
    //1为x，2为y
    SI.varLT[0] = 0;
    SI.varLT[1] = 0;
    SI.varLT[2] = 0;
    
    SI.varRT[0] = 0;
    SI.varRT[1] = 0;
    SI.varRT[2] = 0;
    
    SI.varLB[0] = 0;
    SI.varLB[1] = 0;
    SI.varLB[2] = 0;
    
    SI.varRB[0] = 0;
    SI.varRB[1] = 0;
    SI.varRB[2] = 0;
    
    SI.nowSpeedLT = 0;
    SI.nowSpeedRT= 0;
    SI.nowSpeedLB = 0;
    SI.nowSpeedRB = 0;
    SI.aimSpeed = 0;
    
}

void Encoder_clear_all()
{
encoder_clear_count(QTIMER1_ENCODER1 );
encoder_clear_count(QTIMER1_ENCODER2);
encoder_clear_count(QTIMER2_ENCODER1 );
encoder_clear_count(QTIMER3_ENCODER2 );
}

/*转换成距离
void getEncoder_all_1()
{       
    float dat=0;  
    SI.varLB[0]= -(float)qtimer_quad_get(QTIMER_1,QTIMER1_TIMER0_C0 );
    SI.varLT[0]= -(float)qtimer_quad_get(QTIMER_3,QTIMER3_TIMER2_B18);
    SI.varRB[0]= (float)qtimer_quad_get(QTIMER_1,QTIMER1_TIMER2_C2 );
    SI.varRT[0]= (float)qtimer_quad_get(QTIMER_2,QTIMER2_TIMER0_C3 );
    Encoder_clear_all();
   
    if (SI.varLB[0]>0&&SI.varLT[0]>0&&SI.varRB[0]>0&&SI.varRT[0]>0)
    {
    dat=(SI.varLB[0]+SI.varLT[0]+SI.varRB[0]+SI.varRT[0])/4;
    SI.distance+=dat/1024*30/70*wheel_diam*3.14159265358979;
    x_now+=dat/1024*30/70*wheel_diam*3.14159265358979*cos(angle.Yaw/180*3.1415926);
    y_now+=dat/1024*30/70*wheel_diam*3.14159265358979*sin(angle.Yaw/180*3.1415926);
    }
    
    if (SI.varLB[0]>0&&SI.varLT[0]<0&&SI.varRB[0]<0&&SI.varRT[0]>0)
    { dat=(SI.varLB[0]-SI.varLT[0]-SI.varRB[0]+SI.varRT[0])/4;
    x_now-=dat/1024*30/70*wheel_diam;}
    
    
    if (SI.varLB[0]<0&&SI.varLT[0]>0&&SI.varRB[0]>0&&SI.varRT[0]<0)
    {dat=(-SI.varLB[0]+SI.varLT[0]+SI.varRB[0]-SI.varRT[0])/4;
    x_now+=dat/1024*30/70*wheel_diam;}
    
    
    if (SI.varLB[0]<0&&SI.varLT[0]<0&&SI.varRB[0]>0&&SI.varRT[0]>0)
    { dat=(SI.varLB[0]+SI.varLT[0]+SI.varRB[0]+SI.varRT[0])/4;
    x_now+=dat/1024*30/70*wheel_diam*3.14159265358979*cos(angle.Yaw/180*3.1415926);
    y_now+=dat/1024*30/70*wheel_diam*3.14159265358979*sin(angle.Yaw/180*3.1415926);
    }    
    
    
    if (SI.varLB[0]>0&&SI.varLT[0]>0&&SI.varRB[0]<0&&SI.varRT[0]<0)
    {dat=(SI.varLB[0]+SI.varLT[0]+SI.varRB[0]+SI.varRT[0])/4;
    x_now+=dat/1024*30/70*wheel_diam*3.14159265358979*cos(angle.Yaw/180*3.1415926);
    y_now+=dat/1024*30/70*wheel_diam*3.14159265358979*sin(angle.Yaw/180*3.1415926);}
    
    
    if (SI.varLB[0]<0&&SI.varLT[0]<0&&SI.varRB[0]<0&&SI.varRT[0]<0)
    {dat=(SI.varLB[0]+SI.varLT[0]+SI.varRB[0]+SI.varRT[0])/4;
    SI.distance+=dat/1024*30/70*wheel_diam*3.14159265358979;
    x_now+=dat/1024*30/70*wheel_diam*3.14159265358979*cos(angle.Yaw/180*3.1415926);
    y_now+=dat/1024*30/70*wheel_diam*3.14159265358979*sin(angle.Yaw/180*3.1415926);
    }
    
    test_site[0]=(int)(x_now/20);
    test_site[1]=(int)(y_now/20);    
    SI.nowSpeed=dat;
    SI.encodernow+=(float)dat;
}*/

    
extern int intervene_can ;  

  float datx=0;
  float daty=0;

/*麦轮编码器获取和位移积分*/
void get_site_data()
{

    SI.varLT[0]= -(float)encoder_get_count(QTIMER1_ENCODER1 );
    SI.varRT[0]= (float)encoder_get_count(QTIMER1_ENCODER2);//
    SI.varRB[0]= (float)encoder_get_count(QTIMER2_ENCODER1 );//
    SI.varLB[0]= -(float)encoder_get_count(QTIMER3_ENCODER2 );
    Encoder_clear_all();
    //1为x，2为y
    datx=(SI.varLB[0]+SI.varLT[0]+SI.varRB[0]+SI.varRT[0])/4.0;
    daty=(SI.varLB[0]-SI.varLT[0]-SI.varRB[0]+SI.varRT[0])/4.0;
    /*SI.varLB[1]=-SI.varLB[0]*tan(45/180.0*PI);//左后
    SI.varLB[2]=SI.varLB[0];//y方向速度
    SI.varLT[1]=SI.varLT[0]*tan(45/180.0*PI);//左前
    SI.varLT[2]=SI.varLT[0];
    SI.varRB[1]=SI.varRB[0]*tan(45/180.0*PI);//右后
    SI.varRB[2]=SI.varRB[0];
    SI.varRT[1]=-SI.varRT[0]*tan(45/180.0*PI);//右前
    SI.varRT[2]=SI.varRT[0];*/
    //单位还是cm

    
    SI.nowSpeed=datx;
    SI.encodernow[0]+=datx;
    SI.encodernow[1]+=datx;//x方向
    SI.encodernow[2]+=daty;//y方向
    if(intervene_can!=1){
      ST_car1.x+=-(datx*sin(AD_L_Yaw_angle/180.0*PI)
                  +daty*cos(AD_L_Yaw_angle/180.0*PI))/1024*30/70*wheel_diam*PI/1.37;
      ST_car1.y+=(datx*cos(AD_L_Yaw_angle/180.0*PI)
                  -daty*sin(AD_L_Yaw_angle/180.0*PI))/1024*30/70*wheel_diam*PI;

    }
    else{
      ST_car1.x+=-(datx*sin(AD_L_Yaw_angle/180.0*PI)
                  +daty*cos(AD_L_Yaw_angle/180.0*PI))/1024*30/70*wheel_diam*PI;
      ST_car1.y+=(datx*cos(AD_L_Yaw_angle/180.0*PI)
                  -daty*sin(AD_L_Yaw_angle/180.0*PI))/1024*30/70*wheel_diam*PI;

    }

}


// zzh handsome
/*触发pwm保护蜂鸣器一直叫直到不触发*/
void pwm_protect_pro(int a[])
{
  int t=0;
  for(int i=0;i<4;i++){
    if(a[i]<0){
      if((-a[i])>t){
        t=-a[i];
      }
    }
    else{
      if(a[i]>t){
        t=a[i];
      }
    }
  }
  
  if (t>MOTOR_PWM_MAX){
    //BEEP(100);
    a[0]=a[0]*MOTOR_PWM_MAX/t;
    a[1]=a[1]*MOTOR_PWM_MAX/t;
    a[2]=a[2]*MOTOR_PWM_MAX/t; 
    a[3]=a[3]*MOTOR_PWM_MAX/t; 
  }   
}

float pwm_protect(float a)
{
if (a>MOTOR_PWM_MAX)
  a=MOTOR_PWM_MAX;
if (a<MOTOR_PWM_MIN)
  a=MOTOR_PWM_MIN;      
return a;
}

void speed_protect(float *speed_val)
{

  /*目标速度上限处理*/
  if (*speed_val > 150){
    *speed_val = 150;

  }
  else if (*speed_val < -150){
    *speed_val = -150;
  }	
}

/*触发速度保护蜂鸣器叫一次,结束保护叫一次*/
void speed_protect_pro(float *speed_val,int max)
{
  static int cnt=0;

  /*目标速度上限处理*/
  if (*speed_val > max){
    cnt++;
    //if(cnt==1) BEEP(100);
    *speed_val = max;
    return;
  }
  else if (*speed_val < -max){
    cnt++;
    //if(cnt==1) BEEP(100);
    *speed_val = -max;
    return;
  }
  //if(cnt>=1) BEEP(100);  
  cnt=0;
  return;
}

void speed_protect_pro_2(float *speed_val_x,float *speed_val_y,int max)
{
  float speed_val=sqrt((*speed_val_x)*(*speed_val_x)+(*speed_val_y)*(*speed_val_y));
  static int cnt=0;

  /*目标速度上限处理*/
  if (speed_val > max){
    cnt++;
    if(cnt==1) BEEP(100);
    *speed_val_x = max*(*speed_val_x)/(speed_val);
    *speed_val_y = max*(*speed_val_y)/(speed_val);
    return;
  }
  if(cnt>=1) BEEP(100);  
  cnt=0;
  return;
}

void speed_protect_1(float *speed_val)
{
        /*目标速度上限处理*/
	if (*speed_val > 40)
	{
		*speed_val = 40;
	}
	else if (*speed_val < -40)
	{
		*speed_val = -40;
	}	

}
/*********************************************
以下是伺服电机的控制
**********************************************/

extern target tempTGT_servo;

void servo_control_se(int *flag,int *cnt){
  if(*flag==1){
    if(tempTGT_servo.main_class==1){
      //BEEP(200);
      if(*cnt<TIME_1){
        //BEEP(200);
        gpio_set_level(magnet_pin1,1);
        pwm_set_duty(servo_pin_1, servo_duty1[6]); 
        pwm_set_duty(servo_pin_2, servo_duty2[6]);
      }
      else if(*cnt>=TIME_1&&*cnt<TIME_2){//吸取臂摆到对应位置
        servo_flag=0;//main中等待伺服结束标志     
        pwm_set_duty(servo_pin_1, servo_duty1[7]);
      }
      else if(*cnt>=TIME_2&&*cnt<=TIME_3-10){  //吸取臂摆到对应位置
        servo_flag=0;//main中等待伺服结束标志 
        pwm_set_duty(servo_pin_2, servo_duty2[7]);      
      }
      else if(*cnt>=TIME_3-15&&*cnt<=TIME_3-10){
        servo_flag=-1;//图像看不到舵机标志
      }
      else if(*cnt>=TIME_3-10+50&&*cnt<TIME_4+10){
        gpio_set_level(magnet_pin1,0);        
        pwm_set_duty(servo_pin_1, servo_duty1[7]);      //放置
        pwm_set_duty(servo_pin_2, servo_duty2[7]);
      }
      else if(*cnt>=TIME_4+10&&*cnt<TIME_5){
        gpio_set_level(magnet_pin1,0);
      }
      else if(*cnt>=TIME_5&&*cnt<TIME_5+10) {
        pwm_set_duty(servo_pin_1, servo_duty1[5]);//回正 
        pwm_set_duty(servo_pin_2, servo_duty2[5]);
      }  
      else if(*cnt>=TIME_5+10){  
        *cnt=0;
        *flag=0;//舵机中断结束标志
        return;
      }
    }
    else{//run_back时
      if(*cnt<TIME_1){
        gpio_set_level(magnet_pin1,1);
        pwm_set_duty(servo_pin_1, servo_duty1[1]); 
        pwm_set_duty(servo_pin_2, servo_duty2[1]);
        if(tempTGT_servo.main_class==2){
          pwm_set_duty(servo_pin_3, servo_duty3[0]);
        }
        else if(tempTGT_servo.main_class==3){
          pwm_set_duty(servo_pin_3, servo_duty3[1]);
        }
        else if(tempTGT_servo.main_class==4) {
          pwm_set_duty(servo_pin_3, servo_duty3[2]);
        }
        else if(tempTGT_servo.main_class==5){
          pwm_set_duty(servo_pin_3, servo_duty3[3]);
        }
      }
      else if(*cnt>=TIME_1&&*cnt<TIME_2){//吸取臂摆到对应位置
        servo_flag=0;//main中等待伺服结束标志     
        pwm_set_duty(servo_pin_1, servo_duty1[2]);
      }
      else if(*cnt>=TIME_2&&*cnt<=TIME_3-10){  //吸取臂摆到对应位置
        servo_flag=0;//main中等待伺服结束标志 
        pwm_set_duty(servo_pin_2, servo_duty2[2]);      
      }  
      else if(*cnt>=TIME_3-10&&*cnt<=TIME_3){
        servo_flag=-1;//图像看不到舵机标志
      }
      else if(*cnt>=TIME_3&&*cnt<TIME_4){
        pwm_set_duty(servo_pin_1, servo_duty1[3]);      //放置并回正
        pwm_set_duty(servo_pin_2, servo_duty2[3]);
      }
      else if(*cnt>=TIME_4&&*cnt<TIME_5){
        pwm_set_duty(servo_pin_1, servo_duty1[4]);      //放置并回正
        pwm_set_duty(servo_pin_2, servo_duty2[4]);
      
      }
      else if(*cnt>=TIME_5&&*cnt<TIME_6){
        gpio_set_level(magnet_pin1,0);
        //pwm_set_duty(servo_pin_1, servo_duty1[0]);      
        //pwm_set_duty(servo_pin_2, servo_duty2[0]);
      }
      else if(*cnt>=TIME_6&&*cnt<TIME_6+50){
        gpio_set_level(magnet_pin1,0);
        pwm_set_duty(servo_pin_1, servo_duty1[0]);      
        pwm_set_duty(servo_pin_2, servo_duty2[0]);
      }
      else if(*cnt>=TIME_5+50){
        *cnt=0;
        *flag=0;//舵机中断结束标志
        return;
      }
    }
    
  }
  if(*flag==2){//下分类
    if(*cnt<TIME_BACK){
      gpio_set_level(magnet_pin2,1); 
      pwm_set_duty(servo_pin_3, servo_duty3_back[0]);
    }
    else{
      gpio_set_level(magnet_pin2,0);  
      *cnt=0;
      *flag=0;
      return;
    }
  } 
  if(*flag==3){//右分类
    if(*cnt<TIME_BACK){
      gpio_set_level(magnet_pin3,1);
      pwm_set_duty(servo_pin_3, servo_duty3_back[1]);
    }
    else{
      gpio_set_level(magnet_pin3,0);  
      *cnt=0;
      *flag=0;
      return;
    }
  }
  if(*flag==4){
    if(*cnt<TIME_BACK){//上分类
      gpio_set_level(magnet_pin4,1);
      pwm_set_duty(servo_pin_3, servo_duty3_back[2]);
    }
    else{
      gpio_set_level(magnet_pin4,0);  
      *cnt=0;
      *flag=0;
      return;
    }
  }
  
  if(*flag==5){
    if(*cnt<TIME_BACK){//左分类
      gpio_set_level(magnet_pin5,1);
      pwm_set_duty(servo_pin_3, servo_duty3_back[3]);
    }
    else{
      gpio_set_level(magnet_pin5,0);  
      *cnt=0;
      *flag=0;
      return;
    }
  }
  
}

/*void servo_control_se(int *flag,int *cnt){
  if(*flag==1){    
    if(*cnt<TIME_1){
      if(TGT[nowTGT].real_class==1){
        servo_flag=0;
        *cnt=0;
        *flag=0;
        return;
      }
      if(TGT[nowTGT].main_class==4){
        servo_flag=0;
        *cnt=0;
        *flag=0;
        return;
      }
      if(TGT[nowTGT].main_class==5){
        servo_flag=0;
        *cnt=0;
        *flag=0;
        return;
      }
      gpio_set_level(magnet_pin1,1);
      pwm_set_duty(servo_pin_1, servo_duty1[1]); 
      pwm_set_duty(servo_pin_2, servo_duty2[1]);
      if(TGT[nowTGT].main_class==1){
        pwm_set_duty(servo_pin_3, servo_duty3[2]);
      }
      else if(TGT[nowTGT].main_class==3){
        pwm_set_duty(servo_pin_3, servo_duty3[3]);
      }
      else if(TGT[nowTGT].main_class==2) {
        pwm_set_duty(servo_pin_3, servo_duty3[4]);
      }
      else{
        pwm_set_duty(servo_pin_3, servo_duty3[4]);
      
      }
    }
    else if(*cnt>=TIME_1&&*cnt<TIME_2){//吸取臂摆到对应位置
      servo_flag=0;//main中等待伺服结束标志     
      pwm_set_duty(servo_pin_1, servo_duty1[2]);
    }
    else if(*cnt>=TIME_2&&*cnt<=TIME_3-50){  //吸取臂摆到对应位置
      servo_flag=0;//main中等待伺服结束标志 
      pwm_set_duty(servo_pin_2, servo_duty2[2]);      
    }  
    else if(*cnt>=TIME_3-50&&*cnt<=TIME_3){
      servo_flag=-1;//图像看不到舵机标志
    }
    else if(*cnt>=TIME_3&&*cnt<TIME_4){
      pwm_set_duty(servo_pin_1, servo_duty1[3]);      //放置并回正
      pwm_set_duty(servo_pin_2, servo_duty2[3]);
    }
    else if(*cnt>=TIME_4&&*cnt<TIME_5){
      gpio_set_level(magnet_pin1,0);
      pwm_set_duty(servo_pin_1, servo_duty1[0]);      //放置并回正
      pwm_set_duty(servo_pin_2, servo_duty2[0]);
    }
    else {
      *cnt=0;
      *flag=0;//舵机中断结束标志
      return;
    }
  }
  if(*flag==2){
    if(*cnt<TIME_BACK){
      gpio_set_level(magnet_pin2,1); 
      pwm_set_duty(servo_pin_3, servo_duty3_back[0]);
    }
    else{
      gpio_set_level(magnet_pin2,0);  
      *cnt=0;
      *flag=0;
      return;
    }
  } 
  if(*flag==3){
    if(*cnt<TIME_BACK){//左上角一类一定要稳放
      gpio_set_level(magnet_pin2,1);
      pwm_set_duty(servo_pin_3, servo_duty3_back[1]);
    }
    else{
      gpio_set_level(magnet_pin2,0);  
      *cnt=0;
      *flag=0;
      return;
    }
  }  
  if(*flag==4){
    if(*cnt<TIME_BACK){
      gpio_set_level(magnet_pin3,1);
      pwm_set_duty(servo_pin_3, servo_duty3_back[2]);
    }
    else{
      gpio_set_level(magnet_pin3,0);  
      *cnt=0;
      *flag=0;
      return;
    }
  }
}*/

/*void servo_control(int *flag,int *cnt){
  if (*flag==1){                //主车拿取分类
    if(*cnt<TIME_1){               //吸取
      pwm_set_duty(servo_pin_1, servo_duty1[1]);  
      if(TGT[nowTGT].main_class==1){
        pwm_set_duty(servo_pin_2, servo_duty2[0]);
      }
      else if(TGT[nowTGT].main_class==2){
        pwm_set_duty(servo_pin_2, servo_duty2[1]);
      }
      else if(TGT[nowTGT].main_class==3){
        pwm_set_duty(servo_pin_2, servo_duty2[2]);
      }
      else if(TGT[nowTGT].main_class==4){
        pwm_set_duty(servo_pin_2, servo_duty2[3]);
      }
      else {
        pwm_set_duty(servo_pin_2, servo_duty2[4]);
      }

      gpio_set_level(magnet_pin,1);
    }
    else if(*cnt>=TIME_1&&*cnt<TIME_2){  //吸取臂摆到对应位置
      servo_flag=0;                     //main中等待伺服结束标志 
      pwm_set_duty(servo_pin_1, servo_duty1[2]);
    }  
    else if(*cnt>=TIME_2&&*cnt<TIME_3){
      gpio_set_level(magnet_pin,0);     
      pwm_set_duty(servo_pin_1, servo_duty1[0]);      //放置并回正
    }
    else {
      *cnt=0;
      *flag=0;
      return;
    }
  }
  else if(*flag==2||*flag==3){                   //与辅车对接时的主车抓取分类//
    if(*cnt<TIME_1){ //阶段1主车抓取和与辅车对齐//时间参考：辅车对齐时间
      gpio_set_level(magnet_pin,1);
      pwm_set_duty(servo_pin_1, servo_duty1[1]);  
      pwm_set_duty(servo_pin_2, servo2_duty); 
    }
    else if(*cnt>=TIME_1&&*cnt<1500){  //阶段2主车卡片转移辅车//时间参考：转移时间，总时间满足大于吸取时间
      if(*flag==2)
        pwm_set_duty(servo_pin_transfer_1, servo_transfer_duty1[1]);
      else
        pwm_set_duty(servo_pin_transfer_2, servo_transfer_duty2[1]);
    }
    else if(*cnt>=1500&&*cnt<2000){
      if(*flag==2)
        pwm_set_duty(servo_pin_transfer_1, servo_transfer_duty1[0]);
      else
        pwm_set_duty(servo_pin_transfer_2, servo_transfer_duty2[0]);    
    }
    else if(*cnt>=1500&&*cnt<2500){    //阶段3吸取臂摆到对应位置，不放置，分类盒摆到目标位
      servo_flag=0;             //main中等待伺服结束标志    
      pwm_set_duty(servo_pin_1, servo_duty1[2]);  
      if(TGT[nowTGT].main_class==1){
        pwm_set_duty(servo_pin_2, servo_duty2[0]);
      }
      else if(TGT[nowTGT].main_class==2){
        pwm_set_duty(servo_pin_2, servo_duty2[1]);
      }
      else if(TGT[nowTGT].main_class==3){
        pwm_set_duty(servo_pin_2, servo_duty2[2]);
      }
      else if(TGT[nowTGT].main_class==4){
        pwm_set_duty(servo_pin_2, servo_duty2[3]);
      }
      else {
        pwm_set_duty(servo_pin_2, servo_duty2[4]);    
    }
  }
    else if(*cnt>=2500&&*cnt<3000){  //放置并回正
      gpio_set_level(magnet_pin,0);     
      pwm_set_duty(PWM1_MODULE0_CHB_D13, servo_duty1[0]);
    }
    else{
      *cnt=0;
      *flag=0;
      return;    
    }   
  }
}*/