#include "mymooncake.h"
#include "headfile.h"
#include "math.h"
//y<25,x<35

extern float datx;
extern float daty;

//target TGT[31];

target TGT[44];

/*target TGT[44] = {{0,0,0,0,0},
    {210,202,0,0,0} ,{276,371,0,0,0} ,{119,292,0,0,0} ,{157,400,0,0,0} ,{15,128,0,0,0} ,
    {696,135,0,0,0} ,{573,106,0,0,0} ,{264,299,0,0,0} ,{317,137,0,0,0} ,{475,142,0,0,0} ,
    {442,7,0,0,0} ,{217,365,0,0,0} ,{196,271,0,0,0} ,{82,437,0,0,0} ,{46,3,0,0,0} ,
    {258,182,0,0,0} ,{556,442,0,0,0} ,{73,127,0,0,0} ,{275,38,0,0,0} ,{391,270,0,0,0} ,
    {506,282,0,0,0} ,{429,138,0,0,0} ,{216,457,0,0,0} ,{368,225,0,0,0} ,{374,353,0,0,0} ,
    {676,433,0,0,0} ,{390,416,0,0,0} ,{128,354,0,0,0} ,{415,233,0,0,0} ,{156,348,0,0,0},
    {676,433,0,0,0} ,{390,416,0,0,0} ,{128,354,0,0,0} ,{415,233,0,0,0} ,{156,348,0,0,0},
    {676,433,0,0,0} ,{390,416,0,0,0} ,{128,354,0,0,0} ,{415,233,0,0,0} ,{156,348,0,0,0},
    {676,433,0,0,0} ,{390,416,0,0,0} ,{128,354,0,0,0} ,{415,233,0,0,0}  
};*/

int class_table[16][3]={{0,0,0},
      {3,1,1},{2,2,1},{4,3,3},{4,4,2},{3,5,2},
      {1,6,3},{5,7,2},{5,8,1},{1,9,2},{1,10,1},
      {4,11,1},{3,12,3},{5,13,3},{2,14,2},{2,15,3}};

char* kws_table[9]={"back","carry","front","left","leftturn","null","right","rightturn","stop"};
int kws_label[3]={-1,-1,-1};

int total_TGT=0;
int nowTGT=0;
int car_flag=1;
int img_flag=0;
int get_TGT_flag=0;
int get_class_flag=0;
int small_move_flag=0;
int intervene_flag=0;
int update_path_flag=0;
int ACO_flag=0;
int send_carST_flag=0;
int add_finish_flag=0;
target tempTGT_servo = {0,0,0,0,0};
int8 virtual_flag=0; 
int8 classify_flag=0;

extern int dis_x;
extern int dis_y;

extern uint8_t nowData;

int move_back_can=0;

//int servo_flag_2=0;
int servo_flag=0;
int second_classify_flag=0;
int second_classify_finish_flag=1;

float x_drift=0;
float y_drift=0;

float zap_angle;

int uart_CNM_flag=0;


float aim_angle;
float aim_distance;

site ST_car1={0,0};
site ST_car2={0,0};
site zap_ST={0,0};
site aim_site={0,0};

LinkQueue temp_TGT_Q;


int error_finish_flag;

extern int move_can;//位置速度串级pid的标志位
extern int small_move_can;
extern int servo_can;
extern int cnt; 
extern int img_can;
extern int intervene_can;

extern Speed_Info SI;

extern uint8 uart3_receive[24];
extern uint8 uart4_receive[65];
extern uint8 uart5_receive[24];

extern float error;
extern int speed_max;

extern int servo_duty1[3];
extern int servo_duty2[3];
extern int servo_duty3[5];

extern PID Location_pid;
extern PID Speed_pid[4];
extern PID Angle_pid;
extern PID SMALL_pid;

extern int8 realMap[36][26];
uint8 realMap_binary[36][26];
uint8 path_realMap[36][26];


typedef struct {
   uint8 center_x;
   uint8 center_y;
   uint8 visited_flag;
   uint8 area;
   uint8 position;
}unvisited_blobs_information;

unvisited_blobs_information u_blobs_info[20];


void init_carST(site *ST){
ST->x=20;
ST->y=20;
TGT[0].x=20;
TGT[0].y=20;

}

int UpdateTGT(target newTGT){
    for(int i =  nowTGT; i <= total_TGT; i++){
        if(TGT[i].sigma < 0) continue;
        if(RectIntersect(newTGT,TGT[i]) && newTGT.sigma <= TGT[i].sigma){
            TGT[i].x = newTGT.x;
            TGT[i].y = newTGT.y;
            TGT[i].sigma = newTGT.sigma;
            TGT[i].main_class = newTGT.main_class;
            TGT[i].second_class = newTGT.second_class;
            TGT[i].real_class = newTGT.real_class;
            return 0;
        }
        else if(RectIntersect(newTGT,TGT[i]) && newTGT.sigma > TGT[i].sigma){
          return 0;
        }
    }
    total_TGT++;
    if(total_TGT>=40){
      BEEP(6000);

      total_TGT--;
      return 0;
    }
    TGT[total_TGT].x = newTGT.x;
    TGT[total_TGT].y = newTGT.y;
    TGT[total_TGT].sigma = newTGT.sigma;
    TGT[total_TGT].main_class = newTGT.main_class;
    TGT[total_TGT].second_class = newTGT.second_class;
    TGT[total_TGT].real_class = newTGT.real_class;
    return 1;

}

int RectIntersect(target TGT1,target TGT2){
    int x1,y1,x2,y2,x3,y3,x4,y4;
    x1 = (int)(TGT1.x - TGT1.sigma);
    y1 = (int)(TGT1.y - TGT1.sigma);
    x2 = (int)(TGT1.x + TGT1.sigma);
    y2 = (int)(TGT1.y + TGT1.sigma);
    x3 = (int)(TGT2.x - TGT2.sigma);
    y3 = (int)(TGT2.y - TGT2.sigma);
    x4 = (int)(TGT2.x + TGT2.sigma);
    y4 = (int)(TGT2.y + TGT2.sigma);
    return ! (x1 > x4 || y1 > y4 || x3 > x2 || y3 > y2);
}

int error_judge(float begin_x,float begin_y,float end_x,float end_y,float dis){
  if(abs_float(end_x-begin_x)>dis||abs_float(end_y-begin_y)>dis) return 0;
  else return 1;
}

void get_class(){
  if(uart4_receive[4]!=0xff){
    //BEEP(2000);
    return;
  }
  TGT[nowTGT].main_class=uart4_receive[1];
  TGT[nowTGT].second_class=uart4_receive[2];
  TGT[nowTGT].real_class=uart4_receive[3];
  //BEEP(200);
  /*if(uart4_receive[1]==1) left[++left[0]]=nowTGT;
  else if(uart4_receive[1]==2) up[++up[0]]=nowTGT;
  else if(uart4_receive[1]==3) down[++down[0]]=nowTGT;
  else if(uart4_receive[1]==4) right[++right[0]]=nowTGT;
  else if(uart4_receive[1]==5) in[++in[0]]=nowTGT;*/
  //uart_putchar(UART_4,0xff);  
}

void clear_class(int a[]){
  for(int i=0;i<6;i++){
    a[i]=0;
  }
}



void send_nowTGT_aimST(){
  return;
  char send[8];
  send[0]=0xf0;
  send[1]=0xfd;
  send[2]=(int)TGT[nowTGT].x/20;
  send[3]=(int)TGT[nowTGT].y/20;
  send[4]=(int)TGT[nowTGT-1].x/20;
  send[5]=(int)TGT[nowTGT-1].y/20;    
  send[6]=0xff;
  send[7]=0;
  if(send[2]==0) send[2]=0xfa;
  if(send[3]==0) send[3]=0xfa;
  if(send[4]==0) send[4]=0xfa;
  if(send[5]==0) send[5]=0xfa;  
  uart_putstr(UART_3,send);


}

/*void send_nowTGT_ori(){
  char send[8];
  send[0]=0xfe;
  send[1]=(int)TGT[nowTGT].x/20;
  send[2]=(int)TGT[nowTGT].y/20;
  //send[3]=(int)TGT[nowTGT-1].x/20;
  //send[4]=(int)TGT[nowTGT-1].y/20;    
  send[3]=0xff;
  send[4]=0;
  uart_putstr(UART_1,send);
}*/

/*  
void send_carsite(){
  char send[7];
  send[0]=0xfe;
  send[1]=(int)TGT[nowTGT].x/20;
  send[2]=(int)TGT[nowTGT].y/20+35;
  send[3]=send[1]+send[2]+25;
  send[4]=0xff;
  send[5]=0;
  uart_putstr(UART_1,send);
}*/
void send_carsite(site *ST){
 return;

}
void send_nowTGT_ori(site *ST,int flag){
  char send[11];
  send[0]=0xfe;
  send[1]=0xfe;
  send[2]=0xfe;
  send[3]=0xfe;
  if(flag==1){
    send[4]=((int)((ST->x)*10))/100;
    send[5]=((int)((ST->x)*10))%100;
    send[6]=((int)((ST->y)*10))/100;
    send[7]=((int)((ST->y)*10))%100;
    send[8]=TGT[nowTGT].ori;
  }
  else{
  
    send[4]=((int)(TGT[nowTGT].x*10))/100;
    send[5]=((int)(TGT[nowTGT].x*10))%100;
    send[6]=((int)(TGT[nowTGT].y*10))/100;
    send[7]=((int)(TGT[nowTGT].y*10))%100;
    send[8]=TGT[nowTGT].ori;
  
  }
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
}

/*int receive_unknownTGT(uint8 uart_receive[]){
  static int flag1=0; 
  int flag=0;
  target tempTGT={0,0,0,0,0,0};
  int i;
  uart_putstr(UART_8,uart_receive);
  for(i=1;uart_receive[i]!=0xff;i++){
    tempTGT.x=uart_receive[i++]*20;
    tempTGT.y=uart_receive[i++]*20;
    //tempTGT.id=uart_receive[i++];
    tempTGT.sigma=uart_receive[i];
    EnQueue(&temp_TGT_Q,tempTGT);
  }
  if((i-1)%3!=0||flag1==1){
    BEEP(2000);
    flag1=1;
    return 0;
  }
  if(ACO_flag==0){
    while(!QueueEmpty(temp_TGT_Q)){
      DeQueue(&temp_TGT_Q,&tempTGT);
      flag+=UpdateTGT(tempTGT);
    }
  }

  return flag;
}*/

void receive_unknownTGT(uint8 uart_receive[]){
  target* tempTGT;
  //tempTGT=(target*)malloc(sizeof(target));
  
  int i;
  uart_putstr(UART_8,(char*)uart_receive);
  for(i=1;uart_receive[i]!=0xff;i++){
    tempTGT=(target*)malloc(sizeof(target));
    tempTGT->x=uart_receive[i++]*20;
    tempTGT->y=uart_receive[i++]*20;
    //tempTGT.id=uart_receive[i++];
    tempTGT->sigma=uart_receive[i];
    EnQueue(&temp_TGT_Q,(void*)tempTGT);
  }

}

void receive_carsite(site *ST){
  ST->x=uart4_receive[1]/10.0;
  ST->y=uart4_receive[2]/10.0;
}



void send_TGT_Upper_computer(){
  uart_putchar(UART_8,100);
  for (int i=1;i<=total_TGT;i++){
    uart_putchar(UART_8,(char)((int)TGT[i].x/20));
    uart_putchar(UART_8,(char)((int)TGT[i].y/20)); 
    if(TGT[i].sigma==0) uart_putchar(UART_8,222);
    else uart_putchar(UART_8,TGT[i].sigma);
    uart_putchar(UART_8,TGT[i].ori);
    uart_putchar(UART_8,200);
  }
  uart_putchar(UART_8,100);
}



float index=0.342;//像素点转实际距离//0.2
void img_intervene(int *flag){
  if(*flag==1){
    if(tgt_info.errorX!=0&&tgt_info.errorY!=0){
      x_drift=tgt_info.errorX*index;
      y_drift=tgt_info.errorY*index;
      
      ST_car1.x=TGT[nowTGT].x+x_drift;
      ST_car1.y=TGT[nowTGT].y+y_drift;
      intervene_can=1;
      BEEP(200);
    }
  }
  else if(*flag==2){//下分类
    if(line_info[1][0].dis!=0){
      if(line_info[1][1].dis!=0) y_drift=-((line_info[1][0].dis+line_info[1][1].dis)/2-view_center_y)*index;
      else y_drift=-(line_info[1][0].dis-view_center_y)*index;
      
      ST_car1.y=0+y_drift;
      intervene_can=2;
      BEEP(200);
      
    }
  }
  else if(*flag==3){//右分类
    if(line_info[0][0].dis!=0){
      if(line_info[0][1].dis!=0) x_drift=-((line_info[0][0].dis+line_info[0][1].dis)/2-view_center_x)*index;
      else  x_drift=-(line_info[0][0].dis-view_center_x)*index;      
      
      ST_car1.x=700+x_drift;
      intervene_can=3;
      BEEP(200);
    }
    if(line_info[1][0].dis!=0){
      if(line_info[1][1].dis!=0) y_drift=-((line_info[1][0].dis+line_info[1][1].dis)/2-view_center_y)*index;
      else  y_drift=-(line_info[1][0].dis-view_center_y)*index;      
      
      ST_car1.y=0+y_drift;
      
    }
  }
  else if(*flag==4){//上分类
    if(line_info[0][0].dis!=0){
      if(line_info[0][1].dis!=0) x_drift=-((line_info[0][0].dis+line_info[0][1].dis)/2-view_center_x)*index;
      else  x_drift=-(line_info[0][0].dis-view_center_x)*index;      
      
      ST_car1.x=700+x_drift;

    }
    if(line_info[1][0].dis!=0){
      if(line_info[1][1].dis!=0) y_drift=-((line_info[1][0].dis+line_info[1][1].dis)/2-view_center_y)*index;
      else  y_drift=-(line_info[1][0].dis-view_center_y)*index;      
      
      intervene_can=4;
      ST_car1.y=500+y_drift;
      BEEP(200);
      
    }
  }
  else if(*flag==5){//左分类
    if(line_info[0][0].dis!=0){
      if(line_info[0][1].dis!=0) x_drift=-((line_info[0][0].dis+line_info[0][1].dis)/2-view_center_x)*index;
      else  x_drift=-(line_info[0][0].dis-view_center_x)*index;      
      
      intervene_can=5;
      ST_car1.x=0+x_drift;
      BEEP(200);

    }
    if(line_info[1][0].dis!=0){
      if(line_info[1][1].dis!=0) y_drift=-((line_info[1][0].dis+line_info[1][1].dis)/2-view_center_y)*index;
      else  y_drift=-(line_info[1][0].dis-view_center_y)*index;      
      
      ST_car1.y=500+y_drift;
      
    }
  }  
  else if(*flag==5){//回车库
    if(line_info[0][0].dis!=0&&line_info[1][0].dis==0){
      if(line_info[0][1].dis!=0) x_drift=-((line_info[0][0].dis+line_info[0][1].dis)/2-view_center_x)*index;
      else  x_drift=-(line_info[0][0].dis-view_center_x)*index;      

      ST_car1.x=0+x_drift;

    }
    if(line_info[1][0].dis!=0&&line_info[0][0].dis==0){
      if(line_info[1][1].dis!=0) y_drift=-((line_info[1][0].dis+line_info[1][1].dis)/2-view_center_y)*index;
      else  y_drift=-(line_info[1][0].dis-view_center_y)*index;      
      
      intervene_can=5;      
      ST_car1.y=0+y_drift;
      
    }
    if(line_info[1][0].dis!=0&&line_info[0][0].dis!=0){
      if(line_info[1][1].dis!=0) y_drift=-((line_info[1][0].dis+line_info[1][1].dis)/2-view_center_y)*index;
      else  y_drift=-(line_info[1][0].dis-view_center_y)*index;      

      if(line_info[0][1].dis!=0) x_drift=-((line_info[0][0].dis+line_info[0][1].dis)/2-view_center_x)*index;
      else  x_drift=-(line_info[0][0].dis-view_center_x)*index; 
      
      ST_car1.y=0+y_drift;
      ST_car1.x=0+x_drift;
      intervene_can=5;
      BEEP(200);
      
    }
  }   
}

uint8 uart_SBCNM[9];

void get_error(uint8 uart_receive[]){
  if(uart_receive[1]>=222||uart_receive[2]>=222){
    //BEEP(2000);
    for(int i=0;i<8;i++){
      uart_SBCNM[i]=uart_receive[i];
    }
    return;
  }
  if(uart_receive[3]==0xff){
    uart_putstr(UART_8,(char*)uart_receive);
    x_drift=(uart_receive[1]-128)*0.512;
    y_drift=-(uart_receive[2]-128)*0.352;
      
    if((uart_receive[1]-128<4)&&(uart_receive[2]-128<4)&&(uart_receive[1]-128>-4)&&(uart_receive[2]-128>-4)){
      error_finish_flag=1;
    }
    else {
      error_finish_flag=1;
    }
    if(move_can==1&&(abs_float(ST_car1.x-TGT[nowTGT].x)>20||abs_float(ST_car1.y-TGT[nowTGT].y)>20)||(servo_can==1&&servo_flag!=-1)){//||(servo_can==1&&servo_flag!=-1)
      //BEEP(200);
    }
    else {
      if(move_can==1||(small_move_can==1&&move_can==1)){
        //BEEP(200);
        intervene_can=1;
        ST_car1.x=TGT[nowTGT].x+x_drift;
        ST_car1.y=TGT[nowTGT].y+y_drift;
      } 
    }
    
    
    if(move_back_can==1){
      BEEP(200);
      intervene_can=2;
      ST_car1.y=0+y_drift;
    } 
    if(move_back_can==2){
      if(uart_receive[2]!=128){
        ST_car1.y=0+y_drift;
      }
      else if(uart_receive[1]!=128){
        BEEP(200);
        intervene_can=3;
        ST_car1.x=700+x_drift;
      }
    }
    
    if(move_back_can==3){
      if(uart_receive[1]!=128){
        ST_car1.x=700+x_drift;
      }
      else if(uart_receive[2]!=128){
        BEEP(200);
        intervene_can=4;
        ST_car1.y=500+y_drift;
      }
    }
    
    if(move_back_can==4){
      if(uart_receive[2]!=128){
        ST_car1.y=500+y_drift;
      }
      else if(uart_receive[1]!=128){
        BEEP(200);
        intervene_can=5;
        ST_car1.x=0+x_drift;
      }
    }
    
    if(move_back_can==5){
      if(uart_receive[1]!=128){
        ST_car1.x=0+x_drift;
      }
      if(uart_receive[2]!=128){
        BEEP(200);
        ST_car1.y=0+y_drift;
        intervene_can=6;
      }
    }
  }
  else{
    //BEEP(2000);
    for(int i=0;i<8;i++){
      uart_SBCNM[i]=uart_receive[i];
    }
    return;
  }
}


void get_drift_updata_STcar1(){
    if(uart4_receive[4]=='+'){
      x_drift=uart4_receive[6]*0.16; 
    }
    else if(uart4_receive[4]=='-'){
      x_drift=-uart4_receive[6]*0.16; 
    }
  
  
    if(uart4_receive[5]=='+'){
      y_drift=uart4_receive[7]*0.16; 
    }
    else if(uart4_receive[5]=='-'){
      y_drift=-uart4_receive[7]*0.16; 
    }
    
    ST_car1.x=TGT[nowTGT].x+x_drift;
    ST_car1.y=TGT[nowTGT].y+y_drift; 
}

/*余弦定理解舵机角度*/
void calculate_servo_duty(){
  
}

void updata_STcar2(){
  float x,y;
  x=uart3_receive[1]*cos(uart3_receive[2]);
  y=uart3_receive[1]*sin(uart3_receive[2]);
  ST_car2.x=ST_car1.x+x;
  ST_car2.y=ST_car1.y+y;
}

void get_TGT(){
  int i;
  for(i=1;uart4_receive[2*(i-1)+1]!=0xff;i++){
    if(i>30){
      while(1){
      BEEP(200);
      }
    
    }
    TGT[i].x=uart4_receive[2*(i-1)+1]*20.0;
    TGT[i].y=uart4_receive[2*(i-1)+2]*20.0;
  }
  total_TGT=i-1;
  //uart_putchar(UART_4,0xff);
}

float calculate_distance(float x,float y,float aimx,float aimy){
  return sqrt((aimx-x)*(aimx-x)+(aimy-y)*(aimy-y));
}


void judge_send_TGTinfo(){
  char uart_send[8];
  uart_send[0]=0xf0;
  uart_send[6]=0xff;
  uart_send[7]=0x00;
  if(TGT[nowTGT].main_class==4||TGT[nowTGT].main_class==5){
    uart_send[1]=(int)TGT[nowTGT].x/20;
    uart_send[2]=(int)TGT[nowTGT].y/20;
    uart_send[3]=(int)TGT[nowTGT].main_class;
    uart_send[4]=(int)TGT[nowTGT].second_class;
    uart_send[5]=(int)TGT[nowTGT].real_class;
    uart_putstr(UART_3,uart_send);
  }
}

float calculate_angle(float x1,float y1,float aimx,float aimy)
{ 
  float x=aimx-x1;
  float y=aimy-y1;
  float angle=0;
  float k;
  if (x!=0&&y!=0){
    k=y/x;
    angle=atan(k)/PI*180; 
    if (x>0&&y>0) return angle;
    if (x>0&&y<0) return angle;
    if (x<0&&y>0) return angle+180;
    if (x<0&&y<0) return angle-180;
  }
  if (x==0&&y>0)
    return 90;
  if (x==0&&y<0)
    return -90;
  if (y==0&&x>0)
    return 0;
  if (y==0&&x<0)
    return 180;
  return angle;
}

float abs_float(float m)
{
  if (m>=0)
    return m;
  else 
    return -m;
}

void run_begin(){
  move_ahead();
  system_delay_ms(60);
}

void run_back_servo_test(){
  //int last_servo_flag=0;
  if(servo_can==0){
    pwm_set_duty(servo_pin_3, 568);
    pwm_set_duty(servo_pin_1, 720);
    pwm_set_duty(servo_pin_2, 1000);
  }  
  system_delay_ms(4000);
  servo_can=2;
  tft180_show_uint(80,1*16,servo_can,4);
  system_delay_ms(4000);  
  pwm_set_duty(servo_pin_3, 310);
  system_delay_ms(4000);
  servo_can=3; 
  pwm_set_duty(servo_pin_3, 490);
  system_delay_ms(4000);
  servo_can=4; 
  pwm_set_duty(servo_pin_3, 320);
  system_delay_ms(4000);
  servo_can=5; 
}



void run_back(){
  //辅车
  

  
  //主车  
  
  int last_servo_flag=0;  
  
  uint8 uart_send='\x08';  //串口字节发送
  uart_putchar(UART_5,uart_send);
  
  

  
  TGT[nowTGT].x=600;
  TGT[nowTGT].y=0;
  TGT[nowTGT+1].x=700;
  TGT[nowTGT+1].y=250;
  TGT[nowTGT+2].x=350;
  TGT[nowTGT+2].y=500;
  TGT[nowTGT+3].x=0;
  TGT[nowTGT+3].y=250;
  TGT[nowTGT+4].x=0;
  TGT[nowTGT+4].y=0;
  
 
  intervene_flag=0;  
  small_move_can=0;  
  speed_max=SPEED_begin;
  intervene_can=0;
  memset(uart4_receive,0,sizeof(uart4_receive));
  memset(uart5_receive,0,sizeof(uart5_receive));
  aim_site.x=TGT[nowTGT].x;//下分类
  aim_site.y=TGT[nowTGT].y;


  int go_flag=1;
  tft180_show_string(0, 5*16, "cnm");  
  move_back_can=1;
  intervene_flag=1;
  BEEP(300);
  if(servo_can==0){
    pwm_set_duty(servo_pin_3, 780);
    pwm_set_duty(servo_pin_1, 720);
    pwm_set_duty(servo_pin_2, 1000);
  }  

  system_delay_ms(20);
  do {

    if(img_finish_flag){
      get_img_info(&intervene_flag,(int)Threshold);
      img_finish_flag=0;
      show_img();
    }

    if(servo_can==0&&last_servo_flag==0){
      pwm_set_duty(servo_pin_3, 1080);
      pwm_set_duty(servo_pin_1, 720);
      pwm_set_duty(servo_pin_2, 1000);
    }
    //show_attitude_site();
    if(ST_car1.y<0-40){
      last_servo_flag=1;
      servo_can=2;
      go_flag=0; 
    }
    if ((SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0)) {
      go_flag=0;      
    }
  }while(go_flag);
  if(intervene_can!=2){
      intervene_flag=0;
      move_back_can=0;
      small_move_can=0;
      system_delay_ms(20);

      aim_site.x=ST_car1.x;//保留每次坐标，作为微调依据
      aim_site.y=ST_car1.y-90;
      //system_delay_ms(20);
      //zap_v=20;
      go_flag=1;

      small_move_can=1;
      move_back_can=1;
      intervene_flag=1;
      system_delay_ms(20);
      do {
        if(img_finish_flag){
          get_img_info(&intervene_flag,(int)Threshold);
          img_finish_flag=0;
          show_img();
        }
        //show_attitude_site();
        if (SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0) {
          go_flag=0;
        }    
      }while(go_flag);
      intervene_flag=0;
      move_back_can=0;
      small_move_can=0;

  }

  if(last_servo_flag==0){
    servo_can=2; 
  }
  last_servo_flag=0;
  
  uart_send=0Xfc;  //串口字节发送
  uart_putchar(UART_1,uart_send);  
  
  
  intervene_flag=0;
  intervene_can=0;
  memset(uart4_receive,0,sizeof(uart4_receive));
  memset(uart5_receive,0,sizeof(uart5_receive));  
  move_back_can=0; 
  system_delay_ms(20);
  nowTGT++;
  speed_max=SPEED_begin;
  aim_site.x=TGT[nowTGT].x;//右分类
  aim_site.y=TGT[nowTGT].y;  
  move_back_can=2;
  system_delay_ms(20);
  intervene_flag=1;
  go_flag=1;  
  do {
    if(img_finish_flag){
      get_img_info(&intervene_flag,(int)Threshold);
      img_finish_flag=0;
      show_img();
    }
    if(servo_can==0){
      pwm_set_duty(servo_pin_3, 810);
    }
    
    show_attitude_site();
    if(ST_car1.x>x_max+40){
      servo_can=3;
      last_servo_flag=1;
      go_flag=0; 
    }
    if ((SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0)) {        
      go_flag=0;
    }
  }while(go_flag);
  if(intervene_can!=3){
      intervene_flag=0;
      move_back_can=0;
      small_move_can=0;
      system_delay_ms(20);
      aim_site.x=ST_car1.x+90;//保留每次坐标，作为微调依据
      aim_site.y=ST_car1.y;
      //system_delay_ms(20);
      //zap_v=20;
      go_flag=1;

      small_move_can=1;
      move_back_can=2;
      intervene_flag=1;
      system_delay_ms(20);
      do {
        if(img_finish_flag){
          get_img_info(&intervene_flag,(int)Threshold);
          img_finish_flag=0;
          show_img();
        }
        show_attitude_site();
        if (SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0) {
          go_flag=0;
        }    
      }while(go_flag); 
      intervene_flag=0;
      move_back_can=0;
      small_move_can=0;
  
  }    
  if(last_servo_flag==0){
    servo_can=3;
  }
  last_servo_flag=0;
  
  
  
  

  intervene_can=0;
  memset(uart4_receive,0,sizeof(uart4_receive));
  memset(uart5_receive,0,sizeof(uart5_receive));
  intervene_flag=0;
  move_back_can=0;
  small_move_can=0;
  nowTGT++;
  speed_max=SPEED_begin;
  aim_site.x=TGT[nowTGT].x;//上分类
  aim_site.y=TGT[nowTGT].y;  
  move_back_can=3;
  intervene_flag=1;
  system_delay_ms(20);   
  go_flag=1;  
  do {
    
    if(servo_can==0){
      pwm_set_duty(servo_pin_3, 560);
    }
    
    show_attitude_site();
    if(ST_car1.y>y_max+40){
      servo_can=4;
      last_servo_flag=1;
      go_flag=0; 
    }
    if ((SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0)) {        
      go_flag=0;
    }
  }while(go_flag);
  if(intervene_can!=4){
      intervene_flag=0;
      move_back_can=0;
      small_move_can=0;
      system_delay_ms(20);

      aim_site.x=ST_car1.x;//保留每次坐标，作为微调依据
      aim_site.y=ST_car1.y+90;
      //system_delay_ms(20);
      //zap_v=20;
      go_flag=1;

      small_move_can=1;
      move_back_can=3;
      intervene_flag=1;
      system_delay_ms(20);
      do {
        if(img_finish_flag){
          get_img_info(&intervene_flag,(int)Threshold);
          img_finish_flag=0;
          show_img();
        }
        //show_attitude_site();
        if (SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0) {
          go_flag=0;
        }    
      }while(go_flag); 
      intervene_flag=0;
      move_back_can=0;
      small_move_can=0;
  
  }    
  if(last_servo_flag==0){
    servo_can=4;
  }
  last_servo_flag=0;
  
  
  intervene_flag=0;
  intervene_can=0;
  memset(uart4_receive,0,sizeof(uart4_receive));
  memset(uart5_receive,0,sizeof(uart5_receive));
  move_back_can=0; 
  small_move_can=0;
  system_delay_ms(20);
  nowTGT++;
  speed_max=SPEED_begin;
  aim_site.x=TGT[nowTGT].x;//左分类
  aim_site.y=TGT[nowTGT].y;  
  move_back_can=4;
  intervene_flag=1;
  system_delay_ms(20);   
  go_flag=1;  
  do {
    if(img_finish_flag){
      get_img_info(&intervene_flag,(int)Threshold);
      img_finish_flag=0;
      show_img();
    }
    if(servo_can==0){
      pwm_set_duty(servo_pin_3, 300);
    }
    
    show_attitude_site();
    if(ST_car1.x<-100&&intervene_can==5){
      servo_can=5;
      last_servo_flag=1;
      go_flag=0; 
    }
    if ((SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0)) {        
      go_flag=0;
    }
  }while(go_flag);
  if(intervene_can!=5){
      intervene_flag=0;
      move_back_can=0;
      small_move_can=0;
      system_delay_ms(20);

      aim_site.x=ST_car1.x-150;//保留每次坐标，作为微调依据
      aim_site.y=ST_car1.y;
      //system_delay_ms(20);
      //zap_v=20;
      go_flag=1;
      small_move_can=1;
      move_back_can=4;
      intervene_flag=1;
      system_delay_ms(20);
      do {
        if(img_finish_flag){
          get_img_info(&intervene_flag,(int)Threshold);
          img_finish_flag=0;
          show_img();
        }
        show_attitude_site();
        if (SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0) {
          go_flag=0;
        }    
      }while(go_flag); 
      intervene_flag=0;
      move_back_can=0;
      small_move_can=0;
  
  }    
  if(last_servo_flag==0){
    servo_can=5;
  }
  last_servo_flag=0;
  
  
  intervene_flag=0;
  intervene_can=0;
  memset(uart4_receive,0,sizeof(uart4_receive));
  memset(uart5_receive,0,sizeof(uart5_receive));
  move_back_can=0;
  small_move_can=0;
  system_delay_ms(20); 
  nowTGT++;//回车库
  speed_max=SPEED_begin;  
  aim_site.x=TGT[nowTGT].x;
  aim_site.y=TGT[nowTGT].y;  
  move_back_can=5;
  intervene_flag=1;
  system_delay_ms(20);   
  go_flag=1;  
  do {
    if(img_finish_flag){
      get_img_info(&intervene_flag,(int)Threshold);
      img_finish_flag=0;
      show_img();
    }
    show_attitude_site();
    if ((SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0)) {
      go_flag=0;
    }    
  }while(go_flag);
  
  
  if(intervene_can!=6){
      intervene_flag=0;
      move_back_can=0;
      small_move_can=0;
      system_delay_ms(20);

      aim_site.x=ST_car1.x;//保留每次坐标，作为微调依据
      aim_site.y=ST_car1.y-100;
      //system_delay_ms(20);
      //zap_v=20;
      go_flag=1;

      small_move_can=1;
      move_back_can=5;
      intervene_flag=1;
      system_delay_ms(20);
      do {
        if(img_finish_flag){
          get_img_info(&intervene_flag,(int)Threshold);
          img_finish_flag=0;
          show_img();
        }
        //show_attitude_site();
        if (SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0) {
          go_flag=0;
        }    
      }while(go_flag); 
  }
  
}

void init_location_pid(){
  Location_pid.integral=0;
  Location_pid.err=0;
  Location_pid.err_last=0;
  Location_pid.err_prelast=0;
  Location_pid.UD=0;
  Location_pid.UD_last=0;
}

void run_back_se(){
  int go_flag=1;
  
  uint8 uart_send='\x08';  //串口字节发送
  uart_putchar(UART_5,uart_send);
  
  
  uart_send=0Xfc;  //串口字节发送
  uart_putchar(UART_1,uart_send);
  
  TGT[nowTGT].x=600;
  TGT[nowTGT].y=0;
  TGT[nowTGT+1].x=700;
  TGT[nowTGT+1].y=250;
  TGT[nowTGT+2].x=350;
  TGT[nowTGT+2].y=500;
  TGT[nowTGT+3].x=0;
  TGT[nowTGT+3].y=250;
  TGT[nowTGT+4].x=0;
  TGT[nowTGT+4].y=0;
  
  move_back_can=0;  
  small_move_can=0;  
  intervene_flag=0;
  intervene_can=0;
  init_location_pid();
  memset(uart4_receive,0,sizeof(uart4_receive));
  aim_site.x=TGT[nowTGT].x;//下分类
  aim_site.y=TGT[nowTGT].y;



  tft180_show_string(0, 5*16, "cnm");  
  BEEP(300);
  if(servo_can==0){
    pwm_set_duty(servo_pin_3, 780);
    pwm_set_duty(servo_pin_1, 720);
    pwm_set_duty(servo_pin_2, 1000);
  }  
  zap_angle=calculate_angle(ST_car1.x,ST_car1.y,aim_site.x,aim_site.y);
  
  move_back_can=1;
  intervene_flag=1;
  
  system_delay_ms(30); 
  go_flag=1;
  do {
    if(servo_can==0){
      pwm_set_duty(servo_pin_3, 780);
      pwm_set_duty(servo_pin_1, 720);
      pwm_set_duty(servo_pin_2, 1000);
    }  
    show_attitude_site();
    if (SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0) {
      go_flag=0;
    }
    if(ST_car1.y<0&&intervene_can==2){
      servo_can=2;
      go_flag=0;
    }
  }while(go_flag&&intervene_can!=2); 
      
  move_back_can=0;
  intervene_flag=0;
  intervene_can=0;
  
  nowTGT++;
  init_location_pid();
  memset(uart4_receive,0,sizeof(uart4_receive));
  aim_site.x=TGT[nowTGT].x;//右分类
  aim_site.y=TGT[nowTGT].y;
  zap_angle=calculate_angle(ST_car1.x,ST_car1.y,aim_site.x,aim_site.y);
  
  move_back_can=2;
  intervene_flag=1;
  
  system_delay_ms(30); 
  go_flag=1;  
  do {
    if(servo_can==0){
      pwm_set_duty(servo_pin_3, 810);
    }
    show_attitude_site();
    if (SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0) {
      go_flag=0;
    }
    if(ST_car1.x>700+10&&intervene_can==3){
      servo_can=3;
      go_flag=0;
    }
  }while(go_flag&&intervene_can!=3); 
  
  move_back_can=0;
  intervene_flag=0;
  intervene_can=0;
  
  nowTGT++;
  init_location_pid();  
  memset(uart4_receive,0,sizeof(uart4_receive));
  aim_site.x=TGT[nowTGT].x;//上分类
  aim_site.y=TGT[nowTGT].y;
  zap_angle=calculate_angle(ST_car1.x,ST_car1.y,aim_site.x,aim_site.y);
  
  move_back_can=3;
  intervene_flag=1;
  
  system_delay_ms(30); 
  go_flag=1;  
  do {
    if(servo_can==0){
      pwm_set_duty(servo_pin_3, 470);
    }
    show_attitude_site();
    if (SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0) {
      go_flag=0;
    }
    if(ST_car1.y>700+20&&intervene_can==4){
      servo_can=4;
      go_flag=0;
    }
  }while(go_flag&&intervene_can!=4);   

  move_back_can=0;
  intervene_flag=0;
  intervene_can=0;
  
  nowTGT++;
  init_location_pid();  
  memset(uart4_receive,0,sizeof(uart4_receive));
  aim_site.x=TGT[nowTGT].x;//左分类
  aim_site.y=TGT[nowTGT].y;
  zap_angle=calculate_angle(ST_car1.x,ST_car1.y,aim_site.x,aim_site.y);
  
  move_back_can=4;
  intervene_flag=1;
  
  system_delay_ms(30); 
  go_flag=1;  
  do {
    if(servo_can==0){
      pwm_set_duty(servo_pin_3, 400);
    }
    show_attitude_site();
    if (SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0) {
      go_flag=0;
    }
    if(ST_car1.x<0&&intervene_can==5){
      servo_can=5;
      go_flag=0;
    }
  }while(go_flag&&intervene_can!=5); 
  
  move_back_can=0;
  intervene_flag=0;
  intervene_can=0;
  
  nowTGT++;
  init_location_pid();
  memset(uart4_receive,0,sizeof(uart4_receive));
  aim_site.x=TGT[nowTGT].x;//回车库
  aim_site.y=TGT[nowTGT].y;
  zap_angle=calculate_angle(ST_car1.x,ST_car1.y,aim_site.x,aim_site.y);
  
  move_back_can=5;
  intervene_flag=1;
  
  system_delay_ms(30); 
  go_flag=1;  
  do {
    show_attitude_site();
    if (SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0) {
      go_flag=0;
    }
    if(ST_car1.y<0&&intervene_can==6){
      servo_can=6;
      go_flag=0;
    }
  }while(go_flag&&intervene_can!=6);
}



int total_blobs=0;
void get_unvisited_blobs(){
  int i,j;
  int area=1;
  int max_x,max_y,min_x,min_y;
  for(i=0;i<36;i++){
    for(j=0;j<26;j++){
    if(realMap[i][j]>0) realMap_binary[i][j]=255;
    }
  }
  memset(path_realMap,0,sizeof(path_realMap));
  memset(u_blobs_info,0,sizeof(u_blobs_info));
  for(i=0;i<36;i++){
    for(j=0;j<26;j++){
      
      if(realMap_binary[i][j]==255&&path_realMap[i][j]==0){
        BFSTraverse(j,i,&realMap_binary[0][0],&path_realMap[0][0],26,36,&max_x,&max_y,&min_x,&min_y,&area);
        total_blobs++;
        if(total_blobs>=19){
          BEEP(2000);
          return;
        }
        u_blobs_info[total_blobs].center_x=(max_x+min_x)/2;
        u_blobs_info[total_blobs].center_y=(max_y+min_y)/2;
        u_blobs_info[total_blobs].area=area;
        if(u_blobs_info[total_blobs].center_x<17&&u_blobs_info[total_blobs].center_y<12) u_blobs_info[total_blobs].position=1;
        else if(u_blobs_info[total_blobs].center_x>=17&&u_blobs_info[total_blobs].center_y<12) u_blobs_info[total_blobs].position=2;
        else if(u_blobs_info[total_blobs].center_x>=17&&u_blobs_info[total_blobs].center_y>=12) u_blobs_info[total_blobs].position=3;
        else  u_blobs_info[total_blobs].position=4;
      
      }
    }
  }
}

void get_aim_site(int flag){
  int position=0;
  if(flag){
    if(kws_label[0]==3&&kws_label[1]==0) position=1;
    else if(kws_label[0]==6&&kws_label[1]==0) position=2;
    else if(kws_label[0]==6&&kws_label[1]==2) position=3;
    else if(kws_label[0]==3&&kws_label[1]==2) position=4;
    else {BEEP(2000); return;}

    for(int i=1;i<=total_blobs;i++){
      if(u_blobs_info[i].position==position){
        if(u_blobs_info[i].area>u_blobs_info[0].area){
          u_blobs_info[i].visited_flag=1;
          u_blobs_info[0]=u_blobs_info[i];
        }
      }
    }
    aim_site.x=u_blobs_info[0].center_x*20;
    aim_site.y=u_blobs_info[0].center_y*20; 
  
  }
  else{
    if(kws_label[0]==3) position=1;
    else if(kws_label[0]==6&&kws_label[1]==0) position=2;
    else if(kws_label[0]==6&&kws_label[1]==2) position=3;
    else if(kws_label[0]==3&&kws_label[1]==2) position=4;
    else {BEEP(2000); return;}
  
  
  }
}


void kws_run(){//语音收尾
  int go_flag=1;
  int cnt=0;
  uint8 uart_send;
  while(kws_label[2]!=8){
    cnt=0;
    while (1){
      if(audio_data_get_finish){
          //"back", "carry", "front", "left", "leftturn", "null", "right", "rightturn", "stop"
          
          kws_label[cnt]=audio_predict();
          if(kws_label[cnt]>=0) tft180_show_string(0,3*16,kws_table[kws_label[cnt]]);            //语音识别
          audio_data_get_finish = 0;
      }
      if(kws_label[cnt]==8) return;
      
      cnt++;
      if(cnt==2) break;
    }
    
    
    
    
    go_flag=1;
    small_move_can=1;
    system_delay_ms(20);
    
    do {
      if(intervene_flag==0&&error_judge(ST_car1.x,ST_car1.y,aim_site.x,aim_site.y,40)) {//&&servo_can==0
        intervene_flag=1;
      }
      if(intervene_flag){
        if(img_finish_flag){
          get_img_info(&intervene_flag,(int)Threshold);
          img_finish_flag=0;
          show_img();
        }
      }
      if(audio_data_get_finish){
          //"back", "carry", "front", "left", "leftturn", "null", "right", "rightturn", "stop"
          
          kws_label[cnt]=audio_predict();
          if(kws_label[cnt]>=0) {
            tft180_show_string(0,3*16,kws_table[kws_label[cnt]]);            //语音识别
            
            
            
            
          }
          audio_data_get_finish = 0;
      }

      //update_path();
      
      if (SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0) {//&&(error_finish_flag==1||intervene_can==0)
        system_delay_ms(10);
        if(SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0){
          go_flag=0;
          move_can=0;
          move_stop();
        }
      }
      if(datx<5&&datx>-5&&daty<5&&daty>-5&&intervene_can==1){//速度小于5则开启分类识别
        uart_send='\x06';  //串口字节发送
        get_class_flag=1;
        uart_putchar(UART_4,uart_send);
      }
    }while(go_flag);
    
    small_move_can=0;
    if(intervene_can==1){
      if(nowData<=15){
        TGT[nowTGT].main_class=class_table[nowData][0];
        TGT[nowTGT].real_class=class_table[nowData][1];
        TGT[nowTGT].second_class=class_table[nowData][2];
      }
      else BEEP(2000);

      
      tft180_full(RGB565_WHITE);
      int t;
      while(TGT[nowTGT].main_class ==0){
        uart_send='\x06';
        uart_putchar(UART_4,uart_send);
        t++;
        tft180_show_int(0,2*16,t,5);
        tft180_show_string(0,1*16,"waiting");
        if(nowData<=15){
          TGT[nowTGT].main_class=class_table[nowData][0];
          TGT[nowTGT].real_class=class_table[nowData][1];
          TGT[nowTGT].second_class=class_table[nowData][2];
        }
        else BEEP(2000);
      }
      get_class_flag=0;
                        //发送开始看未知点标志
      tempTGT_servo.main_class=TGT[nowTGT].main_class;
      tempTGT_servo.second_class=TGT[nowTGT].second_class;
      tempTGT_servo.real_class=TGT[nowTGT].real_class;
      cnt=0;
      
      servo_can=1;
      servo_flag=1;
      system_delay_ms(20);

      while(servo_flag==1){//||add_finish_flag==0
        
      }
    }
  }
}



extern float error;
//测试用，跑个直线调参
void run_straight(){
  int go_flag=1;
  speed_max=SPEED_begin;
  nowTGT=1;
  Angle_pid.target_val=0;
  AD_L_Yaw_angle=0;

  init_carST(&ST_car1);
  move_can=1;
  go_flag=1;
  system_delay_ms(40);

  do {
    if(intervene_flag==0&&error_judge(ST_car1.x,ST_car1.y,TGT[nowTGT].x,TGT[nowTGT].y,40)&&error_judge(ST_car1.x,ST_car1.y,TGT[nowTGT-1].x,TGT[nowTGT-1].y,10)==0) {//&&servo_can==0
      intervene_flag=1;
    }
    if(intervene_flag){
      if(img_finish_flag){
        get_img_info(&intervene_flag,(int)Threshold);
        img_finish_flag=0;
        show_img();
      }
    }
    if (SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0) {
      //go_flag=0;
    }
  }while(go_flag); 
  system_delay_ms(5000);
  move_can=0;
  /*while(1){
    nowTGT=0;
    for(int i=0;i<2;i++){
      speed_max=SPEED_begin;
      //intervene_flag=1;
      go_flag=1;
      system_delay_ms(20);
      do {
        show_attitude_site();
        //show_pid_info();
        //tft180_show_int(0,1*16,(int)x_drift,4);
        //tft180_show_int(70,1*16,(int)y_drift,4);
        if (SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0) {
          go_flag=0;
        }
      }while(go_flag); 
      //intervene_flag=0;
      //intervene_can=0;
      
      SI.encodernow[0]=0;
      SI.encodernow[1]=0;
      SI.encodernow[2]=0;
      go_flag=1;
      nowTGT++;
    }
  }*/
}


//测试用，跑个三角形
void run_triangle(){
  init_carST(&ST_car1);
  int go_flag=1;
  Angle_pid.target_val=0;
  TGT[0].x=0;
  TGT[0].y=0;  
  TGT[1].x=0;
  TGT[1].y=30;
  TGT[2].x=30;
  TGT[2].y=200;
  TGT[3].x=60;
  TGT[3].y=30;
  move_can=1;
  while(1){
    nowTGT=1;
    move_can=1;
    go_flag=1;
    system_delay_ms(20);
    system_delay_ms(20);        
    for(int i=0;i<3;i++){
      speed_max=SPEED_begin;
      move_can=1;
      go_flag=1;
      system_delay_ms(20);      
      do {
        show_attitude_site();
        if (SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0) {
          go_flag=0;
        }
      }while(go_flag);
      move_can=0;
      nowTGT++;
      system_delay_ms(20); 
      SI.encodernow[0]=0;
      SI.encodernow[1]=0;
      SI.encodernow[2]=0;
      go_flag=1;
    }
  }
}



float zap_v;

//测试用，只跑不识别
/*void run_xy(){

  
  init_carST(&ST_car1);
  get_TGT_flag=1;
  uint8 uart_send='\x05';  //串口字节发送
  uart_putchar(UART_4,uart_send);
  while(get_TGT_flag==1){}

  ACO(); 
  BEEP(200);
  //system_delay_ms(500);
    speed_max=SPEED_begin;
  Angle_pid.target_val=0;    
  int go_flag=1;  
  move_can=1;  
  nowTGT=1;
  system_delay_ms(20); 
  intervene_can=0;
  for(int i=0;i<total_TGT;i++){
    memset(uart4_receive,0,sizeof(uart4_receive));   
    intervene_can=0;
    move_can=1;
    go_flag=1;
    system_delay_ms(20);
    zap_angle=calculate_angle(ST_car1.x,ST_car1.y,TGT[nowTGT].x,TGT[nowTGT].y);//保留每次angle，作为微调依据
    
    intervene_flag=1;
    do {
      show_attitude_site();
      if (SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0) {
        go_flag=0;
      }    

    }while(go_flag);
    move_can=0; 
    intervene_flag=0;
    

    
      
     if(intervene_can==0){
      intervene_flag=1;
      aim_site.x=ST_car1.x+cos(zap_angle/180.0*PI)*40;//保留每次坐标，作为微调依据
      aim_site.y=ST_car1.y+sin(zap_angle/180.0*PI)*40;
      //system_delay_ms(20);
      //zap_v=20;
      go_flag=1;
      small_move_can=1;
      system_delay_ms(10);
      do {
        show_attitude_site();
        if (SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0) {
          go_flag=0;
        }    
      }while(go_flag);      
    }
    small_move_can=0;
    intervene_flag=0;    
    if(intervene_can==1){
      uart_send='\x06';  //串口字节发送
      get_class_flag=1;
      uart_putchar(UART_4,uart_send);
      tft180_full(RGB565_WHITE);
      while(get_class_flag ==1){
      tft180_show_string(0,1*16,"waiting");
      }
      //show_class();   
    
      servo_can=1;
      servo_flag=1;
      //judge_send_TGTinfo();
      system_delay_ms(10);
      
      while(servo_flag==1){}
    
      uart_send='\x06';  //串口字节发送
      uart_putchar(UART_5,uart_send);      
    }
    
    intervene_can=0;
    nowTGT++;
    speed_max=SPEED_begin;
    system_delay_ms(20); 
    SI.encodernow[0]=0;
    SI.encodernow[1]=0;
    SI.encodernow[2]=0;
    
  }

  //run_back();
  while(1){
    show_attitude_site();
  }
}*/
extern uint8 uart1_receive[25];


extern int cnt;
void run_xy(){

  
  
  init_carST(&ST_car1);
  send_carsite(&ST_car1);  
  get_TGT_flag=1;
  uint8 uart_send='\x05';  //串口字节发送
  uart_putchar(UART_4,uart_send);
  
  send_carST_flag=1;
  uart_putchar(UART_4,uart_send);  
  while(get_TGT_flag==1){
        tft180_show_string(0,1*16,"waiting");
  }
  /*TGT[5].x=13*20;
  TGT[5].y=23*20;
  TGT[4].x=9*20;
  TGT[4].y=16*20;
  TGT[3].x=3*20;
  TGT[3].y=11*20;
  TGT[2].x=5*20;
  TGT[2].y=7*20;
  TGT[1].x=8*20;
  TGT[1].y=4*20;

  total_TGT=5;*/
  ACO(); 
  BEEP(200);
  
  
  char uart_send_str[5];
  uart_send_str[0]=0xf0;
  uart_send_str[1]=0xfe;
  uart_send_str[2]=3;
  uart_send_str[3]=0xff;
  uart_send_str[4]=0;
  uart_putstr(UART_3,uart_send_str);
  
  speed_max=SPEED_begin;
  Angle_pid.target_val=0;    
  int go_flag=1;  
  move_can=1;  
  nowTGT=1;
  system_delay_ms(20); 
  intervene_can=0;
  send_carST_flag=1;
  
  
  for(int i=0;i<total_TGT;i++){
    /*dis_x=(int)abs_float(TGT[nowTGT].x-TGT[nowTGT-1].x)/40;
    dis_y=(int)abs_float(TGT[nowTGT].y-TGT[nowTGT-1].y)/40;
    if(dis_x>=10) dis_x=9;
    if(dis_y>=10) dis_y=9;*/
    //send_nowTGT_aimST();
    //send_nowTGT();
    TGT[nowTGT].sigma=0;
    memset(uart4_receive,0,sizeof(uart4_receive));
    memset(uart1_receive,0,sizeof(uart1_receive));
    add_finish_flag=0; 
    intervene_can=0;
    move_can=1;
    go_flag=1;
    system_delay_ms(20);
    zap_angle=calculate_angle(ST_car1.x,ST_car1.y,TGT[nowTGT].x,TGT[nowTGT].y);//保留每次angle，作为微调依据
    nowData=0;
    intervene_flag=1;
    do {
      if(intervene_flag==0&&error_judge(ST_car1.x,ST_car1.y,TGT[nowTGT].x,TGT[nowTGT].y,40)&&error_judge(ST_car1.x,ST_car1.y,TGT[nowTGT-1].x,TGT[nowTGT-1].y,10)==0) {//&&servo_can==0
        intervene_flag=1;
      }
      if(intervene_flag){
        if(img_finish_flag){
          get_img_info(&intervene_flag,(int)Threshold);
          img_finish_flag=0;
          show_img();
        }
      }

      update_path();
      
      if (SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0) {//&&(error_finish_flag==1||intervene_can==0)
        system_delay_ms(10);
        if(SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0){
        go_flag=0;
        move_can=0;
        move_stop();
        }
      }
      if(datx<5&&datx>-5&&daty<5&&daty>-5&&intervene_can==1){//速度小于则开启分类识别
        uart_send='\x06';  //串口字节发送
        get_class_flag=1;
        uart_putchar(UART_4,uart_send);
      }
    }while(go_flag);
    move_stop();
    move_can=0; 
    intervene_flag=0;
    
    
    if(intervene_can==0){
      intervene_flag=1;
      aim_site.x=ST_car1.x+cos(zap_angle/180.0*PI)*40;//保留每次坐标，作为微调依据
      aim_site.y=ST_car1.y+sin(zap_angle/180.0*PI)*40;
      //system_delay_ms(20);
      //zap_v=20;
      go_flag=1;

      small_move_can=1;
      move_can=1;
      system_delay_ms(40);
      do {
        
        if(img_finish_flag){
          get_img_info(&intervene_flag,(int)Threshold);
          img_finish_flag=0;
          show_img();
        }
        
        update_path();
      
        if (SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0) {
          system_delay_ms(20);
          if(SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0){
            go_flag=0;
            move_can=0;
            move_stop();
        }
        }
      if(datx<5&&datx>-5&&daty<5&&daty>-5&&intervene_can==1){
        uart_send='\x06';  //串口字节发送
        get_class_flag=1;
        uart_putchar(UART_4,uart_send);
      }
      }while(go_flag);      
    }
    move_stop();
    intervene_flag=0;
    move_can=0; 
    small_move_can=0;
    
    if(intervene_can==1){
      if(nowData<=15){
        TGT[nowTGT].main_class=class_table[nowData][0];
        TGT[nowTGT].real_class=class_table[nowData][1];
        TGT[nowTGT].second_class=class_table[nowData][2];
      }
      else BEEP(2000);
      send_carsite(&ST_car1);
      tft180_full(RGB565_WHITE);
      int t;
      
      while(TGT[nowTGT].main_class ==0){
        uart_send='\x06';
        get_class_flag=1;
        uart_putchar(UART_4,uart_send);
        update_path();
        update_path_flag=0;
        t++;
        tft180_show_int(0,2*16,t,5);
        tft180_show_string(0,1*16,"waiting");
        if(nowData<=15){
          TGT[nowTGT].main_class=class_table[nowData][0];
          TGT[nowTGT].real_class=class_table[nowData][1];
          TGT[nowTGT].second_class=class_table[nowData][2];
        }
        else BEEP(2000);
      }
      get_class_flag=0;
      send_carsite(&ST_car1);
      tempTGT_servo.main_class=TGT[nowTGT].main_class;
      tempTGT_servo.second_class=TGT[nowTGT].second_class;
      tempTGT_servo.real_class=TGT[nowTGT].real_class;
      cnt=0;
      
      servo_can=1;
      servo_flag=1;
      system_delay_ms(20);

      while(servo_flag==1||add_finish_flag==0){//||add_finish_flag==0
        update_path();
        update_path_flag=0;
        
      }

    }
    
    if(i>=total_TGT-1){
      update_path();
    
    }
    
    show_path();
    BEEP(200);
    //system_delay_ms(1000);

    
    uart_CNM_flag=0;    
    intervene_can=0;
    add_finish_flag=0;
    nowTGT++;
    speed_max=SPEED_begin;
    system_delay_ms(20); 
    SI.encodernow[0]=0;
    SI.encodernow[1]=0;
    SI.encodernow[2]=0;
    
  }
  while(servo_can==1);
  system_delay_ms(200);
  
  pwm_set_duty(servo_pin_3, 568);
  pwm_set_duty(servo_pin_1, 950);
  pwm_set_duty(servo_pin_2, 1000);  
  
  run_back();
  while(1){
    tft180_full(RGB565_WHITE);
    show_attitude_site();
  }
}

void run_xy_2(){

  
  
  init_carST(&ST_car1);
  get_TGT_flag=1;
  uint8 uart_send='\x05';  //串口字节发送
  uart_putchar(UART_4,uart_send);
  
  send_carST_flag=1;
  //uart_putchar(UART_4,uart_send);  
  while(get_TGT_flag==1){
        tft180_show_string(0,1*16,"waiting");
  }
  /*TGT[4].x=8*20;
  TGT[4].y=10*20;
  TGT[3].x=19*20;
  TGT[3].y=13*20;
  TGT[2].x=15*20;
  TGT[2].y=3*20;
  TGT[1].x=31*20;
  TGT[1].y=13*20;
  TGT[0].x=29*20;
  TGT[0].y=20*20;
  total_TGT=5;*/
  ACO(); 
  BEEP(200);
  
  
  char uart_send_str[5];
  uart_send_str[0]=0xf0;
  uart_send_str[1]=0xfe;
  uart_send_str[2]=3;
  uart_send_str[3]=0xff;
  uart_send_str[4]=0;
  uart_putstr(UART_3,uart_send_str);
  
  //system_delay_ms(500);
    speed_max=SPEED_begin;
  Angle_pid.target_val=0;    
  int go_flag=1;  
  move_can=1;  
  nowTGT=1;
  system_delay_ms(20); 
  intervene_can=0;
  send_carST_flag=1;
  for(int i=0;i<total_TGT;i++){
    send_nowTGT_aimST();
    TGT[nowTGT].sigma=0;
    memset(uart4_receive,0,sizeof(uart4_receive));
    memset(uart1_receive,0,sizeof(uart1_receive));
    add_finish_flag=0; 
    intervene_can=0;
    move_can=1;
    go_flag=1;
    system_delay_ms(20);
    zap_angle=calculate_angle(ST_car1.x,ST_car1.y,TGT[nowTGT].x,TGT[nowTGT].y);//保留每次angle，作为微调依据
    
    intervene_flag=1;
    do {
      show_attitude_site();

      update_path();
      update_path_flag=0;
      
      if (SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0) {
        go_flag=0;
        move_can=0;
        move_stop();
      }
      if(1){
        uart_send='\x06';  //串口字节发送
        get_class_flag=1;
        uart_putchar(UART_4,uart_send);
      }
    }while(go_flag);
    move_stop();
    move_can=0; 
    intervene_flag=0;
    
/*    if(uart_SBCNM[0]!=0){
    
      while(1){
          tft180_full(RGB565_WHITE);
          tft180_show_int(0,0*16,(int)uart_SBCNM[0],4);
          tft180_show_int(70,0*16,(int)uart_SBCNM[1],4); 
          tft180_show_int(0,1*16,(int)uart_SBCNM[2],4); 
          tft180_show_int(70,1*16,(int)uart_SBCNM[3],4); 
          tft180_show_int(0,2*16,(int)uart_SBCNM[4],4); 
          tft180_show_int(70,2*16,(int)uart_SBCNM[5],4); 
          tft180_show_int(0,3*16,(int)uart_SBCNM[6],4); 
          tft180_show_int(70,3*16,(int)uart_SBCNM[7],4);      
          tft180_show_int(0,4*16,(int)uart_SBCNM[8],4);          
      }
   
    } */ 
    
    if(intervene_can==0){
      uart5_init();
      intervene_flag=1;
      aim_site.x=ST_car1.x+cos(zap_angle/180.0*PI)*40;//保留每次坐标，作为微调依据
      aim_site.y=ST_car1.y+sin(zap_angle/180.0*PI)*40;
      //system_delay_ms(20);
      //zap_v=20;
      go_flag=1;
      move_can=1;
      small_move_can=1;
      system_delay_ms(20);
      do {

        update_path();
        update_path_flag=0;
      
        show_attitude_site();
        if (SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0) {
          go_flag=0;
          move_stop();
        }
      if(intervene_can==1){
        uart_send='\x06';  //串口字节发送
        get_class_flag=1;
        uart_putchar(UART_4,uart_send);
      }
      }while(go_flag);      
    }
    move_stop();
    small_move_can=0;
    intervene_flag=0;
    move_can=0; 
 
    
    if(intervene_can==1){
      //uart_CNM_flag=1;
      //uart_send='\x06';  //串口字节发送
      //get_class_flag=1;
      //uart_putchar(UART_4,uart_send);
      //get_class_flag=0;
      //send_carsite(&ST_car1);
      tft180_full(RGB565_WHITE);
      if(nowData<=15){
        TGT[nowTGT].main_class=class_table[nowData][0];
        TGT[nowTGT].real_class=class_table[nowData][1];
        TGT[nowTGT].second_class=class_table[nowData][2];
      }
      else BEEP(2000);
      int t;
      
      while(TGT[nowTGT].main_class ==0){
        //BEEP(200);
        uart_send='\x06';
        get_class_flag=1;
        uart_putchar(UART_4,uart_send);
        update_path();
        update_path_flag=0;
        t++;
        tft180_show_int(0,2*16,t,5);
        tft180_show_string(0,1*16,"waiting");
        if(nowData<=15){
          TGT[nowTGT].main_class=class_table[nowData][0];
          TGT[nowTGT].real_class=class_table[nowData][1];
          TGT[nowTGT].second_class=class_table[nowData][2];
        }
        else BEEP(2000);
      }
      get_class_flag=0;
      //send_carsite(&ST_car1);
      //show_class();   
      //send_carsite(&ST_car1);    
      servo_can=1;
      servo_flag=1;
      judge_send_TGTinfo();
      //system_delay_ms(1000);
      
      while(servo_flag==1){//||add_finish_flag==0
        update_path();
        update_path_flag=0;
        
      }
      uart_send='\x06';  //串口字节发送
      uart_putchar(UART_5,uart_send);
    
    }
    show_path();
    BEEP(200);
    //system_delay_ms(1000);

    
    uart_CNM_flag=0;    
    intervene_can=0;
    add_finish_flag=0;
    nowTGT++;
    speed_max=SPEED_begin;
    system_delay_ms(20); 
    SI.encodernow[0]=0;
    SI.encodernow[1]=0;
    SI.encodernow[2]=0;
    
  }
  
  uart_send_str[0]=0xf0;
  uart_send_str[1]=0xfe;
  uart_send_str[2]=1;
  uart_send_str[3]=0xff;
  uart_send_str[4]=0;
  uart_putstr(UART_3,uart_send_str);
  /*if(uart_SBCNM[0]!=0){
    
      while(1){
          tft180_full(RGB565_WHITE);
          tft180_show_int(0,0*16,(int)uart_SBCNM[0],4);
          tft180_show_int(70,0*16,(int)uart_SBCNM[1],4); 
          tft180_show_int(0,1*16,(int)uart_SBCNM[2],4); 
          tft180_show_int(70,1*16,(int)uart_SBCNM[3],4); 
          tft180_show_int(0,2*16,(int)uart_SBCNM[4],4); 
          tft180_show_int(70,2*16,(int)uart_SBCNM[5],4); 
          tft180_show_int(0,3*16,(int)uart_SBCNM[6],4); 
          tft180_show_int(70,3*16,(int)uart_SBCNM[7],4);      
          tft180_show_int(0,4*16,(int)uart_SBCNM[8],4);          
      }
   
    }*/
  //run_back();
  uart_send_str[0]=0xf0;
  uart_send_str[1]=0xfe;
  uart_send_str[2]=2;
  uart_send_str[3]=0xff;
  uart_send_str[4]=0;
  uart_putstr(UART_3,uart_send_str);
  while(1){
    tft180_full(RGB565_WHITE);
    show_attitude_site();
  }
}


void run_xy_pro(){

  
  
  init_carST(&ST_car1);
  get_TGT_flag=1;
  uint8 uart_send='\x05';  //串口字节发送
  uart_putchar(UART_4,uart_send);
   
  while(get_TGT_flag==1){}
  /*TGT[4].x=8*20;
  TGT[4].y=10*20;
  TGT[3].x=19*20;
  TGT[3].y=13*20;
  TGT[2].x=15*20;
  TGT[2].y=3*20;
  TGT[1].x=31*20;
  TGT[1].y=13*20;
  TGT[0].x=29*20;
  TGT[0].y=20*20;
  total_TGT=5;*/
  ACO(); 


  if(virtual_flag){//开启虚点解决覆盖率问题
    PSODetermineSite();  //加入虚点数为1
    show_path();    
    ACO();
    show_path();    
  }
  else{
    SimpleOriDeside();
  }
  BEEP(200);  

  UpdateRealMap(TGT[0]);
  
  speed_max=SPEED_begin;
  Angle_pid.target_val=0;    
  int go_flag=1;  
  move_can=1;  
  nowTGT=1;
  system_delay_ms(20); 
  intervene_can=0;
  //timer_start(GPT_TIM_1);
  for(int i=0;i<total_TGT;i++){
    nowData=0;
    memset(uart4_receive,0,sizeof(uart4_receive));
    memset(uart1_receive,0,sizeof(uart1_receive));
    send_nowTGT_ori(&ST_car1,0);
    UpdateRealMap(TGT[nowTGT]);
    //send_nowTGT_ori(&ST_car1,0);
    add_finish_flag=0;  
    intervene_can=0;
    move_can=1;
    go_flag=1;
    system_delay_ms(20);

    zap_angle=calculate_angle(ST_car1.x,ST_car1.y,TGT[nowTGT].x,TGT[nowTGT].y);//保留每次angle，作为微调依据
    intervene_flag=0;

    if(TGT[nowTGT].sigma==-1){//虚点
      TGT[nowTGT].sigma=-2;
      intervene_flag=0;
      do {
        
        update_path();
        
        if (SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0) {
          go_flag=0;
          move_can=0;
          move_stop();
        }
      }while(go_flag);
      move_can=0; 
      intervene_flag=0;
    
      uart_send=0Xfd;  //串口字节发送
      uart_putchar(UART_1,uart_send);                         //发送开始看未知点标志
      
      while(add_finish_flag==0);//||add_finish_flag==0
      system_delay_ms(500); 
      
      update_path();
      
    }
    
    else {//实点
      TGT[nowTGT].sigma=0;
      do {
        if(intervene_flag==0&&error_judge(ST_car1.x,ST_car1.y,TGT[nowTGT].x,TGT[nowTGT].y,40)&&error_judge(ST_car1.x,ST_car1.y,TGT[nowTGT-1].x,TGT[nowTGT-1].y,10)==0) {//&&servo_can==0
          intervene_flag=1;
        }
        /*if(intervene_flag){
          if(img_finish_flag){
            get_img_info(&intervene_flag,(int)Threshold);
            img_finish_flag=0;
            show_img();
          }
        }*/
        show_attitude_site();
        update_path();
        
        if (SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0) {//&&(error_finish_flag==1||intervene_can==0)
          system_delay_ms(10);
          if(SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0){
            go_flag=0;
            move_can=0;
            move_stop();
          }
        }
        if(datx<5&&datx>-5&&daty<5&&daty>-5&&intervene_can==1){//速度小于5则开启分类识别
          uart_send='\x06';  //串口字节发送
          get_class_flag=1;
          uart_putchar(UART_4,uart_send);
        }
      }while(go_flag);
      move_can=0; 
      intervene_flag=0;
      
      
      if(intervene_can==0){

        intervene_flag=1;
        aim_site.x=ST_car1.x+cos(zap_angle/180.0*PI)*40;//保留每次坐标，作为微调依据
        aim_site.y=ST_car1.y+sin(zap_angle/180.0*PI)*40;
        
        go_flag=1;
        move_can=1; 
        small_move_can=1;
        system_delay_ms(10);
        do {
          /*if(intervene_flag){
            if(img_finish_flag){
              get_img_info(&intervene_flag,(int)Threshold);
              img_finish_flag=0;
              show_img();
            }
          }*/

          update_path();
          
          if (SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0) {//&&(error_finish_flag==1||intervene_can==0)
            system_delay_ms(10);
            if(SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0){
              go_flag=0;
              move_can=0;
              move_stop();
            }
          }
          if(datx<5&&datx>-5&&daty<5&&daty>-5&&intervene_can==1){//速度小于5则开启分类识别
            uart_send='\x06';  //串口字节发送
            get_class_flag=1;
            uart_putchar(UART_4,uart_send);
          }
        }while(go_flag);      
      }
      small_move_can=0;
      intervene_flag=0;
      move_can=0;
      move_stop();
      
      if(intervene_can==1){
        if(nowData<=15){
          TGT[nowTGT].main_class=class_table[nowData][0];
          TGT[nowTGT].real_class=class_table[nowData][1];
          TGT[nowTGT].second_class=class_table[nowData][2];
        }
        else BEEP(2000);

        uart_send=0Xfd;  //串口字节发送
        uart_putchar(UART_1,uart_send);          //发送开始看未知点标志

        tft180_full(RGB565_WHITE);
        int t;
        while(TGT[nowTGT].main_class ==0){
          uart_send='\x06';
          uart_putchar(UART_4,uart_send);
          update_path();
          t++;
          tft180_show_int(0,2*16,t,5);
          tft180_show_string(0,1*16,"waiting");
          if(nowData<=15){
            TGT[nowTGT].main_class=class_table[nowData][0];
            TGT[nowTGT].real_class=class_table[nowData][1];
            TGT[nowTGT].second_class=class_table[nowData][2];
          }
          else BEEP(2000);
        }
        get_class_flag=0;
        
        
        
        tempTGT_servo.main_class=TGT[nowTGT].main_class;
        tempTGT_servo.second_class=TGT[nowTGT].second_class;
        tempTGT_servo.real_class=TGT[nowTGT].real_class;
        cnt=0;
        
        servo_can=1;
        servo_flag=1;
        system_delay_ms(20);

        while(servo_flag==1||add_finish_flag==0){//||add_finish_flag==0
          update_path();
          
        }

      }
      show_path();
      BEEP(200);

    }    
    intervene_can=0;
    add_finish_flag=0;
    nowTGT++;
    speed_max=SPEED_begin;
    system_delay_ms(20); 
    SI.encodernow[0]=0;
    SI.encodernow[1]=0;
    SI.encodernow[2]=0;
    
  }
  
  Show_path_Map_coverage();  
  
  system_delay_ms(200);
  
  pwm_set_duty(servo_pin_3, 568);
  pwm_set_duty(servo_pin_1, 950);
  pwm_set_duty(servo_pin_2, 1000);  
  
  
  //run_back();
  while(1){
    tft180_full(RGB565_WHITE);
    //show_attitude_site();
    Show_path_Map_coverage(); 
  }
}

void servo_only(){
  nowTGT=1;
  TGT[0].main_class=3;
  TGT[1].main_class=3;
  TGT[2].main_class=2;
  TGT[3].main_class=2;  
  TGT[4].main_class=1; 
  TGT[5].main_class=3; 
  TGT[5].main_class=1;
  total_TGT=7;
  for(int i=0;i<total_TGT;i++){
    tempTGT_servo.main_class=TGT[nowTGT].main_class;
    tempTGT_servo.second_class=TGT[nowTGT].second_class;
    tempTGT_servo.real_class=TGT[nowTGT].real_class;
    
    servo_can=1;
    servo_flag=1;
    tft180_show_uint(80,1*16,TGT[nowTGT].main_class,4);
    system_delay_ms(20);
    while(servo_flag==1){}
    
    nowTGT++;
    BEEP(200);
    system_delay_ms(3000);
  }
}

void fast_debug(){
  total_TGT=2;
  TGT[0].x=10;
  TGT[0].y=10;
  TGT[1].x=10; 
  TGT[1].y=30;
  ACO(); 
}

void run_TMD(){
  get_TGT_flag=1;
  uint8 uart_send='\x01';  //串口字节发送
  uart_putchar(UART_4,uart_send);
  while(get_TGT_flag==1){}
  ACO(); 
  BEEP(200);

  Angle_pid.target_val=0;    
  int go_flag=1;  
  move_can=1;  
  nowTGT=0;
  system_delay_ms(20);
  
  
  for(int i=0;i<total_TGT;i++){
    move_can=1;
    go_flag=1;
    intervene_flag=1;
    do {
      show_attitude_site();
      if (SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0) {
        go_flag=0;
      }    
    }while(go_flag);
    move_can=0;
    intervene_flag=0;  
    intervene_can=0;
    
    
    uart_send='\x02';  //串口字节发送
    get_class_flag=1;
    uart_putchar(UART_4,uart_send);
    while(get_class_flag==1){}
    show_class();
    //send_uart4receive();
    
    
    servo_can=1;
    servo_flag=1;
    while(servo_flag==1){}
    
    nowTGT++;
    SI.encodernow[0]=0;
    SI.encodernow[1]=0;
    SI.encodernow[2]=0;
    system_delay_ms(20);     
  }
  
  run_back();
}




/*void run_TMD(){
  get_TGT_flag=1;

  while(get_TGT_flag==1){}
  ACO();
  send_uart4receive();
  Angle_pid.target_val=0;    
  int go_flag=1;  
  move_can=1;  
  system_delay_ms(20); 

  for(int i=0;i<total_TGT;i++){
    
    second_classify_flag=check_class();
    if(second_classify_flag>3){
      while(second_classify_finish_flag==0){};//等待辅车完成分类
      second_classify_flag=second_classify_flag-2;
    }
    
    move_can=1;
    system_delay_ms(20);     
    do {
      show_attitude_site();
      if (SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0) {
        go_flag=0;
      }    
    }while(go_flag);
    move_can=0;
    
    small_move_flag=1;
    while(small_move_flag==1){}
    
    small_move_can=1;
    go_flag=1;
    do {
      show_attitude_site();
      if (SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0) {
        go_flag=0;
      }    
    }while(go_flag);
    small_move_can=0;
    send_carsite(&ST_car1);    
    if(second_classify_flag==2){
      second_classify_finish_flag=0;
      clear_class(left);
      clear_class(up);      
    }
    else if(second_classify_flag==3){
      second_classify_finish_flag=0;
      clear_class(right);
      clear_class(down);      
    }
    
    get_class_flag=1;
    while(get_class_flag==1){}
    send_uart4receive();
    
    servo_can=second_classify_flag;
    servo_flag=1;
    while(servo_flag==1){}
    
    nowTGT++;
    
  
  }
}*/

/*void run_triangle_1(){//测试速度规划用
  
  int go_flag=1;
  Angle_pid.target_val=0;
  TGT[0].x=0;
  TGT[0].y=30;
  TGT[1].x=40;
  TGT[1].y=200;
  TGT[2].x=80;
  TGT[2].y=30;
  move_can=1;
  while(1){
    nowTGT=0;
    systick_delay_ms(20); 
    set_tsp_target(&tsp_x,&tsp_y,ST_car1.x,ST_car1.y,TGT[nowTGT].x,TGT[nowTGT].y);
    time_optimize(&tsp_x);
    time_optimize(&tsp_y);
    
    for(int i=0;i<3;i++){
      
      do {
        show_attitude_site();
        if (SI.varRB[0]==0&&SI.varLB[0]==0&&SI.varRT[0]==0&&SI.varLT[0]==0) {
          go_flag=0;
        }    
      }while(go_flag);
      nowTGT++;
      systick_delay_ms(20); 
      set_tsp_target(&tsp_x,&tsp_y,ST_car1.x,ST_car1.y,TGT[nowTGT].x,TGT[nowTGT].y);
      time_optimize(&tsp_x);
      time_optimize(&tsp_y);   
      SI.encodernow[0]=0;
      SI.encodernow[1]=0;
      SI.encodernow[2]=0;
      go_flag=1;
      cnt=0;
    }
  }
}*/
    








