#include "pid.h"
extern Speed_Info SI;
volatile PID pid_location;
volatile PID pid_location_x;
volatile PID pid_location_y;
volatile PID pid_speed[4];
volatile PID pid_angle;
volatile PID pid_SMALL;

int intervene_can=0;

volatile T_speed_plan tsp_x;
volatile T_speed_plan tsp_y;

extern EulerAngleTypedef angle;

extern target TGT[44];
extern int nowTGT;
extern float aim_angle;
extern float aim_distance;
extern site ST_car1;
extern site ST_car2;
extern site aim_site;
extern int img_flag;
extern int intervene_can;
extern float x_drift;
extern float y_drift;

extern float small_actual_val;
extern float small_distance;
extern float first_point[2];
extern float last_point[2];
extern float top_point[2];
extern float line_point[188][2];
extern float kl1,kl2; 
extern float bottom_point;
extern int small_flag; 
extern int move_back_can;
extern int move_can;

void init_pid(void)
{
    Location_pid.err=0;
    Location_pid.err_last=0;
    Location_pid.err_prelast=0;//���ϴ�ƫ��
    Location_pid.integral=0;
    Location_pid.output_val=0;
    Location_pid.target_val=0;
    Location_pid.target_val_1=0;
    Location_pid.target_val_2=0;
    Location_pid.UD=0;
    Location_pid.UD_last=0;

    Speed_pid[0].err=0;
    Speed_pid[0].err_last=0;
    Speed_pid[0].err_prelast=0;//���ϴ�ƫ��
    Speed_pid[0].integral=0;
    Speed_pid[0].output_val=0;
    Speed_pid[0].target_val=0;
    Speed_pid[0].target_val_1=0;
    Speed_pid[0].target_val_2=0;

    Speed_pid[1].err=0;
    Speed_pid[1].err_last=0;
    Speed_pid[1].err_prelast=0;//���ϴ�ƫ��
    Speed_pid[1].integral=0;
    Speed_pid[1].output_val=0;
    Speed_pid[1].target_val=0;
    Speed_pid[1].target_val_1=0;
    Speed_pid[1].target_val_2=0;

    Speed_pid[2].err=0;
    Speed_pid[2].err_last=0;
    Speed_pid[2].err_prelast=0;//���ϴ�ƫ��
    Speed_pid[2].integral=0;
    Speed_pid[2].output_val=0;
    Speed_pid[2].target_val=0;
    Speed_pid[2].target_val_1=0;
    Speed_pid[2].target_val_2=0;

    Speed_pid[3].err=0;
    Speed_pid[3].err_last=0;
    Speed_pid[3].err_prelast=0;//���ϴ�ƫ��
    Speed_pid[3].integral=0;
    Speed_pid[3].output_val=0;
    Speed_pid[3].target_val=0;
    Speed_pid[3].target_val_1=0;
    Speed_pid[3].target_val_2=0;
    
    Angle_pid.err=0;
    Angle_pid.err_last=0;
    Angle_pid.err_prelast=0;//���ϴ�ƫ��
    Angle_pid.integral=0;
    Angle_pid.output_val=0;
    Angle_pid.target_val=0;
    Angle_pid.target_val_1=0;
    Angle_pid.target_val_2=0;

    SMALL_pid.err=0;
    SMALL_pid.err_last=0;
    SMALL_pid.err_prelast=0;//���ϴ�ƫ��
    SMALL_pid.integral=0;
    SMALL_pid.output_val=0;
    SMALL_pid.target_val=0;
    SMALL_pid.target_val_1=0;
    SMALL_pid.target_val_2=0;

    Location_pid.Kd=0;
    Location_pid.Ki=0;
    Location_pid.Kp=100;

    Speed_pid[0].Kd=0;
    Speed_pid[0].Ki=0;
    Speed_pid[0].Kp=300;
    
    Speed_pid[1].Kd=0;
    Speed_pid[1].Ki=0;
    Speed_pid[1].Kp=300;
    
    Speed_pid[2].Kd=0;
    Speed_pid[2].Ki=0;
    Speed_pid[2].Kp=300;

    Speed_pid[3].Kd=0;
    Speed_pid[3].Ki=0;
    Speed_pid[3].Kp=300;
    
    Angle_pid.Kd=1;
    Angle_pid.Ki=0;
    Angle_pid.Kp=4;

    SMALL_pid.Kd=0.5;
    SMALL_pid.Ki=0.1;
    SMALL_pid.Kp=1;
}

void init_T_speed_plan(){
  tsp_x.a_acc=0;
  tsp_x.a_dec=0;
  tsp_x.a_max=500;
  tsp_x.l_begin=0;
  tsp_x.l_end=0;
  tsp_x.t_acc=0;
  tsp_x.t_begin=0;
  tsp_x.t_dec=0;
  tsp_x.t_end=0;
  tsp_x.t_tal=0;
  tsp_x.t_uv=0;
  tsp_x.v_begin=0;
  tsp_x.v_end=0;
  tsp_x.v_max=300;
  tsp_x.v_uv=0;
    
  tsp_y.a_acc=0;
  tsp_y.a_dec=0;
  tsp_y.a_max=500;
  tsp_y.l_begin=0;
  tsp_y.l_end=0;
  tsp_y.t_acc=0;
  tsp_y.t_begin=0;
  tsp_y.t_dec=0;
  tsp_y.t_end=0;
  tsp_y.t_tal=0;
  tsp_y.t_uv=0;
  tsp_y.v_begin=0;
  tsp_y.v_end=0;
  tsp_y.v_max=300;
  tsp_y.v_uv=0;
}

/*******
2023.1.9����
�����ٶȹ滮
�о�����������ٶȹ滮��λ�û�pidЧ������
��Ȼ����pid���ݵ�Ҳ�Ϳ������
�ͷ�������ļ��°�
********/

void set_tsp_target(T_speed_plan volatile *tspx,T_speed_plan volatile *tspy, float x,float y,float x1,float y1){
  tspx->l_begin=x;
  tspy->l_begin=y;
  tspx->l_end=x1;
  tspy->l_end=y1;
}

/*******
�滮��ʱ������

********/
void time_optimize(T_speed_plan volatile *tsp){
  float h=abs_float(((tsp->l_end)-(tsp->l_begin)));
  
  if((tsp->v_max)*(tsp->v_max)-((tsp->v_begin)*(tsp->v_begin)+(tsp->v_end)*(tsp->v_end))/2.0<=h*tsp->a_max){
    tsp->v_uv=tsp->v_max;
    tsp->t_acc=((tsp->v_max)-(tsp->v_begin))/(tsp->a_max);
    tsp->t_dec=-((tsp->v_end)-(tsp->v_max))/(tsp->a_max);
    tsp->t_uv=(h-(((tsp->v_max)*(tsp->v_max)-(tsp->v_begin)*(tsp->v_begin))/2*(tsp->a_max)+((tsp->v_max)*(tsp->v_max)-(tsp->v_end)*(tsp->v_end))/2*(tsp->a_max)))/(tsp->v_max);
    tsp->t_tal=(tsp->t_acc)+(tsp->t_dec)+(tsp->t_uv);
  }
  else{
    tsp->v_uv=sqrt(h*(tsp->a_max)+((tsp->v_begin)*(tsp->v_begin)+(tsp->v_end)*(tsp->v_end))/2.0);
    tsp->t_acc=((tsp->v_uv)-(tsp->v_begin))/(tsp->a_max);
    tsp->t_dec=((tsp->v_uv)-(tsp->v_end))/(tsp->a_max);
    tsp->t_uv=0;
    tsp->t_tal=(tsp->t_acc)+(tsp->t_dec)+(tsp->t_uv);   
  }
}

/*******
�滮��ʱ���ڵ���
v����vbegin��vendʱ����
*******/
void time_limit(T_speed_plan volatile *tsp,float t){
  tsp->t_tal=t;
  float h=abs_float((tsp->l_end-tsp->l_begin));
  float v1=t*tsp->a_max/2.0+tsp->v_begin/2.0+tsp->v_end/2.0-
       sqrt(t*t*(tsp->a_max)*(tsp->a_max)+2*t*(tsp->a_max)*(tsp->v_end)-4*h*(tsp->a_max)-(tsp->v_begin)*(tsp->v_begin)+2*(tsp->v_begin)*(tsp->v_end)-(tsp->v_end)*(tsp->v_end))/2.0;   
  float v2=t*tsp->a_max/2.0+tsp->v_begin/2.0+tsp->v_end/2.0+
       sqrt(t*t*(tsp->a_max)*(tsp->a_max)+2*t*(tsp->a_max)*(tsp->v_end)-4*h*(tsp->a_max)-(tsp->v_begin)*(tsp->v_begin)+2*(tsp->v_begin)*(tsp->v_end)-(tsp->v_end)*(tsp->v_end))/2.0; 
  if(v1>=0&&v2>=0){
    if(v1<v2)
      tsp->v_uv=v1;
    else
      tsp->v_uv=v2;
  }
  else if(v1>=0&&v2<0)
    tsp->v_uv=v1;
  else if(v1<0&&v2>=0)
    tsp->v_uv=v2;
  
  tsp->t_acc=(tsp->v_uv-tsp->v_begin)/(tsp->a_max);
  tsp->t_dec=(tsp->v_uv-tsp->v_end)/(tsp->a_max);  
  tsp->t_uv=t-tsp->t_acc-tsp->t_dec;
}

/*******
���㵱ǰʱ��Ŀ���ٶ�

*******/
float calculate_speed(T_speed_plan volatile *tsp,float t){
  if(t <= (tsp->t_acc))
    return (tsp->v_begin)+(tsp->a_max)*t;
  else if((t > (tsp->t_acc)) && (t <= (tsp->t_acc) + (tsp->t_uv)))
    return (tsp->v_uv);
  else if((t > (tsp->t_acc) + (tsp->t_uv)) && (t < (tsp->t_tal)))
    return (tsp->v_uv)-(tsp->a_max)*(t-((tsp->t_acc) + (tsp->t_uv)));
  return 0.0;
}

/*******
·��ʵ��
�����ٶȹ滮��ת��pid�ó�
ÿ�������ٶ�
�ٶȻ�ʵ��Ŀ���ٶ�

*******/
float control_val=0;
float control_val_x=0;//��λcm/s
float control_val_y=0;//��λcm/s
float actual_speed[4]={0.0,0.0,0.0,0.0};
int res_pwm[4]={0,0,0,0};
int TO_STR=0;
float error_x=0;
float error_y=0;
float error=0;
int speed_max=SPEED_begin;

extern int motor_test_flag;
//#define  DEBUG_pid
void control_path(){
  float vx;
  float vy; 
  
  #ifdef DEBUG_pid
    static int cnt=0;
    if(speed_max<250){
      cnt+=2;
    }
    if (cnt==40){
      speed_max+=cnt;
      cnt=0;
    }
  #else
    if(speed_max<250){//400ms�ڵ�������ٶ�
      speed_max+=4;
    }
  #endif  
    
  #ifdef DEBUG_pid
    if  (motor_test_flag==0){
      send_int16_data((int16)(actual_speed[0]*100),(int16)(Speed_pid[0].target_val*100),(int16)(actual_speed[1]*100),(int16)(Speed_pid[1].target_val*100));
    }
    else{
      send_int16_data((int16)(actual_speed[2]*100),(int16)(Speed_pid[2].target_val*100),(int16)(actual_speed[3]*100),(int16)(Speed_pid[3].target_val*100));
    }
  #endif 
    
  TO_STR=controlAS_pwm_STR();
  error_x=(TGT[nowTGT].x-ST_car1.x)/wheel_diam/PI/30*1024*70;
  error_y=(TGT[nowTGT].y-ST_car1.y)/wheel_diam/PI/30*1024*70;
  //aim_angle=calculate_angle(ST_car1.x,ST_car1.y,TGT[nowTGT].x,TGT[nowTGT].y);
  control_val_x=location_pid_realize(&Location_pid_x,error_x);//������ֵ���ɾ���,��λcm������1s����
  control_val_y=location_pid_realize(&Location_pid_y,error_y);//������ֵ���ɾ���,��λcm������1s����
  //speed_protect_pro(&control_val_x,speed_max);
  //speed_protect_pro(&control_val_y,speed_max);
  speed_protect_pro_2(&control_val_x,&control_val_y,speed_max);
  vx=control_val_x;
  vy=control_val_y;
  set_pid_target(&Speed_pid[0], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))+(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))-TO_STR/100.0);//motor1
  set_pid_target(&Speed_pid[1], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))-(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))+TO_STR/100.0);//�趨�ٶ�pid��Ŀ��ֵ
  set_pid_target(&Speed_pid[2], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))-(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))-TO_STR/100.0);
  set_pid_target(&Speed_pid[3], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))+(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))+TO_STR/100.0);
  
 // ʵ���ٶ�
  actual_speed[0]=(float)(SI.varLB[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI); //cm/s*/
  actual_speed[1]=(float)(SI.varRB[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);
  actual_speed[2]=(float)(SI.varLT[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);
  actual_speed[3]=(float)(SI.varRT[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);

  res_pwm[0]=(int)(calc_vPID(&Speed_pid[0],actual_speed[0]));//���p=1,��ô���ٶ�Ҳ��3000
  res_pwm[1]=(int)(calc_vPID(&Speed_pid[1],actual_speed[1]));
  res_pwm[2]=(int)(calc_vPID(&Speed_pid[2],actual_speed[2]));
  res_pwm[3]=(int)(calc_vPID(&Speed_pid[3],actual_speed[3]));
  pwm_protect_pro(res_pwm);
  move_set(res_pwm[0],res_pwm[1],res_pwm[2],res_pwm[3]);
  
  
} 


extern float zap_v;
extern float zap_angle;
void control_path_small(){
  float vx;
  float vy; 
  float k=1;
  TO_STR=controlAS_pwm_STR();
  if(intervene_can==0){  
    aim_angle=calculate_angle(ST_car1.x,ST_car1.y,aim_site.x,aim_site.y);
    //BEEP(200);
    error=sqrt((aim_site.x-ST_car1.x)*(aim_site.x-ST_car1.x)+(aim_site.y-ST_car1.y)*(aim_site.y-ST_car1.y))/wheel_diam/PI/30*1024*70;
    control_val=location_pid_realize(&Location_pid,error); //������ֵ���ɾ���,��λcm������1s����
  }
  else {
    aim_angle=calculate_angle(ST_car1.x,ST_car1.y,TGT[nowTGT].x,TGT[nowTGT].y);
    error=sqrt((TGT[nowTGT].x-ST_car1.x)*(TGT[nowTGT].x-ST_car1.x)+(TGT[nowTGT].y-ST_car1.y)*(TGT[nowTGT].y-ST_car1.y))/wheel_diam/PI/30*1024*70;
    control_val=location_pid_realize(&Location_pid,error); //������ֵ���ɾ���,��λcm������1s����
  }
  if(move_can) speed_protect_pro(&control_val,80);
  else if(move_back_can) speed_protect_pro(&control_val,250);
  vx=k*cos(aim_angle/180.0*PI)*control_val;
  vy=sin(aim_angle/180.0*PI)*control_val;
  set_pid_target(&Speed_pid[0], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))+(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))-TO_STR/100.0);//motor1
  set_pid_target(&Speed_pid[1], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))-(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))+TO_STR/100.0);//�趨�ٶ�pid��Ŀ��ֵ
  set_pid_target(&Speed_pid[2], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))-(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))-TO_STR/100.0);
  set_pid_target(&Speed_pid[3], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))+(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))+TO_STR/100.0);
  
 // ʵ���ٶ�
  actual_speed[0]=(float)(SI.varLB[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI); //cm/s*/
  actual_speed[1]=(float)(SI.varRB[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);
  actual_speed[2]=(float)(SI.varLT[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);
  actual_speed[3]=(float)(SI.varRT[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);

  res_pwm[0]=(int)(calc_vPID(&Speed_pid[0],actual_speed[0]));//���p=1,��ô���ٶ�Ҳ��3000
  res_pwm[1]=(int)(calc_vPID(&Speed_pid[1],actual_speed[1]));
  res_pwm[2]=(int)(calc_vPID(&Speed_pid[2],actual_speed[2]));
  res_pwm[3]=(int)(calc_vPID(&Speed_pid[3],actual_speed[3]));
  pwm_protect_pro(res_pwm);
  move_set(res_pwm[0],res_pwm[1],res_pwm[2],res_pwm[3]);
} 

/*void control_path(){
  float vx;
  float vy; 
  float k=1;  
  TO_STR=controlAS_pwm_STR();
  img_flag=0;
  if(img_flag==0){
    aim_angle=calculate_angle(ST_car1.x,ST_car1.y,TGT[nowTGT].x,TGT[nowTGT].y);

    error=sqrt((TGT[nowTGT].x-ST_car1.x)*(TGT[nowTGT].x-ST_car1.x)+(TGT[nowTGT].y-ST_car1.y)*(TGT[nowTGT].y-ST_car1.y))/wheel_diam/PI/30*1024*70;
    control_val=location_pid_realize(&Location_pid,error);//������ֵ���ɾ���,��λcm������1s����
  
  }
  else{
    aim_angle=calculate_angle(TGT_info[0].center_x,TGT_info[0].center_y,view_center_x,view_center_y);

    error=sqrt((TGT_info[0].center_x-view_center_x)*(TGT_info[0].center_x-view_center_x)+(TGT_info[0].center_y-view_center_y)*(TGT_info[0].center_y-view_center_y))/wheel_diam/PI/30*1024*70;
    control_val=location_pid_realize(&Location_pid,error);//������ֵ���ɾ���,��λcm������1s����
  }
  speed_protect_pro(&control_val,250);
  if(aim_angle<0&&aim_angle>-90){
    //k=(90+aim_angle)/90.0;
    k=1;
  }
  if(aim_angle<-90&&aim_angle>-180){
    //k=(-aim_angle-90)/90.0;
    k=1;
  }
  vx=k*cos(aim_angle/180.0*PI)*control_val;
  vy=sin(aim_angle/180.0*PI)*control_val;
  set_pid_target(&Speed_pid[0], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))+(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))-TO_STR/100.0);//motor1
  set_pid_target(&Speed_pid[1], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))-(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))+TO_STR/100.0);//�趨�ٶ�pid��Ŀ��ֵ
  set_pid_target(&Speed_pid[2], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))-(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))-TO_STR/100.0);
  set_pid_target(&Speed_pid[3], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))+(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))+TO_STR/100.0);
  
 // ʵ���ٶ�
  actual_speed[0]=(float)(SI.varLB[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI); //cm/s
  actual_speed[1]=(float)(SI.varRB[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);
  actual_speed[2]=(float)(SI.varLT[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);
  actual_speed[3]=(float)(SI.varRT[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);

  res_pwm[0]=(int)(calc_vPID(&Speed_pid[0],actual_speed[0]));//���p=1,��ô���ٶ�Ҳ��3000
  res_pwm[1]=(int)(calc_vPID(&Speed_pid[1],actual_speed[1]));
  res_pwm[2]=(int)(calc_vPID(&Speed_pid[2],actual_speed[2]));
  res_pwm[3]=(int)(calc_vPID(&Speed_pid[3],actual_speed[3]));
  pwm_protect_pro(res_pwm);
  move_set(res_pwm[0],res_pwm[1],res_pwm[2],res_pwm[3]);
} 
*/

void control_path_on_back(int speed_max_max){
  float vx;
  float vy;
  if(speed_max<speed_max_max){//400ms�ڵ�������ٶ�
    speed_max+=4;
  }
  aim_angle=calculate_angle(ST_car1.x,ST_car1.y,aim_site.x,aim_site.y);
  TO_STR=controlAS_pwm_STR();
  error=sqrt((aim_site.x-ST_car1.x)*(aim_site.x-ST_car1.x)+(aim_site.y-ST_car1.y)*(aim_site.y-ST_car1.y))/wheel_diam/PI/30*1024*70;
  control_val=location_pid_realize(&Location_pid,error);//������ֵ���ɾ���,��λcm������1s����
  speed_protect_pro(&control_val,speed_max);

  vx=cos(aim_angle/180.0*PI)*control_val;
  vy=sin(aim_angle/180.0*PI)*control_val;
  set_pid_target(&Speed_pid[0], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))+(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))-TO_STR/100.0);//motor1
  set_pid_target(&Speed_pid[1], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))-(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))+TO_STR/100.0);//�趨�ٶ�pid��Ŀ��ֵ
  set_pid_target(&Speed_pid[2], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))-(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))-TO_STR/100.0);
  set_pid_target(&Speed_pid[3], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))+(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))+TO_STR/100.0);
  
 // ʵ���ٶ�
  actual_speed[0]=(float)(SI.varLB[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI); //cm/s*/
  actual_speed[1]=(float)(SI.varRB[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);
  actual_speed[2]=(float)(SI.varLT[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);
  actual_speed[3]=(float)(SI.varRT[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);

  res_pwm[0]=(int)(calc_vPID(&Speed_pid[0],actual_speed[0]));//���p=1,��ô���ٶ�Ҳ��3000
  res_pwm[1]=(int)(calc_vPID(&Speed_pid[1],actual_speed[1]));
  res_pwm[2]=(int)(calc_vPID(&Speed_pid[2],actual_speed[2]));
  res_pwm[3]=(int)(calc_vPID(&Speed_pid[3],actual_speed[3]));
  pwm_protect_pro(res_pwm);
  move_set(res_pwm[0],res_pwm[1],res_pwm[2],res_pwm[3]);
} 

/*void control_path_on_back_se(int speed_max){
  float vx;
  float vy;   
  
  aim_angle=zap_angle;
  TO_STR=controlAS_pwm_STR();
  control_val=speed_max;
  
  
  vx=cos(aim_angle/180.0*PI)*control_val;
  vy=sin(aim_angle/180.0*PI)*control_val;
  set_pid_target(&Speed_pid[0], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))+(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI)));//motor1
  set_pid_target(&Speed_pid[1], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))-(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI)));//�趨�ٶ�pid��Ŀ��ֵ
  set_pid_target(&Speed_pid[2], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))-(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI)));
  set_pid_target(&Speed_pid[3], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))+(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI)));
  
  
  res_pwm[0]=(int)(calc_vPID(&Speed_pid[0],actual_speed[0]));//���p=1,��ô���ٶ�Ҳ��3000
  res_pwm[1]=(int)(calc_vPID(&Speed_pid[1],actual_speed[1]));
  res_pwm[2]=(int)(calc_vPID(&Speed_pid[2],actual_speed[2]));
  res_pwm[3]=(int)(calc_vPID(&Speed_pid[3],actual_speed[3]));
  pwm_protect_pro(res_pwm);
  move_set(res_pwm[0]-TO_STR,res_pwm[1]+TO_STR,res_pwm[2]-TO_STR,res_pwm[3]+TO_STR);

  
}*/


void control_path_on_back_se(int speed_max){
  float vx;
  float vy;   
  float step=50;
  

  
  aim_angle=calculate_angle(ST_car1.x,ST_car1.y,aim_site.x,aim_site.y);
  TO_STR=controlAS_pwm_STR();
  error=sqrt((aim_site.x-ST_car1.x)*(aim_site.x-ST_car1.x)+(aim_site.y-ST_car1.y)*(aim_site.y-ST_car1.y))/wheel_diam/PI/30*1024*70;
  control_val=location_pid_realize(&Location_pid,error);//������ֵ���ɾ���,��λcm������1s����
  if(intervene_can==0&&control_val<50){
    aim_angle=calculate_angle(ST_car1.x,ST_car1.y,aim_site.x+cos(zap_angle/180.0*PI)*step,aim_site.y+cos(zap_angle/180.0*PI)*step);
    step+=50;
  }  
  
  speed_protect_pro(&control_val,speed_max);
  
  
  vx=cos(aim_angle/180.0*PI)*control_val;
  vy=sin(aim_angle/180.0*PI)*control_val;
  set_pid_target(&Speed_pid[0], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))+(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI)));//motor1
  set_pid_target(&Speed_pid[1], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))-(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI)));//�趨�ٶ�pid��Ŀ��ֵ
  set_pid_target(&Speed_pid[2], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))-(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI)));
  set_pid_target(&Speed_pid[3], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))+(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI)));
  
  
  res_pwm[0]=(int)(calc_vPID(&Speed_pid[0],actual_speed[0]));//���p=1,��ô���ٶ�Ҳ��3000
  res_pwm[1]=(int)(calc_vPID(&Speed_pid[1],actual_speed[1]));
  res_pwm[2]=(int)(calc_vPID(&Speed_pid[2],actual_speed[2]));
  res_pwm[3]=(int)(calc_vPID(&Speed_pid[3],actual_speed[3]));
  pwm_protect_pro(res_pwm);
  move_set(res_pwm[0],res_pwm[1],res_pwm[2],res_pwm[3]);

  
}

void set_pid_target(PID volatile *pid, float temp_val)
{  
	pid->target_val = temp_val;    // ���õ�ǰ��Ŀ��ֵ
} 

/*******
����ʽpid��delta_u(k)=u(k)-u(k-1)
                     =kp*[e(k)-e(k-1)]+ki*e(k)+kd*[e(k)-2e(k-1)+e(k-2)]
}*******/
/*****************
�ٶȻ�,��������ʽȥʵ��λ��ʽ����Ȼ��������������
�����ֵ�pid������ڵĿ���������
����Ϊʲô�õ�������λ��ʽȴд������ʽ
p������540�о����    
*****************/
#define SPE_DEAD_ZONE 3.0f /*�ٶȻ�����*/
float Incremental_PID_realize(PID volatile *pid, float actual_val)
{  
  float increase;
 
  pid->err = pid->target_val - actual_val;
  
  /* �趨�ջ����� */
   if( (pid->err>-SPE_DEAD_ZONE) && (pid->err<SPE_DEAD_ZONE ) )
      {
      increase=0;
      pid->err=0;
      pid->err_last=0;
      pid->err_prelast=0;
        }        
  
  increase  =  pid->Kp*(pid->err-pid->err_last)
            +pid->Ki*pid->err
            +pid->Kd*(pid->err-2*pid->err_last+pid->err_prelast);
  
 
  
  pid->err_last=pid->err;
  pid->err_prelast=pid->err_last;
  
  pid->output_val+=increase;
  
  return pid->output_val;
}


void clear_output_val(){
  Speed_pid[0].output_val=0;
  Speed_pid[1].output_val=0;
  Speed_pid[2].output_val=0;
  Speed_pid[3].output_val=0;

}
//λ��ʽpid��u(k)=kp*e(k)+ki*�ۼ�e(i)+kd[e(k)-e(k-1)]
/*****************
λ�û�,���ַ������������
*****************/
#define LOC_DEAD_ZONE 66 /*λ�û�����*///115 80
//2cm�Ͳ��ٵ���
#define LOC_INTEGRAL_START_ERR 2400 /*���ַ���ʱ��Ӧ����Χ*///800//20cm
//200->1.7355,������20cm�������֣�����pid����
#define LOC_INTEGRAL_MAX_VAL 8000 /*���ַ�Χ�޶�����ֹ���ֱ���*/ 
//��ֱ�Ӵ����������ܶ����ɣ�����������cmת��������������dat=(int)(distance/wheel_diam/PI/30*1024*70)  m��
///dat/1024*30/70*wheel_diam*PI;//������ֵ���ɾ���,��λcm
//��ɵķ���Ӧ�����ȼ��ٿ쵽Ŀ����ʱ����٣�����������ٷǳ���˳������֪������ôʵ�ֵ�
int stop_run=0;
float location_pid_realize(PID volatile *pid, float error)
{
  float a=0.5,index=0;
  float kp,ki,kd;
    
    //dat=pid->target_val/wheel_diam/PI/30*1024*70;//ת���ɱ�������������
    //pid->err = dat - actual_val;/*����Ŀ��ֵ��ʵ��ֵ�����*/
    pid->err = error;
    /* �趨�ջ����� */
      if((pid->err >= -LOC_DEAD_ZONE) && (pid->err <= LOC_DEAD_ZONE))
      {
        pid->err = 0;
        pid->integral = 0;
        pid->err_last = 0;
        stop_run=1;
      }
    /*��������ַ��룬ƫ��ϴ�ʱȥ����������*/
      if(abs_float(pid->err) <(LOC_INTEGRAL_START_ERR))
        {
          index=1;
          pid->integral += pid->err;
          kp=pid->Kp;
          ki=pid->Ki;
          kd=pid->Kd;
        }
      else if (abs_float(pid->err) >LOC_INTEGRAL_START_ERR)
      {
        index=0.01;
        pid->integral += 0*pid->err;
        kp=pid->Kp;
        ki=pid->Ki;
        kd=pid->Kd;
      }

      /*���ַ�Χ�޶�����ֹ���ֱ���*/
           if(pid->integral > LOC_INTEGRAL_MAX_VAL)
        {
          pid->integral = LOC_INTEGRAL_MAX_VAL;
          }
         else if(pid->integral < -LOC_INTEGRAL_MAX_VAL)
        {
            pid->integral = -LOC_INTEGRAL_MAX_VAL;
            }     
    //����ȫ΢��pid
        pid->UD=kd /10.0*(1-a)*(pid->err - pid->err_last)+a*pid->UD_last;
    /*PID�㷨ʵ��*/
    pid->output_val = kp /10* pid->err +
                      index*ki /10* pid->integral +
                      pid->UD;

    /*����*/
    pid->err_last = pid->err;
    pid->UD_last=pid->UD;
    /*���ص�ǰʵ��ֵ*/
    return pid->output_val/1024*30/70*wheel_diam*PI*2;//������ֵ���ɾ���,��λcm������0.5s����

}


/*�ٶȻ�pid*/
//��λcm/s
//�����ֵĴ�����ֲ,����
//ȷʵ���㣬�����������ʽ���ٶȻ��ö���
float calc_vPID(PID volatile *pid, float actual_val)// �����������ַ�+���ַ��뷨
{
    float error;
    float duty = 0.0;
    float once_i;
    float kp_t,ki_index,kd_t;
   
  
    kp_t = (float)((pid->Kp) /10.0);              //��ΪҪ�ֵ�ź�ͼ��ͬ��PID����û���ýṹ��?
    kd_t = (float)((pid->Kd) /10.0);				//��ʵ���� �ټ�3��������
    error = pid->target_val - actual_val;

    //���ֿ���
    if(error + pid->err_last>= 0)                   //����ֿ��ƣ�����ʱ?��ƫ��ϴ��ʱ���������ֵ�Ч��,��Ȼ���׳�����(���Ŀ��-���ʵ��ֵ+�ϴ�Ŀ��-�ϴ�ʵ��ֵ)>0����Ϊ����
    {
        ki_index = (pid->Ki / 10.0) - (pid->Ki / 10.0) / (1 + exp((float)pid->Ki-  1* abs_float(error)));    
    }
    else                                            //����ʱ? (ƫ����������仯��ƫ���С����ʱ��Ӵ�������ã���Ϊƫ��С)  ���ñ��ٻ���
    {
        ki_index =  pid->Ki / 10.0;  //0.9;//ki - ki / (1 + exp(kib - 0.2*abs(error)));
    }

	
	//7.12����������ʱ��ѻ�����ȫ���ˣ������ܲ���ɲ��������Ϊ��ʱ��ɲ�ıȽ���
	if(stop_run==1)    //ֻ�����ĵ�һʱ�̿���
	{
	    pid->integral=0;   // ��������    ɲ������
	}

	
    //��ͨ��΢�ֻ��ڻ��� ϵͳ���ֽ�Ծ�ź�ʱ�����ϴ���Ӧ����ʱ��ͷ�����΢�ֱ����ˣ����ʱ�򲢲��ǳ����ˣ����ǽ�����ϵͳ����Ӧ�ٶȣ������˶�̬���̣���̬������Ӧ�ٶȣ�
    //������������ֻ��һ˲�䣬��û��������������Ծ�źŵ����á���������������½�Ϊ0����Ҳ�ᵼ��ϵͳ����
    //����ȫ΢��  ���ڳ��ֽ�Ծ�źź󣬷ּ�������У�����������΢�ֱ��ͣ���΢��ʧ�أ������
    //����ȫ΢��  ����ȫ��΢����ʹ����ƫ������Ծʽ�仯ʱ���ֵ����˲ʱ����õ�һ���̶ȵĻ���
    //΢���źŵ�������Ը���ϵͳ�Ķ�̬���ԣ���Ҳ�������Ƶ���ţ�������Ŷ�ͻ���ʱ�������Գ�
    //΢����Ĳ��㡣Ҫ����������⣬�����ڿ����㷨�м����ͨ�˲���������֮һ������PID�㷨��
    //����һ��һ�׹��Ի��ڣ���ͨ�˲�������ʹ��ϵͳ�����ܵõ����ơ�
    
    pid->UD = (float)kd_t * (error - pid->err_last) * 0.5 + 0.5 * (float)pid->UD;	//����ȫ΢�֣�����֮ǰ��΢������͵�ǰ��΢��ֵ���м�Ȩ��Ŀǰ������0.5��һ��һ��
  
    once_i = 0.5 * (float)ki_index * (error + pid->err_last);                //�Ա��κ��ϴ�ƫ������ۻ������λ��֦�(error+preError)*T/2   ��С��ֵ����߾���
  
    duty = pid->integral + kp_t * error + pid->UD;                         
  
    //����һ�������;Ͱѻ���ֵ����ס�����ֱ��;���˵�ڻ���������ռ�ձ��Ѿ�������ֵ��
    //����ƫ����Ȼ���ڣ����ʱ�����������ס�������ô��ƫ���ʱ�������ڻ���ֵ����
    //�޷�������ȷ����Ӧ
    //��Чƫ�
    //˼·��������Ŀ������������Ʒ�Χʱ������Ӧ����һ��������ƫ��ֵ��Ϊ��Чƫ��ֵ���л��֣�
    //      �����ǽ�ʵ��ƫ��ֵ���л��֡�
    
    if(duty > MOTOR_PWM_MIN && duty < MOTOR_PWM_MAX)      
    {
        duty += once_i;                        //ռ�ձȼ��ϻ����� (���û�й�����������������)
        if(duty > MOTOR_PWM_MAX)              //ռ�ձȼ��ϻ����� ������ռ�ձȵĻ���˵�����ֹ�������
        {
            float temp;
            temp = duty - MOTOR_PWM_MAX;       //temp�ǻ��ֹ����͵Ĳ���
            once_i -= temp;			           //�ѻ��ֵ��� ���������͵Ĳ���            
            duty = MOTOR_PWM_MAX;		       //ռ�ձ���Ϊ��	
        }
        else if(duty < MOTOR_PWM_MIN)
        {
            float temp;
            temp = duty - MOTOR_PWM_MIN;
            once_i -= temp;
            duty = MOTOR_PWM_MIN;
        }
        pid->integral += once_i;	                   //�������������ۼ�???	//��ֹ���ֹ�����      
    }
    else if((duty >= MOTOR_PWM_MAX && once_i < 0) || (duty <= MOTOR_PWM_MIN && once_i > 0))
    {     
        pid->integral += once_i;
        duty += once_i;
    }
    
    pid->err_last = error;
  
    if(duty > MOTOR_PWM_MAX)
    {
        duty = MOTOR_PWM_MAX;
    }
    else if(duty < MOTOR_PWM_MIN)
    {
        duty = MOTOR_PWM_MIN;
    }

    
	
    
    return duty;
    
}

/**************************
�ǶȻ�
mΪ��־λ����Ϊ��ֱ�ߺ�תȦ����
***************************/
#define ANGLE_DEAD_ZONE 0.5f /*�ǶȻ�����*/
//��ʱp=4,i=0,d=1
#define ANGLE_INTEGRAL_START_ERR 30 /*���ַ���ʱ��Ӧ����Χ*/
//ƫ��30������ʱ����������
#define ANGLE_INTEGRAL_MAX_VAL 30  /*���ַ�Χ�޶�����ֹ���ֱ���*/
float angle_pid_realize(PID volatile *pid, float actual_val,int m)
{
  if (m==0)
  {
      /*����Ŀ��ֵ��ʵ��ֵ�����*/
      pid->err = (pid->target_val - actual_val);

      /* �趨�ջ����� */
            if( (pid->err>-ANGLE_DEAD_ZONE) && (pid->err<ANGLE_DEAD_ZONE ) )
            {
            pid->err = 0;
            pid->integral = 0;
            pid->err_last = 0;
              }

      /*��������ַ��룬ƫ��ϴ�ʱȥ����������*/
        if(pid->err > -ANGLE_INTEGRAL_START_ERR && pid->err < ANGLE_INTEGRAL_START_ERR)
          {
          pid->integral += pid->err;
            /*���ַ�Χ�޶�����ֹ���ֱ���*/
              if(pid->integral > ANGLE_INTEGRAL_MAX_VAL)
                {
                    pid->integral = ANGLE_INTEGRAL_MAX_VAL;
                          }
               else if(pid->integral < -ANGLE_INTEGRAL_MAX_VAL)
                {
                      pid->integral = -ANGLE_INTEGRAL_MAX_VAL;
                    }
              }

      /*PID�㷨ʵ��*/
            pid->output_val = pid->Kp * pid->err +
                        pid->Ki * pid->integral +
                        pid->Kd *(pid->err - pid->err_last);

      /*����*/
            pid->err_last = pid->err;

      /*���ص�ǰʵ��ֵ*/
  }
  else
  { /*����Ŀ��ֵ��ʵ��ֵ�����*/
      pid->err = (pid->target_val - actual_val);

      /* �趨�ջ����� */
            if( (pid->err>-ANGLE_DEAD_ZONE) && (pid->err<ANGLE_DEAD_ZONE ) )
            {
            pid->err = 0;
            pid->integral = 0;
            pid->err_last = 0;
              }

      /*��������ַ��룬ƫ��ϴ�ʱȥ����������*/
        if(pid->err > -ANGLE_INTEGRAL_START_ERR && pid->err < ANGLE_INTEGRAL_START_ERR)
          {
          pid->integral += pid->err;
            /*���ַ�Χ�޶�����ֹ���ֱ���*/
              if(pid->integral > ANGLE_INTEGRAL_MAX_VAL)
                {
                    pid->integral = ANGLE_INTEGRAL_MAX_VAL;
                          }
               else if(pid->integral < -ANGLE_INTEGRAL_MAX_VAL)
                {
                      pid->integral = -ANGLE_INTEGRAL_MAX_VAL;
                    }
              }

      /*PID�㷨ʵ��*/
            pid->output_val = 1 * pid->err +
                       0.1 * pid->integral +
                        0.5 *(pid->err - pid->err_last);

      /*����*/
            pid->err_last = pid->err;
  }
      return pid->output_val;
}

//�Ƕ��ٶȿ��ƴ���
float AScontrol_val=0;
void controlAS_pwm(void)//д��ת�򻷵�
{ 
  float L;
  AScontrol_val=0;  
  AScontrol_val=angle_pid_realize(&Angle_pid,AD_L_Yaw_angle,0);
  L=AScontrol_val*100;
  if (L > 15000)
	{
		L = 15000;
	}
	else if (L < -15000)
	{
		L = -15000;
	}	
  move_turn((int)L);
}

int controlAS_pwm_STR(void)//������ֱ������
{   
   AScontrol_val=0;
   AScontrol_val=angle_pid_realize(&Angle_pid,AD_L_Yaw_angle,0);
   speed_protect_1(&AScontrol_val);
  return (int)AScontrol_val*150;//120
}
//------------------------------------------------------------------------------------
//�������������ƶ�ֱ��,��Ү��Ч�����
//����λ���ٶȿ��ƴ���
int location_time=0;

void control_straight()
{ 
  aim_angle=calculate_angle(ST_car1.x,ST_car1.y,TGT[nowTGT].x,TGT[nowTGT].y);
  TO_STR=controlAS_pwm_STR();
  float vx;
  float vy;
  /*if (location_time++%10){*/
     error=sqrt((TGT[nowTGT].x-ST_car1.x)*(TGT[nowTGT].x-ST_car1.x)+(TGT[nowTGT].y-ST_car1.y)*(TGT[nowTGT].y-ST_car1.y))/wheel_diam/PI/30*1024*70;
    control_val=location_pid_realize(&Location_pid,error);//������ֵ���ɾ���,��λcm������1s����
    speed_protect(&control_val);
    vx=cos(aim_angle/180.0*PI)*control_val;
    vy=sin(aim_angle/180.0*PI)*control_val;
    set_pid_target(&Speed_pid[0], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))+(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))-TO_STR/100.0);//motor1
    set_pid_target(&Speed_pid[1], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))-(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))+TO_STR/100.0);//�趨�ٶ�pid��Ŀ��ֵ
    set_pid_target(&Speed_pid[2], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))-(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))-TO_STR/100.0);
    set_pid_target(&Speed_pid[3], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))+(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))+TO_STR/100.0);
  
 // ʵ���ٶ�
  actual_speed[0]=(float)(SI.varLB[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI); //cm/s*/
  actual_speed[1]=(float)(SI.varRB[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);
  actual_speed[2]=(float)(SI.varLT[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);
  actual_speed[3]=(float)(SI.varRT[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);

  res_pwm[0]=(int)(calc_vPID(&Speed_pid[0],actual_speed[0]));//���p=1,��ô���ٶ�Ҳ��3000
  res_pwm[1]=(int)(calc_vPID(&Speed_pid[1],actual_speed[1]));
  res_pwm[2]=(int)(calc_vPID(&Speed_pid[2],actual_speed[2]));
  res_pwm[3]=(int)(calc_vPID(&Speed_pid[3],actual_speed[3]));
  pwm_protect_pro(res_pwm);
  move_set(res_pwm[0],res_pwm[1],res_pwm[2],res_pwm[3]);
  //move_set(-TO_STR,TO_STR,-TO_STR,TO_STR);
}

void straight_test_on_angle()
{ 
  aim_angle=calculate_angle(ST_car1.x,ST_car1.y,TGT[nowTGT].x,TGT[nowTGT].y);
  TO_STR=controlAS_pwm_STR();
  /*float vx;
  float vy;
  if (location_time++%10){//
    control_val=location_pid_realize(&Location_pid,sqrt(SI.encodernow[1]*SI.encodernow[1]+SI.encodernow[2]*SI.encodernow[2]));//������ֵ���ɾ���,��λcm������1s����
    speed_protect(&control_val);
    vx=cos(aim_angle/180.0*PI)*control_val;
    vy=sin(aim_angle/180.0*PI)*control_val;

  }*/
 // ʵ���ٶ� 
  set_pid_target(&Speed_pid[0], -TO_STR/100.0);//motor1
  set_pid_target(&Speed_pid[1], +TO_STR/100.0);//�趨�ٶ�pid��Ŀ��ֵ
  set_pid_target(&Speed_pid[2], -TO_STR/100.0);
  set_pid_target(&Speed_pid[3], +TO_STR/100.0);
  actual_speed[0]=(float)(SI.varLB[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI); //cm/s*/
  actual_speed[1]=(float)(SI.varRB[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);
  actual_speed[2]=(float)(SI.varLT[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);
  actual_speed[3]=(float)(SI.varRT[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);

  res_pwm[0]=(int)(Incremental_PID_realize(&Speed_pid[0],actual_speed[0]));//���p=1,��ô���ٶ�Ҳ��3000
  res_pwm[1]=(int)(Incremental_PID_realize(&Speed_pid[1],actual_speed[1]));
  res_pwm[2]=(int)(Incremental_PID_realize(&Speed_pid[2],actual_speed[2]));
  res_pwm[3]=(int)(Incremental_PID_realize(&Speed_pid[3],actual_speed[3]));
  pwm_protect_pro(res_pwm);

  move_set(res_pwm[0],res_pwm[1],res_pwm[2],res_pwm[3]);
  //move_set(-TO_STR,TO_STR,-TO_STR,TO_STR);
}


//--------------------------------------------------
//λ��΢��pid
//------------------------------------------
#define SMALL_INTEGRAL_START_ERR 10 /*���ַ���ʱ��Ӧ����Χ*/
#define SMALL_INTEGRAL_MAX_VAL 260  /*���ַ�Χ�޶�����ֹ���ֱ���*/
float SMALL_pid_realize(PID volatile *pid, float target_val ,float actual_val,float SMALL_CHANGE_ZONE)
{
/*����Ŀ��ֵ��ʵ��ֵ�����*/
pid->err = (target_val - actual_val);

/* �趨�ջ����� */
      if( (pid->err>-SMALL_CHANGE_ZONE) && (pid->err<SMALL_CHANGE_ZONE ) )
      {
      pid->err = 0;
      pid->integral = 0;
      pid->err_last = 0;
        }
      

/*��������ַ��룬ƫ��ϴ�ʱȥ����������*/
  if(pid->err > -SMALL_INTEGRAL_START_ERR && pid->err < SMALL_INTEGRAL_START_ERR)
    {
    pid->integral += pid->err;
      /*���ַ�Χ�޶�����ֹ���ֱ���*/
        if(pid->integral > SMALL_INTEGRAL_MAX_VAL)
          {
              pid->integral = SMALL_INTEGRAL_MAX_VAL;
                    }
         else if(pid->integral < -SMALL_INTEGRAL_MAX_VAL)
          {
                pid->integral = -SMALL_INTEGRAL_MAX_VAL;
              }
        }

/*PID�㷨ʵ��*/
      pid->output_val = pid->Kp * pid->err +
                  pid->Ki*1.25 * pid->integral +
                  pid->Kd *(pid->err - pid->err_last);

/*����*/
      pid->err_last = pid->err;

/*���ص�ǰʵ��ֵ*/
      return pid->output_val;
}
//ǰ�����˵�
#define SMALL_CHANGE_ZONE_1 5.0f /*��ϻ�����*/
#define SMALL_INTEGRAL_START_ERR_1 20 /*���ַ���ʱ��Ӧ����Χ*/
#define SMALL_INTEGRAL_MAX_VAL_1 260  /*���ַ�Χ�޶�����ֹ���ֱ���*/
float SMALL_pid_realize_1(PID volatile *pid, float actual_val)
{
/*����Ŀ��ֵ��ʵ��ֵ�����*/
pid->err = (pid->target_val_1 - actual_val);

/* �趨�ջ����� */
      if( (pid->err>-SMALL_CHANGE_ZONE_1) && (pid->err<SMALL_CHANGE_ZONE_1 ) )
      {
      pid->err = 0;
      pid->integral = 0;
      pid->err_last = 0;
        }

/*��������ַ��룬ƫ��ϴ�ʱȥ����������*/
  if(pid->err > -SMALL_INTEGRAL_START_ERR_1 && pid->err < SMALL_INTEGRAL_START_ERR_1)
    {
    pid->integral += pid->err;
      /*���ַ�Χ�޶�����ֹ���ֱ���*/
        if(pid->integral > SMALL_INTEGRAL_MAX_VAL_1)
          {
              pid->integral = SMALL_INTEGRAL_MAX_VAL_1;
                    }
         else if(pid->integral < -SMALL_INTEGRAL_MAX_VAL_1)
          {
                pid->integral = -SMALL_INTEGRAL_MAX_VAL_1;
              }
        }

/*PID�㷨ʵ��*/
      pid->output_val = pid->Kp * pid->err +
                  pid->Ki *0.1* pid->integral +
                  pid->Kd *(pid->err - pid->err_last);

/*����*/
      pid->err_last = pid->err;

/*���ص�ǰʵ��ֵ*/
      return pid->output_val;
}
#define SMALL_CHANGE_ZONE_2 10.0f /*ת������*/
#define SMALL_INTEGRAL_START_ERR_2 100 /*���ַ���ʱ��Ӧ����Χ*/
#define SMALL_INTEGRAL_MAX_VAL_2 260  /*���ַ�Χ�޶�����ֹ���ֱ���*/
float SMALL_pid_realize_2(PID volatile *pid, float actual_val)
{
/*����Ŀ��ֵ��ʵ��ֵ�����*/
pid->err = (pid->target_val_2 - actual_val);

/* �趨�ջ����� */
      if( (pid->err>-SMALL_CHANGE_ZONE_2) && (pid->err<SMALL_CHANGE_ZONE_2 ) )
      {
      pid->err = 0;
      pid->integral = 0;
      pid->err_last = 0;
        }

/*��������ַ��룬ƫ��ϴ�ʱȥ����������*/
  if(pid->err > -SMALL_INTEGRAL_START_ERR_2 && pid->err < SMALL_INTEGRAL_START_ERR_2)
    {
    pid->integral += pid->err;
      /*���ַ�Χ�޶�����ֹ���ֱ���*/
        if(pid->integral > SMALL_INTEGRAL_MAX_VAL_2)
          {
              pid->integral = SMALL_INTEGRAL_MAX_VAL_2;
                    }
         else if(pid->integral < -SMALL_INTEGRAL_MAX_VAL_2)
          {
                pid->integral = -SMALL_INTEGRAL_MAX_VAL_2;
              }
        }

/*PID�㷨ʵ��*/
      pid->output_val = pid->Kp * pid->err +
                  pid->Ki * pid->integral +
                  pid->Kd *(pid->err - pid->err_last);

/*����*/
      pid->err_last = pid->err;

/*���ص�ǰʵ��ֵ*/
      return pid->output_val;
}
//----------------------------------------------------------------------------------
//����pid����С������΢��
float small_pwm=0;
float small_pwm_1=0;
int str_control=0;
//��ʵ������ʱ�����ֱ�ӿ���λ�ã�Ȼ����ƽ��֮���
void small_control()
{  
   AScontrol_val=0;
   if (last_point[0]<20||first_point[0]>160&&first_point[0]!=0&& last_point[0]!=0)
   {
     if (last_point[0]<20)
     {
       small_pwm=SMALL_pid_realize(&SMALL_pid,40 ,last_point[0],5.0);//���һ����С��20������ƽ��
       speed_protect_1(&small_pwm);
       move_set((int)small_pwm*200,(int)-small_pwm*200,(int)-small_pwm*200,(int)small_pwm*200);
     }
     else
     {
      small_pwm=SMALL_pid_realize(&SMALL_pid,140 ,first_point[0],5.0);
      speed_protect_1(&small_pwm);
      move_set((int)small_pwm*200,(int)-small_pwm*200,(int)-small_pwm*200,(int)small_pwm*200);
     }
   }
   else
   {
    /*if (kl2!=0&&kl1!=0)
    {
      AScontrol_val=SMALL_pid_realize_2(&SMALL_pid,kl1);//Ŀ��ֵ��0
      speed_protect_1(&AScontrol_val);
      str_control=(int)AScontrol_val*200;
      move_set(str_control,-str_control,0,0);
      
    }
    else 
    {*/
      if(first_point[0]!=0&& last_point[0]!=0)
      {
        small_pwm_1 =SMALL_pid_realize_1(&SMALL_pid,small_distance);//Ŀ��ֵ��95��small_distance=(first_point[1]+last_point[1])/2;
        speed_protect_1(&small_pwm_1);//ǰ������
        
      }
      else
        small_pwm_1 =0;
     
       small_pwm=SMALL_pid_realize(&SMALL_pid, 0,small_actual_val,25);//Ŀ��ֵ��0��small_actual_val=kl1-kl2;�����������ԣ���������ʶ���
       speed_protect_1(&small_pwm);
    
     TO_STR=controlAS_pwm_STR();
     move_set((int)(-(small_pwm*200)-TO_STR+small_pwm_1*200),(int)((small_pwm*200)+TO_STR+small_pwm_1*200),(int)((small_pwm*200)-TO_STR+small_pwm_1*200),(int)(-(small_pwm*200)+TO_STR+small_pwm_1*200));
    }
   if (last_point[0]==0)
     move_behind();
   
   AScontrol_val=0;
}

