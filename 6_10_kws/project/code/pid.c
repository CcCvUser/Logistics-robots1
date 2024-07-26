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
    Location_pid.err_prelast=0;//上上次偏差
    Location_pid.integral=0;
    Location_pid.output_val=0;
    Location_pid.target_val=0;
    Location_pid.target_val_1=0;
    Location_pid.target_val_2=0;
    Location_pid.UD=0;
    Location_pid.UD_last=0;

    Speed_pid[0].err=0;
    Speed_pid[0].err_last=0;
    Speed_pid[0].err_prelast=0;//上上次偏差
    Speed_pid[0].integral=0;
    Speed_pid[0].output_val=0;
    Speed_pid[0].target_val=0;
    Speed_pid[0].target_val_1=0;
    Speed_pid[0].target_val_2=0;

    Speed_pid[1].err=0;
    Speed_pid[1].err_last=0;
    Speed_pid[1].err_prelast=0;//上上次偏差
    Speed_pid[1].integral=0;
    Speed_pid[1].output_val=0;
    Speed_pid[1].target_val=0;
    Speed_pid[1].target_val_1=0;
    Speed_pid[1].target_val_2=0;

    Speed_pid[2].err=0;
    Speed_pid[2].err_last=0;
    Speed_pid[2].err_prelast=0;//上上次偏差
    Speed_pid[2].integral=0;
    Speed_pid[2].output_val=0;
    Speed_pid[2].target_val=0;
    Speed_pid[2].target_val_1=0;
    Speed_pid[2].target_val_2=0;

    Speed_pid[3].err=0;
    Speed_pid[3].err_last=0;
    Speed_pid[3].err_prelast=0;//上上次偏差
    Speed_pid[3].integral=0;
    Speed_pid[3].output_val=0;
    Speed_pid[3].target_val=0;
    Speed_pid[3].target_val_1=0;
    Speed_pid[3].target_val_2=0;
    
    Angle_pid.err=0;
    Angle_pid.err_last=0;
    Angle_pid.err_prelast=0;//上上次偏差
    Angle_pid.integral=0;
    Angle_pid.output_val=0;
    Angle_pid.target_val=0;
    Angle_pid.target_val_1=0;
    Angle_pid.target_val_2=0;

    SMALL_pid.err=0;
    SMALL_pid.err_last=0;
    SMALL_pid.err_prelast=0;//上上次偏差
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
2023.1.9更新
加入速度规划
感觉我们这个组速度规划比位置环pid效果更好
虽然不是pid内容但也和控制相关
就放在这个文件下吧
********/

void set_tsp_target(T_speed_plan volatile *tspx,T_speed_plan volatile *tspy, float x,float y,float x1,float y1){
  tspx->l_begin=x;
  tspy->l_begin=y;
  tspx->l_end=x1;
  tspy->l_end=y1;
}

/*******
规划求时间最优

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
规划定时间内到达
v大于vbegin和vend时适用
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
计算当前时间目标速度

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
路径实现
梯形速度规划和转向环pid得出
每个轮子速度
速度环实现目标速度

*******/
float control_val=0;
float control_val_x=0;//单位cm/s
float control_val_y=0;//单位cm/s
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
    if(speed_max<250){//400ms内到达最快速度
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
  control_val_x=location_pid_realize(&Location_pid_x,error_x);//编码器值换成距离,单位cm，并在1s到达
  control_val_y=location_pid_realize(&Location_pid_y,error_y);//编码器值换成距离,单位cm，并在1s到达
  //speed_protect_pro(&control_val_x,speed_max);
  //speed_protect_pro(&control_val_y,speed_max);
  speed_protect_pro_2(&control_val_x,&control_val_y,speed_max);
  vx=control_val_x;
  vy=control_val_y;
  set_pid_target(&Speed_pid[0], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))+(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))-TO_STR/100.0);//motor1
  set_pid_target(&Speed_pid[1], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))-(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))+TO_STR/100.0);//设定速度pid的目标值
  set_pid_target(&Speed_pid[2], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))-(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))-TO_STR/100.0);
  set_pid_target(&Speed_pid[3], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))+(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))+TO_STR/100.0);
  
 // 实际速度
  actual_speed[0]=(float)(SI.varLB[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI); //cm/s*/
  actual_speed[1]=(float)(SI.varRB[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);
  actual_speed[2]=(float)(SI.varLT[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);
  actual_speed[3]=(float)(SI.varRT[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);

  res_pwm[0]=(int)(calc_vPID(&Speed_pid[0],actual_speed[0]));//如果p=1,那么满速度也是3000
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
    control_val=location_pid_realize(&Location_pid,error); //编码器值换成距离,单位cm，并在1s到达
  }
  else {
    aim_angle=calculate_angle(ST_car1.x,ST_car1.y,TGT[nowTGT].x,TGT[nowTGT].y);
    error=sqrt((TGT[nowTGT].x-ST_car1.x)*(TGT[nowTGT].x-ST_car1.x)+(TGT[nowTGT].y-ST_car1.y)*(TGT[nowTGT].y-ST_car1.y))/wheel_diam/PI/30*1024*70;
    control_val=location_pid_realize(&Location_pid,error); //编码器值换成距离,单位cm，并在1s到达
  }
  if(move_can) speed_protect_pro(&control_val,80);
  else if(move_back_can) speed_protect_pro(&control_val,250);
  vx=k*cos(aim_angle/180.0*PI)*control_val;
  vy=sin(aim_angle/180.0*PI)*control_val;
  set_pid_target(&Speed_pid[0], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))+(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))-TO_STR/100.0);//motor1
  set_pid_target(&Speed_pid[1], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))-(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))+TO_STR/100.0);//设定速度pid的目标值
  set_pid_target(&Speed_pid[2], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))-(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))-TO_STR/100.0);
  set_pid_target(&Speed_pid[3], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))+(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))+TO_STR/100.0);
  
 // 实际速度
  actual_speed[0]=(float)(SI.varLB[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI); //cm/s*/
  actual_speed[1]=(float)(SI.varRB[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);
  actual_speed[2]=(float)(SI.varLT[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);
  actual_speed[3]=(float)(SI.varRT[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);

  res_pwm[0]=(int)(calc_vPID(&Speed_pid[0],actual_speed[0]));//如果p=1,那么满速度也是3000
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
    control_val=location_pid_realize(&Location_pid,error);//编码器值换成距离,单位cm，并在1s到达
  
  }
  else{
    aim_angle=calculate_angle(TGT_info[0].center_x,TGT_info[0].center_y,view_center_x,view_center_y);

    error=sqrt((TGT_info[0].center_x-view_center_x)*(TGT_info[0].center_x-view_center_x)+(TGT_info[0].center_y-view_center_y)*(TGT_info[0].center_y-view_center_y))/wheel_diam/PI/30*1024*70;
    control_val=location_pid_realize(&Location_pid,error);//编码器值换成距离,单位cm，并在1s到达
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
  set_pid_target(&Speed_pid[1], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))-(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))+TO_STR/100.0);//设定速度pid的目标值
  set_pid_target(&Speed_pid[2], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))-(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))-TO_STR/100.0);
  set_pid_target(&Speed_pid[3], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))+(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))+TO_STR/100.0);
  
 // 实际速度
  actual_speed[0]=(float)(SI.varLB[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI); //cm/s
  actual_speed[1]=(float)(SI.varRB[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);
  actual_speed[2]=(float)(SI.varLT[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);
  actual_speed[3]=(float)(SI.varRT[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);

  res_pwm[0]=(int)(calc_vPID(&Speed_pid[0],actual_speed[0]));//如果p=1,那么满速度也是3000
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
  if(speed_max<speed_max_max){//400ms内到达最快速度
    speed_max+=4;
  }
  aim_angle=calculate_angle(ST_car1.x,ST_car1.y,aim_site.x,aim_site.y);
  TO_STR=controlAS_pwm_STR();
  error=sqrt((aim_site.x-ST_car1.x)*(aim_site.x-ST_car1.x)+(aim_site.y-ST_car1.y)*(aim_site.y-ST_car1.y))/wheel_diam/PI/30*1024*70;
  control_val=location_pid_realize(&Location_pid,error);//编码器值换成距离,单位cm，并在1s到达
  speed_protect_pro(&control_val,speed_max);

  vx=cos(aim_angle/180.0*PI)*control_val;
  vy=sin(aim_angle/180.0*PI)*control_val;
  set_pid_target(&Speed_pid[0], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))+(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))-TO_STR/100.0);//motor1
  set_pid_target(&Speed_pid[1], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))-(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))+TO_STR/100.0);//设定速度pid的目标值
  set_pid_target(&Speed_pid[2], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))-(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))-TO_STR/100.0);
  set_pid_target(&Speed_pid[3], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))+(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))+TO_STR/100.0);
  
 // 实际速度
  actual_speed[0]=(float)(SI.varLB[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI); //cm/s*/
  actual_speed[1]=(float)(SI.varRB[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);
  actual_speed[2]=(float)(SI.varLT[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);
  actual_speed[3]=(float)(SI.varRT[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);

  res_pwm[0]=(int)(calc_vPID(&Speed_pid[0],actual_speed[0]));//如果p=1,那么满速度也是3000
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
  set_pid_target(&Speed_pid[1], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))-(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI)));//设定速度pid的目标值
  set_pid_target(&Speed_pid[2], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))-(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI)));
  set_pid_target(&Speed_pid[3], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))+(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI)));
  
  
  res_pwm[0]=(int)(calc_vPID(&Speed_pid[0],actual_speed[0]));//如果p=1,那么满速度也是3000
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
  control_val=location_pid_realize(&Location_pid,error);//编码器值换成距离,单位cm，并在1s到达
  if(intervene_can==0&&control_val<50){
    aim_angle=calculate_angle(ST_car1.x,ST_car1.y,aim_site.x+cos(zap_angle/180.0*PI)*step,aim_site.y+cos(zap_angle/180.0*PI)*step);
    step+=50;
  }  
  
  speed_protect_pro(&control_val,speed_max);
  
  
  vx=cos(aim_angle/180.0*PI)*control_val;
  vy=sin(aim_angle/180.0*PI)*control_val;
  set_pid_target(&Speed_pid[0], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))+(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI)));//motor1
  set_pid_target(&Speed_pid[1], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))-(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI)));//设定速度pid的目标值
  set_pid_target(&Speed_pid[2], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))-(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI)));
  set_pid_target(&Speed_pid[3], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))+(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI)));
  
  
  res_pwm[0]=(int)(calc_vPID(&Speed_pid[0],actual_speed[0]));//如果p=1,那么满速度也是3000
  res_pwm[1]=(int)(calc_vPID(&Speed_pid[1],actual_speed[1]));
  res_pwm[2]=(int)(calc_vPID(&Speed_pid[2],actual_speed[2]));
  res_pwm[3]=(int)(calc_vPID(&Speed_pid[3],actual_speed[3]));
  pwm_protect_pro(res_pwm);
  move_set(res_pwm[0],res_pwm[1],res_pwm[2],res_pwm[3]);

  
}

void set_pid_target(PID volatile *pid, float temp_val)
{  
	pid->target_val = temp_val;    // 设置当前的目标值
} 

/*******
增量式pid：delta_u(k)=u(k)-u(k-1)
                     =kp*[e(k)-e(k-1)]+ki*e(k)+kd*[e(k)-2e(k-1)+e(k-2)]
}*******/
/*****************
速度环,得用增量式去实现位置式，不然会咔咔咔咔咔咔
老四轮的pid花里胡哨的看都看不懂
他们为什么用的明明是位置式却写着增量式
p调到了540感觉差不多    
*****************/
#define SPE_DEAD_ZONE 3.0f /*速度环死区*/
float Incremental_PID_realize(PID volatile *pid, float actual_val)
{  
  float increase;
 
  pid->err = pid->target_val - actual_val;
  
  /* 设定闭环死区 */
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
//位置式pid：u(k)=kp*e(k)+ki*累加e(i)+kd[e(k)-e(k-1)]
/*****************
位置环,积分分离加死区控制
*****************/
#define LOC_DEAD_ZONE 66 /*位置环死区*///115 80
//2cm就不再调节
#define LOC_INTEGRAL_START_ERR 2400 /*积分分离时对应的误差范围*///800//20cm
//200->1.7355,现在是20cm开启积分，进行pid控制
#define LOC_INTEGRAL_MAX_VAL 8000 /*积分范围限定，防止积分饱和*/ 
//就直接传编码器的总读数吧，参数：距离cm转编码器脉冲数：dat=(int)(distance/wheel_diam/PI/30*1024*70)  m车
///dat/1024*30/70*wheel_diam*PI;//编码器值换成距离,单位cm
//逐飞的方法应该是先加速快到目标点的时候减速，但是这个减速非常的顺滑，不知道是怎么实现的
int stop_run=0;
float location_pid_realize(PID volatile *pid, float error)
{
  float a=0.5,index=0;
  float kp,ki,kd;
    
    //dat=pid->target_val/wheel_diam/PI/30*1024*70;//转换成编码器的脉冲数
    //pid->err = dat - actual_val;/*计算目标值与实际值的误差*/
    pid->err = error;
    /* 设定闭环死区 */
      if((pid->err >= -LOC_DEAD_ZONE) && (pid->err <= LOC_DEAD_ZONE))
      {
        pid->err = 0;
        pid->integral = 0;
        pid->err_last = 0;
        stop_run=1;
      }
    /*积分项，积分分离，偏差较大时去掉积分作用*/
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

      /*积分范围限定，防止积分饱和*/
           if(pid->integral > LOC_INTEGRAL_MAX_VAL)
        {
          pid->integral = LOC_INTEGRAL_MAX_VAL;
          }
         else if(pid->integral < -LOC_INTEGRAL_MAX_VAL)
        {
            pid->integral = -LOC_INTEGRAL_MAX_VAL;
            }     
    //不完全微分pid
        pid->UD=kd /10.0*(1-a)*(pid->err - pid->err_last)+a*pid->UD_last;
    /*PID算法实现*/
    pid->output_val = kp /10* pid->err +
                      index*ki /10* pid->integral +
                      pid->UD;

    /*误差传递*/
    pid->err_last = pid->err;
    pid->UD_last=pid->UD;
    /*返回当前实际值*/
    return pid->output_val/1024*30/70*wheel_diam*PI*2;//编码器值换成距离,单位cm，并在0.5s到达

}


/*速度环pid*/
//单位cm/s
//老四轮的代码移植,真香
//确实真香，比上面的增量式的速度环好多了
float calc_vPID(PID volatile *pid, float actual_val)// 遇限削弱积分法+积分分离法
{
    float error;
    float duty = 0.0;
    float once_i;
    float kp_t,ki_index,kd_t;
   
  
    kp_t = (float)((pid->Kp) /10.0);              //因为要分电磁和图像不同的PID所以没有用结构体?
    kd_t = (float)((pid->Kd) /10.0);				//其实可以 再加3个参数啊
    error = pid->target_val - actual_val;

    //积分控制
    if(error + pid->err_last>= 0)                   //变积分控制（加速时?即偏差较大的时候，削弱积分的效果,不然容易超调）(这次目标-这次实际值+上次目标-上次实际值)>0即认为加速
    {
        ki_index = (pid->Ki / 10.0) - (pid->Ki / 10.0) / (1 + exp((float)pid->Ki-  1* abs_float(error)));    
    }
    else                                            //减速时? (偏差出现正负变化即偏差很小，这时候加大积分作用，因为偏差小)  不用变速积分
    {
        ki_index =  pid->Ki / 10.0;  //0.9;//ki - ki / (1 + exp(kib - 0.2*abs(error)));
    }

	
	//7.12尝试在入库的时候把积分量全砍了，看看能不能刹下来，因为这时候刹的比较猛
	if(stop_run==1)    //只在入库的第一时刻砍掉
	{
	    pid->integral=0;   // 不砍积分    刹不下来
	}

	
    //普通的微分环节会在 系统出现阶跃信号时产生较大响应，这时候就发生了微分饱和了，这个时候并不是超调了，而是降低了系统的响应速度，减慢了动态过程（暂态，即响应速度）
    //而且他的作用只有一瞬间，并没有起到慢慢消除阶跃信号的作用。而是输出量急剧下降为0，这也会导致系统的振荡
    //不完全微分  是在出现阶跃信号后，分几次慢慢校正，避免出现微分饱和（即微分失控）的情况
    //不完全微分  不完全的微分它使得在偏差作阶跃式变化时出现的输出瞬时跳变得到一定程度的缓和
    //微分信号的引入可以改善系统的动态特性，但也易引入高频干扰，在误差扰动突变的时候尤其显出
    //微分项的不足。要想解决这个问题，可以在控制算法中加入低通滤波器，方法之一就是在PID算法中
    //加入一个一阶惯性环节（低通滤波器）可使得系统的性能得到改善。
    
    pid->UD = (float)kd_t * (error - pid->err_last) * 0.5 + 0.5 * (float)pid->UD;	//不完全微分，即对之前的微分输出和当前的微分值进行加权，目前给的是0.5即一半一半
  
    once_i = 0.5 * (float)ki_index * (error + pid->err_last);                //对本次和上次偏差进行累积，梯形积分Σ(error+preError)*T/2   减小残值，提高精度
  
    duty = pid->integral + kp_t * error + pid->UD;                         
  
    //积分一旦过饱和就把积分值限制住，积分饱和就是说在积分作用下占空比已经超过满值了
    //但是偏差仍然存在，这个时候如果不限制住积分项，那么当偏差反向时，会由于积分值过大
    //无法快速正确的相应
    //有效偏差法
    //思路：当算出的控制量超出限制范围时，将相应的这一控制量的偏差值作为有效偏差值进行积分，
    //      而不是将实际偏差值进行积分。
    
    if(duty > MOTOR_PWM_MIN && duty < MOTOR_PWM_MAX)      
    {
        duty += once_i;                        //占空比加上积分项 (如果没有过饱和这个就是输出项)
        if(duty > MOTOR_PWM_MAX)              //占空比加上积分项 大于满占空比的话，说明积分过饱和了
        {
            float temp;
            temp = duty - MOTOR_PWM_MAX;       //temp是积分过饱和的部分
            once_i -= temp;			           //把积分的量 砍掉过饱和的部分            
            duty = MOTOR_PWM_MAX;		       //占空比设为满	
        }
        else if(duty < MOTOR_PWM_MIN)
        {
            float temp;
            temp = duty - MOTOR_PWM_MIN;
            once_i -= temp;
            duty = MOTOR_PWM_MIN;
        }
        pid->integral += once_i;	                   //积分项的输出量累加???	//防止积分过饱和      
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
角度环
m为标志位，分为走直线和转圈控制
***************************/
#define ANGLE_DEAD_ZONE 0.5f /*角度环死区*/
//暂时p=4,i=0,d=1
#define ANGLE_INTEGRAL_START_ERR 30 /*积分分离时对应的误差范围*/
//偏差30度左右时，开启积分
#define ANGLE_INTEGRAL_MAX_VAL 30  /*积分范围限定，防止积分饱和*/
float angle_pid_realize(PID volatile *pid, float actual_val,int m)
{
  if (m==0)
  {
      /*计算目标值与实际值的误差*/
      pid->err = (pid->target_val - actual_val);

      /* 设定闭环死区 */
            if( (pid->err>-ANGLE_DEAD_ZONE) && (pid->err<ANGLE_DEAD_ZONE ) )
            {
            pid->err = 0;
            pid->integral = 0;
            pid->err_last = 0;
              }

      /*积分项，积分分离，偏差较大时去掉积分作用*/
        if(pid->err > -ANGLE_INTEGRAL_START_ERR && pid->err < ANGLE_INTEGRAL_START_ERR)
          {
          pid->integral += pid->err;
            /*积分范围限定，防止积分饱和*/
              if(pid->integral > ANGLE_INTEGRAL_MAX_VAL)
                {
                    pid->integral = ANGLE_INTEGRAL_MAX_VAL;
                          }
               else if(pid->integral < -ANGLE_INTEGRAL_MAX_VAL)
                {
                      pid->integral = -ANGLE_INTEGRAL_MAX_VAL;
                    }
              }

      /*PID算法实现*/
            pid->output_val = pid->Kp * pid->err +
                        pid->Ki * pid->integral +
                        pid->Kd *(pid->err - pid->err_last);

      /*误差传递*/
            pid->err_last = pid->err;

      /*返回当前实际值*/
  }
  else
  { /*计算目标值与实际值的误差*/
      pid->err = (pid->target_val - actual_val);

      /* 设定闭环死区 */
            if( (pid->err>-ANGLE_DEAD_ZONE) && (pid->err<ANGLE_DEAD_ZONE ) )
            {
            pid->err = 0;
            pid->integral = 0;
            pid->err_last = 0;
              }

      /*积分项，积分分离，偏差较大时去掉积分作用*/
        if(pid->err > -ANGLE_INTEGRAL_START_ERR && pid->err < ANGLE_INTEGRAL_START_ERR)
          {
          pid->integral += pid->err;
            /*积分范围限定，防止积分饱和*/
              if(pid->integral > ANGLE_INTEGRAL_MAX_VAL)
                {
                    pid->integral = ANGLE_INTEGRAL_MAX_VAL;
                          }
               else if(pid->integral < -ANGLE_INTEGRAL_MAX_VAL)
                {
                      pid->integral = -ANGLE_INTEGRAL_MAX_VAL;
                    }
              }

      /*PID算法实现*/
            pid->output_val = 1 * pid->err +
                       0.1 * pid->integral +
                        0.5 *(pid->err - pid->err_last);

      /*误差传递*/
            pid->err_last = pid->err;
  }
      return pid->output_val;
}

//角度速度控制代码
float AScontrol_val=0;
void controlAS_pwm(void)//写进转向环的
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

int controlAS_pwm_STR(void)//放在走直线里面
{   
   AScontrol_val=0;
   AScontrol_val=angle_pid_realize(&Angle_pid,AD_L_Yaw_angle,0);
   speed_protect_1(&AScontrol_val);
  return (int)AScontrol_val*150;//120
}
//------------------------------------------------------------------------------------
//尝试用陀螺仪制定直线,妈耶，效果真好
//串级位置速度控制代码
int location_time=0;

void control_straight()
{ 
  aim_angle=calculate_angle(ST_car1.x,ST_car1.y,TGT[nowTGT].x,TGT[nowTGT].y);
  TO_STR=controlAS_pwm_STR();
  float vx;
  float vy;
  /*if (location_time++%10){*/
     error=sqrt((TGT[nowTGT].x-ST_car1.x)*(TGT[nowTGT].x-ST_car1.x)+(TGT[nowTGT].y-ST_car1.y)*(TGT[nowTGT].y-ST_car1.y))/wheel_diam/PI/30*1024*70;
    control_val=location_pid_realize(&Location_pid,error);//编码器值换成距离,单位cm，并在1s到达
    speed_protect(&control_val);
    vx=cos(aim_angle/180.0*PI)*control_val;
    vy=sin(aim_angle/180.0*PI)*control_val;
    set_pid_target(&Speed_pid[0], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))+(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))-TO_STR/100.0);//motor1
    set_pid_target(&Speed_pid[1], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))-(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))+TO_STR/100.0);//设定速度pid的目标值
    set_pid_target(&Speed_pid[2], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))-(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))-TO_STR/100.0);
    set_pid_target(&Speed_pid[3], (vy*cos(AD_L_Yaw_angle/180.0*PI)-vx*sin(AD_L_Yaw_angle/180.0*PI))+(-vy*sin(AD_L_Yaw_angle/180.0*PI)-vx*cos(AD_L_Yaw_angle/180.0*PI))+TO_STR/100.0);
  
 // 实际速度
  actual_speed[0]=(float)(SI.varLB[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI); //cm/s*/
  actual_speed[1]=(float)(SI.varRB[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);
  actual_speed[2]=(float)(SI.varLT[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);
  actual_speed[3]=(float)(SI.varRT[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);

  res_pwm[0]=(int)(calc_vPID(&Speed_pid[0],actual_speed[0]));//如果p=1,那么满速度也是3000
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
    control_val=location_pid_realize(&Location_pid,sqrt(SI.encodernow[1]*SI.encodernow[1]+SI.encodernow[2]*SI.encodernow[2]));//编码器值换成距离,单位cm，并在1s到达
    speed_protect(&control_val);
    vx=cos(aim_angle/180.0*PI)*control_val;
    vy=sin(aim_angle/180.0*PI)*control_val;

  }*/
 // 实际速度 
  set_pid_target(&Speed_pid[0], -TO_STR/100.0);//motor1
  set_pid_target(&Speed_pid[1], +TO_STR/100.0);//设定速度pid的目标值
  set_pid_target(&Speed_pid[2], -TO_STR/100.0);
  set_pid_target(&Speed_pid[3], +TO_STR/100.0);
  actual_speed[0]=(float)(SI.varLB[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI); //cm/s*/
  actual_speed[1]=(float)(SI.varRB[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);
  actual_speed[2]=(float)(SI.varLT[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);
  actual_speed[3]=(float)(SI.varRT[0]/encoder_period/1024.0*30/70.0*wheel_diam*PI);

  res_pwm[0]=(int)(Incremental_PID_realize(&Speed_pid[0],actual_speed[0]));//如果p=1,那么满速度也是3000
  res_pwm[1]=(int)(Incremental_PID_realize(&Speed_pid[1],actual_speed[1]));
  res_pwm[2]=(int)(Incremental_PID_realize(&Speed_pid[2],actual_speed[2]));
  res_pwm[3]=(int)(Incremental_PID_realize(&Speed_pid[3],actual_speed[3]));
  pwm_protect_pro(res_pwm);

  move_set(res_pwm[0],res_pwm[1],res_pwm[2],res_pwm[3]);
  //move_set(-TO_STR,TO_STR,-TO_STR,TO_STR);
}


//--------------------------------------------------
//位置微调pid
//------------------------------------------
#define SMALL_INTEGRAL_START_ERR 10 /*积分分离时对应的误差范围*/
#define SMALL_INTEGRAL_MAX_VAL 260  /*积分范围限定，防止积分饱和*/
float SMALL_pid_realize(PID volatile *pid, float target_val ,float actual_val,float SMALL_CHANGE_ZONE)
{
/*计算目标值与实际值的误差*/
pid->err = (target_val - actual_val);

/* 设定闭环死区 */
      if( (pid->err>-SMALL_CHANGE_ZONE) && (pid->err<SMALL_CHANGE_ZONE ) )
      {
      pid->err = 0;
      pid->integral = 0;
      pid->err_last = 0;
        }
      

/*积分项，积分分离，偏差较大时去掉积分作用*/
  if(pid->err > -SMALL_INTEGRAL_START_ERR && pid->err < SMALL_INTEGRAL_START_ERR)
    {
    pid->integral += pid->err;
      /*积分范围限定，防止积分饱和*/
        if(pid->integral > SMALL_INTEGRAL_MAX_VAL)
          {
              pid->integral = SMALL_INTEGRAL_MAX_VAL;
                    }
         else if(pid->integral < -SMALL_INTEGRAL_MAX_VAL)
          {
                pid->integral = -SMALL_INTEGRAL_MAX_VAL;
              }
        }

/*PID算法实现*/
      pid->output_val = pid->Kp * pid->err +
                  pid->Ki*1.25 * pid->integral +
                  pid->Kd *(pid->err - pid->err_last);

/*误差传递*/
      pid->err_last = pid->err;

/*返回当前实际值*/
      return pid->output_val;
}
//前进后退的
#define SMALL_CHANGE_ZONE_1 5.0f /*拟合环死区*/
#define SMALL_INTEGRAL_START_ERR_1 20 /*积分分离时对应的误差范围*/
#define SMALL_INTEGRAL_MAX_VAL_1 260  /*积分范围限定，防止积分饱和*/
float SMALL_pid_realize_1(PID volatile *pid, float actual_val)
{
/*计算目标值与实际值的误差*/
pid->err = (pid->target_val_1 - actual_val);

/* 设定闭环死区 */
      if( (pid->err>-SMALL_CHANGE_ZONE_1) && (pid->err<SMALL_CHANGE_ZONE_1 ) )
      {
      pid->err = 0;
      pid->integral = 0;
      pid->err_last = 0;
        }

/*积分项，积分分离，偏差较大时去掉积分作用*/
  if(pid->err > -SMALL_INTEGRAL_START_ERR_1 && pid->err < SMALL_INTEGRAL_START_ERR_1)
    {
    pid->integral += pid->err;
      /*积分范围限定，防止积分饱和*/
        if(pid->integral > SMALL_INTEGRAL_MAX_VAL_1)
          {
              pid->integral = SMALL_INTEGRAL_MAX_VAL_1;
                    }
         else if(pid->integral < -SMALL_INTEGRAL_MAX_VAL_1)
          {
                pid->integral = -SMALL_INTEGRAL_MAX_VAL_1;
              }
        }

/*PID算法实现*/
      pid->output_val = pid->Kp * pid->err +
                  pid->Ki *0.1* pid->integral +
                  pid->Kd *(pid->err - pid->err_last);

/*误差传递*/
      pid->err_last = pid->err;

/*返回当前实际值*/
      return pid->output_val;
}
#define SMALL_CHANGE_ZONE_2 10.0f /*转向环死区*/
#define SMALL_INTEGRAL_START_ERR_2 100 /*积分分离时对应的误差范围*/
#define SMALL_INTEGRAL_MAX_VAL_2 260  /*积分范围限定，防止积分饱和*/
float SMALL_pid_realize_2(PID volatile *pid, float actual_val)
{
/*计算目标值与实际值的误差*/
pid->err = (pid->target_val_2 - actual_val);

/* 设定闭环死区 */
      if( (pid->err>-SMALL_CHANGE_ZONE_2) && (pid->err<SMALL_CHANGE_ZONE_2 ) )
      {
      pid->err = 0;
      pid->integral = 0;
      pid->err_last = 0;
        }

/*积分项，积分分离，偏差较大时去掉积分作用*/
  if(pid->err > -SMALL_INTEGRAL_START_ERR_2 && pid->err < SMALL_INTEGRAL_START_ERR_2)
    {
    pid->integral += pid->err;
      /*积分范围限定，防止积分饱和*/
        if(pid->integral > SMALL_INTEGRAL_MAX_VAL_2)
          {
              pid->integral = SMALL_INTEGRAL_MAX_VAL_2;
                    }
         else if(pid->integral < -SMALL_INTEGRAL_MAX_VAL_2)
          {
                pid->integral = -SMALL_INTEGRAL_MAX_VAL_2;
              }
        }

/*PID算法实现*/
      pid->output_val = pid->Kp * pid->err +
                  pid->Ki * pid->integral +
                  pid->Kd *(pid->err - pid->err_last);

/*误差传递*/
      pid->err_last = pid->err;

/*返回当前实际值*/
      return pid->output_val;
}
//----------------------------------------------------------------------------------
//并级pid控制小车距离微调
float small_pwm=0;
float small_pwm_1=0;
int str_control=0;
//其实比赛的时候可以直接控制位置，然后再平移之类的
void small_control()
{  
   AScontrol_val=0;
   if (last_point[0]<20||first_point[0]>160&&first_point[0]!=0&& last_point[0]!=0)
   {
     if (last_point[0]<20)
     {
       small_pwm=SMALL_pid_realize(&SMALL_pid,40 ,last_point[0],5.0);//最后一个点小于20就向左平移
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
      AScontrol_val=SMALL_pid_realize_2(&SMALL_pid,kl1);//目标值是0
      speed_protect_1(&AScontrol_val);
      str_control=(int)AScontrol_val*200;
      move_set(str_control,-str_control,0,0);
      
    }
    else 
    {*/
      if(first_point[0]!=0&& last_point[0]!=0)
      {
        small_pwm_1 =SMALL_pid_realize_1(&SMALL_pid,small_distance);//目标值是95，small_distance=(first_point[1]+last_point[1])/2;
        speed_protect_1(&small_pwm_1);//前进后退
        
      }
      else
        small_pwm_1 =0;
     
       small_pwm=SMALL_pid_realize(&SMALL_pid, 0,small_actual_val,25);//目标值是0，small_actual_val=kl1-kl2;调大死区试试，反正都能识别的
       speed_protect_1(&small_pwm);
    
     TO_STR=controlAS_pwm_STR();
     move_set((int)(-(small_pwm*200)-TO_STR+small_pwm_1*200),(int)((small_pwm*200)+TO_STR+small_pwm_1*200),(int)((small_pwm*200)-TO_STR+small_pwm_1*200),(int)(-(small_pwm*200)+TO_STR+small_pwm_1*200));
    }
   if (last_point[0]==0)
     move_behind();
   
   AScontrol_val=0;
}

