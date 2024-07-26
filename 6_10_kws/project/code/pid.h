#ifndef _PID_H
#define _PID_H

#include "headfile.h"

#define SI speed_info
#define  Location_pid pid_location
#define  Location_pid_x pid_location_x
#define  Location_pid_y pid_location_y
#define  Speed_pid pid_speed 
#define  Angle_pid pid_angle 
#define  SMALL_pid pid_SMALL 
#define  SPEED_begin 30



typedef struct PID{
  float Kp;
  float Ki;
  float Kd;
  float err;//ƫ��
  float err_last;//��һ��ƫ��
  float err_prelast;//���ϴ�ƫ��
  float integral;//������
  float target_val;//Ŀ��ֵ
  float target_val_1;//�ڶ���Ŀ��ֵ
  float target_val_2;//������Ŀ��ֵ
  float output_val;//�����
  float UD;
  float UD_last;
}PID;



typedef struct T_speed_plan{
  /*�޶�����*/
  float t_begin;
  float l_begin;
  float v_begin;
  float t_end;
  float l_end;
  float v_end;
  float v_max;
  float a_max;//�������
  
  /*�������*/
  float t_acc;//����ʱ��
  float t_uv; //����ʱ��
  float t_dec;//����ʱ��
  float v_uv; //�����ٶ�
  float a_acc;//���ټ��ٶ�
  float a_dec;//���ټ��ٶ�
  float t_tal;//��ʱ��

}T_speed_plan;


void control_path_on_back_se(int speed_max_max);
void control_path_small();
void control_path();
void control_path_on_back(int speed_max);
float calculate_speed(T_speed_plan volatile *tsp,float t);
void time_limit(T_speed_plan volatile *tsp,float t);
void time_optimize(T_speed_plan volatile *tsp);
void set_tsp_target(T_speed_plan volatile *tspx,T_speed_plan volatile *tspy, float x,float y,float x1,float y1);
void straight_test_on_angle(void);
void clear_output_val(void);
void init_pid(void);
void init_T_speed_plan(void);
void set_pid_target(PID volatile *pid, float temp_val);
float Incremental_PID_realize(PID volatile *pid, float actual_val);
float calc_vPID(PID volatile *pid, float actual_val);
float location_pid_realize(PID volatile *pid, float error);
float angle_pid_realize(PID volatile *pid, float actual_val,int m);
void controlAS_pwm(void);
int controlAS_pwm_STR(void);
void control_straight();
float SMALL_pid_realize(PID volatile *pid, float target_val ,float actual_val,float SMALL_CHANGE_ZONE);
float SMALL_pid_realize_1(PID volatile *pid, float actual_val);
float SMALL_pid_realize_2(PID volatile *pid, float actual_val);
void small_control();
void border_extract();
#endif