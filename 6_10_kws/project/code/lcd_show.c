#include "lcd_show.h"
#include "math.h"

extern uint8   uart4_receive[65];
extern EulerAngleTypedef angle;
extern float AD_L_Yaw_angle;
extern site ST_car1;
extern site ST_car2;
extern Speed_Info SI;
extern int32 motor1,motor2,motor3,motor4;
extern float error;
extern int TO_STR;
extern target TGT[44];
extern int nowTGT;
int test=0;
float test_x=0,test_y=0;
extern int res_pwm[4];
extern PID Speed_pid[4];
extern float datx;
extern float daty;

void show_Speed_Info(){
  tft180_set_color( RGB565_BLUE, RGB565_WHITE);
  tft180_show_string(0, 1*16,"varLB");  
  tft180_show_float(70, 1*16,SI.varLB[0],5,2);
  tft180_show_string(0, 2*16, "varLT");  
  tft180_show_float(70, 2*16,SI.varLT[0],5,2);
  tft180_show_string(0, 3*16, "varRB");  
  tft180_show_float(70, 3*16,SI.varRB[0],5,2);
  tft180_show_string(0, 4*16, "varRT");  
  tft180_show_float(70, 4*16,SI.varRT[0],5,2);
  tft180_show_string(0, 5*16, "car1x");
  tft180_show_float(70, 5*16,ST_car1.x,3,4);
  tft180_show_string(0, 6*16, "car1y");
  tft180_show_float(70, 6*16,ST_car1.y,3,4);  
    tft180_show_int(0,7*16,(int)datx,4); 
    tft180_show_int(70,7*16,(int)daty,4); 
    tft180_show_string(0, 0*16, "Yaw:");
  tft180_show_float(70,0*16,angle.Yaw,3,1); 
}

extern float aim_angle;


void show_attitude_site(){
  //tft180_set_color( RGB565_BLUE, RGB565_WHITE);
  tft180_show_float(70, 0*16 ,error,3,4); 
  tft180_show_string(0, 0*16 ,"error"); 
    tft180_show_int(70,1*16,nowTGT,4);  
  tft180_show_string(0, 2*16,"car1x");
  tft180_show_float(70, 2*16,ST_car1.x,3,4);
  tft180_show_string(0, 3*16, "car1y");
  tft180_show_float(70, 3*16 ,ST_car1.y,3,4);
    tft180_show_int(0,4*16,res_pwm[0],5);
    tft180_show_int(70,4*16,res_pwm[1],5);
  tft180_show_int(0,5*16,res_pwm[2],5);
  tft180_show_int(70,5*16,res_pwm[3],5);
  tft180_show_string(0, 6*16, "angle:");
  tft180_show_float(70,6*16,aim_angle,3,1); 
  tft180_show_string(0, 7*16, "ADLYaw");
  tft180_show_float(70,7*16,AD_L_Yaw_angle,3,1);

}


void show_pid_info(){
  tft180_set_color( RGB565_BLUE, RGB565_WHITE);
  tft180_show_float(70, 0*16 ,Speed_pid[0].Kp,4,4); 
  tft180_show_string(0, 0*16 ,"P"); 
  tft180_show_string(0, 2*16,"I");
  tft180_show_float(70, 2*16,Speed_pid[0].Ki,4,4);
  tft180_show_string(0, 3*16, "D");
  tft180_show_float(70, 3*16 ,Speed_pid[0].Kd,4,4);
    tft180_show_int(0,4*16,res_pwm[0],5);
    tft180_show_int(70,4*16,res_pwm[1],5);
  tft180_show_int(0,5*16,res_pwm[2],5);
  tft180_show_int(70,5*16,res_pwm[3],5);
  tft180_show_string(0, 6*16, "Yaw:");
  tft180_show_float(70,6*16,angle.Yaw,3,1); 
  tft180_show_string(0, 7*16, "ADLYaw");
  tft180_show_float(70,7*16,AD_L_Yaw_angle,3,1);

}
void show_angle()
{
  //float g=45.0/180.0*3.1415926;
  //test_x=cos(g);
  //test_y=sin(wheel_angle/180.0*PI);
   //lcd_showuint8(0, 1,BLUE, WHITE,test);  
    //test++;
  
    tft180_set_color( RGB565_BLUE, RGB565_WHITE);
    
    
    tft180_show_string(0, 2*16,"car1x");
    tft180_show_float(70, 2*16,ST_car1.x,3,4);
    tft180_show_string(0, 3*16, "car1y");
    tft180_show_float(70, 3*16,ST_car1.y,3,4);
    tft180_show_string(0, 2*16,"car2x");
    tft180_show_float(70, 2*16,ST_car2.x,3,4);
    tft180_show_string(0, 3*16, "car2y");
    tft180_show_float(70, 3*16,ST_car2.y,3,4);
    tft180_show_string(0, 4*16, "Yaw:");
    tft180_show_float(70,4*16,angle.Yaw,3,1); 
    tft180_show_string(0, 5*16, "ADLYawangle");
    tft180_show_float(70,5*16,AD_L_Yaw_angle,3,1);
    //lcd_showstr(0, 6,BLUE, WHITE, "x_now:");
    //lcd_showfloat(70,6,BLUE, WHITE,x_now/20,3,3);
    //lcd_showstr(0, 7,BLUE, WHITE, "y_now:");
    //lcd_showfloat(70,7,BLUE, WHITE,y_now/20,3,3);
}

void show_class(){
    tft180_show_string(0,0*16,"main_class");
    tft180_show_uint(80,0*16,TGT[nowTGT].main_class,4);
    tft180_show_string(0,1*16,"second_class");
    tft180_show_uint(80,1*16,TGT[nowTGT].second_class,4);
    tft180_show_string(0,2*16,"real_class");
    tft180_show_uint(80,2*16,TGT[nowTGT].real_class,4); 
    tft180_show_string(0,3*16,"carsitex");
    tft180_show_float(80,3*16,ST_car1.x,2,1);
    tft180_show_string(0,4*16,"carsitey");
    tft180_show_float(80,4*16,ST_car1.y,2,1);
    tft180_show_string(0,5*16,"x");
    tft180_show_uint(80,5*16,(int)TGT[nowTGT].x/20,4); 
    tft180_show_string(0,6*16,"y");
    tft180_show_uint(80,6*16,(int)TGT[nowTGT].y/20,4); 
}
extern int total_TGT;
void show_TGT_site(){
    tft180_set_color( RGB565_BLUE, RGB565_WHITE);
    tft180_show_float(0,0*16,TGT[0].x,3,1);
    tft180_show_float(60,0*16,TGT[0].y,3,1);
    tft180_show_float(110,0*16,TGT[1].x,3,1);
    tft180_show_float(0,1*16,TGT[1].y,3,1);
    tft180_show_float(60,1*16,TGT[2].x,3,1);
    /*tft180_show_float(100,1*16,TGT[2].y,3,0);
    tft180_show_float(20,2*16,TGT[3].x,3,0);
    tft180_show_float(60,2*16,TGT[3].y,3,0);
    tft180_show_float(100,2*16,TGT[4].x,3,0);
    tft180_show_float(20,3*16,TGT[4].y,3,0);
    tft180_show_float(60,3*16,TGT[5].x,3,0);
    tft180_show_float(100,3*16,TGT[5].y,3,0);
    tft180_show_float(20,4*16,TGT[6].x,3,0);
    tft180_show_float(60,4*16,TGT[6].y,3,0);
    tft180_show_float(100,4*16,TGT[7].x,3,0);
    tft180_show_float(20,5*16,TGT[7].y,3,0);
    tft180_show_float(60,5*16,TGT[8].x,3,0);
    tft180_show_float(100,5*16,TGT[8].y,3,0);
    tft180_show_float(20,6*16,TGT[9].x,3,0);
    tft180_show_float(60,6*16,TGT[9].y,3,0);
    tft180_show_float(100,6*16,TGT[10].x,3,0);
    tft180_show_float(20,7*16,TGT[10].y,3,0); */
    tft180_show_uint(60,2*16,total_TGT,4); 


}

void show_path(){
  send_TGT_Upper_computer();
  tft180_full(RGB565_WHITE);
  tft180_show_uint(60,2*16,total_TGT,4); 
  tft180_draw_line((int)700/5,127,(int)700/5,127-500/5,RGB565_RED);
  tft180_draw_line(0,127-500/5,(int)700/5,127-500/5,RGB565_RED);   
  for(int i=nowTGT;i<total_TGT;i++){
    tft180_draw_line((int)TGT[i].x/5,127-(int)TGT[i].y/5,(int)TGT[i+1].x/5,127-(int)TGT[i+1].y/5,RGB565_RED);
  }

}


void show_orginal_site()
{/*
      lcd_showuint8(20,0,BLUE, WHITE,uart4_receive[0]);
       lcd_showuint8(60,0,BLUE, WHITE,uart4_receive[1]);
      lcd_showuint8(100,0,BLUE, WHITE,uart4_receive[2]);
       lcd_showuint8(20,1,BLUE, WHITE,uart4_receive[3]);
       lcd_showuint8(60,1,BLUE, WHITE,uart4_receive[4]);
      lcd_showuint8(100,1,BLUE, WHITE,uart4_receive[5]);
       lcd_showuint8(20,2,BLUE, WHITE,uart4_receive[6]);
       lcd_showuint8(60,2,BLUE, WHITE,uart4_receive[7]);
      lcd_showuint8(100,2,BLUE, WHITE,uart4_receive[8]);
       lcd_showuint8(20,3,BLUE, WHITE,uart4_receive[9]);
       lcd_showuint8(60,3,BLUE, WHITE,uart4_receive[10]);
      lcd_showuint8(100,3,BLUE, WHITE,uart4_receive[11]);
       lcd_showuint8(20,4,BLUE, WHITE,uart4_receive[12]);
       lcd_showuint8(60,4,BLUE, WHITE,uart4_receive[13]);
      lcd_showuint8(100,4,BLUE, WHITE,uart4_receive[14]);
       lcd_showuint8(20,5,BLUE, WHITE,uart4_receive[15]);
       lcd_showuint8(60,5,BLUE, WHITE,uart4_receive[16]);
      lcd_showuint8(100,5,BLUE, WHITE,uart4_receive[17]);
       lcd_showuint8(20,6,BLUE, WHITE,uart4_receive[18]);
       lcd_showuint8(60,6,BLUE, WHITE,uart4_receive[19]);
       lcd_showuint8(100,6,BLUE, WHITE,uart4_receive[20]);
       lcd_showuint8(20,7,BLUE, WHITE,uart4_receive[21]);
       lcd_showuint8(60,7,BLUE, WHITE,uart4_receive[22]);
       lcd_showuint8(100,7,BLUE, WHITE,uart4_receive[23]);*/
}


extern uint8 Binary_map[IMG_H][IMG_W];//先行后列
extern uint8 Blur_image[IMG_H][IMG_W];
extern uint8 Border_image[IMG_H][IMG_W];
extern uint8 Binary_map_Inflation[IMG_H][IMG_W];
extern uint8 Binary_map_corrode[IMG_H][IMG_W];
void show_img(){
  tft180_set_color( RGB565_BLUE, RGB565_WHITE);
  //tft180_clear();
  //tft180_show_gray_image(0, 0, (const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H, MT9V03X_W/2, MT9V03X_H/2, (int)Threshold);
  
  tft180_show_gray_image(0, 0, Binary_map[0], IMG_W, IMG_H, IMG_W/2, IMG_H/2, 0);
  //tft180_show_gray_image(0, 0, Binary_map_Inflation[0], MT9V03X_W, MT9V03X_H, MT9V03X_W/2, MT9V03X_H/2, 0);
  //float pix_error;
  //pix_error=sqrt((blob_info[0].center_x-view_center_x)*(blob_info[0].center_x-view_center_x)+(blob_info[0].center_y-view_center_y)*(blob_info[0].center_y-view_center_y));
  //float k;
  show((int)blob_info[0].center_x,(int)blob_info[0].center_y,(int)blob_info[0].center_x,(int)blob_info[0].center_y,RGB565_RED,2);
  //k=calculate_angle(blob_info[0].center_x,blob_info[0].center_y,view_center_x,view_center_y);
  //tft180_show_float(0,7*16,pix_error,3,1);
  //tft180_show_float(60,7*16,k,3,1);
  //tft180_show_float(0,7*16,(float)line_info[0][0].dis,3,1); 
  //tft180_show_float(60,7*16,(float)line_info[1][0].dis,3,1);  
  tft180_show_float(0,6*16,blob_info[0].TGT_area,4,1);  
  tft180_show_float(0,7*16,(float)blob_info[0].delta_x,3,1); 
  tft180_show_float(60,7*16,(float)blob_info[0].delta_y,3,1); 
  //float x_drift=-(line_info[0][0].dis-view_center_x)*0.255;;
  //float y_drift=-(line_info[1][0].dis-view_center_y)*0.255;//(blob_info[0].center_y-60)*0.255;
  //tft180_show_float(0,5*16,(float)x_drift,3,1); 
  //tft180_show_float(60,5*16,(float)y_drift,3,1); 
}

extern int8 realMap[36][26];
void Show_path_Map_coverage(){
  int cnt = 0;
  tft180_show_gray_image(0,0,(uint8*)realMap[0],26,36,26*3,36*3,1);
  for(int i=1;i<=35;i++){
    for(int j=1;j<=25;j++){
      if(realMap[i][j]!=0) cnt++;
    }
  }
  tft180_show_float(80,0*16,cnt*100.0/(35.0*25.0),2,1);

}