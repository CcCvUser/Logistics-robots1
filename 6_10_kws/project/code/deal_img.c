#include "deal_img.h"
#include "math.h"
#include "LinkQueue.h" 

#define DEBUG_img

/**
开DEBUG_img显示图像后只有10帧，不开有24帧

试了openart上注释了屏幕显示一样能冲到20帧
还不和蚁群中断抢时间
所以还是放在art上更新error了

**/


float Threshold=50;
//例如访问第10行 50列的点，mt9v03x_csi_image[10][50]就可以了
uint8 Binary_map[IMG_H][IMG_W];//先行后列
uint8 Blur_image[IMG_H][IMG_W];
uint8 Border_image[IMG_H][IMG_W];
uint8 Binary_map_corrode[IMG_H][IMG_W];
uint8 Binary_map_Inflation[IMG_H][IMG_W];
uint8 pathMap[IMG_H][IMG_W]={0};
uint16 Hough_map[230][18];//霍夫变换量化

static int LabTable[1024];
static int GamaTable1[32];
static int GamaTable2[64];//B有6位，特殊对待

LINE_information line_info[2][2];
BLOB_information blob_info[50];
TGT_information tgt_info;

uint8 R;
uint8 G;
uint8 B;

int lab_color[3];

int LAB_Threshold[3][2]={{0, 65},{-128, 69},{-128, -10}};

int L_min;
int L_max;
int A_min;
int A_max;
int B_min;
int B_max;

int min_y=200;
int min_x=200;
int max_x=0;
int max_y=0;
extern PID SMALL_pid;


float Blur_filter[3][3] = {{0.0947416, 0.118318, 0.0947416}, 
                            {0.118318, 0.147761, 0.118318}, 
                           {0.0947416, 0.118318, 0.0947416 }};//Gaussain_Blur_filter
/*float Blur_filter[3][3] = {{1, 1, 1}, 
                            {1, 1,1}, 
                           {1, 1, 1 }};*/
int sobel_x[3][3]={{-1,0,1},
                  {-2,0,2},
                  {-1,0,1}};
int sobel_y[3][3]={{1,2,1},
                  {0,0,0},
                  {-1,-2,-1}};

int laplac[3][3]={{1,1,1},{1,-8,1},{1,1,1}};

float dealImageTime;
float dealTimeStarts;
float dealTimeEnds;


//zhn handsome
void Binary_deal(float T,uint8 original_img[][IMG_W],uint8 processed_img[][IMG_W]){  
  int i,j;
  for (i=0;i<IMG_H-1;i++){
    for (j=0;j<IMG_W-1;j++){   
      if (original_img[i][j]<T) processed_img[i][j]=0;//小于阈值是白色的，大于是黑色
      else processed_img[i][j]=255;                           //？谁想的小于阈值白色，改成黑色了
    }
  }
  for(i=IMG_H-1-img_cuty;i<IMG_H;i++){
    for(j=img_cutx_begin;j<img_cutx_end;j++){
    
      processed_img[i][j]=0; 
    
    }
  
  
  }
}

void CreateTable()//RGB565(字节交换)->lab空间建表
{
  for (int I = 0; I < 1024; I++)
  {
    if (I > 9)
      LabTable[I] = (int)(pow((float)I / 1024, 1.0F / 3.0F) * 1024 );
    else
      LabTable[I] = (int)(7.787F * I + 141.2 );
  }
  for (int J = 0; J < 32; J++)
  {
    float x = J/32.0F;
    x = x>0.04045?pow((x+0.055f)/1.055f,2.4f):x/12.92;
    GamaTable1[J] = (int)(x*1024);
  }
  for (int K = 0; K < 64; K++)
  {
    float y = K/64.0F;
    y = y>0.04045?pow((y+0.055f)/1.055f,2.4f):y/12.92;
    GamaTable2[K] = (int)(y*1024);
  }

}




void smooth_extract(uint8 original_img[][MT9V03X_W],uint8 processed_img[][IMG_W]){
  int i=0,j=0;
 
  for (i=1;i<IMG_H-1;i++){
    for (j=1;j<IMG_W-1;j++){ 
      processed_img[i][j] = (uint8)((original_img[i][j]*Blur_filter[1][1] + original_img[i-1][j-1]*Blur_filter[0][0]
                           + original_img[i-1][j]*Blur_filter[0][1] + original_img[i-1][j+1]*Blur_filter[0][2] + original_img[i][j-1]*Blur_filter[1][0]
                           + original_img[i][j+1]*Blur_filter[1][2] + original_img[i+1][j+1]*Blur_filter[2][2] + original_img[i+1][j]*Blur_filter[2][1]
                           + original_img[i+1][j-1]*Blur_filter[2][0]));
    }
  }
}

float temp;
void get_border(uint8 original_img[][IMG_W],uint8 processed_img[][IMG_W]){
  int i=0,j=0;

  for (i=2;i<IMG_H-2;i++){
    for (j=2;j<IMG_W-2;j++){ 
      temp = 4*((original_img[i][j]*laplac[1][1] + original_img[i-1][j-1]*laplac[0][0]
                           + original_img[i-1][j]*laplac[0][1] + original_img[i-1][j+1]*laplac[0][2] + original_img[i][j-1]*laplac[1][0]
                           + original_img[i][j+1]*laplac[1][2] + original_img[i+1][j+1]*laplac[2][2] + original_img[i+1][j]*laplac[2][1]
                           + original_img[i+1][j-1]*laplac[2][0]));
                          /*(uint8)((Blur_image[i][j]*sobel_x[1][1] + Blur_image[i-1][j-1]*sobel_x[0][0]
                           + Blur_image[i-1][j]*sobel_x[0][1] + Blur_image[i-1][j+1]*sobel_x[0][2] + Blur_image[i][j-1]*sobel_x[1][0]
                           + Blur_image[i][j+1]*sobel_x[1][2] + Blur_image[i+1][j+1]*sobel_x[2][2] + Blur_image[i+1][j]*sobel_x[2][1]
                           + Blur_image[i+1][j-1]*sobel_x[2][0])/2.0
                           +(Blur_image[i][j]*sobel_y[1][1] + Blur_image[i-1][j-1]*sobel_y[0][0]
                           + Blur_image[i-1][j]*sobel_y[0][1] + Blur_image[i-1][j+1]*sobel_y[0][2] + Blur_image[i][j-1]*sobel_y[1][0]
                           + Blur_image[i][j+1]*sobel_y[1][2] + Blur_image[i+1][j+1]*sobel_y[2][2] + Blur_image[i+1][j]*sobel_y[2][1]
                           + Blur_image[i+1][j-1]*sobel_y[2][0])/2.0);*/
    
      if(temp<0) temp=0;
      if(temp>255) temp=255;      
      processed_img[i][j]=(uint8)temp;
    }
  }
}

void corrode(uint8 original_img[][IMG_W],uint8 processed_img[][IMG_W]){
  int i=0,j=0;
  
  for (i=1;i<IMG_H-1;i++){
    for (j=1;j<IMG_W-1;j++){
      if(original_img[i][j]==255){
        if((original_img[i+1][j]+original_img[i][j+1]+original_img[i-1][j]+original_img[i][j-1]+original_img[i][j-1]+original_img[i-1][j-1]+original_img[i-1][j+1]+original_img[i+1][j-1]+original_img[i+1][j+1])<=255*6){//4
          processed_img[i][j]=0;
        } 
        else {
          processed_img[i][j]=255;
        }
      }
    }
  }
}

void DoubleInflation(uint8 original_img[][IMG_W],uint8 processed_img[][IMG_W]){
  int i=0,j=0;
  
  for (i=2;i<IMG_H-2;i++){
    for (j=2;j<IMG_W-2;j++){
      if(original_img[i][j]==255){
        processed_img[i][j]=255;
        continue;
      }
      if(original_img[i+1][j]==255||original_img[i][j+1]==255||original_img[i-1][j]==255||original_img[i][j-1]==255||original_img[i-1][j-1]==255||original_img[i-1][j+1]==255||original_img[i+1][j-1]==255||original_img[i+1][j+1]==255
      ||(original_img[i+2][j]+original_img[i+2][j-2]+original_img[i+2][j-1]+original_img[i+2][j+1]+original_img[i+2][j+2])>=2*255
      ||(original_img[i+2][j]+original_img[i+2][j-2]+original_img[i+2][j-1]+original_img[i+2][j+1]+original_img[i+2][j+2])>=2*255
      ||(original_img[i][j+2]+original_img[i-2][j+2]+original_img[i-1][j+2]+original_img[i+1][j+2]+original_img[i+2][j+2])>=2*255
      ||(original_img[i][j-2]+original_img[i-2][j-2]+original_img[i-1][j-2]+original_img[i+1][j-2]+original_img[i+2][j-2])>=2*255){
        processed_img[i][j]=255;
      } 
    }
  }
}

/*void DoubleInflation(){
  int i=0,j=0;
  memset(Binary_map_Inflation,0,sizeof(Binary_map_Inflation)); 
  for (i=3;i<MT9V03X_CSI_H-3;i++){
    for (j=3;j<MT9V03X_CSI_W-3;j++){
      if(Binary_map[i][j]==255){
        Binary_map_Inflation[i][j]=255;
        continue;
      }
      if(Binary_map[i+1][j]==255||Binary_map[i][j+1]==255||Binary_map[i-1][j]==255||Binary_map[i][j-1]==255){
        Binary_map_Inflation[i][j]=255;
      } 
    }
  }
}*/

void draw_line(int angle,int dis,const uint16 color,int x,int y){
  
  tft180_draw_line(x,0,x,dis,color);
  
/*int begin_x=(int)dis/cos(angle*PI/180);
  int begin_y=(int)dis/sin(angle*PI/180);
  
  tft180_draw_line(begin_x+x,128-y,0,begin_y+128-y,color);*/
}

void draw_point(int x,int y,const uint16 color){
  if(x<1) x=1;
  if(y<1) y=1;
  if(x>=160-1) x=160-2;
  if(y>=128-1) y=128-2;

  tft180_draw_point(x,y,color);  
  tft180_draw_point(x+1,y,color);
  tft180_draw_point(x-1,y,color);
  tft180_draw_point(x,y+1,color);
  tft180_draw_point(x,y-1,color);
  tft180_draw_point(x+1,y+1,color);
  tft180_draw_point(x+1,y-1,color);
  tft180_draw_point(x-1,y+1,color);
  tft180_draw_point(x-1,y-1,color);
}

void show(int max_x,int max_y,int min_x,int min_y,const uint16 color,int zoom){
  draw_point(max_x/zoom,max_y/zoom,color);
  draw_point(max_x/zoom,min_y/zoom,color);
  draw_point(min_x/zoom,min_y/zoom,color);
  draw_point(min_x/zoom,max_y/zoom,color);  
}

void update_pathmap(int max_x,int max_y,int min_x,int min_y){
  int i;
  for(i=min_x;i<=max_x;i++){
    pathMap[max_y][i]=min_y;
  }
}

void choose_line(int angle,int dis,int T,int k){
  int j=angle/10;
  if(Hough_map[dis][j]>T&&Hough_map[dis][j]>line_info[k][0].dis&&Hough_map[dis][j]>line_info[k][1].dis){
    line_info[k][1].dis=line_info[k][0].dis;
    line_info[k][1].angle=line_info[k][0].angle;          
    
    line_info[k][0].dis=dis;
    line_info[k][0].angle=angle;
  }
  else if(Hough_map[dis][j]>T&&Hough_map[dis][j]<line_info[k][0].dis&&Hough_map[dis][j]>line_info[k][1].dis){
    line_info[k][1].dis=dis;
    line_info[k][1].angle=angle;        
  
  }
  else if(Hough_map[dis][j]>T&&line_info[k][0].dis==0){
    line_info[k][0].dis=dis;
    line_info[k][0].angle=angle;          
  }
}

void Hough_transform(int T){
  memset(Hough_map,0,sizeof(Hough_map));
  memset(line_info,0,sizeof(line_info));
  int i,j,k;
    for (j=IMG_W-2;j>=1;j--){//x
      for (i=IMG_H-2;i>=1;i--){//y
        if(Binary_map[i][j]==255){
          for(k=0;k<18;k++){
            
#ifdef DEBUG_img
            if((int)(j*cos(k*10.0/180.0*PI)+(120-i)*sin(k*10.0/180.0*PI))>=229&&(int)(j*cos(k*10.0/180.0*PI)+(120-i)*sin(k*10.0/180.0*PI))<0 ) {
              BEEP(2000);
              return;
            }    
#endif
       
            Hough_map[(int)(j*cos(k*10.0/180.0*PI)+(120-i)*sin(k*10.0/180.0*PI))][k]++;
          }
        }
      }
    }

      for(i=0;i<230;i++){
        choose_line(0,i,T,0);
        choose_line(10,i,T,0);
        choose_line(20,i,T,0);
        choose_line(160,i,T,0);
        choose_line(170,i,T,0);  
        
        choose_line(90,i,T,1);
        choose_line(80,i,T,1);
        choose_line(70,i,T,1);
        choose_line(100,i,T,1);
        choose_line(110,i,T,1);  
      
      }
#ifdef DEBUG_img
    if(line_info[1][0].dis!=0){
      tft180_draw_line(40,0,40,60-line_info[1][0].dis/2,RGB565_GREEN);
    
    }
    if(line_info[1][1].dis!=0){
      tft180_draw_line(20,0,20,60-line_info[1][1].dis/2,RGB565_GREEN);    
    }
      
      tft180_draw_line(0,40,line_info[0][0].dis/2,40,RGB565_YELLOW);
      tft180_draw_line(0,20,line_info[0][1].dis/2,20,RGB565_YELLOW); 
#endif 
}


void get_line(int T){
  if(img_ID==0){
    memset(Blur_image,0,sizeof(Blur_image));  
    memset(Border_image,0,sizeof(Border_image));
    memset(Binary_map,0,sizeof(Binary_map));
    
    smooth_extract(mt9v03x_image,Blur_image);
    get_border(Blur_image,Border_image);
    Binary_deal(T,Border_image,Binary_map);
  }
  else if(img_ID==1){
    memset(Border_image,0,sizeof(Border_image));
    memset(Binary_map,0,sizeof(Binary_map));
    memset(Binary_map_corrode,0,sizeof(Binary_map_corrode));
    
    RGB565_LAB_Binary_deal(scc8660_image,Binary_map);
    corrode(Binary_map,Binary_map_corrode);
    get_border(Binary_map_corrode,Border_image);
    memset(Binary_map,0,sizeof(Binary_map));
    Binary_deal(T,Border_image,Binary_map);
  }
  Hough_transform(35);
}


void UpdateTGTinfo(int flag){

    tgt_info.lastdt = tgt_info.dt;
    tgt_info.dt = timer_get(GPT_TIM_1);
    timer_stop(GPT_TIM_1);
    timer_clear(GPT_TIM_1);
    timer_start(GPT_TIM_1);
    
#ifdef DEBUG_img
    tft180_show_string(0,0*16,"fps:");
    tft180_show_float(80,0*16,1000000/tgt_info.dt,2,1);
#endif
    
  if(flag){
    if(blob_info[0].center_x == 0 && blob_info[0].center_y == 0){
        memset(&tgt_info, 0, sizeof(&tgt_info));
        return;
    }else{
        tgt_info.preLastX = tgt_info.lastX;
        tgt_info.preLastY = tgt_info.lastY;
        tgt_info.lastX = tgt_info.nowX;
        tgt_info.lastY = tgt_info.nowY;
        tgt_info.nowX = blob_info[0].center_x;
        tgt_info.nowY = blob_info[0].center_y;
        if(tgt_info.dt == 0 || (tgt_info.lastX == 0 && tgt_info.lastY == 0)){
            tgt_info.dxdt = 0;
            tgt_info.dydt = 0;
        }else{
            tgt_info.dxdt = (tgt_info.nowX - tgt_info.lastX) / tgt_info.dt;
            tgt_info.dydt = (tgt_info.nowY - tgt_info.lastY) / tgt_info.dt;
        }
        if(tgt_info.lastdt == 0 || tgt_info.dt == 0 || (tgt_info.preLastX == 0 && tgt_info.preLastY == 0) || (tgt_info.lastX == 0 && tgt_info.lastY == 0)){
            tgt_info.ddxdtt = 0;
            tgt_info.ddydtt = 0;
        }else{
            tgt_info.ddxdtt = ((tgt_info.nowX - tgt_info.lastX) / tgt_info.dt - (tgt_info.lastX - tgt_info.preLastX) / tgt_info.lastdt)/(0.5*(tgt_info.dt+tgt_info.lastdt));
            tgt_info.ddydtt = ((tgt_info.nowY - tgt_info.lastY) / tgt_info.dt - (tgt_info.lastY - tgt_info.preLastY) / tgt_info.lastdt)/(0.5*(tgt_info.dt+tgt_info.lastdt));
        }
        tgt_info.nextX = tgt_info.nowX + tgt_info.dxdt * dealImageTime + tgt_info.ddxdtt * dealImageTime * dealImageTime * 0.5;
        tgt_info.nextY = tgt_info.nowY + tgt_info.dydt * dealImageTime + tgt_info.ddydtt * dealImageTime * dealImageTime * 0.5;
        tgt_info.errorX = -(tgt_info.nextX - view_center_x);
        tgt_info.errorY = tgt_info.nextY - view_center_y;
    }
  }
  else{
    tgt_info.errorX = -(blob_info[0].center_x - view_center_x);
    tgt_info.errorY = blob_info[0].center_y - view_center_y;
  
  }
  
#ifdef DEBUG_img
    draw_point(tgt_info.nextX/2,tgt_info.nowY/2,RGB565_GREEN); 
#endif
  
}


extern float car_angle;
extern float zap_angle;



void get_blobs(int T){
  
  if(img_ID==0){
    memset(Binary_map,0,sizeof(Binary_map));
    memset(Blur_image,0,sizeof(Blur_image));  
    memset(Border_image,0,sizeof(Border_image)); 
    memset(Binary_map_Inflation,0,sizeof(Binary_map_Inflation));   
    memset(Binary_map_corrode,0,sizeof(Binary_map_corrode));
    memset(blob_info,0,sizeof(blob_info));  
    memset(pathMap,0,sizeof(pathMap));
    
    smooth_extract(mt9v03x_image,Blur_image);
    get_border(Blur_image,Border_image);
    Binary_deal(T,Border_image,Binary_map);
    corrode(Binary_map,Binary_map_corrode);
    DoubleInflation(Binary_map_corrode,Binary_map_Inflation);
  }
  
  else if(img_ID==1){ 
    memset(Binary_map,0,sizeof(Binary_map));
    memset(Binary_map_Inflation,0,sizeof(Binary_map_Inflation));   
    memset(Binary_map_corrode,0,sizeof(Binary_map_corrode));
    memset(blob_info,0,sizeof(blob_info));  
    memset(pathMap,0,sizeof(pathMap));
    
    RGB565_LAB_Binary_deal(scc8660_image,Binary_map);
    corrode(Binary_map,Binary_map_corrode);
    DoubleInflation(Binary_map_corrode,Binary_map_Inflation);
  }  
  
#ifdef DEBUG_img
    tft180_show_gray_image(0, 0, Binary_map_Inflation[0], IMG_W, IMG_H, IMG_W/2, IMG_H/2, 0);
#endif 
  
  
  int i,j;
  int k=0;
  int area;
  for (j=IMG_W-2;j>=1;j--){
    for (i=IMG_H-2;i>=1;i--){
      if (Binary_map_Inflation[i][j]==255&&pathMap[i][j]==0){
        max_x=j;
        max_y=i;
        min_x=j;
        min_y=i;
        BFSTraverse(j,i,&Binary_map_Inflation[0][0],&pathMap[0][0],IMG_W,IMG_H,&max_x,&max_y,&min_x,&min_y,&area);
        if((max_x-min_x)*(max_y-min_y)>50){
          update_pathmap(max_x,max_y,min_x,min_y);
          k++;
          if(k>=49){
            BEEP(2000);
            return;
          }
          if((max_y<=IMG_H-5&&min_y>=4&&max_x<=IMG_W-5&&min_x>=4)){
            blob_info[k].center_x=(max_x+min_x)/2.0;
            blob_info[k].center_y=(max_y+min_y)/2.0;
            blob_info[k].TGT_area=(max_y-min_y)*(max_x-min_x);
            blob_info[k].delta_x=(max_x-min_x);
            blob_info[k].delta_y=(max_y-min_y);
            blob_info[k].flag=1;
            blob_info[k].k=calculate_angle(94,60,blob_info[k].center_x,(120-blob_info[k].center_y));
#ifdef DEBUG_img
              show(max_x,max_y,min_x,min_y,RGB565_YELLOW,2);
#endif
          }
          else if(max_x>IMG_W-5&&min_x<=IMG_W-5&&max_y<=IMG_H-5&&min_y>=4){
            if((max_x-min_x)>50) continue;
            blob_info[k].center_x=(min_x+(max_y-min_y)/2.0);
            blob_info[k].center_y=(max_y+min_y)/2.0;
            blob_info[k].TGT_area=(max_y-min_y)*(max_y-min_y);
            blob_info[k].delta_x=(max_x-min_x);
            blob_info[k].delta_y=(max_y-min_y);
            blob_info[k].flag=0;
#ifdef DEBUG_img
              show(max_x,max_y,min_x,min_y,RGB565_BROWN,2);
#endif
          }
          else if(min_x<4&&max_x>=4&&max_y<=IMG_H-5&&min_y>=4){
            if((max_x-min_x)>50) continue;
            blob_info[k].center_x=(max_x-(max_y-min_y)/2.0);
            blob_info[k].center_y=(max_y+min_y)/2.0;
            blob_info[k].TGT_area=(max_y-min_y)*(max_y-min_y);
            blob_info[k].delta_x=(max_x-min_x);
            blob_info[k].delta_y=(max_y-min_y);
            blob_info[k].flag=0;
            blob_info[k].k=calculate_angle(94,60,blob_info[k].center_x,(120-blob_info[k].center_y));
#ifdef DEBUG_img
              show(max_x,max_y,min_x,min_y,RGB565_BROWN,2);
#endif
          }
          else if(min_y<4&&max_y>=4&&max_x<=IMG_W-5&&min_x>=4){
            if((max_y-min_y)>50) continue;
            blob_info[k].center_x=(max_x+min_x)/2.0;
            blob_info[k].center_y=max_y-(max_x-min_x)/2.0;
            blob_info[k].TGT_area=(max_x-min_x)*(max_x-min_x);
            blob_info[k].delta_x=(max_x-min_x);
            blob_info[k].delta_y=(max_y-min_y);
            blob_info[k].flag=0;
            blob_info[k].k=calculate_angle(94,60,blob_info[k].center_x,(120-blob_info[k].center_y));
#ifdef DEBUG_img
              show(max_x,max_y,min_x,min_y,RGB565_BROWN,2);
#endif
          }
          else if(max_y>IMG_H-5&&min_y<=IMG_H-5&&max_x<=IMG_W-5&&min_x>=4){
            if((max_y-min_y)>50) continue;
            blob_info[k].center_x=(max_x+min_x)/2.0;
            blob_info[k].center_y=min_y+(max_x-min_x)/2.0;
            blob_info[k].TGT_area=(max_x-min_x)*(max_x-min_x);
            blob_info[k].delta_x=(max_x-min_x);
            blob_info[k].delta_y=(max_y-min_y);
            blob_info[k].flag=0;
            blob_info[k].k=calculate_angle(94,60,blob_info[k].center_x,(120-blob_info[k].center_y));
#ifdef DEBUG_img
              show(max_x,max_y,min_x,min_y,RGB565_BROWN,2);
#endif
          }
          if(max_y>=IMG_H-2-img_cuty&&max_y<=IMG_H+3-img_cuty){
            //if((max_y-min_y)>60) continue;
            float t_c=(max_x+min_x)/2.0;
            if(t_c>105||t_c<55) continue; 
            blob_info[k].center_x=t_c;
            blob_info[k].center_y=min_y+(max_x-min_x)/2.0;
            blob_info[k].TGT_area=(max_x-min_x)*(max_x-min_x);
            blob_info[k].delta_x=(max_x-min_x);
            blob_info[k].delta_y=(max_y-min_y);
            blob_info[k].flag=-1;
            blob_info[k].k=calculate_angle(94,60,blob_info[k].center_x,(120-blob_info[k].center_y));
#ifdef DEBUG_img
              show(max_x,max_y,min_x,min_y,RGB565_BLUE,2);
#endif
          
          }
#ifdef DEBUG_img
          else {
            //  BEEP(200);
          }
#endif
        }
      }
      else if(pathMap[i][j]>1){
        i=pathMap[i][j];
      }
    }
  }  
  for(i=1;i<=k;i++){
    //if(blob_info[k].delta_x>70||blob_info[k].delta_y>70) continue;
    if(blob_info[i].TGT_area>3000||blob_info[i].TGT_area<1000) continue;
    if(blob_info[i].flag==1&&(blob_info[i].delta_x-blob_info[i].delta_y>30)&&(blob_info[i].delta_x-blob_info[i].delta_y<-30)) continue; 
    //if(blob_info[i].flag==-1&&(blob_info[i].center_x>125||blob_info[i].center_x<75)) continue; 
    if((abs_float(blob_info[i].center_x-view_center_x)>20||abs_float(blob_info[i].center_y-view_center_y)>20)&&abs_float(blob_info[i].k-zap_angle)>=90) continue;
    if(blob_info[i].flag>=blob_info[0].flag){//完整矩形优先级最高，其次面积大小
      if(blob_info[i].TGT_area>=blob_info[0].TGT_area||(0==blob_info[0].flag||-1==blob_info[0].flag)){
        //if(abs_float(blob_info[i].k-zap_angle)>=(abs_float(blob_info[0].k-zap_angle)+10)&&blob_info[0].k!=0) continue;
        blob_info[0].center_x=blob_info[i].center_x;
        blob_info[0].center_y=blob_info[i].center_y;
        blob_info[0].delta_x=blob_info[i].delta_x; 
        blob_info[0].delta_y=blob_info[i].delta_y; 
        blob_info[0].TGT_area=blob_info[i].TGT_area;
        blob_info[0].flag=blob_info[i].flag;
        blob_info[0].k=blob_info[i].k;
#ifdef DEBUG_img
        //BEEP(200);
#endif
      }
    }
    else if(0==blob_info[i].flag||-1==blob_info[i].flag){
      if(blob_info[i].TGT_area>blob_info[0].TGT_area){
        //if(abs_float(blob_info[i].k-zap_angle)>=(abs_float(blob_info[0].k-zap_angle)+10)&&blob_info[0].k!=0) continue;
        blob_info[0].center_x=blob_info[i].center_x;
        blob_info[0].center_y=blob_info[i].center_y;
        blob_info[0].delta_x=blob_info[i].delta_x; 
        blob_info[0].delta_y=blob_info[i].delta_y; 
        blob_info[0].TGT_area=blob_info[i].TGT_area;
        blob_info[0].flag=blob_info[i].flag;
#ifdef DEBUG_img
        //BEEP(200);
#endif
      }
    
    }
  }
#ifdef DEBUG_img
    show((int)blob_info[0].center_x,(int)blob_info[0].center_y,(int)blob_info[0].center_x,(int)blob_info[0].center_y,RGB565_RED,2);
    tft180_show_gray_image(0, IMG_H/2, pathMap[0], IMG_W, IMG_H, IMG_W/2, IMG_H/2, 3);
#endif

#ifdef DEBUG_img
    tft180_show_gray_image(0, IMG_H/2, Binary_map_Inflation[0], IMG_W, IMG_H, IMG_W/2, IMG_H/2, 0);
#endif
}

void RGB565_LAB_Binary_deal(uint16 original_img[][SCC8660_W],uint8 processed_img[][IMG_W]){
  for(int i=0;i<SCC8660_H;i++){
    for(int j=0;j<SCC8660_W;j++){
      R=(original_img[i][j]&0Xf800)>>11;
      G=(original_img[i][j]&0X07e0)>>5;
      B=(original_img[i][j]&0X001f);
      int x=(455026*GamaTable1[R]+394489*GamaTable2[G]+199046*GamaTable1[B])>>20;
      int y=(223002*GamaTable1[R]+749900*GamaTable2[G]+75675*GamaTable1[B])>>20;
      int z=(18619*GamaTable1[R]+114786*GamaTable2[G]+915097*GamaTable1[B])>>20;
      lab_color[0] = y > 9 ? (116 * LabTable[y] - 16384)>> 10: (903 * LabTable[y])>> 10;
      lab_color[1] = (500 * (LabTable[x] - LabTable[y]))>> 10;
      lab_color[2] = (200 * (LabTable[y] - LabTable[z]))>> 10;
      if((lab_color[0]<LAB_Threshold[0][1]&&lab_color[0]>LAB_Threshold[0][0])&&(lab_color[1]<LAB_Threshold[1][1]&&lab_color[1]>LAB_Threshold[1][0])&&(lab_color[2]<LAB_Threshold[2][1]&&lab_color[2]>LAB_Threshold[1][0])){
        processed_img[i][j]=0;
      }
      else processed_img[i][j]=255;
    }
  }

  for(int i=IMG_H-1-img_cuty;i<IMG_H;i++){
    for(int j=img_cutx_begin;j<img_cutx_end;j++){
    
      processed_img[i][j]=0; 
    
    }
  }  

}

void get_img_info(int *flag,int T){

  if(*flag==1){
    dealTimeStarts = timer_get(GPT_TIM_1);
    get_blobs(T);
    dealTimeEnds = timer_get(GPT_TIM_1);
    dealImageTime = dealTimeEnds - dealTimeStarts;
    
    
    UpdateTGTinfo(1);
  }
  else{
    get_line(T);
  }
  
  img_intervene(flag);  
}

/*********
广度优先算y和x最大最小

明明是dfs
似乎递归层数太多
图像全白时从最小坐标递归到最大坐标会溢出

********/
void bfs(int x,int y){
  if (x<IMG_W-1&&x>=1&&y<IMG_H-1&&y>=1){  
    if (Binary_map_Inflation[y+1][x]==255&&pathMap[y+1][x]==0){
      pathMap[y+1][x]=1;
      if (y+1<min_y)
        min_y=y+1;
      if (x<min_x)
        min_x=x;
      if (y+1>max_y)
        max_y=y+1;
      if (x>max_x)
        max_x=x;
      bfs(x,y+1);
    }
    if (Binary_map_Inflation[y-1][x]==255&&pathMap[y-1][x]==0){
      pathMap[y-1][x]=1;    
      if (y-1<min_y)
        min_y=y-1;
      if (x<min_x)
        min_x=x;
      if (y-1>max_y)
        max_y=y-1;
      if (x>max_x)
        max_x=x;
      bfs(x,y-1);
    }
    if (Binary_map_Inflation[y][x+1]==255&&pathMap[y][x+1]==0){
      pathMap[y][x+1]=1;
      if (y<min_y)
        min_y=y;
      if (x+1<min_x)
        min_x=x+1;
      if (y>max_y)
        max_y=y;
      if (x+1>max_x)
        max_x=x+1;
      bfs(x+1,y);
    }
    if (Binary_map_Inflation[y][x-1]==255&&pathMap[y][x-1]==0){
      pathMap[y][x-1]=1;    
      if (y<min_y)
        min_y=y;
      if (x-1<min_x)
        min_x=x-1;
      if (y>max_y)
        max_y=y;
      if (x-1>max_x)
        max_x=x-1;
      bfs(x-1,y);
    }  
    if (Binary_map_Inflation[y-1][x-1]==255&&pathMap[y-1][x-1]==0){
      pathMap[y-1][x-1]=1;    
      if (y-1<min_y)
        min_y=y-1;
      if (x-1<min_x)
        min_x=x-1;
      if (y-1>max_y)
        max_y=y-1;
      if (x-1>max_x)
        max_x=x-1;
      bfs(x-1,y-1);
    }  
    if (Binary_map_Inflation[y+1][x-1]==255&&pathMap[y+1][x-1]==0){
      pathMap[y+1][x-1]=1;    
      if (y+1<min_y)
        min_y=y+1;
      if (x-1<min_x)
        min_x=x-1;
      if (y+1>max_y)
        max_y=y+1;
      if (x-1>max_x)
        max_x=x-1;
      bfs(x-1,y+1);
    }  
    if (Binary_map_Inflation[y-1][x+1]==255&&pathMap[y-1][x+1]==0){
      pathMap[y-1][x+1]=1;    
      if (y-1<min_y)
        min_y=y-1;
      if (x+1<min_x)
        min_x=x+1;
      if (y-1>max_y)
        max_y=y-1;
      if (x+1>max_x)
        max_x=x+1;
      bfs(x+1,y-1);
    }  
    if (Binary_map_Inflation[y+1][x+1]==255&&pathMap[y+1][x+1]==0){
      pathMap[y+1][x+1]=1;    
      if (y+1<min_y)
        min_y=y+1;
      if (x+1<min_x)
        min_x=x+1;
      if (y+1>max_y)
        max_y=y+1;
      if (x+1>max_x)
        max_x=x+1;
      bfs(x+1,y+1);
    }  
  }
}
/*
虽说dfs也能用
但是忍不了递归溢出了
写一个BFS
*/


typedef struct pix_data {
    int x;
    int y;
} pix_data;


void BFSTraverse(int x,int y,uint8* original_img,uint8* path_map,int width,int height,
                 int* max_x,int* max_y,int* min_x,int* min_y,int* cnt){
  int dx,dy,tempx,tempy;
  *min_x=x;
  *max_x=x;
  *min_y=y;
  *max_y=y;
  LinkQueue Q;
  QElemType e;
  pix_data u;
  //pix_data v; 
  pix_data* v=(pix_data*)malloc(sizeof(pix_data));
  pix_data* w;
  u.x=x;
  u.y=y;
  v->x=x;
  v->y=y;
  *(path_map+x+y*width)=1;
  InitQueue(&Q);
  EnQueue(&Q, v);
  //EnQueue(&Q, &v);
  
  
  while(!QueueEmpty(Q)) {
    DeQueue(&Q, &e);
    u=*(pix_data*)e;

    dx=u.x;
    dy=u.y;
    free(e);//醒醒循环第一次free了v  不是malloc获得的空间不能free
    if(dx<width-1&&dx>=1&&dy<height-1&&dy>=1){
      for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
          tempx=dx-1+j;
          tempy=dy-1+i;
          if(i==j&&i==1)  continue;
          else{
            if(*(original_img+tempx+tempy*width)==255&&*(path_map+tempx+tempy*width)==0){
              w=(pix_data*)malloc(sizeof(pix_data));
              w->x=tempx;
              w->y=tempy;
              EnQueue(&Q, w);
              (*cnt)++;
              *(path_map+tempx+tempy*width)=1;
              if (tempy<*min_y)
                *min_y=tempy;
              if (tempx<*min_x)
                *min_x=tempx;
              if (tempy>*max_y)
                *max_y=tempy;
              if (tempx>*max_x)
                *max_x=tempx;
            }
          }
        }
      }
    }
  }
  free(Q.front);
}


/**********************************************************
horizontal filter尝试进行边界提取，
先robet或者sobel然后再二值化再进行搜线会怎么样？？？？
卷积操作：horizontal_map=mt9v03x_csi_image[i][j]*horizontal_filter[1][1]+mt9v03x_csi_image[i-1][j-1]*horizontal_filter[2][2]+
mt9v03x_csi_image[i-1][j]*horizontal_filter[2][1]+mt9v03x_csi_image[i-1][j+1]*horizontal_filter[2][0]+mt9v03x_csi_image[i][j-1]*horizontal_filter[1][2]
mt9v03x_csi_image[i][j+1]*horizontal_filter[1][0]+mt9v03x_csi_image[i+1][j+1]*horizontal_filter[0][0]+mt9v03x_csi_image[i+1][j]*horizontal_filter[0][1]
+mt9v03x_csi_image[i+1][j-1]*horizontal_filter[0][2]
**********************************************************/
/*int horizontal_filter[3][3]={-1,-2,-1,0,0,0,1,2,1};
float smooth_filter[3][3]={1,1,1,1,1,1,1,1,1};
uint8 horizontal_map[MT9V03X_CSI_H][MT9V03X_CSI_W];
uint8 smooth_map[MT9V03X_CSI_H][MT9V03X_CSI_W];
//horizontal_filter提取上下边界尝试
void convolution_process()
{       
  int L=0;
  int i=0,j=0;
  for (i=0;i<MT9V03X_CSI_H;i++)
    for (j=0;j<MT9V03X_CSI_W;j++)
    horizontal_map[i][j]=255;//初始化成白色
   for (i=1;i<MT9V03X_CSI_H-1;i++)
    for (j=1;j<MT9V03X_CSI_W-1;j++)
    { L=(int)( mt9v03x_image[i][j]*horizontal_filter[1][1] + mt9v03x_image[i-1][j-1]*horizontal_filter[2][2]
                           + mt9v03x_image[i-1][j]*horizontal_filter[2][1] + mt9v03x_image[i-1][j+1]*horizontal_filter[2][0] + mt9v03x_image[i][j-1]*horizontal_filter[1][2]
                           + mt9v03x_image[i][j+1]*horizontal_filter[1][0] + mt9v03x_image[i+1][j+1]*horizontal_filter[0][0] + mt9v03x_image[i+1][j]*horizontal_filter[0][1]
                           + mt9v03x_image[i+1][j-1]*horizontal_filter[0][2]);
    if (L<0)
      L=0;
    if (L>255)
      L=255;
    horizontal_map[i][j]=(uint8)L;
    }
   for (i=0;i<MT9V03X_CSI_H;i++)
      for (j=0;j<MT9V03X_CSI_W;j++)   
      if (horizontal_map[i][j]<Threshold)
        Binary_map[i][j]=255;           //小于阈值是白色的，大于是黑的
      else Binary_map[i][j]=0;
}
//平滑卷积
void smooth_extract()
{
  int i=0,j=0;
   for (i=0;i<MT9V03X_CSI_H;i++)
    for (j=0;j<MT9V03X_CSI_W;j++)
    smooth_map[i][j]=255;//初始化成白色
    
  for (i=1;i<MT9V03X_CSI_H-1;i++)
    for (j=1;j<MT9V03X_CSI_W-1;j++)
    { 
      smooth_map[i][j] = (uint8)((mt9v03x_image[i][j]*smooth_filter[1][1] + mt9v03x_image[i-1][j-1]*smooth_filter[2][2]
                           + mt9v03x_image[i-1][j]*smooth_filter[2][1] + mt9v03x_image[i-1][j+1]*smooth_filter[2][0] + mt9v03x_image[i][j-1]*smooth_filter[1][2]
                           + mt9v03x_image[i][j+1]*smooth_filter[1][0] + mt9v03x_image[i+1][j+1]*smooth_filter[0][0] + mt9v03x_image[i+1][j]*smooth_filter[0][1]
                             + mt9v03x_image[i+1][j-1]*smooth_filter[0][2])/9);}
}
//逐飞的大津法，据说要优化，但是对我们好像没啥用
uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row)
{
    #define GrayScale 256
    uint16 width = col;
    uint16 height = row;
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j, pixelSum = width * height;
    uint8 threshold = 0;
    uint8* data = image;  //指向像素数据的指针
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }

    //统计灰度级中每个像素在整幅图像中的个数  
    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            pixelCount[(int)data[i * width + j]]++;  //将像素值作为计数数组的下标
        }
    }

    //计算每个像素在整幅图像中的比例  
    float maxPro = 0.0;
    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
        if (pixelPro[i] > maxPro)
        {
            maxPro = pixelPro[i];
        }
    }

    //遍历灰度级[0,255]  
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    for (i = 0; i < GrayScale; i++)     // i作为阈值
    {
        w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
        for (j = 0; j < GrayScale; j++)
        {
            if (j <= i)   //背景部分  
            {
                w0 += pixelPro[j];
                u0tmp += j * pixelPro[j];
            }
            else   //前景部分  
            {
                w1 += pixelPro[j];
                u1tmp += j * pixelPro[j];
            }
        }
        u0 = u0tmp / w0;
        u1 = u1tmp / w1;
        u = u0tmp + u1tmp;
        deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            threshold = i;
        }
    }

    return threshold;
}

//加了电磁铁之后，改进一下搜线
//如果没加的话上面的已经够用了
float bottom_point=0;
void get_line_plus()
{
  Binary_deal();
  black_num=0;
  k_point=0;
  bottom_point=0;
    kl1=0;
  int i,j; 
  for (i=0;i<188;i++)
  {
    line_point[i][0]=0;
    line_point[i][1]=0;//线的点坐标初始化，最多也就188个
  }
  for (i=0;i<MT9V03X_CSI_H;i++)
    for (j=0;j<MT9V03X_CSI_W;j++)
        Line_map[i][j]=255;//初始化成白色

  for (j=MT9V03X_CSI_W-1;j>0;j--)      //遍历下半平面的点，寻找黑色的个数
    for (i=MT9V03X_CSI_H-1;i>prospect;i--)    
      if  (Binary_map[i][j]==0)      
        black_num++;
      
    if (black_num>20)
    {
    for (j=MT9V03X_CSI_W-1;j>0;j--)      
      for (i=MT9V03X_CSI_H-1;i>prospect;i--)    
      { 
        if (Binary_map[i][j]==0)          
        {
          Line_map[i][j]=0;
          line_point[k_point][0]=j;//x
          line_point[k_point][1]=i;//y
          k_point++;
          break;
        }
      }
    //如果第一个点跟第五个点的差距不大，说明调整完成，如果差距过大说明得左移 : >
    //如果第一个点在太左边说明得右移，就判断第一个点在哪
    if (k_point>7)
    {
      if ((line_point[0][1]-line_point[5][1])>4) //右移，等等好像也不用
    {
      kl1=(float)(line_point[0][1]-line_point[5][1])/(line_point[0][0]-line_point[5][0]);
      small_actual_val=-atan(kl1)/3.141596*180;
    }
     else
       small_actual_val=0;
    }
    bottom_point=line_point[0][1];
    for (i=0;i<k_point-1;i++)
      if (bottom_point<line_point[i+1][1])
        bottom_point=line_point[i+1][1];
    
    }
    
    
}


//得到视野里面所有黑点的位置
void get_line()
{
  black_num=0;
  first_point[0]=0;
  first_point[1]=0;  
  last_point[0]=0;
  last_point[1]=0;
  top_point[0]=0;
  top_point[1]=0;
  kl1=0;
  kl2=0;
  first_see_flag=0;
  Binary_deal();
  int i,j;
  k_point=0;
  
  for (i=0;i<MT9V03X_CSI_H;i++)
    for (j=0;j<MT9V03X_CSI_W;j++)
        Line_map[i][j]=255;//初始化成白色
  for (i=0;i<188;i++)
  {
    line_point[i][0]=0;
    line_point[i][1]=0;//线的点坐标初始化，最多也就188个
  }
  for (j=MT9V03X_CSI_W-1;j>=0;j--)      //遍历下半平面的点，寻找黑色的个数
    for (i=MT9V03X_CSI_H-1;i>prospect;i--)    
      if  (Binary_map[i][j]==0)
        black_num++;
  
//从底下开始往上搜寻,从右往左,遇到黑色就记录点位，然后下个循环
//记录黑线的第一个点跟最后一个点
  if (black_num>50)
  {
    for (j=MT9V03X_CSI_W-1;j>=0;j--)      
      for (i=MT9V03X_CSI_H-1;i>prospect;i--)    
      { 
        if (Binary_map[i][j]==0)          
        {
          Line_map[i][j]=0;
          line_point[k_point][0]=j;
          line_point[k_point][1]=i;
          k_point++;
          if (first_see_flag==0)//记录第一个黑点
          {
            first_point[0]=j;
            first_point[1]=i;
            first_see_flag=1;
          } break;
        }
      if (i<117&&j<185)   //j:x   i:y
      {
        if (Line_map[i][j]==255&&Line_map[i][j+1]==0)//记录最后一个黑点
        {
          last_point[0]=j;
          last_point[1]=i;
        }
     
      }
    }
  small_distance=(first_point[1]+last_point[1])/2;
  
  if (k_point>6&&(line_point[0][0]-line_point[5][0])!=0&&(line_point[k_point-6][0]-line_point[k_point-1][0])!=0)
  {
    kl1=(float)(line_point[0][1]-line_point[5][1])/(line_point[0][0]-line_point[5][0]);
    kl2=(float)(line_point[k_point-6][1]-line_point[k_point-1][1])/(line_point[k_point-6][0]-line_point[k_point-1][0]);
    kl1=-atan(kl1)/3.141596*180;
    kl2=atan(kl2)/3.141596*180;
    small_actual_val=kl1-kl2;
  }
  else
    small_actual_val=0;
  }
}*/