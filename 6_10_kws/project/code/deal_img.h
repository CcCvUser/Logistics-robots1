#ifndef DEAL_IMG_H
#define DEAL_IMG_H


#include "headfile.h"

#define SCC8660_img

#ifdef MT9V03_img

#define IMG_W  MT9V03X_W                                                     // 图像宽度     范围 [1-752]
#define IMG_H  MT9V03X_H 
#define view_center_x 94
#define view_center_y 60
#define img_cuty 14
#define img_cutx_begin 50
#define img_cutx_end 150
#define img_ID 0
#define img_finish_flag mt9v03x_finish_flag


#endif

/*
SCC8660_W 160
SCC8660_H 120
*/


#ifdef SCC8660_img

#define IMG_W  SCC8660_W                                                     // 图像宽度     范围 [1-752]
#define IMG_H  SCC8660_H 
#define view_center_x 80
#define view_center_y 60
#define img_cuty 14
#define img_cutx_begin 40
#define img_cutx_end 120
#define img_ID 1
#define img_finish_flag scc8660_finish_flag

#endif


extern float Threshold;
/*extern int prospect;
extern float bottom_point;
extern int black_num;
extern int k_point;
extern float small_actual_val;*/

//extern uint8 basic_map[MT9V03X_CSI_H][MT9V03X_CSI_W];

typedef struct {
   float center_x,center_y;
   int delta_x,delta_y;
   int TGT_area;
   float k;     //TGT方向
   int flag;
}BLOB_information;

typedef struct {
    float nowX, nowY;
    float lastX, lastY, preLastX, preLastY;
    float dxdt, dydt, ddxdtt, ddydtt;
    float dt, lastdt;
    float nextX, nextY;
    float errorX, errorY;
}TGT_information;

typedef struct {//极坐标下的直线方程//dis=xcos+ysin
   int dis;
   int angle;
}LINE_information;

extern BLOB_information blob_info[50];
extern LINE_information line_info[2][2];
extern TGT_information tgt_info;


void CreateTable();
void RGB565_LAB_Binary_deal(uint16 original_img[][SCC8660_W],uint8 processed_img[][IMG_W]);
void get_img_info(int *flag,int T);
void get_line(int T);
void draw_point(int x,int y,const uint16 color);
void get_blobs(int T);
//void get_border_test();
void bfs(int x,int y);
void Binary_deal(float T,uint8 original_img[][IMG_W],uint8 processed_img[][IMG_W]);
void get_border(uint8 original_img[][IMG_W],uint8 processed_img[][IMG_W]);
void smooth_extract(uint8 original_img[][MT9V03X_W],uint8 processed_img[][IMG_W]);
void DoubleInflation(uint8 original_img[][IMG_W],uint8 processed_img[][IMG_W]);
void corrode(uint8 original_img[][IMG_W],uint8 processed_img[][IMG_W]);
void show(int max_x,int max_y,int min_x,int min_y,const uint16 color,int zoom);
void UpdateTGTinfo(int flag);//预测

/*
图像的广度优先

*/
void BFSTraverse(int x,int y,uint8* original_img,uint8* path_map,int width,int height, 
                 int* max_x,int* max_y,int* min_x,int* min_y,int* cnt);

//void get_line();
//void get_line_plus();
//void convolution_process();

//uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row);
#endif