#ifndef ZAP_H
#define ZAP_H
#include "headfile.h"
typedef struct Ant{
    bool visited[44];
    int order[44];
    int current;
    int next;
    float nextp[44]; //stands for the possibility of next destination
    float distance;
}Ant;

typedef struct SitePop{
    float x[5];
    float y[5];
    float vx[5];
    float vy[5];
    int fit;
    int virtualPoints;
}SitePop;

typedef struct OriPop{
    float ori[44];
    float vori[44];
    int fit;
    int TGTnum;
}OriPop;


SitePop PSODetermineSite();
void SimpleOriDeside();
void SelectionSort(int arr[][2], int n);
void ShowWeightMap();
void ACO();
int isCrossed(float xi, float yi, float xj, float yj, float xp, float yp, float xq, float yq);
int SiteGetCoverage(SitePop pop);
int OriGetCoverage(OriPop pop);
void UpdateRealMap(target nowTGT);


int isCrossed(float xi, float yi, float xj, float yj, float xp, float yp, float xq, float yq) ;
void ACO();
void update_path();

#endif