#include "zap.h"
#include "headfile.h"
#include "math.h"
//#include "stdlib"

extern int total_TGT;
extern int nowTGT;
extern target TGT[44];
extern int ACO_flag;
extern LinkQueue temp_TGT_Q;
extern int update_path_flag;

#define siteSizepop 20
#define oriSizepop 100
#define LIMIT_COVERAGE 831 //95%

int crossFlag=0;

float mid1 = 0 , mid2 = 0, mid3 = 0;



int8 realMap[36][26] ={
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 125, 107, 94, 90, 86, 82, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 82, 86, 90, 94, 107, 125, 0},
    {0, 0, 107, 92, 80, 75, 71, 67, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 67, 71, 75, 80, 92, 107, 0},
    {0, 0, 94, 80, 71, 67, 64, 61, 58, 58, 58, 58, 58, 58, 58, 58, 58, 58, 58, 61, 64, 67, 71, 80, 94, 0},
    {0, 0, 90, 75, 67, 64, 61, 58, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 58, 61, 64, 67, 75, 90, 0},
    {0, 0, 86, 71, 64, 61, 58, 55, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 55, 58, 61, 64, 71, 86, 0},
    {0, 0, 82, 67, 61, 58, 55, 52, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 52, 55, 58, 61, 67, 82, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 82, 67, 61, 58, 55, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 55, 58, 61, 67, 82, 0},
    {0, 0, 86, 71, 64, 61, 58, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 52, 58, 61, 64, 71, 86, 0},
    {0, 0, 90, 75, 67, 64, 61, 58, 55, 48, 48, 48, 48, 48, 48, 48, 48, 48, 55, 58, 61, 64, 67, 75, 90, 0},
    {0, 0, 94, 80, 71, 67, 64, 61, 58, 58, 58, 58, 58, 58, 58, 58, 58, 58, 58, 61, 64, 67, 71, 80, 94, 0},
    {0, 0, 107, 92, 80, 75, 71, 67, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 67, 71, 75, 80, 92, 107, 0},
    {0, 0, 125, 107, 94, 90, 86, 82, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 82, 86, 90, 94, 107, 125, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

int8 tempMap[36][26] ={
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 125, 107, 94, 90, 86, 82, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 82, 86, 90, 94, 107, 125, 0},
    {0, 0, 107, 92, 80, 75, 71, 67, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 67, 71, 75, 80, 92, 107, 0},
    {0, 0, 94, 80, 71, 67, 64, 61, 58, 58, 58, 58, 58, 58, 58, 58, 58, 58, 58, 61, 64, 67, 71, 80, 94, 0},
    {0, 0, 90, 75, 67, 64, 61, 58, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 58, 61, 64, 67, 75, 90, 0},
    {0, 0, 86, 71, 64, 61, 58, 55, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 55, 58, 61, 64, 71, 86, 0},
    {0, 0, 82, 67, 61, 58, 55, 52, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 52, 55, 58, 61, 67, 82, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 79, 64, 58, 55, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 52, 55, 58, 64, 79, 0},
    {0, 0, 82, 67, 61, 58, 55, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 50, 55, 58, 61, 67, 82, 0},
    {0, 0, 86, 71, 64, 61, 58, 52, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 52, 58, 61, 64, 71, 86, 0},
    {0, 0, 90, 75, 67, 64, 61, 58, 55, 48, 48, 48, 48, 48, 48, 48, 48, 48, 55, 58, 61, 64, 67, 75, 90, 0},
    {0, 0, 94, 80, 71, 67, 64, 61, 58, 58, 58, 58, 58, 58, 58, 58, 58, 58, 58, 61, 64, 67, 71, 80, 94, 0},
    {0, 0, 107, 92, 80, 75, 71, 67, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 67, 71, 75, 80, 92, 107, 0},
    {0, 0, 125, 107, 94, 90, 86, 82, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 79, 82, 86, 90, 94, 107, 125, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};


/*决定虚点位置*/
SitePop PSODetermineSite(){
    //Parameter Init
    const int L1 = 700, L2 = 500, maxgen = 10;
    const float wMax = 0.9, wMin = 0.4;
    const int c1 = 2, c2 = 2;
    const int vxMax = 175, vyMax = 125;
    const int seed = 114514;
    int minPointNum = 0, maxPointNum = 0;
    srand(seed);

    SitePop pops[siteSizepop],bestPops[siteSizepop], bestPop;
    memset(pops, 0, sizeof(pops));
    memset(&bestPop, 0, sizeof(bestPop));

    //Remove original virtual point
    int noVirtuals = 0;
    do{
        noVirtuals = 0;
        for(int i = 0; i <= total_TGT; i++){
            if(TGT[i].sigma == -1){
                noVirtuals = 1;
                for(int j = i; j < total_TGT; j++){
                    TGT[j] = TGT[j+1];
                    // memcpy(&TGT[j], &TGT[j+1], sizeof(&TGT[j]));
                }
                total_TGT--;
                break;
            }
        }
    }while(noVirtuals);

    //Get Coverage
    int nowCoverage = 0;
    SimpleOriDeside();
    for(int x = 1; x <= 35; x++){
        for(int y = 1; y <= 25; y++){
            if(tempMap[x][y] == 0) nowCoverage++;
        }
    }
    if (nowCoverage > LIMIT_COVERAGE){
      SimpleOriDeside();
      return bestPop;
    }
    else if(nowCoverage >= 700){
      minPointNum = 1;
      maxPointNum = 2;
    }
    else{
      minPointNum = 2;
      maxPointNum = 3;
    }

    //iteration
    for(int nowPoint = minPointNum; nowPoint <= maxPointNum; nowPoint++){

        //Particle Init
        if(nowPoint == minPointNum){
            for(int i = 0; i < siteSizepop; i++){
                pops[i].virtualPoints = nowPoint;
                for(int j = 0; j < pops[i].virtualPoints; j++){
                    int ramX = (rand() % 699) + 1;
                    int ramY = (rand() % 499) + 1;
                    int ramVx = rand() % vxMax;
                    int ramVy = rand() % vyMax;
                    pops[i].x[j] = ramX;
                    pops[i].y[j] = ramY;
                    pops[i].vx[j] = ramVx;
                    pops[i].vy[j] = ramVy;
                }
                pops[i].fit = SiteGetCoverage(pops[i]);
            }
        }
        else{
            for(int i = 0; i < siteSizepop; i++){
                pops[i].virtualPoints = nowPoint;
                for(int j = nowPoint - 1; j < pops[i].virtualPoints; j++){
                    int ramX = (rand() % 699) + 1;
                    int ramY = (rand() % 499) + 1;
                    // int ramVx = rand() % 60;
                    // int ramVy = rand() % 60;
                    pops[i].x[j] = ramX;
                    pops[i].y[j] = ramY;
                    // pops[i].vx[j] = ramVx;
                    // pops[i].vy[j] = ramVy;
                }

                //试试重新给前面的点一个速度，防止进入局部最优
                for(int j = 0; j < pops[i].virtualPoints; j++){
                    int ramVx = rand() % vxMax;
                    int ramVy = rand() % vyMax;
                    pops[i].vx[j] = ramVx;
                    pops[i].vy[j] = ramVy;
                }
                pops[i].fit = SiteGetCoverage(pops[i]);
            }
        }
        
        memcpy(bestPops, pops, sizeof(pops));
        for(int i = 0; i < siteSizepop; i++){
            if(pops[i].fit > bestPop.fit) memcpy(&bestPop, &pops[i], sizeof(pops[i]));
        }


        for(int T = maxgen; T > 0; T--){
            float w = wMax - ((wMax - wMin) / maxgen)*(maxgen - T);

            for(int j = 0; j < siteSizepop; j++){

                //Speed update
                for(int i = 0; i < pops[j].virtualPoints; i++){
                    float rand1 = (rand() % 35565) / 35565.0;
                    float rand2 = (rand() % 35565) / 35565.0;
                    float rand3 = (rand() % 35565) / 35565.0;
                    float rand4 = (rand() % 35565) / 35565.0;
                    pops[j].vx[i] = w * pops[j].vx[i] + c1 * rand1 * (bestPops[j].x[i] - pops[j].x[i]) + c2 * rand2 * (bestPop.x[i] - pops[j].x[i]);
                    pops[j].vy[i] = w * pops[j].vy[i] + c1 * rand3 * (bestPops[j].y[i] - pops[j].y[i]) + c2 * rand4 * (bestPop.x[i] - pops[j].x[i]);
                    if(pops[j].vx[i] > vxMax) pops[j].vx[i] = vxMax;
                    if(pops[j].vy[i] > vyMax) pops[j].vy[i] = vyMax;
                }

                //Position Update
                for(int i = 0; i < pops[j].virtualPoints; i++){
                    pops[j].x[i] += pops[j].vx[i];
                    pops[j].y[i] += pops[j].vx[i];
                    if(pops[j].x[i] > L1) pops[j].x[i] = L1;
                    if(pops[j].x[i] < 0) pops[j].x[i] = 0;
                    if(pops[j].y[i] > L2) pops[j].y[i] = L2;
                    if(pops[j].y[i] < 0) pops[j].y[i] = 0;
                }

                //Fit Update
                pops[j].fit = SiteGetCoverage(pops[j]);

                //Best Update
                for(int i = 0; i <pops[j].virtualPoints; i++){
                    if(pops[j].fit > bestPops[j].fit) memcpy(&bestPops[j], &pops[j], sizeof(pops[j]));//bestPops[j] = pops[j];
                    if(pops[j].fit > bestPop.fit) memcpy(&bestPop, &pops[j], sizeof(pops[j]));//bestPop = pops[j];
                }

            }

        }

        nowCoverage = 0;
        SiteGetCoverage(bestPop);
        for(int x = 1; x <= 35; x++){
            for(int y = 1; y <= 25; y++){
                if(tempMap[x][y] == 0) nowCoverage++;
            }
        }
        if (nowCoverage > LIMIT_COVERAGE) break;
    }

    for(int i = 0; i < bestPop.virtualPoints; i++){
        total_TGT++;
        TGT[total_TGT].x = bestPop.x[i];
        TGT[total_TGT].y = bestPop.y[i];
        TGT[total_TGT].sigma = -1;
    }

    SimpleOriDeside();
    return bestPop;
}

void UpdateRealMap(target nowTGT){
    int nowX = nowTGT.x/20;
    int nowY = nowTGT.y/20;
    int horizon = 3;
    for(int x = 1; x <= 35; x++){
        for(int y = 1; y <= 25; y++){
            if((nowTGT.ori & 0x01) && x - nowX - 1 >= 0 && x - nowX - 1 - 2 * horizon <= 0 && y - nowY + horizon >= 0 && y - nowY - horizon <= 0) realMap[x][y] = 0;
            if((nowTGT.ori & 0x04) && x - nowX + horizon >= 0 && x - nowX - horizon <= 0 && y - nowY - 1 >= 0 && y - nowY - 1 - 2 * horizon <= 0) realMap[x][y] = 0;
            if((nowTGT.ori & 0x10) && x - nowX + 1 <= 0 && x - nowX + 1 + 2 * horizon >= 0 && y - nowY + horizon >= 0 && y - nowY - horizon <= 0) realMap[x][y] = 0;
            if((nowTGT.ori & 0x40) && x - nowX + horizon >= 0 && x - nowX - horizon <= 0 && y - nowY + 1 <= 0 && y - nowY + 1 + 2 * horizon >= 0) realMap[x][y] = 0;
        }
    }
    return;
}

int SiteGetCoverage(SitePop pop){
    int coverage = 0;
    //memcpy(tempMap, realMap, sizeof(realMap));
    for(int i = 0; i <= pop.virtualPoints; i++){
        TGT[total_TGT + 1 + i].x = pop.x[i];
        TGT[total_TGT + 1 + i].y = pop.y[i];
    }
    total_TGT += pop.virtualPoints;
    SimpleOriDeside();
    for(int x = 1; x <= 35; x++){
        for(int y = 1; y <= 25; y++){
            if(tempMap[x][y] == 0) coverage++;
        }
    }
    for(int i = 0; i < pop.virtualPoints; i++){
        TGT[total_TGT - i].x = 0;
        TGT[total_TGT - i].y = 0;
    }
    total_TGT -= pop.virtualPoints;
    // printf("%d\n", coverage);
    return coverage;
}

/*int OriGetCoverage(OriPop pop){
    
}
*/
void SimpleOriDeside(){
    memcpy(tempMap, realMap, sizeof(tempMap));
    for(int i = nowTGT; i <= total_TGT; i++){
        //TGT[i].ori = 0b00000000;
        TGT[i].ori = 0X00;
        int nowX = TGT[i].x/20;
        int nowY = TGT[i].y/20;
        int horizon = 3;
        int tempOri[8][2] = {{0,0},{1,0},{2,0},{3,0},{4,0},{5,0},{6,0},{7,0}};
        //下面两段对坐标遍历的代码我知道很屎且有非常非常大的提速空间，但是如果不急的话就先这样写，改起来实在实在太麻烦了
        //下面这段代码也是决定这段代码总运行时间的，要优化先从这里入手
        for(int x = 1; x <= 35; x++){
            for(int y = 1; y <= 25; y++){
                if(x - nowX - 1 >= 0 && x - nowX - 1 - 2 * horizon <= 0 && y - nowY + horizon >= 0 && y - nowY - horizon <= 0) tempOri[0][1] += tempMap[x][y];
                if(x - nowX + horizon >= 0 && x - nowX - horizon <= 0 && y - nowY - 1 >= 0 && y - nowY - 1 - 2 * horizon <= 0) tempOri[2][1] += tempMap[x][y];
                if(x - nowX + 1 <= 0 && x - nowX + 1 + 2 * horizon >= 0 && y - nowY + horizon >= 0 && y - nowY - horizon <= 0) tempOri[4][1] += tempMap[x][y];
                if(x - nowX + horizon >= 0 && x - nowX - horizon <= 0 && y - nowY + 1 <= 0 && y - nowY + 1 + 2 * horizon >= 0) tempOri[6][1] += tempMap[x][y];
            }
        }
        SelectionSort(tempOri, 8);
        // TGT[i].ori[tempOri[0][0]] = 1;
        // TGT[i].ori[tempOri[1][0]] = 1;
        //TGT[i].ori |= (0b00000001 << tempOri[0][0]);
        //TGT[i].ori |= (0b00000001 << tempOri[1][0]);
        TGT[i].ori |= (0X01 << tempOri[0][0]);
        TGT[i].ori |= (0X01 << tempOri[1][0]);
        for(int x = 1; x <= 35; x++){
            for(int y = 1; y <= 25; y++){
                // if(TGT[i].ori[0] && x - nowX - 1 >= 0 && x - nowX - 1 - 2 * horizon <= 0 && y - nowY + horizon >= 0 && y - nowY - horizon <= 0) tempMap[x][y] = 0;
                // if(TGT[i].ori[2] && x - nowX + horizon >= 0 && x - nowX - horizon <= 0 && y - nowY - 1 >= 0 && y - nowY - 1 - 2 * horizon <= 0) tempMap[x][y] = 0;
                // if(TGT[i].ori[4] && x - nowX + 1 <= 0 && x - nowX + 1 + 2 * horizon >= 0 && y - nowY + horizon >= 0 && y - nowY - horizon <= 0) tempMap[x][y] = 0;
                // if(TGT[i].ori[6] && x - nowX + horizon >= 0 && x - nowX - horizon <= 0 && y - nowY + 1 <= 0 && y - nowY + 1 + 2 * horizon >= 0) tempMap[x][y] = 0;
                // if((TGT[i].ori & 0b00000001) && x - nowX - 1 >= 0 && x - nowX - 1 - 2 * horizon <= 0 && y - nowY + horizon >= 0 && y - nowY - horizon <= 0) tempMap[x][y] = 0;
                // if((TGT[i].ori & 0b00000100) && x - nowX + horizon >= 0 && x - nowX - horizon <= 0 && y - nowY - 1 >= 0 && y - nowY - 1 - 2 * horizon <= 0) tempMap[x][y] = 0;
                // if((TGT[i].ori & 0b00010000) && x - nowX + 1 <= 0 && x - nowX + 1 + 2 * horizon >= 0 && y - nowY + horizon >= 0 && y - nowY - horizon <= 0) tempMap[x][y] = 0;
                // if((TGT[i].ori & 0b01000000) && x - nowX + horizon >= 0 && x - nowX - horizon <= 0 && y - nowY + 1 <= 0 && y - nowY + 1 + 2 * horizon >= 0) tempMap[x][y] = 0;
                if((TGT[i].ori & 0X01) && x - nowX - 1 >= 0 && x - nowX - 1 - 2 * horizon <= 0 && y - nowY + horizon >= 0 && y - nowY - horizon <= 0) tempMap[x][y] = 0;
                if((TGT[i].ori & 0X04) && x - nowX + horizon >= 0 && x - nowX - horizon <= 0 && y - nowY - 1 >= 0 && y - nowY - 1 - 2 * horizon <= 0) tempMap[x][y] = 0;
                if((TGT[i].ori & 0X10) && x - nowX + 1 <= 0 && x - nowX + 1 + 2 * horizon >= 0 && y - nowY + horizon >= 0 && y - nowY - horizon <= 0) tempMap[x][y] = 0;
                if((TGT[i].ori & 0X40) && x - nowX + horizon >= 0 && x - nowX - horizon <= 0 && y - nowY + 1 <= 0 && y - nowY + 1 + 2 * horizon >= 0) tempMap[x][y] = 0;
            
            }
        }


        // for(int j = 0; j < 8; j++){
        //     printf("%d",TGT[i].ori[j]);
        // }
        // printf("\n");
    }
    return;
}


void SelectionSort(int arr[][2], int n) {
    int i, j, maxIndex;
    for (i = 0; i < n - 1; i++) {
        maxIndex = i;
        for (j = i + 1; j < n; j++) {
            if (arr[j][1] > arr[maxIndex][1]) {
                maxIndex = j;
            }
        }
        int temp0 = arr[i][0];
        int temp1 = arr[i][1];
        arr[i][0] = arr[maxIndex][0];
        arr[i][1] = arr[maxIndex][1];
        arr[maxIndex][0] = temp0;
        arr[maxIndex][1] = temp1;
    }
    //if(arr[0][0] == 1 || (arr[1][0] == 1 && arr[0][0] != 0)) BEEP(4000);
    return;
}

void ShowWeightMap(){
    for(int x = 0; x < 36; x++){
        for(int y = 0; y < 26; y++){
            //printf("%d ",tempMap[x][y]);
        }
        //printf("\n");
    }
    return;
}

/*    author:chatGPT   */
/*
int isCrossed(float xi, float yi, float xj, float yj, float xp, float yp, float xq, float yq) {
    float cross = (xj - xi) * (yq - yp) - (yj - yi) * (xq - xp);
    if (cross != 0) {
        // 两向量不共线
        float r = ((yi - yp) * (xq - xp) - (xi - xp) * (yq - yp)) / cross;
        float s = ((yi - yp) * (xj - xi) - (xi - xp) * (yj - yi)) / cross;
        if (r >= 0 && r <= 1 && s >= 0 && s <= 1) {
            // 两条线段相交
            return 1;
        }
    } else {
        // 两向量共线，需要进一步判断是否有重合部分
        if (((xi - xp) * (yq - yp) - (yi - yp) * (xq - xp)) == 0) {
            // 四个点在同一条直线上
            if ((xi >= xp && xi <= xq) || (xi >= xq && xi <= xp) ||
                (xj >= xp && xj <= xq) || (xj >= xq && xj <= xp) ||
                (xp >= xi && xp <= xj) || (xp >= xj && xp <= xi) ||
                (xq >= xi && xq <= xj) || (xq >= xj && xq <= xi)) {
                // 有重合部分
                return 1;
            }
        }
    }
    // 两条线段不相交
    return 0;
}*/

int isCrossed(float xi, float yi, float xj, float yj, float xp, float yp, float xq, float yq) {
    float cross = (xj - xi) * (yq - yp) - (yj - yi) * (xq - xp);
    if (cross != 0) {
        // 两向量不共线
        float r = ((yi - yp) * (xq - xp) - (xi - xp) * (yq - yp)) / cross;
        float s = ((yi - yp) * (xj - xi) - (xi - xp) * (yj - yi)) / cross;
        if (r >= 0 && r <= 1 && s >= 0 && s <= 1) {
            // 两条线段相交
            return 1;
        }
    }
    return 0;
}


void ACO(){

    //system_delay_ms(2000);

    const double alpha = 1.0;
    const double beta = 6.0;
    const double rho = 0.4;
    const int Q = 1, T = 1 * (total_TGT - nowTGT + 1), K = 5 * (total_TGT - nowTGT + 1);
    const int seed = 114514;
    float tau[44][44],dist[44][44],eta[44][44],taueta[44][44],delta[44][44];
    Ant ant,bestAnt;
    target OrderedTGT[44];
    srand(seed);
    bestAnt.distance = 1145141919180;
    //initialization

    for(int i = nowTGT; i <= total_TGT; i++){
        for(int j = nowTGT; j <= total_TGT; j++){
            tau[i][j] = 1;
            dist[i][j] = pow( pow( TGT[i].x - TGT[j].x, 2) + pow( TGT[i].y - TGT[j].y, 2), 0.5);
            eta[i][j] = 1/dist[i][j];
        }
    }
    //init of dist[][], tau[][], eta[][]

    for(int t = 0; t < T; t++){
        for(int i = nowTGT; i <= total_TGT; i++){
            for(int j = nowTGT; j <= total_TGT; j++){
                taueta[i][j] = pow(tau[i][j],alpha) * pow(eta[i][j],beta);
                delta[i][j] = 0;
            }
        }
        //get taueta and delta matrix

        for(int k = 0; k < K; k++){ //k stands for ant
            ant.current = nowTGT;
            ant.distance = 0;
            for(int i = 0; i <= nowTGT; i++) {
                ant.visited[i] = true;
                ant.order[i] = 0;
            }
            for(int i = nowTGT + 1; i <= total_TGT; i++){
                ant.visited[i] = false;
            }
            for(int i = nowTGT + 1; i <= total_TGT; i++){ //i stands for the visit times
                float sum = 0.0;
                for(int j = nowTGT + 1; j <= total_TGT; j++){ //j stands for the tgt that unvisited
                    if(ant.current != j && ant.visited[j] == false)
                    sum += taueta[ant.current][j];
                }
                for(int j = 0; j <= total_TGT; j++){
                    if(ant.visited[j] == false) ant.nextp[j] = taueta[ant.current][j] / sum;
                    else ant.nextp[j] = 0.0;
                }
                for(int j = nowTGT + 1; j <= total_TGT; j++){
                    ant.nextp[j] += ant.nextp[j-1];
                }
                // get the p array

                float ram = (rand() % 35565 )/ 35565.0;
                while(ram == 0) ram = (rand() % 35565 )/ 35565.0;
                for(ant.next = 0; ;ant.next++){
                    if(ant.next == 0){
                        if(ram < ant.nextp[ant.next]) break;
                    }
                    else{
                        if(ant.nextp[ant.next-1] <= ram && ram <= ant.nextp[ant.next]) break;
                    }
                }
                //choose the route

                ant.order[i] = ant.next;
                ant.visited[ant.next] = true;
                ant.current = ant.next;
                //update the ant
            }
            //here need to update the route!!! NO CROSS!! And recalculate the distance!!!!
            /*

            xs, bu hui xie;
            xia mian haoxiang cuole;

            */

            //updated here in 2023 4 14

            /*
            do{
                crossFlag = 0;
                for(int i = nowTGT; i < total_TGT - 2; i++){
                    for(int p = i + 2; p < total_TGT - 4; p++){
                        if(isCrossed(TGT[ant.order[i]].x, TGT[ant.order[i]].y, TGT[ant.order[i+1]].x, TGT[ant.order[i+1]].y,TGT[ant.order[p]].x, TGT[ant.order[p]].y, TGT[ant.order[p+1]].x, TGT[ant.order[p+1]].y)){
                            Ant tempAnt;
                            crossFlag = 1;
                            printf("%d-%d and %d-%d is Crossing\n",i,i+1,p,p+1);
                            for(int j = 0; j <= total_TGT; j++){
                                if(j < i+1 || j > p){
                                    tempAnt.order[j] = ant.order[j];
                                    //printf("--1 %d %d %d", i, p, j);
                                }

                                else{
                                    tempAnt.order[j] = ant.order[1+i+p-j];
                                    //printf("--2 %d %d %d", i, p, j);
                                }
                            }
                            for(int j = 0; j <= total_TGT; j++){
                                ant.order[j] = tempAnt.order[j];
                            };
                            break;
                        }
                    }
                    if(crossFlag == 1) break;
                }
            }while(crossFlag);
            */

            ant.distance = 0;
            ant.order[nowTGT] = nowTGT;

            for(int i = nowTGT; i < total_TGT; i++){
                ant.distance += dist[ant.order[i]][ant.order[i+1]];
            }

            //updated here in 2023 2 26
            ant.distance += pow( pow( TGT[ant.order[total_TGT-1]].x - 700, 2) + pow( TGT[ant.order[total_TGT-1]].y, 2), 0.5);

            if(ant.distance < bestAnt.distance){
                bestAnt.distance = ant.distance;
                for(int i = nowTGT; i <= total_TGT; i++){
                    bestAnt.order[i] = ant.order[i];
                }
            }

            for(int i = nowTGT; i < total_TGT; i++){
                delta[ant.order[i]][ant.order[i+1]] += Q/ant.distance;
            }
            //update delta
        }

        //printf("t = %d, dist = %f\n", t, bestAnt.distance);

        for(int i = nowTGT; i <= total_TGT; i++){
            for(int j = nowTGT; j <= total_TGT; j++){
                tau[i][j] = rho*tau[i][j] + delta[i][j];
                if (tau[i][j] < 0.000005)   tau[i][j] = 0.000005;
            }
        }
        //update tau
    }

    //20230415
    tft180_show_string(60,5*16,"BEGIN_cross");
    do{
          for(int i = nowTGT; i <= total_TGT; i++){
            for (int j = i+1; j <= total_TGT; j++){
              if(TGT[i].x == TGT[j].x && TGT[i].y == TGT[j].y) {
                BEEP(2000);

                tft180_show_float(0,0*16,TGT[i].x,3,1);
                tft180_show_float(60,0*16,TGT[i].y,3,1);
                tft180_show_float(0,1*16,TGT[j].x,3,1);
                tft180_show_float(60,1*16,TGT[j].y,3,1);
                tft180_show_uint(0,2*16,total_TGT,4);
                tft180_show_uint(80,2*16,nowTGT,4); 
                tft180_show_uint(0,3*16,i,4); 
                tft180_show_uint(80,3*16,j,4); 
                send_TGT_Upper_computer();
              }
            }
          }      
        crossFlag = 0;
        tft180_clear();
        tft180_show_string(60,5*16,"BEGIN_cross");
        for(int i = nowTGT; i < total_TGT - 2; i++){
            for(int p = i + 2; p < total_TGT; p++){
                if(isCrossed(TGT[bestAnt.order[i]].x, TGT[bestAnt.order[i]].y, TGT[bestAnt.order[i+1]].x, TGT[bestAnt.order[i+1]].y,TGT[bestAnt.order[p]].x, TGT[bestAnt.order[p]].y, TGT[bestAnt.order[p+1]].x, TGT[bestAnt.order[p+1]].y)){
                    Ant tempAnt;
                    crossFlag = 1;
                    //printf("%d-%d and %d-%d is Crossing\n",i,i+1,p,p+1);
                    for(int j = 0; j <= total_TGT; j++){
                        if(j < i+1 || j > p){
                            tempAnt.order[j] = bestAnt.order[j];
                            //printf("--1 %d %d %d", i, p, j);
                        }

                        else{
                            tempAnt.order[j] = bestAnt.order[1+i+p-j];
                            //printf("--2 %d %d %d", i, p, j);
                        }
                    }
                    for(int j = 0; j <= total_TGT; j++){
                        bestAnt.order[j] = tempAnt.order[j];
                    };
                    break;
                }
            }
            if(crossFlag == 1) break;
        }
    }while(crossFlag);
    tft180_show_string(60,5*16,"END_cross");
    
    
    tft180_show_string(0,5*16,"BEGIN");
    for(int i = 0; i <= nowTGT; i++){
        OrderedTGT[i].x = TGT[i].x;
        OrderedTGT[i].y = TGT[i].y;
        OrderedTGT[i].sigma = TGT[i].sigma;
        OrderedTGT[i].main_class = TGT[i].main_class;
        OrderedTGT[i].second_class = TGT[i].second_class;
        OrderedTGT[i].real_class = TGT[i].real_class;
        OrderedTGT[i].ori = TGT[i].ori;
    }
    tft180_show_string(0,6*16,"END");
    for(int i = nowTGT + 1; i <= total_TGT; i++){
        OrderedTGT[i].x = TGT[bestAnt.order[i]].x;
        OrderedTGT[i].y = TGT[bestAnt.order[i]].y;
        OrderedTGT[i].sigma = TGT[bestAnt.order[i]].sigma;
        OrderedTGT[i].main_class = TGT[bestAnt.order[i]].main_class;
        OrderedTGT[i].second_class = TGT[bestAnt.order[i]].second_class;
        OrderedTGT[i].real_class = TGT[bestAnt.order[i]].real_class;
        OrderedTGT[i].ori = TGT[bestAnt.order[i]].ori;
    }
    OrderedTGT[nowTGT].x = TGT[nowTGT].x;
    OrderedTGT[nowTGT].y = TGT[nowTGT].y;
    OrderedTGT[nowTGT].sigma = TGT[nowTGT].sigma;
    OrderedTGT[nowTGT].main_class = TGT[nowTGT].main_class;
    OrderedTGT[nowTGT].second_class = TGT[nowTGT].second_class;
    OrderedTGT[nowTGT].real_class = TGT[nowTGT].real_class;
    OrderedTGT[nowTGT].ori = TGT[nowTGT].ori;

    for(int i = 0; i <= total_TGT; i++){
        TGT[i].x = OrderedTGT[i].x;
        TGT[i].y = OrderedTGT[i].y;
        TGT[i].sigma = OrderedTGT[i].sigma;
        TGT[i].main_class = OrderedTGT[i].main_class;
        TGT[i].second_class = OrderedTGT[i].second_class;
        TGT[i].real_class = OrderedTGT[i].real_class;
        TGT[i].ori = OrderedTGT[i].ori;
    }
/*
    printf("best distance is %f\n",bestAnt.distance);
    printf("best route is ");
    for(int i = 0; i <= total_TGT; i++){
        printf("%d ",bestAnt.order[i]);
    }
    printf("\n");
*/

    return;
}


extern int8 virtual_flag;
void update_path(){
    int flag_2=1;
    static int cnt=0;
    int flag=0;
    QElemType e;
    target tempTGT={0,0,0,0,0,0,0};
    tft180_full(RGB565_WHITE);
    while(!QueueEmpty(temp_TGT_Q)){
      DeQueue(&temp_TGT_Q,&e);
      
      tempTGT.x=(*(target*)e).x;
      tempTGT.y=(*(target*)e).y;
      tempTGT.sigma=(*(target*)e).sigma;
      
      free(e);
      if(tempTGT.x>700||tempTGT.y>500||tempTGT.x<0||tempTGT.y<0||tempTGT.sigma>150){//如果收崩了则清空队列
        BEEP(2000);
        while(!QueueEmpty(temp_TGT_Q)){
          DeQueue(&temp_TGT_Q,&e);
          free(e);
        }
        return;
      }
      flag+=UpdateTGT(tempTGT);
    }
    cnt++;

    tft180_show_uint(0,0*16,nowTGT,4); 
    tft180_show_uint(0,1*16,total_TGT,4);
    tft180_show_uint(0,2*16,cnt,4);
    if(flag){
      if(virtual_flag){//开启虚点解决覆盖率问题
        PSODetermineSite();
      }
      else{
        SimpleOriDeside();
      }
      /*if(TGT[nowTGT].ori==66||TGT[nowTGT].ori==6){
        while(1){
          tft180_full(RGB565_WHITE);   
          tft180_show_string(0, 3*16, "cnm");
        }
      
      }*/
      do{
        for(int i = nowTGT; i <= total_TGT; i++){
          if((TGT[i].x<=0||TGT[i].y<=0)&&i!=0){//莫名其妙偶发的多0，0点和有重复点的问题，找半天没找到为啥发生，先为了完赛去除该点
              tft180_set_color(RGB565_BLUE, RGB565_GREEN);
                //BEEP(2000);
              flag_2=1;
              TGT[i]=TGT[total_TGT];
              memset(&TGT[total_TGT],0,sizeof(&TGT[total_TGT]));
              total_TGT--;
              break;
          }
            for (int j = i+1; j <= total_TGT; j++){
              if(TGT[i].x == TGT[j].x && TGT[i].y == TGT[j].y) {
                //BEEP(2000);
                tft180_set_color(RGB565_BLUE, RGB565_RED);
                TGT[j]=TGT[total_TGT];
                memset(&TGT[total_TGT],0,sizeof(&TGT[total_TGT]));
                total_TGT--;
              }
              
            }
          }
        flag_2=0;
      }while(flag_2) ;    
        BEEP(200);
        tft180_show_string(0, 3*16 ,"ACO_BEGIN");   
        ACO_flag=1;
        ACO();
        tft180_show_string(0, 4*16 ,"ACO_END");

        
        /*while(!QueueEmpty(temp_TGT_Q)){
            tft180_show_string(0, 3*16 ,"errorupdate_path");
            DeQueue(&temp_TGT_Q,&tempTGT);
            flag+=UpdateTGT(tempTGT);
        }
        if(flag!=0){
            update_path_flag=1;
        }*/
        ACO_flag=0;
    }
    return;
}


/******
4.21
******/
/*void ACO(){
    ACO_flag=1;
    const double alpha = 0.8;
    const double beta = 3.5;
    const double rho = 0.8;
    const int Q = 1, T = 15 * (total_TGT - nowTGT + 1), K = (int)1.5 * (total_TGT - nowTGT + 1);
    const int seed = 20020926;
    float tau[31][31],dist[31][31],eta[31][31],taueta[31][31],delta[31][31];
    Ant ant,bestAnt;
    target OrderedTGT[30];
    srand(seed);
    bestAnt.distance = 1145141919180;
    //initialization

    for(int i = nowTGT; i <= total_TGT; i++){
        for(int j = nowTGT; j <= total_TGT; j++){
            tau[i][j] = 1;
            dist[i][j] = pow( pow( TGT[i].x - TGT[j].x, 2) + pow( TGT[i].y - TGT[j].y, 2), 0.5);
            eta[i][j] = 1/dist[i][j];
        }
    }
    //init of dist[][], tau[][], eta[][]

    for(int t = 0; t < T; t++){

        for(int i = nowTGT; i <= total_TGT; i++){
            for(int j = nowTGT; j <= total_TGT; j++){
                taueta[i][j] = pow(tau[i][j],alpha) * pow(eta[i][j],beta);
                delta[i][j] = 0;
            }
        }
        //get taueta and delta matrix

        for(int k = 0; k < K; k++){ //k stands for ant
            ant.current = nowTGT;
            ant.distance = 0;
            for(int i = 0; i <= nowTGT; i++) ant.visited[i] = true;
            for(int i = nowTGT + 1; i <= total_TGT; i++){
                ant.visited[i] = false;
            }
            for(int i = nowTGT + 1; i <= total_TGT; i++){ //i stands for the visit times
                float sum = 0.0;
                for(int j = nowTGT + 1; j <= total_TGT; j++){ //j stands for the tgt that unvisited
                    if(ant.current != j && ant.visited[j] == false)
                    sum += taueta[ant.current][j];
                }
                for(int j = 0; j <= total_TGT; j++){
                    if(ant.visited[j] == false) ant.nextp[j] = taueta[ant.current][j] / sum;
                    else ant.nextp[j] = 0.0;
                }
                for(int j = nowTGT + 1; j <= total_TGT; j++){
                    ant.nextp[j] += ant.nextp[j-1];
                }
                // get the p array

                float ram = (rand() % 35565 )/ 35565.0;
                while(ram == 0) ram = (rand() % 35565 )/ 35565.0;
                for(ant.next = 0; ;ant.next++){
                    if(ant.next == 0){
                        if(ram < ant.nextp[ant.next]) break;
                    }
                    else{
                        if(ant.nextp[ant.next-1] <= ram && ram <= ant.nextp[ant.next]) break;
                    }
                }
                //choose the route

                ant.order[i] = ant.next;
                ant.visited[ant.next] = true;
                ant.distance += dist[ant.current][ant.next];
                ant.current = ant.next;
                //update the ant
            }

            //updated here in 2023 2 26
            ant.distance += pow( pow( TGT[ant.order[total_TGT-1]].x - 350, 2) + pow( TGT[ant.order[total_TGT-1]].y, 2), 0.5);

            if(ant.distance < bestAnt.distance){
                bestAnt.distance = ant.distance;
                for(int i = nowTGT; i <= total_TGT; i++){
                    bestAnt.order[i] = ant.order[i];
                }
            }

            for(int i = nowTGT; i <= total_TGT; i++){
                for(int j = nowTGT; j <= total_TGT; j++){
                    delta[i][j] += Q/ant.distance;
                }
            }
            //update delta
        }

        printf("t = %d, dist = %f\n", t, bestAnt.distance);

        for(int i = nowTGT; i <= total_TGT; i++){
            for(int j = nowTGT; j <= total_TGT; j++){
                tau[i][j] = rho*tau[i][j] + delta[i][j];
            }
        }
        //update tau
    }

    for(int i = nowTGT + 1; i <= total_TGT; i++){
        OrderedTGT[i].x = TGT[bestAnt.order[i]].x;
        OrderedTGT[i].y = TGT[bestAnt.order[i]].y;
        OrderedTGT[i].sigma =TGT[bestAnt.order[i]].sigma;
        OrderedTGT[i].main_class = TGT[bestAnt.order[i]].main_class;
        OrderedTGT[i].second_class = TGT[bestAnt.order[i]].second_class;
        OrderedTGT[i].real_class = TGT[bestAnt.order[i]].real_class;
    }
    OrderedTGT[nowTGT].x = TGT[nowTGT].x;
    OrderedTGT[nowTGT].y = TGT[nowTGT].y;
    OrderedTGT[nowTGT].sigma =TGT[nowTGT].sigma;
    OrderedTGT[nowTGT].main_class = TGT[nowTGT].main_class;
    OrderedTGT[nowTGT].second_class = TGT[nowTGT].second_class;
    OrderedTGT[nowTGT].real_class = TGT[nowTGT].real_class;

    for(int i = 0; i <= total_TGT; i++){
        TGT[i].x = OrderedTGT[i].x;
        TGT[i].y = OrderedTGT[i].y;
        TGT[i].sigma = OrderedTGT[i].sigma;
        TGT[i].main_class = OrderedTGT[i].main_class;
        TGT[i].second_class = OrderedTGT[i].second_class;
        TGT[i].real_class = OrderedTGT[i].real_class;
    }

    printf("best distance is %f\n",bestAnt.distance);
    printf("best route is ");
    for(int i = 0; i <= total_TGT; i++){
        printf("%d ",bestAnt.order[i]);
    }
    printf("\n");
    ACO_flag=0;
    return;
}*/


//void ACO(){
//    const double alpha = 1.0;
//    const double beta = 2.0;
//    const double rho = 0.5;
//    const int Q = 1, T = 15 * total_TGT, K = (int)(1.5 * total_TGT);
//    const int seed = 114514;
//    float tau[31][31],dist[31][31],eta[31][31],taueta[31][31],delta[31][31];
//    Ant ant,bestAnt;
//    target OrderedTGT[30];
//    srand(seed);
//    bestAnt.distance = 1145141919180;
//    //initialization
//
//    for(int i = 0; i <= total_TGT; i++){
//        for(int j = 0; j < total_TGT; j++){
//            tau[i][j] = 1;
//            dist[i][j] = pow( pow( TGT[i].x - TGT[j].x, 2) + pow( TGT[i].y - TGT[j].y, 2), 0.5);
//            eta[i][j] = 1/dist[i][j];
//        }
//        tau[i][total_TGT] = 1;
//        dist[i][total_TGT] = pow( pow(TGT[i].x , 2) + pow(TGT[i].y , 2), 0.5);
//        eta[i][total_TGT] = 1.0/dist[i][total_TGT];
//    }
//    //init of dist[][], tau[][], eta[][]
//
//    for(int t = 0; t < T; t++){
//
//        for(int i = 0; i <= total_TGT; i++){
//            for(int j = 0; j <= total_TGT; j++){
//                taueta[i][j] = pow(tau[i][j],alpha) * pow(eta[i][j],beta);
//                delta[i][j] = 0;
//            }
//        }
//        //get taueta and delta matrix
//
//        for(int k = 0; k < K; k++){ //k stands for ant
//            ant.current = total_TGT;
//            ant.distance = 0;
//            for(int i = 0; i < total_TGT; i++){
//                ant.visited[i] = false;
//            }
//            for(int i = 0; i < total_TGT; i++){ //i stands for the visit times
//                float sum = 0.0;
//                for(int j = 0; j < total_TGT; j++){ //j stands for the tgt that unvisited
//                    if(ant.current != j && ant.visited[j] == false)
//                    sum += taueta[ant.current][j];
//                }
//                for(int j = 0; j < total_TGT; j++){
//                    if(ant.visited[j] == false) ant.nextp[j] = taueta[ant.current][j] / sum;
//                    else ant.nextp[j] = 0.0;
//                }
//                for(int j = 1; j < total_TGT; j++){
//                    ant.nextp[j] += ant.nextp[j-1];
//                }
//                // get the p array
//
//                float ram = (rand() % 35565 )/ 35565.0;
//                for(ant.next = 0; ;ant.next++){
//                    if(ant.next == 0){
//                        if(ram < ant.nextp[ant.next]) break;
//                    }
//                    else{
//                        if(ant.nextp[ant.next-1] <= ram && ram <= ant.nextp[ant.next]) break;
//                    }
//                }
//                //choose the route
//
//                ant.order[i] = ant.next;
//                ant.visited[ant.next] = true;
//                ant.distance += dist[ant.current][ant.next];
//                ant.current = ant.next;
//                //update the ant
//                
//                
//
//            }
//            
//            //updated here in 2023 3 5
//            ant.distance += pow( pow( TGT[ant.order[total_TGT-1]].x - 350, 2) + pow( TGT[ant.order[total_TGT-1]].y, 2), 0.5);
//            /*
//            mid1 = TGT[ant.order[total_TGT-1]].x - 350.0;
//            mid1 = pow(mid1,2);
//            mid2 = TGT[ant.order[total_TGT-1]].y;
//            mid2 = pow(mid2,2);
//            mid3 = sqrt(mid1 + mid2);
//            ant.distance += mid3;
//            */
//            
//            if(ant.distance < bestAnt.distance){
//                bestAnt.distance = ant.distance;
//                for(int i = 0; i < total_TGT; i++){
//                    bestAnt.order[i] = ant.order[i];
//                }
//            }
//
//            for(int i = 0; i < total_TGT; i++){
//                for(int j = 0; j < total_TGT; j++){
//                    delta[i][j] += Q/ant.distance;
//                }
//            }
//            //update delta
//        }
//
//        //printf("t = %d, dist = %f\n", t, bestAnt.distance);
//
//        for(int i = 0; i < total_TGT; i++){
//            for(int j = 0; j < total_TGT; j++){
//                tau[i][j] = rho*tau[i][j] + delta[i][j];
//            }
//        }
//        //update tau
//    }
//
//    for(int i = 0; i < total_TGT; i++){
//        OrderedTGT[i].x = TGT[bestAnt.order[i]].x;
//        OrderedTGT[i].y = TGT[bestAnt.order[i]].y;
//        OrderedTGT[i].main_class = TGT[bestAnt.order[i]].main_class;
//        OrderedTGT[i].second_class = TGT[bestAnt.order[i]].second_class;
//        OrderedTGT[i].real_class = TGT[bestAnt.order[i]].real_class;
//    }
//
//    for(int i = 0; i < total_TGT; i++){
//        TGT[i].x = OrderedTGT[i].x;
//        TGT[i].y = OrderedTGT[i].y;
//        TGT[i].main_class = OrderedTGT[i].main_class;
//        TGT[i].second_class = OrderedTGT[i].second_class;
//        TGT[i].real_class = OrderedTGT[i].real_class;
//    }
//
//    /*
//    printf("best distance is %f\n",bestAnt.distance);
//    printf("best route is ");
//    for(int i = 0; i < total_TGT; i++){
//        printf("%d ",bestAnt.order[i]);
//    }
//    printf("\n");
//    在Vscode上测试用的，这里没用捏*/
//
//    return;
//}
