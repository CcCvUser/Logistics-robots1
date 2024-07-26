#ifndef SPEED_H
#define SPEED_H

#include "headfile.h" 


#define SI  speedInfo
#define SP  speedParam
#define TDP trackDectionParam

typedef struct SpeedInfo
{
    int varL[3];                //编码器值
    int varR[3];                //编码器值

    int nowSpeedL;              //左轮实际速度
    int nowSpeedR;              //右轮实际速度
    int aimSpeed;	            //目标速度

    int aimSpeedL;	            //左轮目标速度（算上差速的）
    int aimSpeedR;	            //右轮目标速度（算上差速的）

    int motorPWML;              //左轮PWM
    int motorPWMR;              //右轮PWM

    uint32_t differential;      //差速 
    uint32_t adcDifferential;   //电磁差速
    uint32_t encoderThreshold;  //编码器限幅，防止抬轮和滑移
} SpeedInfo;



typedef enum SpeedType
{
    NORMAL_SHIFT,	    //二次公式
    FULL_ACCELE,	    //直线全速
    HALF_ACCELE,        //出弯加速  车身已经摆正的时候
    BRAKE,              //刹车
    EXIT_CURVE_ACCELE,	//出弯加速，刚出弯还没摆正的时候
} SpeedType;

typedef enum BrakeType
{
    STRIGHT_BRAKE,          //长直道刹车
    SHORT_STRIGHT_BRAKE,    //短直道刹车
    EXIT_ENTER_BRAKE,       //出弯再入弯刹车
} BrakeType;

typedef struct SpeedParam
{
    uint32_t addSpeed;                     //出弯加速一点点加

    uint32_t maxSpeed;                      // 直道最大速度   长直道速度
    uint32_t max2Speed;                     // 二次公式 最大速度限幅
    uint32_t minSpeed;                      // 最小速度 (全部的)

    uint32_t curveSpeed;                    //二次公式弯道  基础速度
    uint32_t exitSpeed;                     //出弯速度基础
    uint32_t maxExitSpeed;                  //出弯加速的最大速度
    uint32_t exitEnterSpeed;                //出弯又入弯的速度
    
    uint32_t speedK;                        //刹车系数
    uint32_t speedK2;                       //二次公式系数
    uint32_t annulusSpeedK2;                //圆环二次公式系数    
    
    
    uint32_t constantSpeed;                 //匀速
    uint32_t annulusSpeed;                  //圆环速度
    uint32_t annulusMinSpeed;               //圆环最小速度 
    
    //uint32_t rampUpSpeed;		            //过坡道速度
    //uint32_t rampOnSpeed;                 //坡上速度
    //uint32_t rampDownSpeed;               //下坡速度
    uint32_t rampUpSpeed[5];		        //过坡道速度
    uint32_t rampOnSpeed[5];                //坡上速度
    uint32_t rampDownSpeed[5];              //下坡速度
    
    uint32_t adcSpeed;                      //电磁速度(匀速)
    
    uint32_t strightSpeedCut;               //长直道刹车一点点减速
    uint32_t shortStrightSpeedCut;          //短直道刹车一点点减速
    uint32_t exitEnterSpeedCut;             //出弯又入弯一点点减速


	//车库速度
	uint32_t garageSpeed;
} SpeedParam;

typedef struct TrackDectionParam
{
    uint32_t strightAD;              //入直道的判断
    uint32_t enterCurveAD;           //入弯减速的判断
    uint32_t exitCurveAD;            //出弯加速的判断
    uint32_t halfAD;                 //出弯短直道判断   
    uint32_t cancelExitAccAD;         //解除出弯加速判断，用来防止抬轮时误判出弯加速一直加

    uint32_t brakeTime;              //刹车时间
    uint32_t limitTime;              //砍积分时间（没用到）
    
    uint32_t brakeTest;              //刹车类型
    uint32_t brakeTop_1;
    uint32_t brakeTop_2;
    uint32_t brakeTop_3;
    uint32_t brakeTop_4;
    uint32_t brakeTop_0;
    
    uint32_t brakeSpeedTop_1;        //各种刹车时的TOP点
    uint32_t brakeSpeedTop_2;
    uint32_t brakeSpeedTop_3;
    uint32_t brakeSpeedTop_4;
    uint32_t brakeSpeedTop_0;
    
    uint32_t brakeSpeed_1;           //各种刹车的速度
    uint32_t brakeSpeed_2;
    uint32_t brakeSpeed_3;
    uint32_t brakeSpeed_4;
    
    uint32_t exitEnterTop;           //出弯又入弯的top点
    uint32_t exitEnterSpeedTop;      //出弯又入弯的SPEEDTOP点
    uint32_t halfTop;                //出弯短直道的top点
    uint32_t halfSpeedTop;           //出弯短直道的SPEEDTOP点
} TrackDectionParam;

extern SpeedType speedType;
extern BrakeType brakeType;

void speed_init();
void speedParam_init();
void SpeedInfo_Init();
void trackDectionParam_init();
SpeedType getSpeedType();
 uint8_t isBrakeFinished(int aimSpeed);
int getBrakeSpeed();
int getAimSpeed();

#endif