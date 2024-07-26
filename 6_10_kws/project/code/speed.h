#ifndef SPEED_H
#define SPEED_H

#include "headfile.h" 


#define SI  speedInfo
#define SP  speedParam
#define TDP trackDectionParam

typedef struct SpeedInfo
{
    int varL[3];                //������ֵ
    int varR[3];                //������ֵ

    int nowSpeedL;              //����ʵ���ٶ�
    int nowSpeedR;              //����ʵ���ٶ�
    int aimSpeed;	            //Ŀ���ٶ�

    int aimSpeedL;	            //����Ŀ���ٶȣ����ϲ��ٵģ�
    int aimSpeedR;	            //����Ŀ���ٶȣ����ϲ��ٵģ�

    int motorPWML;              //����PWM
    int motorPWMR;              //����PWM

    uint32_t differential;      //���� 
    uint32_t adcDifferential;   //��Ų���
    uint32_t encoderThreshold;  //�������޷�����ֹ̧�ֺͻ���
} SpeedInfo;



typedef enum SpeedType
{
    NORMAL_SHIFT,	    //���ι�ʽ
    FULL_ACCELE,	    //ֱ��ȫ��
    HALF_ACCELE,        //�������  �����Ѿ�������ʱ��
    BRAKE,              //ɲ��
    EXIT_CURVE_ACCELE,	//������٣��ճ��仹û������ʱ��
} SpeedType;

typedef enum BrakeType
{
    STRIGHT_BRAKE,          //��ֱ��ɲ��
    SHORT_STRIGHT_BRAKE,    //��ֱ��ɲ��
    EXIT_ENTER_BRAKE,       //����������ɲ��
} BrakeType;

typedef struct SpeedParam
{
    uint32_t addSpeed;                     //�������һ����

    uint32_t maxSpeed;                      // ֱ������ٶ�   ��ֱ���ٶ�
    uint32_t max2Speed;                     // ���ι�ʽ ����ٶ��޷�
    uint32_t minSpeed;                      // ��С�ٶ� (ȫ����)

    uint32_t curveSpeed;                    //���ι�ʽ���  �����ٶ�
    uint32_t exitSpeed;                     //�����ٶȻ���
    uint32_t maxExitSpeed;                  //������ٵ�����ٶ�
    uint32_t exitEnterSpeed;                //������������ٶ�
    
    uint32_t speedK;                        //ɲ��ϵ��
    uint32_t speedK2;                       //���ι�ʽϵ��
    uint32_t annulusSpeedK2;                //Բ�����ι�ʽϵ��    
    
    
    uint32_t constantSpeed;                 //����
    uint32_t annulusSpeed;                  //Բ���ٶ�
    uint32_t annulusMinSpeed;               //Բ����С�ٶ� 
    
    //uint32_t rampUpSpeed;		            //���µ��ٶ�
    //uint32_t rampOnSpeed;                 //�����ٶ�
    //uint32_t rampDownSpeed;               //�����ٶ�
    uint32_t rampUpSpeed[5];		        //���µ��ٶ�
    uint32_t rampOnSpeed[5];                //�����ٶ�
    uint32_t rampDownSpeed[5];              //�����ٶ�
    
    uint32_t adcSpeed;                      //����ٶ�(����)
    
    uint32_t strightSpeedCut;               //��ֱ��ɲ��һ������
    uint32_t shortStrightSpeedCut;          //��ֱ��ɲ��һ������
    uint32_t exitEnterSpeedCut;             //����������һ������


	//�����ٶ�
	uint32_t garageSpeed;
} SpeedParam;

typedef struct TrackDectionParam
{
    uint32_t strightAD;              //��ֱ�����ж�
    uint32_t enterCurveAD;           //������ٵ��ж�
    uint32_t exitCurveAD;            //������ٵ��ж�
    uint32_t halfAD;                 //�����ֱ���ж�   
    uint32_t cancelExitAccAD;         //�����������жϣ�������ֹ̧��ʱ���г������һֱ��

    uint32_t brakeTime;              //ɲ��ʱ��
    uint32_t limitTime;              //������ʱ�䣨û�õ���
    
    uint32_t brakeTest;              //ɲ������
    uint32_t brakeTop_1;
    uint32_t brakeTop_2;
    uint32_t brakeTop_3;
    uint32_t brakeTop_4;
    uint32_t brakeTop_0;
    
    uint32_t brakeSpeedTop_1;        //����ɲ��ʱ��TOP��
    uint32_t brakeSpeedTop_2;
    uint32_t brakeSpeedTop_3;
    uint32_t brakeSpeedTop_4;
    uint32_t brakeSpeedTop_0;
    
    uint32_t brakeSpeed_1;           //����ɲ�����ٶ�
    uint32_t brakeSpeed_2;
    uint32_t brakeSpeed_3;
    uint32_t brakeSpeed_4;
    
    uint32_t exitEnterTop;           //�����������top��
    uint32_t exitEnterSpeedTop;      //�����������SPEEDTOP��
    uint32_t halfTop;                //�����ֱ����top��
    uint32_t halfSpeedTop;           //�����ֱ����SPEEDTOP��
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