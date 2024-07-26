#include "speed.h"

volatile SpeedParam speedParam;
volatile SpeedInfo speedInfo;
SpeedType speedType = FULL_ACCELE;
BrakeType brakeType = STRIGHT_BRAKE;
uint8_t brakeTime = 0;
uint32_t speedUpTime = 0;
uint32_t halfSpeedUpTime = 0;
TrackDectionParam trackDectionParam;

extern  RunSpeedMode runSpeedMode;

extern IMG_FLAGS IF;
extern IMG_FLAGS LAST_IF;
extern IMG_STATUS IS;
extern DeviationVAR  myDeviationVar;

uint8_t limitFlag = 0;
extern int top;
extern AnnulusDEV AD;

extern uint32_t RampCircleNum;
extern EulerAngleTypedef angle;

extern uint32_t myRampReleaseDelay[5];

uint8_t  myQtDebugSpeedType;    //QT�ϵ��ı��ٲ���





void speed_init()
{
    speedParam_init();
    SpeedInfo_Init();
    trackDectionParam_init();
}


void speedParam_init()
{
    SP.addSpeed = 1; 

    SP.maxSpeed =  250;     //270;    ��̥ɲ����������250��������� 
    SP.max2Speed = 220;     //230;
    SP.minSpeed = 200;

    SP.curveSpeed = 230; 
    SP.exitSpeed = 230;   
    SP.maxExitSpeed = 240; 
    SP.exitEnterSpeed = 205;

    //SP.rampUpSpeed = 170; 
    //SP.rampOnSpeed = 165;
   // SP.rampDownSpeed = 163;
    for(int i=0;i<5;i++)
    {
        SP.rampUpSpeed[i]=220;
        SP.rampOnSpeed[i] = 220;
        SP.rampDownSpeed[i] = 180;
		myRampReleaseDelay[i]=5;
    }
    SP.adcSpeed = 200;
    SP.annulusSpeed = 240;
    SP.annulusMinSpeed = 200;
    SP.constantSpeed = 200;
	SP.garageSpeed=175;    //����ǰ���ٶ�

    SP.speedK = 112;    //124; 
    SP.speedK2 = 118;
    SP.annulusSpeedK2 = 120;
	
    SP.strightSpeedCut = 12;
    SP.shortStrightSpeedCut = 10;
    SP.exitEnterSpeedCut = 10;
}

void trackDectionParam_init()
{
    TDP.strightAD = 16 ;       //20;    
    TDP.enterCurveAD = 10;    //12; 
    TDP.exitCurveAD = 23;     //25;  
    TDP.halfAD = 16;          //20;
    TDP.cancelExitAccAD = 100;          //7.25�¼ӵ�̧�����г�����ٴ�����ֵ����������    ������Ǻ���Ҫ
    
    TDP.brakeTime = 7;                 //����ͼ����ʱ����20ms,����ɲ��������С��
    TDP.limitTime = 20;
    
    TDP.brakeTest = 0;
    TDP.brakeTop_1 = 76;        //79;
    TDP.brakeTop_2 = 75;        //78;
    TDP.brakeTop_3 = 73;        //76;
    TDP.brakeTop_4 = 72;        //75;
    TDP.brakeTop_0 = 70;        //73;
    
    TDP.brakeSpeedTop_1 = 75;    //78;
    TDP.brakeSpeedTop_2 = 74;    //77;
    TDP.brakeSpeedTop_3 = 72;    //75;
    TDP.brakeSpeedTop_4 = 70;    //73;
    TDP.brakeSpeedTop_0 = 68;    //71;
    
    TDP.brakeSpeed_1 = 250; //270;
    TDP.brakeSpeed_2 = 240; //260;
    TDP.brakeSpeed_3 = 228; //245;
    TDP.brakeSpeed_4 = 215; //230;
    
    TDP.exitEnterTop = 75;         //78;
    TDP.exitEnterSpeedTop = 73;    //76;
    TDP.halfTop = 76;              //79;
    TDP.halfSpeedTop = 74;         //77;

}

void SpeedInfo_Init()
{
    SI.varL[0] = 0;
    SI.varL[1] = 0;
    SI.varL[2] = 0;
    
    SI.varR[0] = 0;
    SI.varR[1] = 0;
    SI.varR[2] = 0;
    
    SI.nowSpeedL = 0;
    SI.nowSpeedR = 0;
    SI.aimSpeedL = 0;
    SI.aimSpeedR = 0;
    SI.aimSpeed = 0;

    SI.motorPWML = 0;
    SI.motorPWMR = 0;
    SI.differential = 50;    //���ǳ��ӱȽ��أ����ٽo���˲���˳���ĸо�
    SI.adcDifferential = 56;
    SI.encoderThreshold = 70;
}


int getAimSpeed()
{
    if(RampCircleNum>5)    //�µ��ٶ� ���� ѭ������
        RampCircleNum=5;

    int aimSpeed;
    SpeedType runRunSpeedType;
	
    if(runSpeedMode == CONSTANT_SPEED)                 //����
    {
        //���趨�ٶ�С���µ��ٶȵ�ʱ��ֱ�����趨���ٶȣ���Ȼ�������ºͳ�����ʱ��ɼ�����
        if(SP.constantSpeed<SP.rampUpSpeed[(judgedRampNum-1)%RampCircleNum])
        {
            return SP.constantSpeed;
        }
        else
        {
            if(IF.ramp)
            {
                if(1==IF.ramp)
                {
                    aimSpeed = SP.rampUpSpeed[(judgedRampNum-1)%RampCircleNum];
                }
                else if(2==IF.ramp)
                {
                    aimSpeed = SP.rampOnSpeed[(judgedRampNum-1)%RampCircleNum];
                }
                else if(3==IF.ramp)
                {               
                    aimSpeed = SP.rampDownSpeed[(judgedRampNum-1)%RampCircleNum];
                }

                return aimSpeed;
            }
	        else if(!IF.ramp && (angle.Pitch>5 || angle.Pitch<-6))    //1.18�¼� ��Ϊһ�����¶��� ��ǰ������״̬�����Լ�һ���±�־�����Ƕ���һ����Χ��ʱʹ��
	        {
               aimSpeed = SP.rampDownSpeed[(judgedRampNum-1)%RampCircleNum];   //Ҳ�������ٶȾ�����  ��������´���ȥ�����⣬��Ϊ���±����ˣ������гɳ�ֱ����
	        }
			else if(1==IF.cheku || 2==IF.cheku || 3==IF.cheku || 4==IF.cheku)    //�ȸ����̶��ٶ�
			{
			    aimSpeed = SP.garageSpeed;    
			}
			else if(5==IF.cheku || 6==IF.cheku)    //��״̬ɲ��
			{
			    //aimSpeed = 0; 
			    aimSpeed = SP.garageSpeed; 
			}
            else
               return SP.constantSpeed;
        }
    }
    else if(runSpeedMode == ADC_SPEED)                 //ADC
    {
        return SP.adcSpeed;
    }
    else if(runSpeedMode == NORMAL_SHIFT_SPEED)        //���ι�ʽ
    {
        if(IF.ramp)
        {
            if(1==IF.ramp)
            {
                aimSpeed = SP.rampUpSpeed[(judgedRampNum-1)%RampCircleNum];
            }
            else if(2==IF.ramp)
            {
                aimSpeed = SP.rampOnSpeed[(judgedRampNum-1)%RampCircleNum];
            }
            else if(3==IF.ramp)
            {               
                aimSpeed = SP.rampDownSpeed[(judgedRampNum-1)%RampCircleNum];
            }
			
            return aimSpeed;
        }
		else if(1==IF.cheku || 2==IF.cheku || 3==IF.cheku || 4==IF.cheku)    //�ȸ����̶��ٶ�
		{
			aimSpeed = SP.garageSpeed;    
		}
		else if(5==IF.cheku || 6==IF.cheku)    //��״̬ɲ��
		{
			//aimSpeed = 0;    
			aimSpeed = SP.garageSpeed; 
		}
	    else if(!IF.ramp && (angle.Pitch>5 || angle.Pitch<-6))    //1.18�¼� ��Ϊһ�����¶��� ��ǰ������״̬�����Լ�һ���±�־�����Ƕ���һ����Χ��ʱʹ��
	    {
            aimSpeed = SP.rampDownSpeed[(judgedRampNum-1)%RampCircleNum];  //Ҳ�������ٶȾ�����  ��������´���ȥ�����⣬��Ϊ���±����ˣ������гɳ�ֱ����
	    }
        else
            runRunSpeedType = NORMAL_SHIFT;
    }
    else if(runSpeedMode == VARIABLE_SPEED)            //����
    {
        //����
		runRunSpeedType = getSpeedType();
    }

	myQtDebugSpeedType = (uint8_t)runRunSpeedType;    //�ŵ�qt��
	

    switch(runRunSpeedType)
    {
    case NORMAL_SHIFT:                       //���ι�ʽ
    {
        int referenceSpeed;
        int minSpeed;
        float speedK2;
        if(IF.annulus)    //Բ��
        {
            referenceSpeed = SP.annulusSpeed;
            minSpeed = SP.annulusMinSpeed;
            speedK2 = (float)SP.annulusSpeedK2;

			aimSpeed = (int)(referenceSpeed - (float)(referenceSpeed - minSpeed)
                             * myDeviationVar.DK * myDeviationVar.DK / speedK2 / speedK2);    //Բ�����뱣��,��ͬԲ����ͬ�ٶ�
        }
        else    //��ͨ���
        {
            referenceSpeed = SP.curveSpeed;
            minSpeed = SP.minSpeed;
            speedK2 =(float)SP.speedK2;
			
			//aimSpeed = SP.constantSpeed;    //����normalshifit��,·��̫����,�ĳ�����,����������ٺͳ�����ٻ᲻���������,

			aimSpeed = (int)(referenceSpeed - (float)(referenceSpeed - minSpeed)
                             * myDeviationVar.DK * myDeviationVar.DK / speedK2 / speedK2);    //Բ�����뱣��,��ͬԲ����ͬ�ٶ�
        }
        //aimSpeed = (int)(referenceSpeed - (float)(referenceSpeed - minSpeed)
        //                     * myDeviationVar.DK * myDeviationVar.DK / speedK2 / speedK2);

		//7.19����״̬2����ô�Բ������ȥ
        if(AL1==IF.annulus || AR1==IF.annulus
			|| AL2==IF.annulus || AR2==IF.annulus) //�뻷ʱ����С�ٶ�   
        {
            aimSpeed = SP.annulusMinSpeed;
        }

        if(aimSpeed < minSpeed)
        {
            aimSpeed = minSpeed;
        }
        if(aimSpeed > SP.max2Speed)
        {
            aimSpeed = SP.max2Speed;
        }   
        break;
    }
    case FULL_ACCELE:                      //ȫ�� ��ֱ��
    {
        aimSpeed = SP.maxSpeed;
        break;
    }
    case BRAKE:                            //ɲ��
    {   
        aimSpeed = getBrakeSpeed();
        if(isBrakeFinished(aimSpeed))     //�Ƿ����
        {
            ++brakeTime;
        }
        if(brakeTime >= TDP.brakeTime)    //ɲ�공�ű��normalshift
        {
            brakeTime = 0;
            speedType = NORMAL_SHIFT;
        }
        break;
    }
    case EXIT_CURVE_ACCELE:                //�������  �ճ���ʱ
    {
        aimSpeed = (SI.aimSpeed + SP.addSpeed) < SP.exitSpeed
                       ? (SI.aimSpeed + SP.addSpeed)
                       : SP.exitSpeed;
        break;
    }
    case HALF_ACCELE:                      //�������  ��ȫ���䳵�����ʱ
    {
        aimSpeed = SP.maxExitSpeed;
        break;
    }
    default:
        break;
    }

    if(IF.ramp)
    {
        if(1==IF.ramp)
        {
            aimSpeed = SP.rampUpSpeed[(judgedRampNum-1)%RampCircleNum];
        }
        else if(2==IF.ramp)
        {
            aimSpeed = SP.rampOnSpeed[(judgedRampNum-1)%RampCircleNum];
        }
        else if(3==IF.ramp)
        {
            aimSpeed = SP.rampDownSpeed[(judgedRampNum-1)%RampCircleNum];
        }
    }
	else if(1==IF.cheku || 2==IF.cheku || 3==IF.cheku || 4==IF.cheku)    //�ȸ����̶��ٶ�
	{
		aimSpeed = SP.garageSpeed;	  
	}
	else if(5==IF.cheku || 6==IF.cheku)    //��״̬ɲ��
	{
		//aimSpeed = 0;
		aimSpeed = SP.garageSpeed; 
    }
	else if(!IF.ramp && (angle.Pitch>5 || angle.Pitch<-6))    //1.18�¼� ��Ϊһ�����¶��� ��ǰ������״̬�����Լ�һ���±�־�����Ƕ���һ����Χ��ʱʹ��
    {
	    aimSpeed = SP.rampDownSpeed[(judgedRampNum-1)%RampCircleNum];    //Ҳ�������ٶȾ�����  ��������´���ȥ�����⣬��Ϊ���±����ˣ������гɳ�ֱ����
    }
    else if(IF.annulusDelay)   //��Բ��ʱ����û������������Ҫ����һ��
    {
        float DK = AD.sumDK / AD.cnt;
        int annulusSpeed = (int)(SP.annulusSpeed - (SP.annulusSpeed - SP.annulusMinSpeed)
                             * DK * DK / SP.annulusSpeedK2 / SP.annulusSpeedK2);
        annulusSpeed = annulusSpeed > SP.annulusMinSpeed ? annulusSpeed : SP.annulusMinSpeed;
        aimSpeed = aimSpeed < annulusSpeed ? aimSpeed : annulusSpeed;
    }
 
    int maxSpeed = SP.maxSpeed;
    
    switch(TDP.brakeTest)              //�������� ��ֱ������ɲ������ٶȵ�
    {
    case 1:
        maxSpeed = TDP.brakeSpeed_1;
        break;
    case 2:
        maxSpeed = TDP.brakeSpeed_2;
        break;
    case 3:
        maxSpeed = TDP.brakeSpeed_3;
        break;
    case 4:
        maxSpeed = TDP.brakeSpeed_4;
        break;
    default:
        break;
    }
    
    if(aimSpeed > maxSpeed)    //��������ٶ�ʱ���޷�
    {
        aimSpeed = maxSpeed;
    }
	
    return aimSpeed;
    
}



SpeedType getSpeedType()
{

    /************************** �ٶȹ滮 ******************************/
    int maxSpeed = SI.varL[0] > SI.varR[0] ? SI.varL[0] : SI.varR[0];   //��ǰ�����������ٶȴ���Ǹ�
    
    if(IF.annulus)
    {
        speedType = NORMAL_SHIFT;
        speedUpTime = 0;
        halfSpeedUpTime = 0;
        if(IF.annulus && LAST_IF.annulus == 0)
        {
            limitFlag = 1;
        }
    }
    else if(IF.ramp)
    {
        speedType = EXIT_CURVE_ACCELE; 
        speedUpTime = 0;   
        halfSpeedUpTime = 0;
        if(IF.ramp && LAST_IF.ramp == 0)
        {
            limitFlag = 1;
        }
    }   
	else if(IF.rampDelay)
	{
	    speedType = NORMAL_SHIFT;    //rampDelayʱ����normalshift,�������Ը�����г������
	    speedUpTime = 0;   
        halfSpeedUpTime = 0;
	}
    /*
    else if (IF.obstacle)
    {
        if (GET_SWITCH4())
        {
            speedType = NORMAL_SHIFT;
        }
        else
        {
            speedType = EXIT_CURVE_ACCELE; 
        }
        speedUpTime = 0;     
        halfSpeedUpTime = 0;
        if(IF.obstacle && LAST_IF.obstacle == 0) 
        {
            limitFlag = 1;
        }
    }
  */
    else if(IS.annulusTop <= YY && speedType == NORMAL_SHIFT)
    {
        speedType = NORMAL_SHIFT;
        speedUpTime = 0;
        halfSpeedUpTime = 0;
    }
    else if(top >= YY && IS.speedTop >= YM && IS.annulusTop >= YM && 
              myDeviationVar.absDeviation[0] <= TDP.strightAD &&
              speedType != BRAKE
              && 0==IF.rampDelay)    //7.27�¼ӵķ�ֹ������ǰ������г�ֱ
    {
        ++speedUpTime;      
    }
    else if((top <= TDP.brakeTop_1 || IS.speedTop <= TDP.brakeSpeedTop_1 || IS.annulusTop <= YY ||
              myDeviationVar.absDeviation[0] >= TDP.strightAD) &&
              (speedType == FULL_ACCELE || speedType == HALF_ACCELE) && 
              maxSpeed >= TDP.brakeSpeed_1 && 
              TDP.brakeTest <= 1)
    {
        speedType = BRAKE; 
        brakeType = STRIGHT_BRAKE;
        speedUpTime = 0;
        halfSpeedUpTime = 0;
        limitFlag = 1;
    }
    else if((top <= TDP.brakeTop_2 || IS.speedTop <= TDP.brakeSpeedTop_2 || IS.annulusTop <= YY || 
              myDeviationVar.absDeviation[0] >= TDP.strightAD) &&
              (speedType == FULL_ACCELE || speedType == HALF_ACCELE) && 
              maxSpeed >= TDP.brakeSpeed_2 &&
              TDP.brakeTest <= 2)
    {
        speedType = BRAKE; 
        brakeType = STRIGHT_BRAKE;
        speedUpTime = 0;
        halfSpeedUpTime = 0;
        limitFlag = 1;
    }
    else if((top <= TDP.brakeTop_3 || IS.speedTop <= TDP.brakeSpeedTop_3 || IS.annulusTop <= YY ||
              myDeviationVar.absDeviation[0] >= TDP.strightAD) &&
              (speedType == FULL_ACCELE || speedType == HALF_ACCELE) && 
              maxSpeed >= TDP.brakeSpeed_3 &&
              TDP.brakeTest <= 3)
    {
        speedType = BRAKE;
        brakeType = STRIGHT_BRAKE;
        speedUpTime = 0;
        halfSpeedUpTime = 0;
        limitFlag = 1;
    }
    else if((top <= TDP.brakeTop_4 || IS.speedTop <= TDP.brakeSpeedTop_4 || IS.annulusTop <= YY ||
              myDeviationVar.absDeviation[0] >= TDP.strightAD) &&
              (speedType == FULL_ACCELE || speedType == HALF_ACCELE) &&
              maxSpeed >= TDP.brakeSpeed_4 &&
              TDP.brakeTest <= 4)
    {
        speedType = BRAKE; 
        brakeType = STRIGHT_BRAKE;
        speedUpTime = 0;
        halfSpeedUpTime = 0;
        limitFlag = 1;
    }
    else if((top <= TDP.brakeTop_0 || IS.speedTop <= TDP.brakeSpeedTop_0 || IS.annulusTop <= YY ||
             myDeviationVar.absDeviation[0] >= TDP.strightAD) &&
             (speedType == FULL_ACCELE || speedType == HALF_ACCELE))
    {
        speedType = BRAKE; 
        brakeType = STRIGHT_BRAKE;
        speedUpTime = 0;
        halfSpeedUpTime = 0;
        limitFlag = 1;
    }
    else if(myDeviationVar.absDeviation[0] <= TDP.exitCurveAD &&
             myDeviationVar.diffDeviation[0] < 0 && myDeviationVar.diffDeviation[1] < 0 &&
             myDeviationVar.diffDeviation[2] < 0 && speedType == NORMAL_SHIFT)
    {
        speedType = EXIT_CURVE_ACCELE; 
        speedUpTime = 0;
        halfSpeedUpTime = 0;
    }   
    else if(top >= TDP.halfTop && IS.speedTop >= TDP.halfSpeedTop && 
             myDeviationVar.absDeviation[0] <= TDP.halfAD &&
             speedType != FULL_ACCELE && speedType != BRAKE
             && 0==IF.rampDelay)     //7.27�¼ӵķ�ֹ������ǰ������г�ֱ
    {
        ++halfSpeedUpTime;
    }
    else if((IS.annulusTop <= YY || ((top <= TDP.exitEnterTop || IS.speedTop <= TDP.exitEnterSpeedTop || 
             myDeviationVar.absDeviation[0] >= TDP.enterCurveAD) &&
             ((myDeviationVar.diffDeviation[0] > 0 && myDeviationVar.diffDeviation[1] > 0 &&
             myDeviationVar.diffDeviation[2] > 0) ||
             (((myDeviationVar.diffDeviation[0] > 0 && myDeviationVar.diffDeviation[1] > 0) ||
             (myDeviationVar.diffDeviation[1] > 0 && myDeviationVar.diffDeviation[2] > 0)) &&
             myDeviationVar.diffDeviation[0] + myDeviationVar.diffDeviation[1] + myDeviationVar.diffDeviation[2] > 3)))) && 
             speedType == EXIT_CURVE_ACCELE &&
             maxSpeed >= SP.exitEnterSpeed)
    {
        speedType = BRAKE; 
        brakeType = EXIT_ENTER_BRAKE;
        speedUpTime = 0;
        halfSpeedUpTime = 0;
        limitFlag = 1;
    }
    else if((IS.annulusTop <= YY || (myDeviationVar.absDeviation[0] >= TDP.enterCurveAD &&
             ((myDeviationVar.diffDeviation[0] > 0 && myDeviationVar.diffDeviation[1] > 0 &&
             myDeviationVar.diffDeviation[2] > 0) ||
             (((myDeviationVar.diffDeviation[0] > 0 && myDeviationVar.diffDeviation[1] > 0) ||
             (myDeviationVar.diffDeviation[1] > 0 && myDeviationVar.diffDeviation[2] > 0)) &&
             myDeviationVar.diffDeviation[0] + myDeviationVar.diffDeviation[1] + myDeviationVar.diffDeviation[2] > 3)))) && 
             speedType == EXIT_CURVE_ACCELE)
    {
        speedType = BRAKE; 
        brakeType = EXIT_ENTER_BRAKE;
        speedUpTime = 0;
        halfSpeedUpTime = 0;
        limitFlag = 1;
    }
	//7.25 �¼ӷ�ֹ̧��һֱ�ڳ������    ���normalshift���ǳ������������Կ���
	
	else if(speedType == EXIT_CURVE_ACCELE && myDeviationVar.absDeviation[0] >= TDP.cancelExitAccAD)
	{
		speedType = BRAKE; 
		brakeType = EXIT_ENTER_BRAKE;
		speedUpTime = 0;
		halfSpeedUpTime = 0;
	    limitFlag = 1;

	}
	
    /******************************************************************/
    if(speedUpTime >= 3 && 0==IF.rampDelay)    //7.27�¼ӵķ�ֹ������ǰ������г�ֱ
    {
        speedType = FULL_ACCELE;
    }
    else if(halfSpeedUpTime >= 1 && 0==IF.rampDelay)    //7.27�¼ӵķ�ֹ������ǰ������г�ֱ
    {
        speedType = HALF_ACCELE;
    }
  
    return speedType;

}

int getBrakeSpeed()
{
    int brakeSpeed;
    int speedCut;

    switch(brakeType)
    {
    case STRIGHT_BRAKE:         
        speedCut = SP.strightSpeedCut;
        break;
    case SHORT_STRIGHT_BRAKE:     
        speedCut = SP.shortStrightSpeedCut;
        break;
    case EXIT_ENTER_BRAKE:     
        speedCut = SP.exitEnterSpeedCut;
        break;
    default:
        break;
    }
	
    brakeSpeed = (int)(SP.curveSpeed - (SP.curveSpeed - SP.minSpeed)
                               * myDeviationVar.DK * myDeviationVar.DK / SP.speedK / SP.speedK - speedCut);
	
    if(brakeSpeed + speedCut < SP.minSpeed)
    {
        brakeSpeed = SP.minSpeed - speedCut;
    }
    if(brakeSpeed + speedCut > SP.max2Speed)
    {
        brakeSpeed = SP.max2Speed - speedCut;
    }

    return brakeSpeed;

}

uint8_t isBrakeFinished(int aimSpeed)
{

    return ((SI.varL[0] > SI.varR[0] ? SI.varL[0] : SI.varR[0]) <= aimSpeed);

}
