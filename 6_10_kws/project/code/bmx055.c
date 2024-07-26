
#include "bmx055.h"
#include "zf_driver_iic.h"
#include "SEEKFREE_IIC.h"
/////��1�ķ�ʽ����ͨ�� �������߱�Ĳ���
vuint8                 IsGyroOffsetReset = 1;              /////�����Ҫ������������Ʈ�����򽫸ı�����Ϊ   1
BMX055Datatypedef      BMX055_data;
volatile EulerAngleTypedef      SystemAttitude;            /////��̬��
volatile EulerAngleTypedef      SystemAttitudeRate;        /////��̬���ٶ�
volatile AttitudeDatatypedef    GyroOffset;

volatile EulerAngleTypedef firstAngle;
volatile EulerAngleTypedef angle;
extern int buzzerTime;
/**************************************************/
/*��ȫ���Ŀ��Ƽ��ɼ��������ڸ��ж�*/
/*************************************************/

volatile float AccZAngle = 0;

volatile EulerAngleTypedef previousSystemAttitude;
volatile float relativeYaw = 0;

float AD_L_Yaw_angle = 0;//�Ի�ƫ����

//extern AttitudeDatatypedef         Acc;
//extern AttitudeDatatypedef         Gyro;

volatile float myImgAnnulusYaw = 0;    //��Բ���õ�ƫ����
extern uint8_t annulusStartAccumulateFlag;


float subAngle(float op1, float op2)
{
    float angle;
    if (op1 - op2 <= 180 && op1 - op2 >= -180)
    {
        angle = op1 - op2;
    }
    else if (op1 - op2 > 180)
    {
        angle = op1 - op2 - 360;
    }
    else if (op1 - op2 < -180)
    {
        angle = op1 - op2 + 360;
    }      
    return angle;
}


void getBMXData(void)
{
    static uint8_t IsAttitudeinit = 0;
    static uint8_t first = 0;
    static uint8_t num = 0;

    /*****************************************************************/
    /*���Ƿֽ��ߣ���������ƫ����*/
    /******************************************************************/
    
    BMX055_DataRead(&BMX055_data, 0);
    BMX055_data.GYROXdata = (BMX055_data.GYROXdata - GyroOffset.Xdata) * 0.030517578;   
    BMX055_data.GYROYdata = (BMX055_data.GYROYdata - GyroOffset.Ydata) * 0.030517578;
    BMX055_data.GYROZdata = (BMX055_data.GYROZdata - GyroOffset.Zdata) * 0.030517578;
    
    ///////1000 / 32768     //////BMX055������Ʈ�������Ժ��Բ��ƣ����ǰ�ȫ������ǽ���һ��
    BMX055_data.ACCXdata *= 0.001953125;      ///////4 / 2048
    BMX055_data.ACCYdata *= 0.001953125;
    BMX055_data.ACCZdata *= 0.001953125;
    
    
    Acc.Xdata = BMX055_data.ACCXdata;
    Acc.Ydata = BMX055_data.ACCYdata;
    Acc.Zdata = BMX055_data.ACCZdata;
    Gyro.Xdata = BMX055_data.GYROXdata;
    Gyro.Ydata = BMX055_data.GYROYdata;
    Gyro.Zdata = BMX055_data.GYROZdata;

    
    if(IsAttitudeinit == 0)
    {
        Quaternion_init();                    ////��̬�����ʼ��        
        IsAttitudeinit = 1;
    }
    else
    {
        Attitude_UpdateGyro();                /////���ٸ���
        Attitude_UpdateAcc();                 /////����ںϸ���
        
        
        SystemAttitude.Pitch = -EulerAngle.Roll / PI * 180;            ////תΪ�Ƕ�   ������
        SystemAttitude.Roll = EulerAngle.Pitch / PI * 180;             ////������
        SystemAttitude.Yaw = EulerAngle.Yaw / PI * 180;                ////ƫ����
        SystemAttitudeRate.Pitch = -EulerAngleRate.Roll / PI * 180;    ////�������ٶ�
        SystemAttitudeRate.Roll = EulerAngleRate.Pitch / PI * 180;
        SystemAttitudeRate.Yaw = EulerAngleRate.Yaw / PI * 180;        ////ƫ�����ٶ�
        
        AD_L_Yaw_angle = AD_L_Yaw_angle - PERIODS*SystemAttitudeRate.Yaw;//�Ի�ƫ����
        
        relativeYaw += subAngle(SystemAttitude.Yaw, previousSystemAttitude.Yaw);

       /* if(1==annulusStartAccumulateFlag)    //Բ���ۻ�
        {
		    myImgAnnulusYaw += PERIODS*SystemAttitudeRate.Yaw;    //Բ���뻷���ڷ�ֹ���г�����
		    
        }
		else
		{
		    myImgAnnulusYaw=0;
		}*/
		
        float AccZ, AccZAdjust;    
        AccZ = -Acc.Zdata;
        if (AccZ > 1)
        {
            AccZ = 1;
        }
        if (AccZ < -1)
        {
            AccZ = -1;            
        }
        AccZAngle = asinf(AccZ) * 180 / PI;
        AccZAdjust = (AccZAngle - SystemAttitude.Pitch);
        SystemAttitude.Pitch += (-Gyro.Xdata + AccZAdjust) * PERIODS;
        if (first == 0)
        {
            if (num < 100)
            {
                ++num;
                firstAngle.Pitch += SystemAttitude.Pitch;
                firstAngle.Roll += SystemAttitude.Roll;
                firstAngle.Yaw += SystemAttitude.Yaw;
            }
            else
            {
                first = 1;
                firstAngle.Pitch /= 100;
                firstAngle.Roll /= 100;
                firstAngle.Yaw /= 100;
                
                /*
                if (runState == CAR_READY)
                {
                    runState = CAR_RUNNING;
                }
                else if (runState == CAR_STOP)
                {
                    //BEEP(200);
                }
                */
            }
        }
        else
        {
            angle.Pitch = subAngle(SystemAttitude.Pitch, firstAngle.Pitch);    //װ���ˣ�ȡ������
            angle.Roll = subAngle(SystemAttitude.Roll, firstAngle.Roll);
            angle.Yaw =- subAngle(SystemAttitude.Yaw, firstAngle.Yaw);    
        }
        previousSystemAttitude.Pitch = SystemAttitude.Pitch;
        previousSystemAttitude.Roll = SystemAttitude.Roll;
        previousSystemAttitude.Yaw = SystemAttitude.Yaw;
    }
}


void GyroOffset_init(void)      /////////��������Ʈ��ʼ��
{
    GyroOffset.Xdata = 0;
    GyroOffset.Ydata = 0;
    GyroOffset.Zdata = 0;
    for (uint16_t i = 0; i < 2000; ++i)
    {
        BMX055_DataRead(&BMX055_data, 0);
        GyroOffset.Xdata += BMX055_data.GYROXdata;
        GyroOffset.Ydata += BMX055_data.GYROYdata;
        GyroOffset.Zdata += BMX055_data.GYROZdata;
    }
  
    GyroOffset.Xdata /= 2000;
    GyroOffset.Ydata /= 2000;
    GyroOffset.Zdata /= 2000;
    IsGyroOffsetReset = 0;
}


uint8_t BMX055_init(void)
{
    simiic_init();
	system_delay_ms(100);
	
	/************************************/
  	/*���ٶ�����*/
  	/************************************/  	
	uint8 ErrCount = 0; 
  	while(simiic_read_reg(IIC_BMX055_ACC_ADR, BMX055_ACC_ID,SIMIIC) != 0xFA )   /////ȷ��оƬID
  	{
                ErrCount++;
		system_delay_ms(10);
    	if(ErrCount > 5)
     	return 0;
 	} 
	simiic_write_reg(IIC_BMX055_ACC_ADR, BMX055_ACC_PMURANGE, 0x05);   ///4G  3 2G  5 4G  8 8G 
	system_delay_ms(10);
	simiic_write_reg(IIC_BMX055_ACC_ADR, BMX055_ACC_PMUBW, 0x0F);      ///1000HZ      
  	system_delay_ms(10);
  	simiic_write_reg(IIC_BMX055_ACC_ADR, BMX055_ACC_PMULPM, 0x00);     ///Normal MODE      
 	system_delay_ms(10); 
  
  	/************************************/
  	/*����������*/
  	/************************************/
  	ErrCount = 0;
  	while(simiic_read_reg(IIC_BMX055_GYRO_ADR, BMX055_GYRO_ID,SIMIIC) != 0x0F )   /////ȷ��оƬID
  	{
    	ErrCount++;
		system_delay_ms(10);
    	if(ErrCount > 5)
      	return 0;
 	}
  	simiic_write_reg(IIC_BMX055_GYRO_ADR, BMX055_GYRO_RANGE, 0x01);     ///+-1000      
  	system_delay_ms(10);
  	simiic_write_reg(IIC_BMX055_GYRO_ADR, BMX055_GYRO_BW, 0x02);        ///1000HZ      
  	system_delay_ms(10);
  	simiic_write_reg(IIC_BMX055_GYRO_ADR, BMX055_GYRO_LPM, 0x00);       ///Normal MODE      
  	system_delay_ms(10);
  	simiic_write_reg(IIC_BMX055_GYRO_ADR, BMX055_GYRO_RATEHBW, 0x08);   ///��ͨ�˲� �����Բ�Ҫ      
  	system_delay_ms(10);

  	//BMX055OFFSET_init();
  	/************************************/
  	/*����������*/
  	/************************************/
  	ErrCount = 0;
 	simiic_write_reg(IIC_BMX055_MAG_ADR, BMX055_MAG_POM, 0x81);
  	system_delay_ms(10);
  	while(simiic_read_reg(IIC_BMX055_MAG_ADR, BMX055_MAG_ID,SIMIIC) != 0x32 )   /////ȷ��оƬID
  	{
    	ErrCount++;
		system_delay_ms(10);
    	if(ErrCount > 5)
      	return 0;
  	}
  	simiic_write_reg(IIC_BMX055_MAG_ADR, BMX055_MAG_DATARATE, 0x38);   ///�������30HZ       
  	system_delay_ms(10);
  	simiic_write_reg(IIC_BMX055_MAG_ADR, BMX055_MAG_INTEN, 0x00);      ///��ʹ���ж�      
  	system_delay_ms(10);
  	return 1;

}



uint8_t BMX055_DataRead(BMX055Datatypedef *Q, uint8_t type)
{
    uint8 datatemp[6] = {0};
  	simiic_read_regs(IIC_BMX055_GYRO_ADR, BMX055_GYRO_XDATALSB, datatemp, 6, SIMIIC);
  	Q->GYROXdata = (float)((int16)((datatemp[1] << 8) | datatemp[0]));    //-GyroOffset.Xdata;
  	Q->GYROYdata = (float)((int16)((datatemp[3] << 8) | datatemp[2]));    //-GyroOffset.Ydata;
  	Q->GYROZdata = (float)((int16)((datatemp[5] << 8) | datatemp[4]));    //-GyroOffset.Zdata;

	simiic_read_regs(IIC_BMX055_ACC_ADR, BMX055_ACC_XDATALSB, datatemp, 6, SIMIIC);
  	Q->ACCXdata = (float)((int16)((datatemp[1] << 8) | datatemp[0]) >> 4);
  	Q->ACCYdata = (float)((int16)((datatemp[3] << 8) | datatemp[2]) >> 4);
  	Q->ACCZdata = (float)((int16)((datatemp[5] << 8) | datatemp[4]) >> 4);

  	/************************************/
  	/*���������ݶ�ȡ*/
  	/************************************/
  	if(type)
  	{
            simiic_read_regs(IIC_BMX055_MAG_ADR, BMX055_MAG_XDATALSB, datatemp, 6, SIMIIC);
            Q->MAGXdata = (float)((int16)((datatemp[1] << 8) | datatemp[0]) >> 3);
            Q->MAGYdata = (float)((int16)((datatemp[3] << 8) | datatemp[2]) >> 3);
            Q->MAGZdata = (float)((int16)((datatemp[5] << 8) | datatemp[4]) >> 1);

  	}	
  	return 1;
}