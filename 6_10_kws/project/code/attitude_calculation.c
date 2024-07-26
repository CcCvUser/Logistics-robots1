/*********************************************************************************************************************
 *
 * @file       		Attitude_Calculation.c
 * @author     		Alex
 * @version    		v1.0
 * @Software 		IAR 8.1
 * @date       		2016-11-9
 ********************************************************************************************************************/

#include "headfile.h"
#include "Attitude_Calculation.h"

AttitudeDatatypedef         Acc;
AttitudeDatatypedef         Gyro;


#define  XA    (-Acc.Xdata)
#define  YA     (Acc.Zdata)  
#define  ZA    (-Acc.Ydata)

#define  XG    (Gyro.Xdata)
#define  YG   (-Gyro.Zdata)
#define  ZG   (Gyro.Ydata)


#define  ATTITUDE_COMPENSATE_LIMIT   ((float)1 / 180 * PI / PERIODHZ)


QuaternionTypedef    Quaternion;
EulerAngleTypedef    EulerAngle;
QuaternionTypedef    AxisAngle;
EulerAngleTypedef    EulerAngleRate;

QuaternionTypedef    MeaQuaternion;
EulerAngleTypedef    MeaEulerAngle;
QuaternionTypedef    MeaAxisAngle;

QuaternionTypedef    ErrQuaternion;
EulerAngleTypedef    ErrEulerAngle;
QuaternionTypedef    ErrAxisAngle;


float XN, XE, XD;
float YN, YE, YD;
float ZN, ZE, ZD;


void Quaternion_Normalize(QuaternionTypedef *Qu)       //��Ԫ����һ��
{
	float Normal = 0;
	Normal = sqrtf(Qu->W * Qu->W + Qu->X * Qu->X + Qu->Y * Qu->Y + Qu->Z * Qu->Z);
	if (isnan(Normal) || Normal <= 0)
	{
		Qu->W = 1;
		Qu->X = 0;
		Qu->Y = 0;
		Qu->Z = 0;
	}
	else
	{
		Qu->W /= Normal;
		Qu->X /= Normal;
		Qu->Y /= Normal;
		Qu->Z /= Normal;
	}
}


// ��Ԫ������, p=q^-1=q*/(|q|^2), ԭʼ��Ԫ����Ϊ��λ��Ԫ��
void Quaternion_Invert(QuaternionTypedef *p, const QuaternionTypedef *q)
{
	p->W = q->W;
	p->X = -q->X;
	p->Y = -q->Y;
	p->Z = -q->Z;
}


// ��Ԫ���˷�, result=pq
void Quaternion_Multi(QuaternionTypedef *result, const QuaternionTypedef *p, const QuaternionTypedef *q)
{
	result->W = p->W * q->W - p->X * q->X - p->Y * q->Y - p->Z * q->Z;
	result->X = p->W * q->X + p->X * q->W - p->Y * q->Z + p->Z * q->Y;
	result->Y = p->W * q->Y + p->X * q->Z + p->Y * q->W - p->Z * q->X;
	result->Z = p->W * q->Z - p->X * q->Y + p->Y * q->X + p->Z * q->W;
}



void Quaternion_ToEulerAngle(const QuaternionTypedef *q, EulerAngleTypedef *e)  //��Ԫ��תŷ����
{
	e->Roll = atan2f(2 * (q->W * q->X + q->Y * q->Z), 1 - 2 * (q->X * q->X + q->Y * q->Y));
	float k = 2 * (q->W * q->Y - q->Z * q->X);
	if (k > 1) k = 1;
	else if (k < -1) k = -1;
	e->Pitch = asinf(k);
	e->Yaw = atan2f(2 * (q->W * q->Z + q->X * q->Y), 1 - 2 * (q->Y * q->Y + q->Z * q->Z));
}


void Quaternion_FromEulerAngle(QuaternionTypedef *q, const EulerAngleTypedef *e) //ŷ����ת��Ԫ��
{
	float cosx = cosf(e->Roll / 2);
	float sinx = sinf(e->Roll / 2);
	float cosy = cosf(e->Pitch / 2);
	float siny = sinf(e->Pitch / 2);
	float cosz = cosf(e->Yaw / 2);
	float sinz = sinf(e->Yaw / 2);
	q->W = cosx * cosy * cosz + sinx * siny * sinz;
	q->X = sinx * cosy * cosz - cosx * siny * sinz;
	q->Y = cosx * siny * cosz + sinx * cosy * sinz;
	q->Z = cosx * cosy * sinz - sinx * siny * cosz;
}

void Quaternion_ToAxisAngle(const QuaternionTypedef *q, QuaternionTypedef *a) //��Ԫ��ת���
{
	a->W = 0;
	a->X = q->X;
	a->Y = q->Y;
	a->Z = q->Z;
	Quaternion_Normalize(a);
	a->W = acosf(q->W) * 2;
}

// ���ת��Ϊ��Ԫ��
void Quaternion_FromAxisAngle(QuaternionTypedef *q, const QuaternionTypedef *a)
{
	float c = cosf(a->W / 2);
	float s = sinf(a->W / 2);
	q->W = 0;
	q->X = a->X;
	q->Y = a->Y;
	q->Z = a->Z;
	Quaternion_Normalize(q);
	q->W = c;
	q->X *= s;
	q->Y *= s;
	q->Z *= s;
}



//���ٶ�����ת����Ԫ������
void Quaternion_FromGyro(QuaternionTypedef *q, float wx, float wy, float wz, float dt)
{
	q->W = 1;
	q->X = wx * dt / 2;
	q->Y = wy * dt / 2;
	q->Z = wz * dt / 2;
	Quaternion_Normalize(q);
}


// ʹ�ý��ٶȸ�����Ԫ��(һ�����������Runge-Kunta, �ȼ���ʹ�ýǶ�����)
void Quaternion_UpdateFromGyro(QuaternionTypedef *q, float x, float y, float z, float dt)
{
	float dW = 0.5 * (-q->X * x - q->Y * y - q->Z * z) * dt;
	float dX = 0.5 * (q->W * x + q->Y * z - q->Z * y) * dt;
	float dY = 0.5 * (q->W * y - q->X * z + q->Z * x) * dt;
	float dZ = 0.5 * (q->W * z + q->X * y - q->Y * x) * dt;
	q->W += dW;
	q->X += dX;
	q->Y += dY;
	q->Z += dZ;
	Quaternion_Normalize(q);
}



void QuaternionFromAcc(QuaternionTypedef *Qu, float ax, float ay, float az, float mx, float my, float mz)//�ɼ����ٶȼƻ����Ԫ��
{
	float Normal = 0;
	XD = ax;
	YD = ay;
	ZD = az;                                              //ȡ��������
	Normal = sqrtf(XD * XD + YD * YD + ZD * ZD);
	XD /= Normal;
	YD /= Normal;
	ZD /= Normal;                                         //��һ��


	XN = -mx; YN = -my; ZN = -mz;                         //ȡ����������Ϊ��
	Normal = XD * XN + YD * YN + ZD * ZN;                 //��
	XN -= Normal*XD;
	YN -= Normal*YD;
	ZN -= Normal*ZD;                                      //������
	Normal = sqrtf(XN * XN + YN * YN + ZN * ZN);
	XN /= Normal;
	YN /= Normal;
	ZN /= Normal;                                         //��һ��

	XE = YD*ZN - YN*ZD;                                   //��//������
	YE = ZD*XN - ZN*XD;
	ZE = XD*YN - XN*YD;
	Normal = sqrtf(XE*XE + YE*YE + ZE*ZE);
	XE /= Normal;
	XE /= Normal;
	XE /= Normal;                                         //��һ��

	Qu->W = 0.5 * sqrtf(XN + YE + ZD + 1);                //��ת����ת����Ԫ��
	Qu->X = (YD - ZE) / (4 * Qu->W);
	Qu->Y = (ZN - XD) / (4 * Qu->W);
	Qu->Z = (XE - YN) / (4 * Qu->W);
	Quaternion_Normalize(Qu);
	return;
}


void Quaternion_init()
{
	if (XA != 0 || YA != 0 || ZA != 0)
	{
		QuaternionFromAcc(&Quaternion, XA, YA, ZA, -1, 0, 0);
	}
	else
	{
		Quaternion.W = 1;
		Quaternion.X = 0;
		Quaternion.Y = 0;
		Quaternion.Z = 0;
	}
	Quaternion_ToEulerAngle(&Quaternion, &EulerAngle);
	Quaternion_ToAxisAngle(&Quaternion, &AxisAngle);
	EulerAngleRate.Pitch = 0;
	EulerAngleRate.Roll = 0;
	EulerAngleRate.Yaw = 0;
}


void Attitude_UpdateAcc(void)            //����ںϸ���
{
	QuaternionTypedef    EstQuaternion;
	EulerAngleTypedef    EstEulerAngle;
	QuaternionTypedef    DivQuaternion;
	QuaternionTypedef    ComAxisangle;
	QuaternionTypedef    Compensate;
	QuaternionTypedef    Last;


	QuaternionFromAcc(&MeaQuaternion, 0, YA, ZA, -1, 0, 0);
	Quaternion_ToEulerAngle(&MeaQuaternion, &MeaEulerAngle);
	Quaternion_ToAxisAngle(&MeaQuaternion, &MeaAxisAngle);      //���㵱ǰ���ٶȼ���̬

	EstEulerAngle.Roll = EulerAngle.Roll;
	EstEulerAngle.Pitch = EulerAngle.Pitch;
	EstEulerAngle.Yaw = 0;

	Quaternion_FromEulerAngle(&EstQuaternion, &EstEulerAngle);  //����ŷ����ת��Ԫ��

	//�������������Ԫ��ƫ��
	Quaternion_Invert(&DivQuaternion, &EstQuaternion);
	Quaternion_Multi(&ErrQuaternion, &DivQuaternion, &MeaQuaternion);
	Quaternion_Normalize(&ErrQuaternion);
	Quaternion_ToEulerAngle(&ErrQuaternion, &ErrEulerAngle);
	Quaternion_ToAxisAngle(&ErrQuaternion, &ErrAxisAngle);


	//���У���޷�

	memcpy(&ComAxisangle, &ErrAxisAngle, sizeof(QuaternionTypedef));
	if (ComAxisangle.W > ATTITUDE_COMPENSATE_LIMIT)
	{
		ComAxisangle.W = ATTITUDE_COMPENSATE_LIMIT;
	}
	Quaternion_FromAxisAngle(&Compensate, &ComAxisangle);


	//ִ��У��

	memcpy(&Last, &EstQuaternion, sizeof(QuaternionTypedef));
	Quaternion_Multi(&EstQuaternion, &Last, &Compensate);


	Quaternion_ToEulerAngle(&EstQuaternion, &EstEulerAngle);
	EstEulerAngle.Yaw = EulerAngle.Yaw;                //��ʹ�ü��ٶȼƲ�ƫ����
	Quaternion_FromEulerAngle(&Quaternion, &EstEulerAngle);
	Quaternion_ToEulerAngle(&Quaternion, &EulerAngle);
	Quaternion_ToAxisAngle(&Quaternion, &AxisAngle);
}


void Attitude_UpdateGyro(void)  //���ٸ���
{

	QuaternionTypedef   g1, tmp;
	EulerAngleTypedef   LastEulerAngle;
	QuaternionTypedef   LastQuanternion;

	//������һ�ε�ŷ���Ǻ���Ԫ��
	memcpy(&LastEulerAngle, &EulerAngle, sizeof(EulerAngleTypedef));
	memcpy(&LastQuanternion, &Quaternion, sizeof(QuaternionTypedef));

	//������̬����
	float gx = XG / 180 * PI;
	float gy = YG / 180 * PI;
	float gz = ZG / 180 * PI;

	Quaternion_UpdateFromGyro(&Quaternion, gx, gy, gz, PERIODS);
	Quaternion_ToEulerAngle(&Quaternion, &EulerAngle);
	Quaternion_ToAxisAngle(&Quaternion, &AxisAngle);

	// ����ŷ�����ٶ�
	// YawΪƫ�����ٶ�,Ϊ��NED�е�D��(Z��)��ת�Ľ��ٶ�,ʹ����Ԫ������

	g1.W = 0; g1.X = gx; g1.Y = gy; g1.Z = gz;

	Quaternion_Invert(&LastQuanternion, &LastQuanternion);
	Quaternion_Multi(&tmp, &LastQuanternion, &g1);
	Quaternion_Invert(&LastQuanternion, &LastQuanternion);
	Quaternion_Multi(&g1, &tmp, &LastQuanternion);

	EulerAngleRate.Yaw = g1.Z;
	//PitchΪ�������ٶ�, Ϊ��Y����ת�Ľ��ٶ�, ʹ��???����
	if (fabs(LastEulerAngle.Pitch - EulerAngle.Pitch) < PI / 2)
	{
		EulerAngleRate.Pitch = EulerAngle.Pitch - LastEulerAngle.Pitch;
	}
	else if (EulerAngle.Pitch - LastEulerAngle.Pitch > PI / 2)
	{
		EulerAngleRate.Pitch = -PI + (EulerAngle.Pitch - LastEulerAngle.Pitch);
	}
	else if (EulerAngle.Pitch - LastEulerAngle.Pitch < -PI / 2)
	{
		EulerAngleRate.Pitch = PI + (EulerAngle.Pitch - LastEulerAngle.Pitch);
	}

	EulerAngleRate.Pitch /= PERIODS;
	//RollΪ������ٶ�, ��X''����ת�Ľ��ٶ�, ֱ��ʹ������������
	EulerAngleRate.Roll = gx;
}

//static const float Q_angle=0.005,Q_gyro=0.001,R_angle=40,dt=0.005;
//static float P[2][2] = {
//    { 1, 0 },
//    { 0, 1 }
//};
//static float Pdot[4] ={0,0,0,0};
//static const char C_0 = 1;
//static float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
////******************************************************************************
////      ������Filter
////******************************************************************************
//void Kalman_Filter(float angle_m,float gyro_m)          //gyro_m:gyro_measure
//{
//    attitudeTemp.angle+=(gyro_m-q_bias) * dt;
//    
//    Pdot[0]=Q_angle - P[0][1] - P[1][0];
//    Pdot[1]=- P[1][1];
//    Pdot[2]=- P[1][1];
//    Pdot[3]=Q_gyro;
//    
//    P[0][0] += Pdot[0] * dt;
//    P[0][1] += Pdot[1] * dt;
//    P[1][0] += Pdot[2] * dt;
//    P[1][1] += Pdot[3] * dt;
//    
//    angle_err = angle_m - attitudeTemp.angle;
//    
//    PCt_0 = C_0 * P[0][0];
//    PCt_1 = C_0 * P[1][0];
//    
//    E = R_angle + C_0 * PCt_0;
//    
//    K_0 = PCt_0 / E;
//    K_1 = PCt_1 / E;
//    
//    t_0 = PCt_0;
//    t_1 = C_0 * P[0][1];
//
//    P[0][0] -= K_0 * t_0;
//    P[0][1] -= K_0 * t_1;
//    P[1][0] -= K_1 * t_0;
//    P[1][1] -= K_1 * t_1;
//    
//    
//    attitudeTemp.angle   += K_0 * angle_err;
//    q_bias  += K_1 * angle_err;
//    attitudeTemp.angle_dot = gyro_m-q_bias;
//}
/*******************************************************************************
* Function Name  : Kalman_Filter
* Description    : �������˲�
* Input          : None
* Output         : None
* Return         : None

Q:����������Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵��ϵͳ����
R:����������R���󣬶�̬��Ӧ�����������ȶ��Ա��
ResrcData:	ADC�ɼ�������

ע��	1	A=1��	��������U(k)=0;	
		2	���˸о�float�㹻���ˣ�double̫�˷��ˣ�
   			����û��о����Ȳ��������Ը�Ϊdouble

*******************************************************************************/

/*******************************************************************************
		�������˲���������Ĺ�ʽ����
NO.1	Ԥ�����ڵ�״̬,����ϵͳ���
	x(k|k-1)=A*x(k-1|k-1)+B*U(k)
	x(k|k-1):������һ״̬Ԥ��Ľ��
	x(k-1|k-1):��һʱ�̵�����Ԥ��ֵ
	U(k):����״̬�Ŀ����������û�У�����Ϊ0	

NO.2	����x(k|k-1)��covariance��p��ʾcovariance
	p(k|k-1)=A*p(k-1|k-1)*AT+Q
	p_mid=p(k|k-1),p_last=p(k-1|k-1)

NO.1 NO.2�Ƕ�ϵͳ��Ԥ��

NO.3	�ο�����ֵ���й���
	x(k|k)=x(k|k-1)+kg*(Z(k)-H*x(k|k-1)) 
	Ϊ��ʵ�ֵݹ飬ÿ�ε�kg����ʵʱ���µ�

NO.4	���������� Kalman Gain
	kg=p(k|k-1)*HT/(H*p(k|k-1)*HT+R)

NO.5	p(k|k)=(1-kg*H)*p(k|k-1)
	����ÿ��p(k|k)��kg����Ҫǰһʱ�̵�ֵ�����£��ݹ�Ĺ�����ȥ
*******************************************************************************/

float Kalman_Filter(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R)
{
	float R = MeasureNoise_R;
	float Q = ProcessNiose_Q;

	static float x_last;

	float x_mid;
	float x_now;

	static float p_last;

	float p_mid;
	float p_now;
	
	float kg;	

	x_mid = x_last;				//x_last=x(k-1|k-1),x_mid=x(k|k-1)
	p_mid = p_last+Q;			//p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=��������
	kg = p_mid/(p_mid+R);		//kgΪKalman Gain��RΪ��������
	x_now = x_mid+kg*(ResrcData-x_mid);	//���Ƴ�������ֵ
	p_now = (1-kg)*p_mid;		//����ֵ��Ӧ��covariance	

	p_last = p_now;				//����covarianceֵ
	x_last = x_now;				//����ϵͳ״ֵ̬

	return x_now;		
}