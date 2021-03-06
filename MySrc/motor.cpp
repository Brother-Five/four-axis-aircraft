#include "bsp.h"
#include "filter.h"
#include "cpid.h"
#include "mpu6050.h"

/**
 * [Motor_Aguest description]
 * @param Ax     [遥控的一个通道值]
 * @param Ay     [遥控的一个通道值]
 * @param Az [CarFramePID.AdjustPID()解算出来的CarFramePID.Out]
 * @param _mAngle [选择正前方的角度]
 */
void Motor_Aguest(u16 Ax,u16 Ay,float Az,float _mAngle)
{
//	static u8 count;
    int MotorData[4]={0};
	float xpower,ypower,zpower;
	float AxChange,AyChange;
    AxChange = ((float)Ax - 1024.0f)*No1;
    AyChange = ((float)Ay - 1024.0f)*No1;

    xpower = ((float)AyChange*arm_sin_f32(_mAngle*0.0174532f)+(float)AxChange*arm_cos_f32(_mAngle*0.0174532f));
    ypower = ((float)AyChange*arm_cos_f32(_mAngle*0.0174532f)-(float)AxChange*arm_sin_f32(_mAngle*0.0174532f));
	zpower = Az;
//	zpower  = (int)(((float)Az - 1024.0f)*No1);//(Az - 1024)*No1;

    X_Axic(MotorData,xpower);
    Y_Axic(MotorData,ypower);
    Z_Axic(MotorData,zpower);
	Motor_DataScale(MotorData,5000);
//	u1_printf("%d\t%d\t%d\t%d\r\n\r\n",MotorData[Motor1],MotorData[Motor2],MotorData[Motor3],MotorData[Motor4]);
    Motor_CarFrame(MotorData[Motor1],MotorData[Motor2],MotorData[Motor3],MotorData[Motor4]);//经过Can给底盘4个电机发送的向量相加后的值

}


void X_Axic(int *MotorData,int power)
{
    Motor(MotorData,Motor1,-power);
    Motor(MotorData,Motor2,-power);
    Motor(MotorData,Motor3,power);
    Motor(MotorData,Motor4,power);
}

void Y_Axic(int *MotorData,int power)
{
    Motor(MotorData,Motor1,-power);
    Motor(MotorData,Motor2,power);
    Motor(MotorData,Motor3,power);
    Motor(MotorData,Motor4,-power);
}

void  Z_Axic(int *MotorData,int power)
{

	Motor(MotorData,Motor1,-power);
    Motor(MotorData,Motor2,-power);
    Motor(MotorData,Motor3,-power);
    Motor(MotorData,Motor4,-power);
}
void Motor_DataScale(int *MotorData,int max)
{
    int TempBuf[4]={0};
    u8 k;
    float scale;
    for ( k = 0; k < 4; k++)
    {
        TempBuf[k] = myabs(MotorData[k]);
    }
//	u1_printf("%d\t%d\t%d\t%d\r\n",TempBuf[0],TempBuf[1],TempBuf[2],TempBuf[3]);
	bubble_sort(TempBuf,4);
//    u1_printf("%d\t%d\t%d\t%d\r\n",TempBuf[0],TempBuf[1],TempBuf[2],TempBuf[3]);
    if(TempBuf[3] >= max)
        scale = ((float)max)/((float)TempBuf[3]);
    else
        scale = 1.0f;
//	u1_printf("%d\t%d\t%d\t%d\r\n",(int)(((float)TempBuf[0])*scale),(int)(((float)TempBuf[1])*scale),(int)(((float)TempBuf[2])*scale),(int)(((float)TempBuf[3])*scale));
    MotorData[Motor1] =  (int)(((float)MotorData[Motor1]) * scale);
    MotorData[Motor2] =  (int)(((float)MotorData[Motor2]) * scale);
    MotorData[Motor3] =  (int)(((float)MotorData[Motor3]) * scale);
    MotorData[Motor4] =  (int)(((float)MotorData[Motor4]) * scale);
}
void Motor(int *MotorData,u8 motorx,int power)
{
    MotorData[motorx] += power;
}


void Motor_CarFrame(int16_t current_201,int16_t current_202,int16_t current_203,int16_t current_204)
{
    CanTxMsg msg_send = { 0x110, 0x110, CAN_Id_Standard, CAN_RTR_Data, 0x08, {0, 0, 0, 0, 0, 0, 0, 0}};

	// msg_send.StdId = 0x110 + 0;
    msg_send.Data[0] = (unsigned char)(current_201 >> 8);
    msg_send.Data[1] = (unsigned char)current_201;
    msg_send.Data[2] = (unsigned char)(current_202 >> 8);
    msg_send.Data[3] = (unsigned char)current_202;
    msg_send.Data[4] = (unsigned char)(current_203 >> 8);
    msg_send.Data[5] = (unsigned char)current_203;
    msg_send.Data[6] = (unsigned char)(current_204 >> 8);
    msg_send.Data[7] = (unsigned char)current_204;
	while(CAN_Transmit(CAN1,&msg_send) == CAN_TxStatus_NoMailBox);
//    CAN_send(CAN1, &msg_send);
}

void Motor_Power(u16 id,u16 current,u16 volate)
{
    CanTxMsg msg_send = { 0, 0, CAN_Id_Standard, CAN_RTR_Data, 0x08, {0, 0, 0, 0, 0, 0, 0, 0}};
	
	msg_send.StdId = 0x131;
    msg_send.Data[0] = 0x55;
    msg_send.Data[1] = 0xaa;
    msg_send.Data[2] = (u8)((id>>8)&0xff);
    msg_send.Data[3] = (u8)(id&0xff);
    msg_send.Data[4] = (u8)((volate>>8)&0xff);
	
    msg_send.Data[5] = (u8)((current>>8)&0xff);
    msg_send.Data[6] = (u8)(current&0xff);
	
    msg_send.Data[7] = (u8)(volate);
	CAN_Transmit(CAN1,&msg_send);
//    while(CAN_Transmit(CAN1,&msg_send) == CAN_TxStatus_NoMailBox);

}
void Motor_Other(int16_t current_201,int16_t current_202,int16_t current_203,int16_t current_204)
{
    CanTxMsg msg_send = { 0x02, 0x02, CAN_Id_Standard, CAN_RTR_Data, 0x08, {0, 0, 0, 0, 0, 0, 0, 0}};

    msg_send.Data[0] = (unsigned char)(current_201 >> 8);
    msg_send.Data[1] = (unsigned char)current_201;
    msg_send.Data[2] = (unsigned char)(current_202 >> 8);
    msg_send.Data[3] = (unsigned char)current_202;
    msg_send.Data[4] = (unsigned char)(current_203 >> 8);
    msg_send.Data[5] = (unsigned char)current_203;
    msg_send.Data[6] = (unsigned char)(current_204 >> 8);
    msg_send.Data[7] = (unsigned char)current_204;
    while(CAN_Transmit(CAN1,&msg_send) == CAN_TxStatus_NoMailBox);
}


void Motor_GetMassage(u16 index)
{
    CanTxMsg msg_send = { 0x130, 0x130, CAN_Id_Standard, CAN_RTR_Data, 0x08, {0, 0, 0, 0, 0, 0, 0, 0}};

    msg_send.Data[0] = 0x55;
    msg_send.Data[1] = 0xaa;
    msg_send.Data[2] = (index>>8)&0xff;
    msg_send.Data[3] = index&0xff;
    while(CAN_Transmit(CAN1,&msg_send) == CAN_TxStatus_NoMailBox);
}

void MOTOR_SetPWM(short pwm,short id)
{

    CanTxMsg msg_send = { 0, 0, CAN_Id_Standard, CAN_RTR_Data, 0x08,
                         {0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55}};
	if(pwm < -5000) pwm = -5000;
	if(pwm > 5000) pwm = 5000;
    // msg_send.StdId = id<<4 | 4;
    // msg_send.Data[0] = (unsigned char)((5000>>8)&0xff);
    // msg_send.Data[1] = (unsigned char)(5000&0xff);
    // msg_send.Data[2] = (unsigned char)((pwm/10>>8)&0xff);
    // msg_send.Data[3] = (unsigned char)(pwm/10&0xff);
    // CAN_send(CAN1,&msg_send);
	msg_send.StdId = id<<4 | 2;
    msg_send.Data[0] = (pwm>>8)&0xff;
    msg_send.Data[1] = (unsigned char)(pwm&0xff);
	while(CAN_Transmit(CAN1,&msg_send) == CAN_TxStatus_NoMailBox);
//	CAN_send(CAN1,&msg_send);
}






void Motor_PwrAdd(float throttle,float controlout[],float motorout[],float addout)
{
		motorout[0]=throttle-  controlout[0]+0*controlout[1]-controlout[2];
		motorout[1]=throttle+0*controlout[0]+  controlout[1]+controlout[2];
		motorout[2]=throttle+ controlout[0]-0*controlout[1]-controlout[2];
		motorout[3]=throttle-0*controlout[0]-  controlout[1]+controlout[2];
		motorout[4]=0.0f;
		motorout[5]=0.0f;
//		u1_printf("%7.3f\t%7.3f\t%7.3f\t%7.3f\t\r\n",motorout[3],motorout[2],motorout[1],motorout[0]);
}



void Motor_PwmValSet(float motorout[])
{
	unsigned char i=0;
	float thr[6]={0.0f};
	for(i=0;i<6;i++)
	{
		thr[i]=motorout[i]*55;
	}
	PWM1=1300+thr[0];
	PWM2=1300+thr[1];
	PWM3=1300+thr[2];//加30补偿
	PWM4=1300+thr[3];
	if(PWM1>_FLY_MAX_OUT)	PWM1=_FLY_MAX_OUT+30;
	if(PWM1<1200.0f)					PWM1=1200.0f;
	if(PWM2>_FLY_MAX_OUT)	PWM2=_FLY_MAX_OUT;
	if(PWM2<1200.0f)					PWM2=1200.0f;
	if(PWM3>_FLY_MAX_OUT)	PWM3=_FLY_MAX_OUT;
	if(PWM3<1200.0f)					PWM3=1200.0f;
	if(PWM4>_FLY_MAX_OUT)	PWM4=_FLY_MAX_OUT;
	if(PWM4<1200.0f)					PWM4=1200.0f;
	
//	u1_printf("PWM1:%d\tPWM2:%d\tPWM3:%d\tPWM4:%d\t\r\n",PWM1,PWM2,PWM3,PWM4);

}

float ControlOut[3]={0},MotorOut[6]={0.5,0.5,0.5,0.5};//{0.5,0.5,0.5,0.5}
extern float RC_control;
extern float addout;
extern int pwm_flag;
void controlnormalmode_update(float dt)
{
		if(pwm_flag)
		{
	  Pidcontrol_Altitude_VEL(&Target,&Eulla,&velocity,ControlOut,dt);
		extern float RC_control;
		Motor_PwrAdd_Control(RC_control,addout);
		Motor_PwmValSet(MotorOut);
		}
}

void Motor_PwrAdd_Control(float DR16_Control,float addout)
{
		Motor_PwrAdd(DR16_Control,ControlOut,MotorOut,addout);	
}
