#include "cpid.h"
#include "bsp.h"
#include "SetParameter.h"
void tips(void)
{
	unsigned char i;
	for(i=0;i<dev.fnum-1;i++)
	{
		printf("%s\r\n",dev.tab[i].tips);
	}
}


/**********************************
 * 给C语言编译的函数调用的接口
 *********************************/
extern PID Pitchpid,Rollpid,Yawpid;
extern PID Gyropitchpid,Gyrorollpid,Gyroyawpid;
void SetPitchPid(float m_Kp, float m_Ki,float m_Kd)
{
	Pitchpid.kp=m_Kp;
	Pitchpid.ki=m_Ki;
	Pitchpid.kd=m_Kd;
	u1_printf("p:%f\ti:%f\td:%f\r\n",m_Kp,m_Ki,m_Kd);
}
void SetRollpid(float m_Kp, float m_Ki, float m_Kd)
{
	Rollpid.kp=m_Kp;
	Rollpid.ki=m_Ki;
	Rollpid.kd=m_Kd;
	u1_printf("p:%f\ti:%f\td:%f\r\n",m_Kp,m_Ki,m_Kd);
}
void SetYawpid(float m_Kp, float m_Ki, float m_Kd)
{
	Yawpid.kp=m_Kp;
	Yawpid.ki=m_Ki;
	Yawpid.kd=m_Kd;
	u1_printf("p:%f\ti:%f\td:%f\r\n",m_Kp,m_Ki,m_Kd);
}

void SetGyropitchpid(float m_Kp, float m_Kd)
{
	Pitchpid.kp=m_Kp;
	Pitchpid.kd=m_Kd;
	u1_printf("p:%f\ti:%f\td:%f\r\n",m_Kp,m_Kd);
}
void SetGyrorollpid(float m_Kp, float m_Kd)
{
	Pitchpid.kp=m_Kp;
	Pitchpid.kd=m_Kd;
	u1_printf("p:%f\ti:%f\td:%f\r\n",m_Kp,m_Kd);
}
void SetGyroyawpid(float m_Kp, float m_Kd)
{
	Pitchpid.kp=m_Kp;
	Pitchpid.ki=m_Kd;
	u1_printf("p:%f\ti:%f\td:%f\r\n",m_Kp,m_Kd);
}
void PitchPid_GET(float on)
{
	if(on)
	u1_printf("p:%f\ti:%f\td:%f\r\n",Pitchpid.kp,Pitchpid.ki,Pitchpid.kd);
}

void Rollpid_GET(float on)
{
	if(on)
	u1_printf("p:%f\ti:%f\td:%f\r\n",Rollpid.kp,Rollpid.ki,Rollpid.kd);
}

void SetPWM(float index,float value){
	switch((u8)index)
	{
		case 1:PWM1 = (int)value;break;
		case 2:PWM2 = (int)value;break;
		case 3:PWM3 = (int)value;break;
		case 4:PWM4 = (int)value;break;
	}
	u1_printf("%d:%d\r\n",(u8)index,(int)value);
}
//extern void SetPara_DR(float _DR_SPEED,float _PitchScale,float _YawScale,float CARFRAMEWEIGHT);
//extern void SetPara_KM(float SPEED1,float SPEED2,float PITCHWEIGHT,float YAWWEIGHT,float CARFRAMEWEIGHT);
//extern void ParaSetting(float _pFlag);
//extern void SetPara_Operater(u8 mode);
extern void ESCSetPara(float _mESC_PWM_speed,float _mESC_PWM_volate);
// *********************************

struct _m_nametab nametab[]=
{

	{(void *)SetPitchPid,"SetPitchPid","SetPitchPid(float m_Kp, float m_Ki, float m_Kd)"},
	{(void *)SetRollpid,"SetRollpid","SetRollpid(float m_Kp, float m_Ki, float m_Kd)"},
	{(void *)SetYawpid,"SetYawpid","SetYawpid(float m_Kp, float m_Ki, float m_Kd)"},
	
	{(void *)SetGyropitchpid,"SetGyropitchpid","SetGyropitchpid(float m_Kp, float m_Kd)"},
	{(void *)SetGyrorollpid,"SetGyrorollpid","SetGyrorollpid(float m_Kp, float m_Kd)"},
	{(void *)SetGyroyawpid,"SetGyroyawpid","SetGyroyawpid(float m_Kp, float m_Kd)"},
	
	{(void *)SetPWM,"SetPWM","SetPWM(float index,float value)"},
	
	{(void *)PitchPid_GET,"PitchPid_GET","PitchPid_GET(float on)"},
	{(void *)Rollpid_GET,"Rollpid_GET","Rollpid_GET(float on)"},
	
	{(void*)tips,"tips",""}
};

struct _m_dev dev=
{
	nametab,
	sizeof(nametab)/sizeof(struct _m_nametab),//函数数量
	0,	  	//参数数量
	0,	 	//函数ID
};

void DebugToolsInit(void)
{
	Initial_System_Timer();
//	UsartDMA_Init();
//	TIMResetModeForDMA();
//	EXTI_Configuration();
}


















