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
extern FUZZYPID PitchPID,YawPID;
extern FUZZYSPID CarFramePID;
extern FUZZYSPID TurnplatePID,CVPID;
void SetPitchPid(float m_Kp, float m_Ki, float m_Kd)
{
	TurnplatePID.SetPID(m_Kp, m_Ki,m_Kd);
	u1_printf("p:%f\ti:%f\td:%f\r\n",m_Kp,m_Ki,m_Kd);
}
void SetTarget(float m_T){
	TurnplatePID.SetTarget(m_T);
}
void SetPMaxMin(float m_Rpi, float m_Rpa){
	TurnplatePID.SetPMaxMin(m_Rpi,m_Rpa);
}
void SetIMaxMin(float m_Rii, float m_Ria){
	TurnplatePID.SetIMaxMin(m_Rii,m_Ria);
}
void SetDMaxMin(float m_Rdi, float m_Rda){
	TurnplatePID.SetDMaxMin(m_Rdi,m_Rda);
}
void SetDeadZone(float m_DeadZone){
//	CVPID.DeadZone = m_DeadZone;
	TurnplatePID.SetDeadZone(m_DeadZone);

}
void SetFuzzy(float mode,float _NB_PB,float _NM_PM,float _NS_PS){
	TurnplatePID.SetFuzzy(mode,_NB_PB,_NM_PM,_NS_PS);
	switch((u8)mode){
			case 0:u1_printf("mode0:\t::%f\t::%f\t::%f\r\n",TurnplatePID.NB_PB,TurnplatePID.NM_PM,TurnplatePID.NS_PS);break;
			case 1:u1_printf("mode1:\t::%f\t::%f\t::%f\r\n",TurnplatePID.NB_PB_kp,TurnplatePID.NM_PM_kp,TurnplatePID.NS_PS_kp);break;
			case 2:u1_printf("mode2:\t::%f\t::%f\t::%f\r\n",TurnplatePID.NB_PB_ki,TurnplatePID.NM_PM_ki,TurnplatePID.NS_PS_ki);break;
			case 3:u1_printf("mode3:\t::%f\t::%f\t::%f\r\n",TurnplatePID.NB_PB_kd,TurnplatePID.NM_PM_kd,TurnplatePID.NS_PS_kd);break;
			default:break;
			}
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
	{(void *)SetTarget,"SetTarget","SetTarget(float m_T)"},
	{(void *)SetPWM,"SetPWM","(float index,float value)"},
	{(void *)SetFuzzy,"SetFuzzy","SetFuzzy(float mode,float _NB_PB,float _NM_PM,float _NS_PS)"},
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


















