/**
 * RTX系统中DR16任务中的执行的子函数
 */
#include "RTL.h"
#include "bsp.h"
#include "cpid.h"
#include "filter.h"
#include "SetParameter.h"

extern OS_TID HandleIdle_Task;
extern OS_TID HandleDR16_SxValue;
float RC_control=0;
float addout=0;
int pwm_flag=0;

/*
//简单测试模式
*/

//void os_dr16(RC_Value *ptrmsg)					
//{
//	
//	//			printf("ch0:%d\tch1:%d\tch2:%d\tch3:%d\t\r\n",ptrmsg->ch0,ptrmsg->ch1,ptrmsg->ch2,ptrmsg->ch3);	
////	if((ptrmsg->s1)==1)
////			os_evt_set (BIT_1, HandleDR16_SxValue);
//		if((ptrmsg->s1)==3)
//	{	
//		delay_ms(5);
//		if((ptrmsg->s1)==3)
//		PWM1=0;
//		PWM2=0;
//		PWM3=0;
//		PWM4=0;
//		while(1);
//	}
//	else if((ptrmsg->s1)==1)
//	{
//		addout=float(((ptrmsg->ch1)-1024+50)/25000.0);							//0.0025附近
//		RC_control=(ptrmsg->ch3)/(1684.0-364.0);				//油门的范围是 0.05/1320*1024=0.07附近
//		Motor_PwrAdd_Control(RC_control,addout);
//	}
//}

///*
////直接给油门模式
//*/

void os_dr16(RC_Value *ptrmsg)					
{
		if((ptrmsg->s1)==3)
	{	
		RC_control=0.0;				//油门的范围是 0.05/1320*1024=0.07附近
//		Motor_PwrAdd_Control(RC_control,addout);
		PWM1=0;
		PWM2=0;
		PWM3=0;
		PWM4=0;
		pwm_flag=0;
	}
	else if((ptrmsg->s1)==1)
	{
		pwm_flag=1;
		RC_control=float(0.004*((ptrmsg->ch3)-1024));				//油门的范围是 0.05/1320*1024=0.07附近
//		Motor_PwrAdd_Control(RC_control,addout);
	}
}


/////*
////累加给油门模式
////*/

//void os_dr16(RC_Value *ptrmsg)					
//{
//		if((ptrmsg->s2)==3)
//	{	
//		RC_control=0.0;				//油门的范围是 0.05/1320*1024=0.07附近
//		Motor_PwrAdd_Control(RC_control,addout);
//		PWM1=0;
//		PWM2=0;
//		PWM3=0;
//		PWM4=0;
//		pwm_flag=0;
//	}
//	else if((ptrmsg->s2)==1)
//	{
//		pwm_flag=1;
//		if(((ptrmsg->ch3)-1024)>0)
//		RC_control=float(0.01*((ptrmsg->ch3)-1024));				//油门的范围是 0.05/1320*1024=0.07附近
//		Motor_PwrAdd_Control(RC_control,addout);
//	}
//}