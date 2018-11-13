#ifndef _SYSTEMTIMER_H
#define _SYSTEMTIMER_H
#ifdef __cplusplus
 extern "C" {
#endif

#define StartCheck 1
#define StopCheck  0

#define micros() TIM5->CNT	  //读取系统时钟
void Initial_System_Timer(void);
u32 RunTime1(unsigned char flag);
u32 RunTime2(unsigned char flag);
void delay_ms_nos(uint32_t cnt);
void delay_us_nos(uint32_t cnt);
typedef struct
{
	float thistime;
	float lasttime;
//	float dot;
}TIMEDOT;
#ifdef __cplusplus
 }
#endif
#endif
