#ifndef __PWM_H__
#define __PWM_H__

#include "stm32f4xx.h"


#define PWM1  TIM2->CCR1
#define PWM2  TIM2->CCR2
#define PWM3  TIM2->CCR3
#define PWM4  TIM2->CCR4
#define PWM5  TIM3->CCR1
#define PWM6  TIM3->CCR2
#define PWM7  TIM3->CCR3
#define PWM8  TIM3->CCR4

void TIM2_PWM_Configuration(void);
void TIM3_PWM_Configuration(void);
void ESC_Init0(void);
void ESC_Init(void);
void ESC_STOP(void);
void ESC_RUN(u8 pwm1,u8 pwm2);
void ESC_RUN1(u8 pwm1,u8 pwm2,u8 pwm3,u8 pwm4);
void ESC_Toggle(void);
u8 Get_ESC_Flag(void);
u8 Set_ESC_Flag(u8 flag);
void Duoji_Open(void);
void Duoji_Close(void);
void Duoji_Toggle(void);
#endif


