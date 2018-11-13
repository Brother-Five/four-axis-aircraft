#include "bsp.h"
//b11 3r CH4
//a2 2t  CH3

#define Laser_On() bsp_LedOff(3)
#define Laser_Off() bsp_LedOn(3)
void TIM2_PWM_Configuration(void)
{
    GPIO_InitTypeDef          gpio;
    TIM_TimeBaseInitTypeDef   tim;
    TIM_OCInitTypeDef         oc;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB,ENABLE);//��ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	//GPIO ��ʼ��
    gpio.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_11;//PB3 PB11
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB,&gpio);

    gpio.GPIO_Pin = GPIO_Pin_15|GPIO_Pin_2;//PA2 PA15
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA,&gpio);

    GPIO_PinAFConfig(GPIOB,GPIO_PinSource3, GPIO_AF_TIM2);//��������
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource15,GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM2);
    //��ʱ��ʱ������
    tim.TIM_Prescaler = 84-1;//1MHZ
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 20000; //20ms
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2,&tim);

    oc.TIM_OCMode = TIM_OCMode_PWM2;
    oc.TIM_OutputState = TIM_OutputState_Enable;//���ʹ��
    oc.TIM_Pulse = 0;
    oc.TIM_OCPolarity = TIM_OCPolarity_Low;

    TIM_OC1Init(TIM2,&oc);
    TIM_OC2Init(TIM2,&oc);
		TIM_OC3Init(TIM2,&oc);
		TIM_OC4Init(TIM2,&oc);

    TIM_OC1PreloadConfig(TIM2,TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM2,TIM_OCPreload_Enable);
		TIM_OC3PreloadConfig(TIM2,TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM2,TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM2,ENABLE);

    TIM_Cmd(TIM2,ENABLE);
    Laser_Off();
//	Duoji_Open();
//		delay_ms(5000);
//	//PWMӦ�ò���Ҫ��ʼ����ֱ�����
//  PWM1=1000;
//	PWM2=1000;
//	PWM3=1000;
//	PWM4=1000;
//	delay_ms(5000);
//	Duoji_Close();
}

 void TIM3_PWM_Configuration(void)
 {
 	GPIO_InitTypeDef          gpio;
     TIM_TimeBaseInitTypeDef   tim;
     TIM_OCInitTypeDef         oc;

     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOB,ENABLE);//��ʱ��
     RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
 	//GPIO ��ʼ��
     gpio.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;//PB4 PB5
     gpio.GPIO_Mode = GPIO_Mode_AF;
     gpio.GPIO_Speed = GPIO_Speed_100MHz;
     GPIO_Init(GPIOB,&gpio);

     gpio.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;//PC8 PC9
     gpio.GPIO_Mode = GPIO_Mode_AF;
     gpio.GPIO_Speed = GPIO_Speed_100MHz;
     GPIO_Init(GPIOC,&gpio);

     GPIO_PinAFConfig(GPIOB,GPIO_PinSource4, GPIO_AF_TIM3);//��������
     GPIO_PinAFConfig(GPIOB,GPIO_PinSource5, GPIO_AF_TIM3);
 	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8, GPIO_AF_TIM3);
     GPIO_PinAFConfig(GPIOC,GPIO_PinSource9, GPIO_AF_TIM3);
     //��ʱ��ʱ������
     tim.TIM_Prescaler = 84-1;//1MHZ
     tim.TIM_CounterMode = TIM_CounterMode_Up;
     tim.TIM_Period = 1000; //20ms
     tim.TIM_ClockDivision = TIM_CKD_DIV1;
     TIM_TimeBaseInit(TIM3,&tim);

     oc.TIM_OCMode = TIM_OCMode_PWM2;
     oc.TIM_OutputState = TIM_OutputState_Enable;//���ʹ��
     oc.TIM_Pulse = 0;
     oc.TIM_OCPolarity = TIM_OCPolarity_Low;

     TIM_OC1Init(TIM3,&oc);
     TIM_OC2Init(TIM3,&oc);
 	TIM_OC3Init(TIM3,&oc);
 	TIM_OC4Init(TIM3,&oc);

     TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);
     TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);
 	TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable);
     TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);

     TIM_ARRPreloadConfig(TIM3,ENABLE);

     TIM_Cmd(TIM3,ENABLE);
 }
void ESC_Init0(void)
{
//	PWM1=1000;
//	PWM2=1000;
//	PWM3=1000;
//	PWM4=1000;
//	
//	u1_printf("222\r\n");
}
void ESC_Init(void)
{
//	PWM1=2000;//�ȸ�����������õ��֪������
//	PWM2=2000;
//	PWM3=2000;
//	PWM4=2000;
//	delay_ms(3000);
//	PWM1=1000;//�������
//	PWM2=1000;
//	PWM3=1000;
//	PWM4=1000;
//	delay_ms(3000);
////	u1_printf("111\r\n");
}
void ESC_RUN(u8 pwm1,u8 pwm2)
{
//	PWM1=1000+10*pwm1;
//	PWM2=1000+10*pwm2;
// 	PWM3=1000+10*pwm3;
// 	PWM4=1000+10*pwm4;
}

void ESC_RUN1(u8 pwm1,u8 pwm2,u8 pwm3,u8 pwm4)
{
//	PWM1=1000+10*pwm1;
//	PWM2=1000+10*pwm2;
// 	PWM3=1000+10*pwm3;
// 	PWM4=1000+10*pwm4;
//	u1_printf("123\r\n");
}

void ESC_STOP(void)
{
//	PWM1=1000;//�������
//	PWM2=1000;
// 	PWM3=1000;
// 	PWM4=1000;
//	Set_ESC_Flag(0);
//	Laser_Off();
}

u8 ESC_Flag=0;
extern u8 ESC_PWM1,ESC_PWM2;
void ESC_Toggle(void)
{

	ESC_Flag = !ESC_Flag;
    if(ESC_Flag == 1)
    {
//        ESC_RUN(ESC_PWM1,ESC_PWM2);
        Laser_On();
        
    }
    else
    {
        ESC_STOP();
        Laser_Off();
	}
}
void ESC_Open(void)
{
	Laser_On();
	ESC_Flag = 1;
}
u8 Get_ESC_Flag(void)
{
    return ESC_Flag;
}
u8 Set_ESC_Flag(u8 flag)
{
    ESC_Flag = flag;
	return 0;
}

extern const u16 DUOJI_OPEN,DUOJI_CLOSE;
void Duoji_Open(void)
{

//	PWM1=DUOJI_OPEN;
//	PWM2=DUOJI_OPEN;
//	PWM3=DUOJI_OPEN;
//	PWM4=DUOJI_OPEN;
}

void Duoji_Close(void)
{
//	PWM1=DUOJI_CLOSE;
//	PWM2=DUOJI_CLOSE;
//	PWM3=DUOJI_CLOSE;
//	PWM4=DUOJI_CLOSE;
}

void Duoji_Toggle(void)
{
    static u8 flag=1;
    if(flag == 0)
    {
        Duoji_Open();
        flag = !flag;
    }
    else
    {
        Duoji_Close();
        flag = !flag;
    }
}
