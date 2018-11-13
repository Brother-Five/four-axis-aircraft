#include "bsp.h"
#include "bsp_can.h"
#include "RTL.h"
#include "cpid.h"
#include "add_bullet.h"
#include "SetParameter.h"
#include "motor.h"
//case KEY_PRESSED_OFFSET_SHIFT +KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_A:bumark=1;lmark=1;break;     //添加于遥控器那里
//case KEY_PRESSED_OFFSET_SHIFT +KEY_PRESSED_OFFSET_CTRL + KEY_PRESSED_OFFSET_F:bumark=1;rmark=1;break;	  //补弹按键设置
//if(!bumark)PitchPID.SetTarget(TempPitch);

//	can_filter_Mask_config(CanFilter_5|CanFifo_0|Can_STDID|Can_DataType,0x207,0x3ff);
//	can_nvic_config(CAN1_R0); 															 //添加于task void EXTITask(void)中

//else if(PIDBegin_CAN&&cir_mark==0&&bu_mark==0&&vmark==0&&bumark==0)    //  添加于 CarFrame_Agues任务那里
//if(PIDBegin_DR16 && PIDBegin_CAN && CAN2_YawAngle && CAN2_YawMotor && CAN2_PitchMotor&&vmark==0&&cir_mark==0&&bu_mark==0&&bumark==0)

//	case BIT_0:	ESC_Toggle();delay_ms(10);vmark=1;USART1_DMA_Send(circlebuf,sizeof(circlebuf));break;//vmark=1;break;//无刷电机delay_ms(10);USART1_DMA_Send(rbuf,sizeof(rbuf));
//				case BIT_1: Turnplate_motor_ONCE();vmark=0;break;//拨盘电机（单次）USART1_DMA_Send(circlebuf,sizeof(circlebuf));
//				case BIT_2:
////						FollowMode =!FollowMode;  1
//						Duoji_Toggle();//舵机
//				bumark=1;

//vmark=!vmark;StrategicMode =vmark*4;StrategicMode =1;

//if(ToCVStructPointer(pMsg->Data)->ID == 0x03)
//	{
//		if(vmark)u1_printf("CV=%d\t%d\t%d\r\n",ToCVStructPointer(pMsg->Data)->CV_X,ToCVStructPointer(pMsg->Data)->CV_Y,ToCVStructPointer(pMsg->Data)->RES);
//		rrx=ToCVStructPointer(pMsg->Data)->CV_X;rry=ToCVStructPointer(pMsg->Data)->CV_Y;rrth=(ToCVStructPointer(pMsg->Data)->RES-1024)*5;
//		if(rrx==888){vmark=0;StrategicMode =0;rrx=1024;rry=1024;rrth=0;}
//	}

//#define xunxian_On() bsp_LedOff(4)
//#define xunxian_Off() bsp_LedOn(4)

//测试用

//u32 UART1_BAUD=115200;
int16_t temp[4],i;
u8 chessbuf[3]={0x55,0x66,0x77}; //85 102 119
u8 circlebuf[3]={0x55,0x88,0x99};// 85 -120 103

u8 lbuf[5]={0xA1,0xF1,0x11,0x22,0x33};     //红外串口发送数组
u8 rbuf[5]={0xA1,0xF1,0x11,0x44,0x55};
u8 DATA=0x33;u8 cnt=0;
u8 bumark=0,timemark=0,vmark=0,fumark=0,lmark=0,rmark=0,bu_mark=0,cir_mark=0;
int16_t rx=1024,rrx=1024,rry=1024,ry=1024,rrth=0,rth=1024;
extern OS_TID HandleDisconnectCheck;
extern FUZZYPID YawPID;
extern FUZZYPID PitchPID;
extern union Parameter_Operater_Union Parameter_Operater;
#define delay_us delay_us_nos

u8 bullet_flag;
u8 Get_bullet_flag(void)
{
	return bullet_flag;
}
void Set_bullet_flag(u8 tflag)
{
	bullet_flag = tflag;
}

void add_bullet_run()
{
	if(bullet_flag == 0)
	{
		Motor_Aguest(1024 + ((float)Get_AxicX() - 1024)/5.0f,1024 + ((float)Get_AxicY() - 1024)/5.0f,Get_AxicZ(),0);//输出底盘电机的电机控制量
	}
	else if(bullet_flag == 1)
	{
		Motor_Aguest(1024 + ((float)Get_AxicX() - 1024)/5.0f,1024 + ((float)Get_AxicY() - 1024)/5.0f,Get_AxicZ(),-90);//输出底盘电机的电机控制量
	}
	else if(bullet_flag == 2)
	{
		Motor_Aguest(1024 + ((float)Get_AxicX() - 1024)/5.0f,1024 + ((float)Get_AxicY() - 1024)/5.0f,Get_AxicZ(),90);//输出底盘电机的电机控制量
	}
}

void Enter_bullet_mode(u8 _mode)
{
	if(StrategicMode != 5)
	{
		Duoji_Open();
		StrategicMode = 5;
		FollowMode =1;
		bullet_flag = _mode;
	}
	else
	{
		Duoji_Close();
		StrategicMode = 0;
		FollowMode =0;
		bullet_flag = 0;
	}
}

void add_base_move(u8 _value)
{
	CanTxMsg msg_toHW = { 0x222, 0x01, CAN_Id_Standard, CAN_RTR_Data, 0x08, {0, 0, 0, 0, 0, 0, 0, 0}};
	msg_toHW.Data[0] = _value;
	{
		u8 i;
		for(i = 0 ;i < 8 ;i++)
		{
			while(CAN_Transmit(CAN1,&msg_toHW) == CAN_TxStatus_NoMailBox)
			{
				u1_printf("fail\r\n");
			}
//			while(CAN_Transmit(CAN1,&msg_toHW) == CAN_TxStatus_NoMailBox);
//			u1_printf("%d\r\n",CAN_Transmit(CAN1,&msg_toHW));
//			CAN_Transmit(CAN1,&msg_toHW);
			// myflag = 0;
			delay_ms(100);
		}
	}
}
void add_base_bullet(u8 _value)
{
	CanTxMsg msg_toHW = { 0x222, 0x01, CAN_Id_Standard, CAN_RTR_Data, 0x08, {0, 0, 0, 0, 0, 0, 0, 0}};
	msg_toHW.Data[0] = _value;
	{
		u8 i;
		if(_value == 0xbb)//开始补弹
			Duoji_Open();
		delay_ms(500);
		for(i = 0 ;i < 8 ;i++)
		{
			while(CAN_Transmit(CAN1,&msg_toHW) == CAN_TxStatus_NoMailBox)
			{
				u1_printf("fail\r\n");
			}
//			while(CAN_Transmit(CAN1,&msg_toHW) == CAN_TxStatus_NoMailBox);
//			u1_printf("%d\r\n",CAN_Transmit(CAN1,&msg_toHW));
//			CAN_Transmit(CAN1,&msg_toHW);
			// myflag = 0;
			delay_ms(100);
		}
		delay_ms(500);
		if(_value == 0xcc)//开始补弹
			Duoji_Close();
	}
}
void add_bullet1(u8 _value)
{
	CanTxMsg msg_toHW = { 0x222, 0x01, CAN_Id_Standard, CAN_RTR_Data, 0x08, {0, 0, 0, 0, 0, 0, 0, 0}};
	if(StrategicMode == 5)
	{
		msg_toHW.Data[0] = _value;
		{
			u8 i;
			Duoji_Open();
	//		delay_ms(500);
			for(i = 0 ;i < 8 ;i++)
			{
				while(CAN_Transmit(CAN1,&msg_toHW) == CAN_TxStatus_NoMailBox)
				{
					u1_printf("fail\r\n");
				}
	//			while(CAN_Transmit(CAN1,&msg_toHW) == CAN_TxStatus_NoMailBox);
	//			u1_printf("%d\r\n",CAN_Transmit(CAN1,&msg_toHW));
	//			CAN_Transmit(CAN1,&msg_toHW);
				// myflag = 0;
				delay_ms(100);
			}
	//		delay_ms(2000);
	//		Duoji_Close();
		}
	}
	// myflag = 1;
	// FollowMode =0;
	// StrategicMode = 0;
	// delay_ms(1500);
	// Duoji_Close();
}

void add_bullet(CanRxMsg *msg_rece)
{


//	if(bumark)//==1||rmark==1)						//补弹或进站巡线
//		{
////			u1_printf("in\r\n");
//			PitchPID.SetTarget(Parameter_Operater.data.PITCH_Encoder_Mid);
//			for(i=0;i<4;i++)																		//循线程序代码
//			{
//				temp[i]=(((int16_t)msg_rece->Data[2*i])<<8)+msg_rece->Data[2*i+1];u1_printf("%d\t",temp[i]);
//			}
//			u1_printf("\r\n");
////			if(temp[0]==1&&temp[1]==1&&temp[2]==1&&temp[3]==1)
////				{bumark=0;USART1_DMA_Send(chessbuf,sizeof(chessbuf));vmark=1;}   //巡到公路出口，依靠棋盘标定

//			if(temp[0]==0&&temp[1]==0&&temp[2]==0&&temp[3]==0)
//			{
//
//				Motor_Aguest(1024,1024,0,0);
////				timemark=1;
////				if(bumark==1)					//如果是补弹，发送红外
//				{
////					send_byte(data);
//				}
//				if(bumark==1)bumark=0;
//				if(rmark==1){rmark=0;bu_mark=1;}
//				printf("out bumark\r\n");
////			}		//printf("out bumark\r\n");}							//发信号延时等待落弹开始
			//else
//			{Motor_CarFrame(temp[0],temp[1],temp[2],temp[3] );u1_printf("send\r\n");}
//		}

}

void go_back()
{

	if(rmark==2)
	 {
		Motor_Aguest(1024,1124,0,0);
		delay_ms(400);
		Motor_Aguest(1024,1124,0,0);
		delay_ms(400);
		Motor_Aguest(1024,1124,0,0);					//进站
		delay_ms(400);
		rmark=1;
	  }
	if(fumark==1||bu_mark==1)
	  {
		Motor_Aguest(1024,900,0,0);
		delay_ms(400);
		Motor_Aguest(1024,900,0,0);
		delay_ms(400);
		Motor_Aguest(1024,900,0,0);					//出站
		delay_ms(400);
		bu_mark=0;
		vmark=1;
//		USART1_DMA_Send(go_buf,sizeof(go_buf));
//		delay_ms(20);USART1_DMA_Send(go_buf,sizeof(go_buf));  		//通知PC要到符点
	  }
	if(fumark==2)
	  {
		Motor_Aguest(1200,1024,0,0);
		delay_ms(400);
		Motor_Aguest(1200,1024,0,0);
		delay_ms(400);
		Motor_Aguest(1200,1024,0,0);					//下坡
		delay_ms(400);
		vmark=1;
//		USART1_DMA_Send(back_buf,sizeof(back_buf));
//		delay_ms(20);USART1_DMA_Send(back_buf,sizeof(back_buf));	//通知PC要回站

	  }
//	if(rx==666){vmark=0;rmark=1;Motor_Aguest(1,1,1);}		//进站
//	if(rx==888){vmark=0;Motor_Aguest(1,1,1);}		//上坡
}
extern u8 cnt;
void send_bit_1()
{
    for(cnt=0;cnt<22;cnt++)
		{GPIO_SetBits(GPIOC,GPIO_Pin_14);//LED0输出高
		delay_us(17);
		GPIO_ResetBits(GPIOC,GPIO_Pin_14); //LED0输出低
		delay_us(9);}
		GPIO_ResetBits(GPIOC,GPIO_Pin_14);
		delay_ms(1);
    delay_us(680);
}

void send_bit_0()
{
    for(cnt=0;cnt<22;cnt++)
		{GPIO_SetBits(GPIOC,GPIO_Pin_14);//LED0输出高
		delay_us(17);
		GPIO_ResetBits(GPIOC,GPIO_Pin_14); //LED0输出低
		delay_us(9);}
		GPIO_ResetBits(GPIOC,GPIO_Pin_14);
    delay_us(560);
}

void send_start_bit()
{
    for(cnt=0;cnt<50;cnt++)
		{GPIO_SetBits(GPIOC,GPIO_Pin_14);//LED0输出高
		delay_us(17);
		GPIO_ResetBits(GPIOC,GPIO_Pin_14); //LED0输出低
		delay_us(9);}
		GPIO_ResetBits(GPIOC,GPIO_Pin_14);
		delay_ms(4);
    delay_us(500);
}

void send_fin_bit()
{
 for(cnt=0;cnt<50;cnt++)
		{GPIO_SetBits(GPIOC,GPIO_Pin_14);//LED0输出高
		delay_us(17);
		GPIO_ResetBits(GPIOC,GPIO_Pin_14); //LED0输出低
		delay_us(9);}
		GPIO_ResetBits(GPIOC,GPIO_Pin_14);
		delay_ms(2);
    delay_us(500);
}
void send_byte(u16 byte)
{
  u8 i;
	send_start_bit();
	for(i=0;i<16;i++)
	{
	 if(byte&0x8000)send_bit_1();
	 else send_bit_0();
	 byte<<=1;
  }
	send_fin_bit();
	for(cnt=0;cnt<22;cnt++)
		{GPIO_SetBits(GPIOC,GPIO_Pin_14);//LED0输出高
		delay_us(17);
		GPIO_ResetBits(GPIOC,GPIO_Pin_14); //LED0输出低
		delay_us(9);}
	delay_ms(150);
}



void send_cmd()
{
//	if(cir_mark)							//由侧面转为正面
//			{
//
//				Motor_Aguest(224,1824,0);
//				delay_ms(500);
//				Motor_Aguest(224,1824,0);
//				delay_ms(500);
//				Motor_Aguest(124,1724,1000);
//				delay_ms(500);
//				Motor_Aguest(24,1624,3200);
//				delay_ms(500);
//				cir_mark=0;
//				//bumark=1;
//				//USART1_DMA_Send(chessbuf,sizeof(chessbuf)); //通知PC启动圆形标定识别
//				vmark=1;
////				bu_mark=1;
//			}
//		if(bu_mark)
//			{
//
//				Motor_Aguest(324,1924,0);
//				delay_ms(500);
//				Motor_Aguest(24,1924,0);
//				delay_ms(500);
//			    Motor_Aguest(1024,1334,0);//			   Motor_Aguest(1024,1424,0);
//				delay_ms(500);
//				bu_mark=0;
////				Motor_Aguest(1024,1424,0);
////				delay_ms(500);
////				printf("t\r\n");
//				Motor_Aguest(1024,1024,0);
//				delay_ms(100);
//				printf("v0");
//				bumark=1;									//圆形标定完后直走后启动循线补弹
//		    }
//
//		if(redmark&&lmark)						//红外发送,通知左边落弹
//			{
//				//delay_ms(200);
//				UART1_BAUD=9600;
//				bsp_InitUart();
//				//delay_ms(200);
//				USART1_DMA_Send(lbuf,sizeof(lbuf));
//				delay_ms(20);USART1_DMA_Send(lbuf,sizeof(lbuf));
//				delay_ms(20);USART1_DMA_Send(lbuf,sizeof(lbuf));
//				lmark=redmark=0;
//				delay_ms(200);
//				UART1_BAUD=115200;
//				bsp_InitUart();
//				//delay_ms(200);
//
//			}
//		if(redmark&&rmark)						//红外发送,通知右边落弹
//			{
//				UART1_BAUD=9600;
//				bsp_InitUart();
//				USART1_DMA_Send(rbuf,sizeof(rbuf));
//				delay_ms(20);USART1_DMA_Send(rbuf,sizeof(rbuf));
//				delay_ms(20);USART1_DMA_Send(rbuf,sizeof(rbuf));
//				delay_ms(200);
//				UART1_BAUD=115200;
//				bsp_InitUart();
//				rmark=redmark=0;
//			}
//		if(timemark)
//		{
//			delay_ms(3000);				//收到信号开始延时2.5s收弹
//			delay_ms(2000);
//			redmark=1;
//			timemark=0;
//			bumark=0;
//		}
//
		delay_ms(10);
}

void rece_usart(u8* uart_rx)
{
//			if(uart_rx[0]==0xaa)
//	       {
//			rx=uart_rx[1]<<8;
//			rx|=uart_rx[2];
//			ry=uart_rx[3]<<8;
//			ry|=uart_rx[4];
//			rth=uart_rx[5]<<8;
//			rth|=uart_rx[6];
//			//printf("%d\t%d\t%d\t",rx,ry,rth);
//			}
//		   if(rx==666&&vmark==1){vmark=0;cir_mark=1;rrx=rry=1024;rrth=0;}
//		   else {rrx=rx;rry=ry;rrth=(int)(rth-1024)*7;}
//		    if(rx==888&&vmark==1){vmark=0;rrx=rry=1024;rrth=0;}//bu_mark=1;
//		   else
//			{rrx=rx;rry=ry;rrth=(int)(rth-1024)*7;}
//	//if(vmark)Motor_Aguest(rrx,rry,rrth);
//	if(rrx==888)vmark=0;
	delay_ms(10);
}

