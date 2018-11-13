#include "includes.h"
#include "cpid.h"
#include "filter.h"
#include "SetParameter.h"
#include "location.h"
#include "bsp_stmflash.h"
#include "Kalman.h"
#include "add_bullet.h"

//#define HERO_R
//#define SPECIAL
//#define SPECIAL1
//#define INDEX1
//#define INDEX2
#define INDEX3
TIMEDOT TIMEALL;

#ifdef INDEX1
u16 DUOJI_OPEN = 700,DUOJI_CLOSE=1585;
#endif

#ifdef INDEX2
u16 DUOJI_OPEN = 700,DUOJI_CLOSE=1585;
#endif

#ifdef INDEX3
u16 DUOJI_OPEN = 700,DUOJI_CLOSE=1585;
#endif
//目标上下限和初始目标值在void bsp_ParaInit()中设置
FUZZYPID PitchPID(0,0,0,0,-6000, +6000, -500, +500, -5000, +5000,0,0,0,\
		 90.0f,60.0f,30.0f,\

#ifdef INDEX1
		 60.00f,50.00f,55.00f,
		 500.0f,500.0f,500.0f,\
		 1.5,1.1,1.0);
#endif

#ifdef INDEX2
/*3333*/
		 50.00f,40.00f,45.00f,
		 500.0f,500.0f,500.0f,\
		 1.5,1.80,1.30);
#endif

#ifdef INDEX3
/*3333*/
		 60.00f,50.00f,55.00f,
		 500.0f,500.0f,500.0f,\
		 2.0,1.8,1.5);
#endif

//CPID PitchPID(40.0,500.0,1.00,0,-6000, +6000, -500, +500, -5000, +5000,0,0);
//SCPID PitchSpeedPID(0.0001,0.0,0.001,0,-6000, +6000, -3000, +3000, -5000, +5000,-500,+500,0,5);

//CPID YawPID(60.0,500.0,1.80,0,-6000, +6000, -1000, +1000, -5000, +5000,-1.701411834604E+38,1.701411834604E+38);
//CPID YawPID(0.0,500.0,0.5,0,-6000, +6000, -500, +500, -5000, +5000,-1.701411834604E+38,1.701411834604E+38);
FUZZYPID YawPID(8.5,0.1,1.1,0,-4500, +4500, -500, +500, -4500, +4500,-1.701411834604E+38,1.701411834604E+38,0,\
		 120.0f,60.0f,0.0f,\
		 80.0f,70.0f,75.00f,\
		 0.0f,0.0f,0.0f,\
		 1.2,4.20,2.80);


//CPID YawSpeedPID(0.1,50.0,0.1,0,-6000, +6000, -3000, +3000, -5000, +5000,-500,+500,0);

//FUZZYSPID CarFramePID(8.5,0.1,1.1,0,-20000, +20000, -3000, +3000, -5000, +5000,-1.701411834604E+38,1.701411834604E+38,60,\
//		 300.0f,200.0f,100.0f,\
//		 10.05f,10.05f,10.05f,\
//		 0.0f,0.0f,0.0f,\
//		 -0.0069f,-0.0069f,-0.0069f);
SCPID CarFramePID(0,0,0,0,-20000, +20000, -3000, +3000, -5000, +5000,-1.701411834604E+38,1.701411834604E+38,60);//PID参数在void bsp_ParaInit()中设置
//SCPID TurnplatePID(25.20,0.0,0.001,0.0,-50, +50, -50, +50, -50, +50,-50,50,1.5,0);

//SCPID TurnplatePID(5.0,0.0,0.0,0,-1000, +1000, -500, +500, -1000, +1000,-1.701411834604E+38,1.701411834604E+38,0);
FUZZYSPID TurnplatePID(5.0,0.0,0.0001,0,-1000, +1000, -500, +500, -1000, +1000,-1.701411834604E+38,1.701411834604E+38,0,\
		 400.0f,300.0f,200.0f,\
		 15.0f,7.0f,3.0f,\
		 0.0f,0.0f,0.0f,\
		 0.0f,0.3f,0.05f);

/**
 *任务函数声明
 */
static void AppTaskCreate (void);
static void AppObjCreate (void);

__task void TaskStart(void);
//__task void DEBUG_DATA(void);
__task void DR16_DATA_Rec(void);
__task void DisconnectCheck(void);
__task void PitchYaw_Aguest(void);
__task void AHRS_Update_Aguest(void);
__task void Imu_data_Aguest(void);
__task void PWM_Task(void);
__task void Boot_Sequence(void);
__task void DR16_SxValue(void);



/***
 *任务栈
 */
static uint64_t TaskStartStk[512/8];
//static uint64_t DEBUG_DATAStk[512/8];
static uint64_t DR16_DATA_RecStk[512/8];
static uint64_t DisconnectCheckStk[512/8];
static uint64_t PitchYaw_AguestStk[1024/8];
static uint64_t AHRS_Update_AguestStk[512/8];
static uint64_t Imu_data_AguestStk[512/8];
static uint64_t PWM_TaskStk[512/8];
static uint64_t Boot_SequenceStk[512/8];
static uint64_t DR16_SxValueStk[512/8];


/**
 *任务句柄
 */
 OS_TID HandleDEBUG_DATA       = NULL;
OS_TID HandleDR16_DATA_Rec    = NULL;
OS_TID HandleDisconnectCheck  = NULL;
OS_TID HandlePitchYaw_Aguest  = NULL;
OS_TID HandleAHRS_Update_Aguest  = NULL;
OS_TID HandleImu_data_Aguest  = NULL;
OS_TID HandlePWM_Task    = NULL;
OS_TID HandleBoot_Sequence    = NULL;
OS_TID HandleDR16_SxValue     = NULL;



u8 timer200msflag = 0,positionreachflag = 0,shootonce = 0,shootcons = 0;
/**
 * [TaskStart 开始任务，新建任务和信息量，同时用来处理固定时间任务，这里用来检测串口的FIFO是否为满]
 */
__task void TaskStart(void)
{
	AppTaskCreate();/* 创建任务 */
	AppObjCreate();/* 创建任务通信机制和有时序需要的初始化步骤 */
	while(1)
	{
		os_dly_wait(1);
	}
}

//__task void DEBUG_DATA(void)
//{
////	delay_ms(5000);
//	while(1)
//	{
//			extern EULLA Eulla/*,LastEulla*/;//互补滤波后的欧拉角结构体
//			u1_printf("%d\t%d\t%d\t%d\t%7.3f\t%7.3f\t%7.3f\t\r\n",PWM1,PWM2,PWM3,PWM4,Eulla.yaw,Eulla.pitch,Eulla.roll);
////			u1_printf("%7.3f\t%7.3f\t%7.3f\r\n",Eulla.yaw,Eulla.pitch,Eulla.roll );
////		ANO_Data3_Send(0xf1,djm.volate*100,djm.current*100,djm.current*djm.volate*100);
////		u1_printf("V:%d\tI:%d\tP:%d\r\n",(int)(djm.volate*100),(int)(djm.current*100),(int)(djm.current*djm.volate*100));
////		if(!GetParaFlag())
////		{
////			ANO_Data1_Send(0xf1,yaw_angle_ADXRS622);
//			// ANO_Data1_Send(0xf1,YAW_Angle);
////			ANO_Data6_Send(0xf1,YawPID.Target,YawPID.Current,aa,CarFramePID.Current,YawPID.ITerm,YawPID.DTerm);
//	//		{
//	//			short gx;
//	//			u8 buf[6],res;
//	//			res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
//	//			if(res==0)
//	//			{
//	//				gx=(short)(((u16)buf[0]<<8)|buf[1]) + 18;
//	//				u1_printf("%d\r\n",gx);
//	//			}
//	//
//	//		}
//			// ANO_Data6_Send(0xf1,YawPID.Kp*100,YawPID.Ki*100,YawPID.Kd*100,YawPID.Target - YawPID.Current,myabs(YawPID.Target - YawPID.Current),YawPID.NS_PS_kd);
//			// ANO_Data6_Send(0xf1,YawPID.Target,YawPID.Current,YawPID.Out,YawPID.PTerm,YawPID.ITerm,YawPID.DTerm);
////			ANO_Data6_Send(0xf1,TurnplatePID.Target,TurnplatePID.Current,TurnplatePID.Out,TurnplatePID.PTerm,TurnplatePID.ITerm,TurnplatePID.DTerm);
//			// ANO_Data6_Send(0xf1,PitchPID.Target,PitchPID.Current,PitchPID.Out,PitchPID.PTerm,PitchPID.ITerm,PitchPID.DTerm);
////			ANO_Data6_Send(0xf1,CarFramePID.Target,CarFramePID.Current,CarFramePID.Out,CarFramePID.PTerm,CarFramePID.ITerm,CarFramePID.DTerm);
//	//		u1_printf("%d\t%d\t%d\r\n",(int)car_location.gobal_X,(int)car_location.gobal_Y,(int)car_location.gobal_Z);
//	//		temp[2] = body_C*0.61591103507271171941830624465355f;
//	//		temp[2] = temp[2] - (((int)temp[2])/360)*360;
//	//		if(temp[2]<0)
//	//			temp[2]+=360;
//	//		u1_printf("%d\t%d\t%d\r\n",temp[0],temp[1],temp[2]);
//	//		u1_printf("%d\t%d\r\n",YawPID.Current,CarFramePID.Current);
//	//		temp[0] = (YawPID.Current -  CarFramePID.Current)/22.7556f;6
//	//		temp[1] = YawPID.Target/22.7556f;
//	//		temp[2] = (CarFramePID.Current)/22.7556f;
//	//		temp[3] = (YawPID.Current/22.7556f);
//	//		u1_printf("%d\t%d\t%d\t%d\r\n",temp[0],temp[1],temp[2],temp[3]);
//	//		u1_printf("%d\t%d\t%d\r\n",0,camera_angle/22.7556f,((float)camera_angle - Middle)/22.7556f);
//	//		ANO_Data1_Send(0xf1,(CarFramePID.Current)/22.7556f);
//	//		ANO_Data6_Send(0xf1,0,(CarFramePID.Current)/22.7556f,camera_angle/22.7556f,((float)camera_angle - Middle)/22.7556f,0,0);
//	//		u1_printf("Cam:%f\r\n",(YawPID.Current - Middle)/22.7556f);
//	//		u1_printf("%f\t%f\t%f\r\n",car_location.gobal_X,car_location.gobal_Y,car_location.car_body_C);
//	//		ANO_Data6_Send(0xf1,car_location.gobal_X,car_location.gobal_Y,car_location.car_body_C,0,0,0);
//	//		ANO_Data6_Send(0xf1,MotorMassage.motor1.position,MotorMassage.motor2.position,MotorMassage.motor3.position,MotorMassage.motor4.position,MotorMassage.motor1.speed,MotorMassage.motor2.speed);
//	//		CANCheckFifo();
//	//		u1_printf("ch0:%d\tch1:%d\tch2:%d\tch3:%dch0:%d\tch1:%d\tch2:%d\tch3:%dch0:%d\tch1:%d\tch2:%d\tch3:%d\r\n",1,2,3,4,5,6,7,8,9,10,11,12);

//	//		u1_printf("ch0:%d\r\n",Get_AxicY);
//	//		CheckParameter();
//	//		CANCheckFifo();
//	//		ANO_Data6_Send(0xf1,PitchPID.Target - PitchPID.Current,PitchPID.Current,errorC,PitchPID.Kp,PitchPID.Ki*100,PitchPID.Kd*100);

//			// ANO_Data3_Send(0xf1,PitchPID.Kp,PitchPID.Ki*100,PitchPID.Kd*100);
//	//		ANO_Data6_Send(0xf1,YawPID.Target,YawPID.Current,YawPID.Out,YawPID.PTerm,YawPID.ITerm,YawPID.DTerm);
//			// ANO_Data6_Send(0xf1,errorC*100,YawPID.Current,YawPID.Kd*100,YawPID.PTerm,YawPID.ITerm,YawPID.DTerm);
////	//		ANO_Data6_Send(0xf1,PitchSpeedPID.Target,PitchSpeedPID.Current,PitchSpeedPID.Out,PitchSpeedPID.PTerm,PitchSpeedPID.ITerm,PitchSpeedPID.DTerm);
////		}
//		os_dly_wait (10);
//	}
//}


//extern float errorC;
//extern float yaw_angle_6050;
//extern union ParameterUnion Parameter;
extern MOTORSTRUCT MotorMassage;
extern LOCATIONSTRUCT car_location;
extern float errorC;
extern int camera_angle;
extern float body_X,body_Y,body_C,YAW_Angle,YAW_Angle_Zero;
float tmee,tmeee;
extern float yaw_angle_ADXRS622;
extern short aa;
extern AngleCorrectStruct AngleCorrect;
extern MotorEncoderStruct MotorEncoder;

/**
 * [DisconnectCheck 断线保护任务]
 */

uint8_t PIDBegin_DR16=0,PIDBegin_CAN=0,CAN2_YawAngle=0,CAN2_PitchMotor=0,CAN2_YawMotor=0,ExtiFlag=1;
__task void DisconnectCheck(void)
{

	while(1)
	{
//		ANO_ImuDataReturn1(&velocity.X,
//						&velocity.Y,&velocity.Z,&acc.X,&acc.Y,
//						&acc.Z,PWM1,PWM2,
//						PWM3,PWM4);	
			extern PID Pitchpid,Rollpid,Yawpid;
      u1_printf("%7.3f\t%7.3f\t%7.3f\r\n",Pitchpid.controutput,Rollpid.controutput,Yawpid.controutput);
//		u1_printf("%7.3f\t%7.3f\t%7.3f\r\n",Pitchpid.error,Rollpid.error,Yawpid.error );
//		u1_printf("%7.3f\t%7.3f\t%7.3f\r\n",Pitchpid.error,Eulla.pitch,Target.pitch );
		
//		if (os_evt_wait_or (BIT_0, 50) == OS_R_TMO)
//		{
//			PIDBegin_DR16 = 0;//若等待超时设置标志量为0
//			ESC_STOP();
//			shootonce = 0;
//			shootcons = 0;
//			os_evt_set (BIT_6, HandleDisconnectCheck);//断线保护
//			u1_printf("PIDBegin_DR16 = 0\r\n");
//		}
      delay_ms(50);
//		else PIDBegin_DR16 = 1;
//		if (os_evt_wait_or (BIT_1, 50) == OS_R_TMO) {}
//		else PIDBegin_CAN = 1;
//		if (os_evt_wait_or (BIT_2, 50) == OS_R_TMO) {
//			CAN2_YawAngle = 0;//若等待超时设置标志量为0
////			u1_printf("CAN2_YawAngle = 0\r\n");
//		}
//		else{
//			CAN2_YawAngle = 1;

//		}
//		if (os_evt_wait_or (BIT_3, 50) == OS_R_TMO)
//		{
//			CAN2_PitchMotor = 0;//若等待超时设置标志量为0
////			u1_printf("CAN2_PitchMotor = 0\r\n");
//		}
//		else CAN2_PitchMotor = 1;
//		if (os_evt_wait_or (BIT_4, 50) == OS_R_TMO)
//		{
//			CAN2_YawMotor = 0;//若等待超时设置标志量为0
////			u1_printf("CAN2_YawMotor = 0\r\n");

//		}
//		else CAN2_YawMotor = 1;

	}
}

/**
 * [EXTITask 断线保护任务]
 */
void ttp(void)
{
	Location(&MotorMassage);
}

extern float YAW_Angle_Zero;

/**
 * [DR16_DATA_Rec DR16数据处理，通过邮箱接收中断的数据]
 */
 __task void DR16_DATA_Rec(void)
 {
 	RC_Value *ptrmsg;
 	while(1)
 	{
 		if(DR16_receive(&ptrmsg, 200) != OS_R_TMO )
 		{
 			os_dr16(ptrmsg);
 		}
	  os_dly_wait (2);
 	}
 }
 
// 
///**
// 遥控键值处理
// */
//__task void DR16_SxValue(void)
//{
//		OS_RESULT xResult;
//		while(1)
//		{
//			if (os_evt_wait_or (BIT_ALL, 200) == OS_R_EVT)
//				{
//					xResult = os_evt_get ();
//					switch (xResult)
//					{
//						case BIT_1:		
//						{	
//											printf("BTT_1");
//						}break;
//						case BIT_2:		
//						{	
//											printf("BTT_2");
//						}break;
//						case BIT_3:		
//						{	
//											printf("BTT_3");
//						}break;
//					}
//				}
//			delay_ms(100);
//		}
//}


 /**
 * [CAN1_DATA0_Rec CAN1数据的处理]
 */
struct {
	float volate;
	u8 flag;
}realshoot;
u16 B_freq,B_speed;
u8 BackupMode = 0;
OS_ID  tmr2;


extern FUZZYPID CVPID,CVPitchPID;
/**
 * [PitchYaw_Aguest PItCH和YAW两个轴的的PID控制]
 */
__task void PitchYaw_Aguest(void)
{
//	TIM1_timer(200,(void *)pitchyaw);
	delay_ms(5000);
	while(1)
	{
	    u32 Dt;
//			delay_ms(1000);
			{
//				extern EULLA Eulla/*,LastEulla*/;//互补滤波后的欧拉角结构体
			//	u1_printf("%7.3f\t%7.3f\t%7.3f\r\n",Eulla.yaw,Eulla.pitch,Eulla.roll );
			}
			Dt=RunTime1(StopCheck);
			controlnormalmode_update(Dt/1000000);
//			delay_ms(2);
//			u1_printf("%d\t%d\t%d\t%d\t\r\n",PWM1,PWM2,PWM3,PWM4);
      RunTime1(StartCheck);
			os_dly_wait(2);

	}
}

/**
 * [CarFrame_Aguest 底盘电机的控制]
 */
__task void AHRS_Update_Aguest(void)
{


//	os_itv_set(10);	/* 设置延迟周期 */

  u32 dt;
	while(1)
	{	
//			u1_printf("%7.3f\t %7.3f\t %7.3f\t %7.3f\t %7.3f\t %7.3f\r\n", ACC.Y, ACC.X, ACC.Z, VELOCITY.X, VELOCITY.Y, VELOCITY.Z);
		  dt=RunTime2(StopCheck);
			AHRS_Update(dt/1000000);
			RunTime2(StartCheck);
//			u1_printf("%7.3f\t%7.3f\t%7.3f\r\n",Eulla.yaw,Eulla.pitch,Eulla.roll );
//			delay_ms(2);
			os_dly_wait(2);
	}
}


/**
 * [Turnplate_Aguest 拨盘电机速度闭环]
 */
extern EncoderStuct EncoderData;
#define DEADTIME 300
__task void Imu_data_Aguest(void)
{

	os_itv_set(4);	/* 设置延迟周期 */

	while(1)
	{
			
			Imu_data_Prepare();//MPU6050滤波
//		u1_printf("123\r\n");
			os_dly_wait (2);
			
	}

}


/**
 * [DR16_SxValue DR16的键值处理，通过事件组]
 */

#define CV_MODE 1

void PWM_Init()
{
	  PWM1=1000;
		PWM2=1000;
		PWM3=1000;
		PWM4=1000;
}
__task void PWM_Task(void)
{
	delay_ms(500);
	//PWM应该不需要初始化，直接输出
  PWM1=1000;
	PWM2=1000;
	PWM3=1000;
	PWM4=1000;
//	delay_ms(500);
//	PWM1=1700;
//	PWM2=1700;
//	PWM3=1700;
//	PWM4=1700;
	while(1)
	{
//	bsp_LedToggle(1);
//	bsp_LedToggle(2);
	delay_ms(50);
	}
}
//junfu   Code1

/**
 * [Boot_Sequence 启动顺序]
 */
__task void Boot_Sequence(void)
{
//	kalman_struct AngleKalman;
//	kalman_init(&AngleKalman,0, 5,0.1,30000);

	delay_ms(2500);
	FollowMode = 1;
	PIDBegin_CAN = 1;
	delay_ms(800);
	FollowMode = 0;
	os_tsk_delete_self();
}

/**
 * [AppTaskCreate  创建任务]
 */
static void AppTaskCreate (void)
{
//HandleDEBUG_DATA = os_tsk_create_user(DEBUG_DATA,             /* 任务函数 */
//	                                      3,                         /* 任务优先级 */
//	                                      &DEBUG_DATAStk,         /* 任务栈 */
//	                                      sizeof(DEBUG_DATAStk)); /* 任务栈大小，单位字节数 */

	HandleDisconnectCheck = os_tsk_create_user(DisconnectCheck,             /* 任务函数 */
	                                      3,                         /* 任务优先级 */
	                                      &DisconnectCheckStk,         /* 任务栈 */
	                                      sizeof(DisconnectCheckStk)); /* 任务栈大小，单位字节数 */
	HandleDR16_DATA_Rec = os_tsk_create_user(DR16_DATA_Rec,              /* 任务函数 */
	                                   2,                       /* 任务优先级 */
	                                   &DR16_DATA_RecStk,          /* 任务栈 */
	                                   sizeof(DR16_DATA_RecStk));  /* 任务栈大小，单位字节数 */
	HandlePitchYaw_Aguest = os_tsk_create_user(PitchYaw_Aguest,              /* 任务函数 */
	                                   2,                       /* 任务优先级 */
	                                   &PitchYaw_AguestStk,          /* 任务栈 */
	                                   sizeof(PitchYaw_AguestStk));  /* 任务栈大小，单位字节数 */

	HandleAHRS_Update_Aguest = os_tsk_create_user(AHRS_Update_Aguest,              /* 任务函数 */
	                                   2,                       /* 任务优先级 */
	                                   &AHRS_Update_AguestStk,          /* 任务栈 */
	                                   sizeof(AHRS_Update_AguestStk));  /* 任务栈大小，单位字节数 */
//	
//		HandleDR16_SxValue = os_tsk_create_user(DR16_SxValue,             /* 任务函数 */
//	                                      6,                         /* 任务优先级 */
//	                                      &DR16_SxValueStk,         /* 任务栈 */
//	                                      sizeof(DR16_SxValueStk)); /* 任务栈大小，单位字节数 */

	HandleImu_data_Aguest = os_tsk_create_user(Imu_data_Aguest,              /* 任务函数 */
	                                   2,                       /* 任务优先级 */
	                                   &Imu_data_AguestStk,          /* 任务栈 */
	                                   sizeof(Imu_data_AguestStk));  /* 任务栈大小，单位字节数 */
	HandleBoot_Sequence = os_tsk_create_user(Boot_Sequence,             /* 任务函数 */
                                      2,                         /* 任务优先级 */
                                      &Boot_SequenceStk,         /* 任务栈 */
                                      sizeof(Boot_SequenceStk)); /* 任务栈大小，单位字节数 */
	HandlePWM_Task = os_tsk_create_user(PWM_Task,             /* 任务函数 */
								 1,                         /* 任务优先级 */
								  &PWM_TaskStk,         /* 任务栈 */
								  sizeof(PWM_TaskStk)); /* 任务栈大小，单位字节数 */
}

/**
 * [AppObjCreate  创建任务通信机制]
 */
static void AppObjCreate (void)
{
	/* 创建消息邮箱 */
	DR16_mailbox_init();//DR16邮箱初始化

	/* 相关初始化 */
	delay_ms(100);
//	Motor_Init(0xf1,0);

}


/**
 * [main  主函数]
 */
int main (void)
{
	/* 初始化外设 */
	bsp_Init();
	ControlTask_Init();
 	os_sys_init_user (TaskStart,              /* 任务函数 */
	                  200,                         /* 任务优先级 */
	                  &TaskStartStk,          /* 任务栈 */
	                  sizeof(TaskStartStk)); /* 任务栈大小，单位字节数 */
	while(1);
}
