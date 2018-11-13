/**
 * RTXϵͳ��CAN2��������е�ִ�е��Ӻ���
 */

#include "bsp.h"
#include "cpid.h"
#include "filter.h"
#include "SetParameter.h"
#include "Kalman.h"
#define Num 5

//#define Middle 2678  //Yaw���м�����D

extern union Parameter_Operater_Union Parameter_Operater;
extern FUZZYPID PitchPID,YawPID;
extern FUZZYSPID CarFramePID;
extern SCPID TurnplatePID;

extern OS_TID HandleDR16_SxValue;
extern OS_TID HandleDisconnectCheck;
u8 YawChangeFlag=0;
float YAW_Angle_Zero;

MotorEncoderStruct MotorEncoder;
AngleCorrectStruct AngleCorrect;

#define PITCHSPEEDCODER 0
#define YAWSPEEDCODER   1

EncoderStuct Speed6623[2];
void Set_Speed6623(int32_t temp,u8 id)
{
	static int last_position[2];
	static int last_coder[2],this_coder[2],turn_cnt[2];

	this_coder[id] = temp;

	if(this_coder[id] - last_coder[id] > 4096)
	{
		turn_cnt[id]--;
	}
	else if(this_coder[id] - last_coder[id] < -4096)
	{
		turn_cnt[id]++;
	}
	last_coder[id] = this_coder[id];

	Speed6623[id].Position = this_coder[id] + turn_cnt[id] * 8192;
	Speed6623[id].Speed = Speed6623[id].Position - last_position[id];
	last_position[id] = Speed6623[id].Position;
}




EncoderStuct Get_Speed6623(u8 id)
{
	return Speed6623[id];
}

/**
 * [PitchMotor Pitch�������ݴ���]
 * @param msg_rece [CAN���յĽṹ��]
 */
void PitchMotor(CanRxMsg *msg_rece)
{
	// short TempPitch[4];
	static u16 flagPitch = 0, initcountPitch = 0;
	static short filter_bufPitch[Num];
	static unsigned char iPitch=0;

//=====================================================================================
	static short last_angle,this_angle;
	static int turn_cnt,first_flag = 1;
	last_angle = this_angle;
	this_angle = (u16)msg_rece->Data[0] <<8 | (u16)msg_rece->Data[1];

	if(first_flag)//��һ�μ�⴦��
	{
		last_angle = this_angle;
		first_flag = 0;
	}
	if(last_angle < 2048 && this_angle > 6144 )
	{
		turn_cnt--;
	}
	else if(this_angle < 2048 && last_angle > 6144)
	{
		turn_cnt++;
	}
	MotorEncoder.PitchEncoder = this_angle + turn_cnt*8192;
//=====================================================================================


	MotorEncoder.PitchEncoder = Middle_filter(filter_bufPitch,&iPitch,Num,MotorEncoder.PitchEncoder);//��ֵ�˲�
//	Set_Speed6623(MotorEncoder.PitchEncoder,PITCHSPEEDCODER);
//	u1_printf("%d\r\n",MotorEncoder.PitchEncoder);//������
	if(flagPitch)
	{
		PitchPID.SetCurrent((float)MotorEncoder.PitchEncoder);//Pitch�����������PID�����п���
	}
	else
	{
		initcountPitch ++;
		PitchPID.SetCurrent((float)MotorEncoder.PitchEncoder);//Pitch�����������PID,�����Ծ��ɴ������
		if(initcountPitch == 50)//Pitch��Yaw��50������800������ÿ��PID������˳��
		{
			flagPitch = 1;
		}
	}
}

/**
 * [YawMotor Yaw�������ݴ���]
 * @param msg_rece [CAN���յĽṹ��]
 */
void YawMotor(CanRxMsg *msg_rece)
{
	// short TempYaw[4];
	static u16 flagYaw = 0, initcount = 0;
	static short filter_bufYaw[Num];
	static unsigned char iYaw=0;

//=====================================================================================
	static short last_angle,this_angle;
	static int turn_cnt,first_flag=1;
	last_angle = this_angle;
	this_angle = (short)msg_rece->Data[0] <<8 | (short)msg_rece->Data[1];
	if(first_flag)//��һ�μ�⴦��
	{
		last_angle = this_angle;
		first_flag = 0;
	}
	if(last_angle < 2048 && this_angle > 6144 )
	{
		turn_cnt--;
	}
	else if(this_angle < 2048 && last_angle > 6144)
	{
		turn_cnt++;
	}
	MotorEncoder.YawEncoder = this_angle + turn_cnt*8192;
//=====================================================================================
//
	MotorEncoder.YawEncoder = Middle_filter(filter_bufYaw,&iYaw,Num,MotorEncoder.YawEncoder);//�����˲�
	AngleCorrect.w3 = MotorEncoder.YawEncoder;
//	Set_Speed6623(MotorEncoder.YawEncoder,YAWSPEEDCODER);

//	u1_printf("%d\r\n",MotorEncoder.YawEncoder);//������
	 if(YawChangeFlag == 0 && initcount == 40)//
	 {
		 YAW_Angle_Zero =  (MotorEncoder.YawEncoder - Parameter_Operater.data.YAW_Encoder_Mid)/CodedDiscToAngle;//����������Ǻ����̳�ʼ�ǶȲ�����������У��
		 YawChangeFlag = 1;//ʹ�������������PID�����п���
	 }
	if(flagYaw)
	{
		{
			if(FollowMode ==1)
			{
				if(Get_bullet_flag() == 0)
					YawPID.SetCurrent((MotorEncoder.YawEncoder - Parameter_Operater.data.YAW_Encoder_Mid  + YawPID.Target));		
				else if(Get_bullet_flag() == 1)
					YawPID.SetCurrent((MotorEncoder.YawEncoder - Parameter_Operater.data.YAW_Encoder_Mid - 90*CodedDiscToAngle  + YawPID.Target));
				else if(Get_bullet_flag() == 2)	
					YawPID.SetCurrent((MotorEncoder.YawEncoder - Parameter_Operater.data.YAW_Encoder_Mid + 90*CodedDiscToAngle  + YawPID.Target));		
			}
			else if(FollowMode == 0)
			{
				if(Get_bullet_flag() == 0)
					CarFramePID.SetCurrent((float)(MotorEncoder.YawEncoder - Parameter_Operater.data.YAW_Encoder_Mid));//Yaw�����������PID�����п���
				else if(Get_bullet_flag() == 1)
					CarFramePID.SetCurrent((float)(MotorEncoder.YawEncoder - Parameter_Operater.data.YAW_Encoder_Mid - 90*CodedDiscToAngle));//Yaw�����������PID�����п���
				else if(Get_bullet_flag() == 2)
					CarFramePID.SetCurrent((float)(MotorEncoder.YawEncoder - Parameter_Operater.data.YAW_Encoder_Mid + 90*CodedDiscToAngle));//Yaw�����������PID�����п���
//				u1_printf("%d\r\n",(int)CarFramePID.Current);
//				ANO_Data3_Send(0xf1,((float)camera_angle - Middle)/22.7556f,camera_angle/22.7556f,(CarFramePID.Current)/22.7556f);
			}
		}
	}
	else
	{
		initcount ++;
		if(initcount == 50)
		{
			flagYaw = 1;
		}
	}
}



/**
 * [YawAngle �������������ݴ���]
 * @param msg_rece [CAN���յĽṹ��]
 */
void YawAngle(CanRxMsg *msg_rece)
{
	float YAW_Angle;
	static float this_yaw_angle,last_yaw_angle;
    float temp_yaw_angle;
	static int turn_cnt;

	temp_yaw_angle = (int32_t)(msg_rece->Data[0]<<24)|(int32_t)(msg_rece->Data[1]<<16)
	| (int32_t)(msg_rece->Data[2]<<8) | (int32_t)(msg_rece->Data[3]);

	last_yaw_angle = this_yaw_angle;
	this_yaw_angle = -((float)temp_yaw_angle*0.01f);
	if(this_yaw_angle - last_yaw_angle > 180)
	{
		turn_cnt++;
	}
	else if(this_yaw_angle - last_yaw_angle < -180)//180
	{
		turn_cnt--;
	}
	YAW_Angle = -(this_yaw_angle + turn_cnt*360);

	if(YawChangeFlag == 1)
	{
		if(FollowMode == 0)
		{
			YawPID.SetCurrent(((YAW_Angle - YAW_Angle_Zero)*CodedDiscToAngle));
//			camera_a = yaw_angle_6050 + Middle/CodedDiscToAngle;
		}
		else if(FollowMode == 1)
			YAW_Angle_Zero = YAW_Angle - YawPID.Target/CodedDiscToAngle;
	}

}



/**
 * [YawAngle_6050 6050���������ݴ���]
 * @param msg_rece [CAN���յĽṹ��]
 */
void YawAngle_6050(CanRxMsg *msg_rece)
{
	float  yaw_angle_6050;
	yaw_angle_6050 = *((__packed float *)msg_rece->Data);
	AngleCorrect.w2 = yaw_angle_6050;
//	ANO_Data1_Send(0xf1,yaw_angle_6050*100);
//	u1_printf("%d\r\n",(short)(yaw_angle_6050*100));
	if(YawChangeFlag == 1)
	{
		if(FollowMode == 0)
		{
			YawPID.SetCurrent(((yaw_angle_6050 - YAW_Angle_Zero)*CodedDiscToAngle));
//			camera_a = yaw_angle_6050 + Middle/CodedDiscToAngle;
		}
		else if(FollowMode == 1)
			YAW_Angle_Zero = yaw_angle_6050 - YawPID.Target/CodedDiscToAngle;
	}
}

void YawAngle_ADXRS622(CanRxMsg *msg_rece)
{
	float yaw_angle_ADXRS622;
	yaw_angle_ADXRS622 = *((__packed float *)&msg_rece->Data)/1000.0f;
	
	AngleCorrect.w2 = yaw_angle_ADXRS622;
	if(YawChangeFlag == 1)
	{
		if(FollowMode == 0)
		{
			YawPID.SetCurrent(((yaw_angle_ADXRS622 - YAW_Angle_Zero)*CodedDiscToAngle));
		}
		else if(FollowMode == 1)
			YAW_Angle_Zero = yaw_angle_ADXRS622 - YawPID.Target/CodedDiscToAngle;
	}
}


MotorEncoderStruct GetMotorEncoder(void)
{
	return	MotorEncoder;
}

