#ifndef __MOTOR_H__
#define __MOTOR_H__
#ifdef __cplusplus
 extern "C" {
#endif

#include <stm32f4xx.h>

#define myabs(x) ((x)>0? (x):(-(x)))

#define M_ID_0                 ((uint16_t)0x0001)  /* ID 0 selected */
#define M_ID_1                 ((uint16_t)0x0002)  /* ID 1 selected */
#define M_ID_2                 ((uint16_t)0x0004)  /* ID 2 selected */
#define M_ID_3                 ((uint16_t)0x0008)  /* ID 3 selected */
#define M_ID_4                 ((uint16_t)0x0010)  /* ID 4 selected */
#define M_ID_5                 ((uint16_t)0x0020)  /* ID 5 selected */
#define M_ID_6                 ((uint16_t)0x0040)  /* ID 6 selected */
#define M_ID_7                 ((uint16_t)0x0080)  /* ID 7 selected */
#define M_ID_8                 ((uint16_t)0x0100)  /* ID 8 selected */
#define M_ID_9                 ((uint16_t)0x0200)  /* ID 9 selected */
#define M_ID_10                ((uint16_t)0x0400)  /* ID 10 selected */
#define M_ID_11                ((uint16_t)0x0800)  /* ID 11 selected */
#define M_ID_12                ((uint16_t)0x1000)  /* ID 12 selected */
#define M_ID_13                ((uint16_t)0x2000)  /* ID 13 selected */
#define M_ID_14                ((uint16_t)0x4000)  /* ID 14 selected */
#define M_ID_15                ((uint16_t)0x8000)  /* ID 15 selected */
#define M_ID_All               ((uint16_t)0xFFFF)  /* All ID selected */

typedef enum {GameInfoID = 0x0001, RealBloodChangedDataID = 0x0002,RealShootDataID = 0x0003} FrameID;
typedef __packed struct
{
	uint8_t SOF;
	uint16_t DataLength;
	uint8_t CRC8;
	uint16_t CmdID;
}FrameHeader;
typedef enum{
	BUFF_TYPE_NONE, //无效
	BUFF_TYPE_ARMOR = 0x01, //防御符
	BUFF_TYPE_SUPPLY = 0x04, //加血符
	BUFF_TYPE_BULLFTS= 0x08, //加弹符
}eBuffType;
typedef __packed struct
{
	uint32_t remainTime;
	uint16_t remainLifeValue;
	float realChassisOutV;
	float realChassisOutA;
	uint8_t runeStatus[4];
	uint8_t bigRune0Status;
	uint8_t bigRune1status;
	uint8_t conveyorBelts0:2;
	uint8_t conveyorBelts1:2;
	uint8_t parkingApron0:1;
	uint8_t parkingApron1:1;
	uint8_t parkingApron2:1;
	uint8_t parkingApron3:1;
	__packed struct{
		uint8_t flag; //0 无效， 1 有效
		uint32_t x;
		uint32_t y;
		uint32_t z;
		uint32_t compass;
	} gpsData;
}tGameInfo;
typedef __packed struct
{
	uint8_t weakId:4;
	uint8_t way:4;
	uint16_t value;
}tRealBloodChangedData;
typedef __packed struct
{
	float realShootSpeed;
	float realShootFreq;

}tRealShootDataBase;

typedef __packed struct
{
	tRealShootDataBase Bullet;
	tRealShootDataBase Golf;
}tRealShootData;
#define Motor1  0
#define Motor2  1
#define Motor3  2
#define Motor4  3

#define No1 7  //4500/660 = 6.8

#define _MOTOR_PWM_PERIOD	(unsigned short)(1450-1)//10KHZ
#define _FLY_MAX_OUT 		  (unsigned short)(_MOTOR_PWM_PERIOD+1)
	

extern float ControlOut[3],MotorOut[6];

void Motor_CarFrame(int16_t current_201,int16_t current_202,int16_t current_203,int16_t current_204);
void Motor_Init(int8_t mode,int8_t ret);
void Motor_Other(int16_t current_201,int16_t current_202,int16_t current_203,int16_t current_204);
void Motor_Position(u8 id,int32_t current);
void Motor_GetMassage(u16 index);
void Motor_Power(u16 id,u16 current,u16 volate);

void Motor_YawPitch(int16_t current_205,int16_t current_206);
void Motor_Aguest(u16 Ax,u16 Ay,float Az,float _mAngle);
void X_Axic(int *temp,int power);
void Y_Axic(int *temp,int power);
void Z_Axic(int *temp,int power);
void Motor_DataScale(int *MotorData,int max);
void MOTOR_Init( short id, short mode);
void MOTOR_SetPWM(short pwm,short id);
void Motor(int *temp,u8 motorx,int power);
void Stop(void);
void Motor_PwrAdd(float throttle,float controlout[],float motorout[],float addout);
void Motor_PwmValSet(float motorout[]);
void controlnormalmode_update(float dt);
void Motor_PwrAdd_Control(float DR16_Control, float addout);
//void Motor_Init(void);
#ifdef __cplusplus
}
#endif
#endif
