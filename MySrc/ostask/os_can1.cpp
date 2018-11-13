#include "bsp.h"
#include "cpid.h"
#include "SetParameter.h"


FUZZYPID CVPID(8.5,0.1,1.1,0,-4500, +4500, -3000, +3000, -4500, +4500,-500,500,0,\
		 100.0f,50.0f,20.0f,\
//		 0.3f,0.6f,0.5f,\
		 0.0f,0.0f,0.0f,\
		 0.8,1.0,0.8);
		  0.5f,0.5f,0.5f,\
//		 0.4f,0.2f,0.25f,
		 1.0f,1.5f,3.0f,\
		 0.1,0.1,0.1);
FUZZYPID CVPitchPID(8.5,0.1,1.1,0,-4500, +4500, -3000, +3000, -4500, +4500,-500,500,0,\
		 100.0f,50.0f,20.0f,\
//		 0.3f,0.6f,0.5f,\
		 0.0f,0.0f,0.0f,\
		 0.8,1.0,0.8);
		  0.5f,0.5f,0.5f,\
//		 0.4f,0.2f,0.25f,
		 1.0f,1.5f,3.0f,\
		 0.1,0.1,0.1);
u8 cv_flag,auto_flag;
extern FUZZYPID YawPID,PitchPID;
extern CPID CarFramePID;
extern SCPID TurnplatePID;
extern Parameter_Operater_Union Parameter_Operater;
extern OS_TID HandleDR16_SxValue;

void CV_ToPC(CVStruct *pMsg)
{
	CanTxMsg msg_send = { 0x500, 0x500, CAN_Id_Standard, CAN_RTR_Data, 0x08, {0, 0, 0, 0, 0, 0, 0, 0}};
	*ToCVStructPointer(msg_send.Data) = *ToCVStructPointer(pMsg);
    while(CAN_Transmit(CAN1,&msg_send) == CAN_TxStatus_NoMailBox);
}
void CV_FromPC(CanRxMsg *pMsg)
{
	static u8 filtercnt = 0;
//	u1_printf("%x\t%x\t%d\t%d\t%d\r\n",ToCVStructPointer(pMsg->Data)->ID,ToCVStructPointer(pMsg->Data)->Flag,ToCVStructPointer(pMsg->Data)->CV_X,ToCVStructPointer(pMsg->Data)->CV_Y,ToCVStructPointer(pMsg->Data)->RES);
	if(cv_flag == 1)
	{
		if(ToCVStructPointer(pMsg->Data)->ID == 0x01)
		{
			if(ToCVStructPointer(pMsg->Data)->Flag == 0x01)
			{
				YawPID.SetTarget( (YawPID.Current -  CarFramePID.Current ) - (ToCVStructPointer(pMsg->Data)->CV_X/100.0f)*22.7556f + CVPID.Out);
				filtercnt = 0;
			}
			else
				filtercnt++;
		}
		if(filtercnt == 100)
		{
			CVPID.ITerm = 0;
			CVPitchPID.ITerm = 0;
			YawPID.SetTarget( (YawPID.Current -  CarFramePID.Current));
//			PitchPID.SetTarget( Parameter_Operater.data.PITCH_Encoder_Mid);
			filtercnt = 0;
		}
//				u1_printf("%x\t%x\t%d\t%d\t%d\t%d\r\n",CVValue->State.ID,CVValue->State.Flag,CVAllValue.Strike_Angle_X,CVAllValue.Strike_Angle_Y,\
//			CVValue->CV_X,CVValue->CV_Y\
//			);
//				if(myabs(CVAllValue.Strike_Coordinate_Y)<3)
//					u1_printf("%d\t%d\r\n",CVAllValue.Strike_Angle_Y,tmeee);
//				ANO_Data1_Send(0xf1,CVAllValue.Strike_Coordinate_X);
//				u1_printf("%x\t%x\t%d\t%d\t%d\t%d\r\n",CVValue->State.ID,CVValue->State.Flag,CVAllValue.Strike_Angle_X,CVAllValue.Strike_Angle_Y,\
	CVAllValue.Strike_Coordinate_X,CVAllValue.Strike_Coordinate_Y\
	);

	}
	else if(auto_flag)
	{
		if(ToCVStructPointer(pMsg->Data)->ID == 0x01)
		{
		//自动打击
				static u16 filtercnt = 0;
				if(ToCVStructPointer(pMsg->Data)->Flag == 0x01)
				{
					//StrategicMode = 0;
					YawPID.SetTarget( (YawPID.Current -  CarFramePID.Current ) - (ToCVStructPointer(pMsg->Data)->CV_X/100.0f)*22.7556f + CVPID.Out);
					PitchPID.SetTarget(Parameter_Operater.data.PITCH_Encoder_Mid + (ToCVStructPointer(pMsg->Data)->CV_Y/100.0f)*22.7556f);
					if(ToCVStructPointer(pMsg->Data)->RES != 0x00 && Get_ESC_Flag())
					{
						os_evt_set (BIT_4, HandleDR16_SxValue);
					}
					else
					{
						os_evt_set (BIT_5, HandleDR16_SxValue);//停止小子弹发射
					}
					filtercnt = 0;
				}
				else
				{
						filtercnt ++;
						if(filtercnt > 180)
							filtercnt = 180;
				}
				if(filtercnt == 180)
				{
					static float yaw_dgree = 0.0f;
					static s8 zhengfu = 1; 
					CVPID.ITerm = 0;
					CVPitchPID.ITerm = 0;
					YawPID.SetTarget( (YawPID.Current - CarFramePID.Current) + yaw_dgree*22.7556f);
					yaw_dgree += (0.3*zhengfu);
					if(yaw_dgree >= 15.0f)
						zhengfu = -1;
					if(yaw_dgree <= -15.0)
						zhengfu = 1;
				}
		}
	}
		
}


u8 Get_cv_flag(void)
{
	return cv_flag;
}
void Set_cv_flag(u8 tflag)
{
	cv_flag = tflag;
}

u8 Get_auto_flag(void)
{
	return auto_flag;
}
void Set_auto_flag(u8 tflag)
{
	auto_flag = tflag;
}

void RealShoot_Massage(uint8_t *pMsg)
{
    CanTxMsg msg_send = { 0x502, 0x502, CAN_Id_Standard, CAN_RTR_Data, 0x08, {0, 0, 0, 0, 0, 0, 0, 0}};
	*(tRealShootData*)(msg_send.Data) = *(tRealShootData*)(pMsg);
    // msg_send.Data[0] = 0x55;
//	*((CVStruct*)Ftemp.DATA) = *((CVStruct*)(&pMsg->Data[1]));
	
//	*((CVStruct*)(&msg_send.Data[0])) = *((CVStruct*)pMsg);
    while(CAN_Transmit(CAN1,&msg_send) == CAN_TxStatus_NoMailBox);

}
