#ifndef _OS_CAN1_H___
#define _OS_CAN1_H___
#include <stm32f4xx.h>

typedef __packed struct
{
	int8_t ID;
	int8_t Flag;
	int16_t CV_X;
	int16_t CV_Y;
	int16_t RES;
}CVStruct;
#define ToCVStructPointer(_ptemp)  ((CVStruct*)_ptemp)

void CV_ToPC(CVStruct *pMsg);
void CV_FromPC(CanRxMsg *pMsg);
u8 Get_cv_flag(void);
void Set_cv_flag(u8 tflag);
u8 Get_auto_flag(void);
void Set_auto_flag(u8 tflag);
#endif

