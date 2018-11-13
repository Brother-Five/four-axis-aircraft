#include "mpu6050.h"
#include "myiic.h"
#include "bsp.h"
#include "includes.h"
#include "Kalman.h"

//////////////////////////////////////////////////////////////////////////////////
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//MPU6050 ��������
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/9
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////


OFFSET OffSet;
Data_To_Imu  acc, velocity;
Data_To_Imu  ACC, VELOCITY;



// Fast inverse square-root
/**************************ʵ�ֺ���********************************************
*����ԭ��:	   float InvSqrt(float x)
*��������:	   ���ټ��� 1/Sqrt(x) 	
��������� Ҫ�����ֵ
��������� ���
*******************************************************************************/
float Invsqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}


///**
// *kalman_filter - �������˲���
// *@kalman:�������ṹ��
// *@measure������ֵ
// *�����˲����ֵ
// */
//float Kalman_filter(kalman_struct *kalman, float measure)
//{
//		u1_printf("123\r\n");
//    /* Predict */
//    kalman->x = kalman->A * kalman->x;//%x�������������һ��ʱ���ĺ������ֵ��������Ϣ����
//    kalman->p = kalman->A * kalman->A * kalman->p + kalman->q;  /*������������� p(n|n-1)=A^2*p(n-1|n-1)+q */

//    /* Measurement */
//    kalman->gain = kalman->p * kalman->H / (kalman->p * kalman->H * kalman->H + kalman->r);
//    kalman->x = kalman->x + kalman->gain * (measure - kalman->H * kalman->x);//���ò������Ϣ���ƶ�x(t)�Ĺ��ƣ�����������ƣ����ֵҲ�������
//    kalman->p = (1 - kalman->gain * kalman->H) * kalman->p;//%������������

//    return kalman->x;
//}

//��ʼ��MPU6050
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Init(void)
{
	u8 res;
	IIC_Init();//��ʼ��IIC����
	
//	RunTime(StartCheck);
//	delay_ms(1000);
//	RunTime(StopCheck);
		
	
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//��λMPU6050
	
    delay_ms_nos(100);
	
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//����MPU6050
	MPU_Set_Gyro_Fsr(3);					//�����Ǵ�����,��2000dps
	MPU_Set_Accel_Fsr(2);					//���ٶȴ�����,��2g//��8g
	MPU_Set_Rate(1000);						//���ò�����50Hz
	MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//�ر������ж�
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C��ģʽ�ر�
	MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//�ر�FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT���ŵ͵�ƽ��Ч
	res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
	if(res==MPU_ADDR)//����ID��ȷ
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//����CLKSEL,PLL X��Ϊ�ο�
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//���ٶ��������Ƕ�����
		MPU_Set_Rate(1000);						//���ò�����Ϊ50Hz
 	}else return 1;
	return 0;
}
//����MPU6050�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ��
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//���������������̷�Χ
}
//����MPU6050���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ��
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȴ����������̷�Χ
}
//����MPU6050�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ��
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6;
	return MPU_Write_Byte(MPU_CFG_REG,data);//�������ֵ�ͨ�˲���
}
//����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ��
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���
 	return MPU_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}

//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��)
short MPU_Get_Temperature(void)
{
    u8 buf[2];
    short raw;
	float temp;
	MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf);
    raw=((u16)buf[0]<<8)|buf[1];
    temp=36.53+((double)raw)/340;
    return temp*100;;
}
//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������

u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    u8 buf[6],res;
	
	res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=(short)(((u16)buf[0]<<8)|buf[1]);
		*gy=(short)(((u16)buf[2]<<8)|buf[3]);
		*gz=(short)(((u16)buf[4]<<8)|buf[5]);
////		*gx=(short)(((u16)buf[0]<<8)|buf[1]) + 21;
////		*gy=(short)(((u16)buf[2]<<8)|buf[3]) - 10;
////		*gz=(short)(((u16)buf[4]<<8)|buf[5]) + 3;
	}
    return res;;
}
//�õ����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    u8 buf[6],res;
	res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];
		*ay=((u16)buf[2]<<8)|buf[3];
		*az=((u16)buf[4]<<8)|buf[5];
	}
    return res;;
}
//IIC����д
//addr:������ַ
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i;
    IIC_Start();
	IIC_Send_Byte((addr<<1)|0);//����������ַ+д����
	if(IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		IIC_Stop();
		return 1;
	}
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ��
	for(i=0;i<len;i++)
	{
		IIC_Send_Byte(buf[i]);	//��������
		if(IIC_Wait_Ack())		//�ȴ�ACK
		{
			IIC_Stop();
			return 1;
		}
	}
    IIC_Stop();
	return 0;
}
//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
 	IIC_Start();
	IIC_Send_Byte((addr<<1)|0);//����������ַ+д����
	if(IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		IIC_Stop();
		return 1;
	}
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ��
    IIC_Start();
	IIC_Send_Byte((addr<<1)|1);//����������ַ+������
    IIC_Wait_Ack();		//�ȴ�Ӧ��
	while(len)
	{
		if(len==1)*buf=IIC_Read_Byte(0);//������,����nACK
		else *buf=IIC_Read_Byte(1);		//������,����ACK
		len--;
		buf++;
	}
    IIC_Stop();	//����һ��ֹͣ����
	return 0;
}
//IICдһ���ֽ�
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
u8 MPU_Write_Byte(u8 reg,u8 data)
{
    IIC_Start();
	IIC_Send_Byte((MPU_ADDR<<1)|0);//����������ַ+д����
	if(IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		IIC_Stop();
		return 1;
	}
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ��
	IIC_Send_Byte(data);//��������
	if(IIC_Wait_Ack())	//�ȴ�ACK
	{
		IIC_Stop();
		return 1;
	}
    IIC_Stop();
	return 0;
}
//IIC��һ���ֽ�
//reg:�Ĵ�����ַ
//����ֵ:����������
u8 MPU_Read_Byte(u8 reg)
{
	u8 res;
    IIC_Start();
	IIC_Send_Byte((MPU_ADDR<<1)|0);//����������ַ+д����
	IIC_Wait_Ack();		//�ȴ�Ӧ��
    IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ��
    IIC_Start();
	IIC_Send_Byte((MPU_ADDR<<1)|1);//����������ַ+������
    IIC_Wait_Ack();		//�ȴ�Ӧ��
	res=IIC_Read_Byte(0);//��ȡ����,����nACK
    IIC_Stop();			//����һ��ֹͣ����
	return res;
}


//**********���ݶ�ȡ******************/
//���ݶ�ȡ���ڽ���ʹ��
void MPU6050_Read_To_Use_(Origial_DATA*Origial_DATA__ACC_XYZ,Origial_DATA*Origial_DATA__GYRO_XYZ,OFFSET*offset)
{
	uint8_t Mpu6050_Data_Buf[14]={0};//������ԭʼ���ݽ��ջ���
	int32_t G_X=0,G_Y=0,G_Z=0,A_X=0,A_Y=0,A_Z=0;
	int32_t temp=0;
	uint8_t res;
	
	MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG,14,Mpu6050_Data_Buf);//ģ��IIC��ȡ����

	
	 A_X=(((int16_t)Mpu6050_Data_Buf[0]<<8)|Mpu6050_Data_Buf[1])-offset->AX_Offset;
	 A_Y=(((int16_t)Mpu6050_Data_Buf[2]<<8)|Mpu6050_Data_Buf[3])-offset->AY_Offset;
	 A_Z=(((int16_t)Mpu6050_Data_Buf[4]<<8)|Mpu6050_Data_Buf[5])+offset->AZ_Offset;
	
	// Temperature=	(((int16_t)Mpu6050_Data_Buf[6]<<8)|Mpu6050_Data_Buf[7]);//�ڲ��¶�ADC
	 G_X=(((int16_t)Mpu6050_Data_Buf[8]<<8)|Mpu6050_Data_Buf[9])-offset->GX_Offset;
	 G_Y=(((int16_t)Mpu6050_Data_Buf[10]<<8)|Mpu6050_Data_Buf[11])-offset->GY_Offset;
	 G_Z=(((int16_t)Mpu6050_Data_Buf[12]<<8)|Mpu6050_Data_Buf[13])-offset->GZ_Offset;
//	 u1_printf("%d\t%d\t%d\t%d\t%d\t%d\r\n",A_X,A_Y,A_Z,G_X,G_Y,G_Z);
	 /***�����޷�***/
// 	if(G_X>Data_Max) G_X=Data_Max;
// 	if(G_X<Data_Min) G_X=Data_Min;
// 	if(G_Y>Data_Max) G_Y=Data_Max;
// 	if(G_Y<Data_Min) G_Y=Data_Min;
// 	if(G_Z>Data_Max) G_Z=Data_Max;
// 	if(G_Z<Data_Min) G_Z=Data_Min;
// 	if(A_X>Data_Max) A_X=Data_Max;
// 	if(A_X<Data_Min) A_X=Data_Min;
// 	if(A_Y>Data_Max) A_Y=Data_Max;
// 	if(A_Y<Data_Min) A_Y=Data_Min;
//  if(A_Z>Data_Max) A_Z=Data_Max;
//  if(A_Z<Data_Min) A_Z=Data_Min;
	
	#if IMU_CHIP_ROTATION==0  //0����MPU6050+X��Ϊ��ͷǰ��
	

	#elif IMU_CHIP_ROTATION==1//1����MPU6050-X��Ϊ��ͷǰ��
	A_X=-A_X;
	A_Y=-A_Y;
	
	G_X=-G_X;
	G_Y=-G_Y;
	
	#elif IMU_CHIP_ROTATION==2//2����MPU6050+Y��Ϊ��ͷǰ��
	temp=A_X;
	A_X = A_Y;
	A_Y =-temp;
	
	temp=G_X;
	G_X = G_Y;
	G_Y = -temp;
	
	#elif IMU_CHIP_ROTATION==3//3����MPU6050-Y��Ϊ��ͷǰ��
	temp=A_X;
	A_X=-A_Y;
	A_Y= temp;

	temp=G_X;
	G_X =-G_Y;
	G_Y = temp;
	#endif
	
	Origial_DATA__ACC_XYZ->X=(int16_t)A_X;
	Origial_DATA__ACC_XYZ->Y=(int16_t)A_Y;
	Origial_DATA__ACC_XYZ->Z=(int16_t)A_Z;
	Origial_DATA__GYRO_XYZ->X=(int16_t)G_X;
	Origial_DATA__GYRO_XYZ->Y=(int16_t)G_Y;
	Origial_DATA__GYRO_XYZ->Z=(int16_t)G_Z;
	
// 	u1_printf("%d\t %d\t %d\t %d\t %d\t %d\r\n",Origial_DATA__ACC_XYZ->X,Origial_DATA__ACC_XYZ->Y,
// 	Origial_DATA__ACC_XYZ->Z,Origial_DATA__GYRO_XYZ->X,Origial_DATA__GYRO_XYZ->Y,Origial_DATA__GYRO_XYZ->Z);

}

//**********���ݶ�ȡ******************/
//���ݶ�ȡ������ƫ����
void MPU6050_Read_To_Calculate(Origial_DATA *Origial_DATA__ACC_XYZ,Origial_DATA *Origial_DATA__GYRO_XYZ)
{
	uint8_t Mpu6050_Data_Buf[14]={0};//������ԭʼ���ݽ��ջ���
	int32_t G_X=0,G_Y=0,G_Z=0,A_X=0,A_Y=0,A_Z=0;
	int32_t temp=0;
	MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG,14,Mpu6050_Data_Buf);//ģ��IIC��ȡ����
	
	 A_X=	(((int16_t)Mpu6050_Data_Buf[0]<<8)|Mpu6050_Data_Buf[1]);
	 A_Y=	(((int16_t)Mpu6050_Data_Buf[2]<<8)|Mpu6050_Data_Buf[3]);
	 A_Z=	(((int16_t)Mpu6050_Data_Buf[4]<<8)|Mpu6050_Data_Buf[5]);
	// Temperature=	(((int16_t)Mpu6050_Data_Buf[6]<<8)|Mpu6050_Data_Buf[7]);//�ڲ��¶�ADC
	 G_X=	(((int16_t)Mpu6050_Data_Buf[8]<<8)|Mpu6050_Data_Buf[9]);
	 G_Y=	(((int16_t)Mpu6050_Data_Buf[10]<<8)|Mpu6050_Data_Buf[11]);
	 G_Z=	(((int16_t)Mpu6050_Data_Buf[12]<<8)|Mpu6050_Data_Buf[13]);

		#if IMU_CHIP_ROTATION==0  //0����MPU6050+X��Ϊ��ͷǰ��
	

	#elif IMU_CHIP_ROTATION==1//1����MPU6050-X��Ϊ��ͷǰ��
	A_X=-A_X;
	A_Y=-A_Y;
	
	G_X=-G_X;
	G_Y=-G_Y;
	
	#elif IMU_CHIP_ROTATION==2//2����MPU6050+Y��Ϊ��ͷǰ��
	temp=A_X;
	A_X = A_Y;
	A_Y =-temp;
	
	temp=G_X;
	G_X = G_Y;
	G_Y = -temp;
	
	#elif IMU_CHIP_ROTATION==3//3����MPU6050-Y��Ϊ��ͷǰ��
	temp=A_X;
	A_X=-A_Y;
	A_Y= temp;

	temp=G_X;
	G_X =-G_Y;
	G_Y = temp;
	#endif
	
	Origial_DATA__ACC_XYZ->X=(int16_t)A_X;
	Origial_DATA__ACC_XYZ->Y=(int16_t)A_Y;
	Origial_DATA__ACC_XYZ->Z=(int16_t)A_Z;
	Origial_DATA__GYRO_XYZ->X=(int16_t)G_X;
	Origial_DATA__GYRO_XYZ->Y=(int16_t)G_Y;
	Origial_DATA__GYRO_XYZ->Z=(int16_t)G_Z;
}

/************���ݱ궨**************/
//���ܣ��ԼӼơ����������ݽ��б궨
//���룺�OOFF_SET *ACC_OFF,OFF_SET *GYRO_OFF
//�������
Origial_DATA Sensor_Acc,Sensor_Gyro;//������ԭʼ����

const unsigned char offsetlength = 100;  //������ƫ�����������ݴ���

void MPU6050_Data_Check_(OFFSET *offset)
{
	u8 num=0;
	int32_t temp[6]={0};
//	LED1_ON();//
	for(num=0;num<offsetlength;num++)
	{
	     MPU6050_Read_To_Calculate(&Sensor_Acc,&Sensor_Gyro);	
		
	   temp[0]+=Sensor_Acc.X;
		 temp[1]+=Sensor_Acc.Y;
		 temp[2]+=Sensor_Acc.Z;
		
		 temp[3]+=Sensor_Gyro.X;
		 temp[4]+=Sensor_Gyro.Y;
		 temp[5]+=Sensor_Gyro.Z;
		 delay_ms(5);
	}
	
	offset->AX_Offset=temp[0]/offsetlength;
	offset->AY_Offset=temp[1]/offsetlength;
	offset->AZ_Offset=ACC_8G_SCALE-temp[2]/offsetlength;
	
	offset->GX_Offset=temp[3]/offsetlength;
	offset->GY_Offset=temp[4]/offsetlength;
	offset->GZ_Offset=temp[5]/offsetlength;
// 	STMFLASH_Write(Read_Addr,(u16*)off_set,6);//���ݱ���
	delay_ms(20);
//	LED1_OFF();
//	printf("offset:%d\t %d\t %d\t %d\t %d\t %d\r\n",off_set[0],off_set[1],off_set[2],off_set[3],off_set[4],off_set[5]);
	
}

/******************/
//���������ݻ����˲����� �Ӽ�����Kalman�˲�
//����������˲������ת�������õ��ļ��ٶ�(g/m^2)�����ٶ�����(rad/s)
//���룺ԭʼ���� �궨�õ�����ƫ
//�˲���Ȳ��˹��󣬷���Ӵ�������ʱ�����Ͷ�̬ЧӦ
const float acc_denominator = 417.95f; //�Ӽ������� 4096/9.8=417.95 ���ݵ����������ٶ�9.788m/s^2
const float gyro_denominator = 1877.2f; //������������ ת��Ϊrad/S
const unsigned char accfilter_Num = 6; //ƽ���˲����
const unsigned char gyrofilter_Num = 6; //ƽ���˲����
//const unsigned char offsetlength = 100;  //������ƫ�����������ݴ���
const float gyroscale=gyro_denominator*gyrofilter_Num;
kalman_struct KalmanfilterAccx,KalmanfilterAccy,KalmanfilterAccz;
void MPU6050_Data_Read_Analys_Kalman(Data_To_Imu *GYRO,Data_To_Imu *Acc)
{
	u8 j=0;
	float a=0.3f,b=0.7f;
	static u8 counter=0;//��������
	float temp4=0,temp5=0,temp6=0;
	static float ACC_TEMP[3]={0};
	static int16_t BUF4[gyrofilter_Num]={0},BUF5[gyrofilter_Num]={0},BUF6[gyrofilter_Num]={0};

     MPU6050_Read_To_Use_(&Sensor_Acc,&Sensor_Gyro,&OffSet);//��ȡ���ݣ��궨

/*******************�������˲�*****************************/
	Acc->X=kalman_filter(&KalmanfilterAccx,Sensor_Acc.X);
	
	u1_printf("%d\r\n",Acc->X);
//	Acc->Y=kalman_filter(&KalmanfilterAccy,Sensor_Acc.Y);
	Acc->Z=kalman_filter(&KalmanfilterAccz,Sensor_Acc.Z);
//	Acc->X=a*Sensor_Acc.X+b*ACC_TEMP[0];
//	ACC_TEMP[0]=Acc->X;

	Acc->Y=a*Sensor_Acc.Y+b*ACC_TEMP[1];
	ACC_TEMP[1]=Acc->Y;

//	Acc->Z=a*Sensor_Acc.Z+b*ACC_TEMP[2];
//	ACC_TEMP[2]=Acc->Z;

	 BUF4[counter]=Sensor_Gyro.X;
	 BUF5[counter]=Sensor_Gyro.Y;
	 BUF6[counter]=Sensor_Gyro.Z;
	
	for(j=0;j<gyrofilter_Num;j++)
	{
		temp4+= BUF4[j];
		temp5+= BUF5[j];
		temp6+= BUF6[j];
	}
	
	
	 Acc->X/=acc_denominator;//g/m^2
	 Acc->Y/=acc_denominator;
	 Acc->Z/=acc_denominator;

	 GYRO->X=temp4/gyroscale;//rad/s
	 GYRO->Y=temp5/gyroscale;
	 GYRO->Z=temp6/gyroscale;
	 u1_printf("%7.3f\t %7.3f\t %7.3f\t %7.3f\t %7.3f\t %7.3f\r\n", Acc->X, Acc->Y, Acc->Z, GYRO->X, GYRO->Y, GYRO->Z);
	counter++;
	if(counter==gyrofilter_Num) counter=0;
	
	
}




/******************/
//˫�����˲�����
//����������˲������ת�������õ��ļ��ٶ�(g/m^2)�����ٶ�����(rad/s)
//���룺ԭʼ���� �궨�õ�����ƫ
//�˲���Ȳ��˹��󣬷���Ӵ�������ʱ�����Ͷ�̬ЧӦ
void MPU6050_Data_Read_Analys(Data_To_Imu *GYRO,Data_To_Imu *Acc)
{
	u8 i=0,j=0;
	static u8 count=0,counter=0;//��������
	int32_t temp1=0,temp2=0,temp3=0,temp4=0,temp5=0,temp6=0;
	static int16_t BUF1[accfilter_Num]={0},BUF2[accfilter_Num]={0},BUF3[accfilter_Num]={0};
	static int16_t BUF4[gyrofilter_Num]={0},BUF5[gyrofilter_Num]={0},BUF6[gyrofilter_Num]={0};
	
     MPU6050_Read_To_Use_(&Sensor_Acc,&Sensor_Gyro,&OffSet);//��ȡ���ݣ��궨

   BUF1[count]=Sensor_Acc.X;
	 BUF2[count]=Sensor_Acc.Y;
	 BUF3[count]=Sensor_Acc.Z;
	
	 BUF4[counter]=Sensor_Gyro.X;
	 BUF5[counter]=Sensor_Gyro.Y;
	 BUF6[counter]=Sensor_Gyro.Z;

	for(i=0;i<accfilter_Num;i++)
	{
		temp1+= BUF1[i];
		temp2+= BUF2[i];
		temp3+= BUF3[i];
	}
	
	temp1/=accfilter_Num;
	temp2/=accfilter_Num;
	temp3/=accfilter_Num;
	
	for(j=0;j<gyrofilter_Num;j++)
	{
		temp4+= BUF4[j];
		temp5+= BUF5[j];
		temp6+= BUF6[j];
	}

	temp4/=gyrofilter_Num;
	temp5/=gyrofilter_Num;
	temp6/=gyrofilter_Num;
	
	 Acc->X=temp1/acc_denominator;//g
	 Acc->Y=temp2/acc_denominator;
	 Acc->Z=temp3/acc_denominator;
		
	 GYRO->X=temp4/gyro_denominator;//rad/s
	 GYRO->Y=temp5/gyro_denominator;	
	 GYRO->Z=temp6/gyro_denominator;
		
	count++;
	if(count==accfilter_Num) count=0;	
	counter++;
	if(counter==gyrofilter_Num) counter=0;
	
//	u1_printf("%7.3f\t %7.3f\t %7.3f\t %7.3f\t %7.3f\t %7.3f\r\n", Acc->X, Acc->Y, Acc->Z, GYRO->X, GYRO->Y, GYRO->Z);
}

/*���ݽ���*/
//�����ǲ�õĽ��ٶ�������imuupdate���������ı�ֵ��������õ�ַ���ݸı����ֵ�Ļ�
void MPU6050_Data_Exchage(Data_To_Imu*accin,Data_To_Imu*gyroin,Data_To_Imu*accout,Data_To_Imu*gyroout)
{
	accout->X=accin->X;
	accout->Y=accin->Y;
	accout->Z=accin->Z;

	gyroout->X=gyroin->X;
	gyroout->Y=gyroin->Y;
	gyroout->Z=gyroin->Z;
//	u1_printf("%7.3f\t %7.3f\t %7.3f\t %7.3f\t %7.3f\t %7.3f\r\n", accout->X, accout->Y, accout->Z, gyroout->X, gyroout->Y, gyroout->Z);
}



#define KP  1.09f//2.0f
float Kp = KP;//���ھ���ACC HMC���ݶ�GYRO�����Ĵ�С   �������ٶȼ�/�����������ٶ�              
float Ki = 0.001f; //��������������Ư�� 

static float exInt = 0.0f, eyInt = 0.0f, ezInt=0.0f;//���ֻ��ڱ���
float q[4]={1.0f,0.0f,0.0f,0.0f};//ȫ����Ԫ��
//float gyro_yaw_scale=0.9;

extern Data_To_Imu ACC,VELOCITY;	
EULLA Eulla/*,LastEulla*/;//�����˲����ŷ���ǽṹ��
INERIAL Inerialacc;
/**************����ŷ����****************/
//����:S.O.H. Madgwick ���� On September 10th,2010
//����:MPU6050���ݻ����˲�����ŷ��������
//���룺���ٶ�(rad/s)�����ٶ�(m/s^2)��������������(��).
//�������Ԫ����ŷ���Ǹ���!
//���أ���
void imu_update(Data_To_Imu*velocity,Data_To_Imu*acc,EULLA*eulla,float dt)
{
  float vx=0.0f, vy=0.0f, vz=0.0f;//��ǰ��ŷ����(��Ԫ��)�Ļ����������ϵ�ϣ����������������λ����
  float ex=0.0f, ey=0.0f, ez=0.0f;//�����ǻ��ֺ����̬�ͼ����������̬���
  //double t[3][3]; //��ŷ������Ҿ�������
  float norm=0.0f;//ģ��������
	//double yaw_mag;//�����ƽ������ƫ��
  float gyro_vel_z=velocity->Z;
/*************Ϊ�������ݼ���׼��*********/
	float q0q0 = q[0]*q[0];
	float q0q1 = q[0]*q[1];
	float q0q2 = q[0]*q[2];
	//float q0q3 = q[0]*q[3];	
	float q1q1 = q[1]*q[1];
	//float q1q2 = q[1]*q[2];	
	float q1q3 = q[1]*q[3];
	float q2q2 = q[2]*q[2];
	float q2q3 = q[2]*q[3];
	float q3q3 = q[3]*q[3];
	
//	u1_printf("1\r\n");
//	u1_printf("%7.3f\t%7.3f\t%7.3f\r\n",acc->X,acc->Y,acc->Z );
	if(acc->X*acc->Y*acc->Z==0) return;	
//	u1_printf("2\r\n");
	if(gyro_vel_z<0.0015f) gyro_vel_z=0;
//u1_printf("3\r\n");
	Inerialacc.acctemp=(acc->X*acc->X + acc->Y*acc->Y + acc->Z*acc->Z)/(9.8f*9.8f);
	if(0.55f<Inerialacc.acctemp && Inerialacc.acctemp<1.21f)
	{
		Kp=KP;
	}
	else
	{
		Kp=0;//�����ü��ٶȼƲ������������ǻ�������ĽǶ�
	}
	norm=Invsqrt(acc->X*acc->X + acc->Y*acc->Y + acc->Z*acc->Z);//�����󷽸�����
//	u1_printf("4\r\n");
	Inerialacc.acclength=acc->Z*norm;//�淶�����acc->Z
	if(Inerialacc.acclength<0.05f)
	{
		Inerialacc.acccorr=0;
	}
	else
	{
		Inerialacc.acccorr=sinf(acosf(Inerialacc.acccorr));
	}
	  acc->X = acc->X * norm;
	  acc->Y = acc->Y * norm;//�ѼӼ���λ����ת��Ϊ��λ����
	  acc->Z = acc->Z * norm;//���ڻ�����������õ���������
				
	  vx = 2*(q1q3 - q0q2);	//�����ǻ��ֺ��������̬											
	  vy = 2*(q0q1 + q2q3);//��ǰ��ŷ����(��Ԫ��)�Ļ����������ϵ�ϣ����������������λ����
	  vz = q0q0 - q1q1 - q2q2 + q3q3 ;//�պ��Ƿ��������е����е�����Ԫ��
//	  vz = -0.5f+q0q0 + q3q3 ;//�պ��Ƿ��������е����е�����Ԫ��
	
	  ex = (acc->Y*vz - acc->Z*vy) ; //�����ǻ��ֺ��������̬�ͼӼƲ��������̬֮������                          					
	  ey = (acc->Z*vx - acc->X*vz) ;
	  ez = (acc->X*vy - acc->Y*vx) ;
	//����֮������������������(Ҳ��������������)����ʾ��ex,ey,ez����������������vxyz ��acc->x y z�Ĳ�ˡ�
	//�����������Ծ���λ�ڻ�������ϵ�ϵģ��������ǻ������Ҳ�ǻ��ڻ�������ϵ�����Ҳ�˵Ĵ�С�������ǻ�������
	//���ȣ������������������ǡ�
	/*********������********/		
	  exInt = exInt + ex * Ki;							 
	  eyInt = eyInt + ey * Ki;
	  ezInt = ezInt + ez * Ki;
/*********�ò���������PI������������ƫ********/
  velocity->X = velocity->X + Kp*ex + exInt;//P��I����					   							
  velocity->Y = velocity->Y + Kp*ey + eyInt;
  velocity->Z = velocity->Z + Kp*ez + ezInt;//����Z��ת���޷�ͨ���ӼƲ��������Բ���Ҫ���ֻ���
													//�����gz����û�й۲��߽��н��������Ư�ƣ����ֳ����ľ��ǻ����������Լ�
 		   							
	/********���������������Ԫ����halfTΪ�������ڵ�һ��**********/				   
	// integrate quaternion rate and normalise //��Ԫ�����·���
	q[0] = q[0] + (-q[1]*velocity->X - q[2]*velocity->Y - q[3]*velocity->Z)*dt/2;
	q[1] = q[1] + ( q[0]*velocity->X + q[2]*velocity->Z - q[3]*velocity->Y)*dt/2;
	q[2] = q[2] + ( q[0]*velocity->Y - q[1]*velocity->Z + q[3]*velocity->X)*dt/2;
	q[3] = q[3] + ( q[0]*velocity->Z + q[1]*velocity->Y - q[2]*velocity->X)*dt/2;

	norm = Invsqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);//�����󷽸�����
	q[0] = q[0] * norm;//������Ԫ��
	q[1] = q[1] * norm;
	q[2] = q[2] * norm;
	q[3] = q[3] * norm;
	
/***********�ٴθ�������*************/
//   q0q0=q[0]*q[0];
//   q0q1=q[0]*q[1];
//   q0q2=q[0]*q[2];
//   //q0q3=q[0]*q[3];
//   q1q1=q[1]*q[1];
//  // q1q2=q[1]*q[2];
//   q1q3=q[1]*q[3];
//   q2q2=q[2]*q[2];
//   q2q3=q[2]*q[3];
//   q3q3=q[3]*q[3];
	
/*************���·������Ҿ���***********/
//     t[1][2]=2.0*(q2q3+q0q1); //y 
//     t[2][2]=q0q0-q1q1-q2q2+q3q3; //z
//     t[0][2]=2.0*(q1q3-q0q2); 	//x
//     t[0][1]=2.0*(q1q2+q0q3);
//     t[0][0]=q0q0+q1q1-q2q2-q3q3;

//	eulla->yaw = atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3]* q[3] + 1)* 57.3; // �� z����ת
	eulla->yaw  += gyro_vel_z*dt*57.3f;
 	eulla->pitch = sinf(-2 * q[1] * q[3] + 2 * q[0]* q[2])* 57.3; //�� y����ת -90��~+90��
 	eulla->roll  = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2]* q[2] + 1) * 57.3; // �� x����ת -180��~+180��
	
//	delay_ms(5);
//u1_printf("5\r\n");
}





/*AcczWithoutG_Get*/
/*�����ȥ�������ٶȵļ��ٶ�*/
//������q[]ȫ����Ԫ������ǰ������̬�ı�ʾ������һ��
//			acc�����õ��Ļ�����ٶ�����
//����ֵ����ȥ�������ٶȵļ��ٶȴ�С
float AcczWithoutG_Get(float q[],Data_To_Imu acc)
{
	float w=q[0],x=q[1],y=q[2],z=q[3];
	float ax=0.0f,ay=0.0f,az=0.0f;
	float accwithoutg=0.0f;
	ax=2*(w*y-x*z);
	ay=2*(w*x+y*z);
	az=w*w-x*x-y*y+z*z;
	
	accwithoutg=ax*acc.X+ay*acc.Y+az*acc.Z-9.8f;
//	u1_printf("%7.3f\r\n",accwithoutg);
	
	return accwithoutg;
}
/**************************/
void AHRS_Update(float dt)
{
	imu_update(&VELOCITY,&ACC,&Eulla,dt);//��̬����
	
	AcczWithoutG_Get(q,ACC);
	
}


void Imu_data_Prepare(void)
{
				#if USE_KALMAN
			{
				MPU6050_Data_Read_Analys_Kalman(&velocity,&acc);//kalman�Ӵ��ڻ����˲�
				
			}
			#else
			{
//				MPU6050_Data_Read_Analys_LPF(&velocity,&acc);//һ�׵�ͨ�Ӵ��ڻ���	
				MPU6050_Data_Read_Analys(&velocity,&acc);		
				
				
			}
			#endif
			
			MPU6050_Data_Exchage(&acc,&velocity,&ACC,&VELOCITY);
//			u1_printf("%7.3f\t %7.3f\t %7.3f\t %7.3f\t %7.3f\t %7.3f\r\n", ACC.Y, ACC.X, ACC.Z, VELOCITY.X, VELOCITY.Y, VELOCITY.Z);
}




