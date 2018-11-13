#include "mpu6050.h"
#include "myiic.h"
#include "bsp.h"
#include "includes.h"
#include "Kalman.h"

//////////////////////////////////////////////////////////////////////////////////
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//MPU6050 驱动代码
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/9
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////


OFFSET OffSet;
Data_To_Imu  acc, velocity;
Data_To_Imu  ACC, VELOCITY;



// Fast inverse square-root
/**************************实现函数********************************************
*函数原型:	   float InvSqrt(float x)
*功　　能:	   快速计算 1/Sqrt(x) 	
输入参数： 要计算的值
输出参数： 结果
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
// *kalman_filter - 卡尔曼滤波器
// *@kalman:卡尔曼结构体
// *@measure；测量值
// *返回滤波后的值
// */
//float Kalman_filter(kalman_struct *kalman, float measure)
//{
//		u1_printf("123\r\n");
//    /* Predict */
//    kalman->x = kalman->A * kalman->x;//%x的先验估计由上一个时间点的后验估计值和输入信息给出
//    kalman->p = kalman->A * kalman->A * kalman->p + kalman->q;  /*计算先验均方差 p(n|n-1)=A^2*p(n-1|n-1)+q */

//    /* Measurement */
//    kalman->gain = kalman->p * kalman->H / (kalman->p * kalman->H * kalman->H + kalman->r);
//    kalman->x = kalman->x + kalman->gain * (measure - kalman->H * kalman->x);//利用残余的信息改善对x(t)的估计，给出后验估计，这个值也就是输出
//    kalman->p = (1 - kalman->gain * kalman->H) * kalman->p;//%计算后验均方差

//    return kalman->x;
//}

//初始化MPU6050
//返回值:0,成功
//    其他,错误代码
u8 MPU_Init(void)
{
	u8 res;
	IIC_Init();//初始化IIC总线
	
//	RunTime(StartCheck);
//	delay_ms(1000);
//	RunTime(StopCheck);
		
	
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//复位MPU6050
	
    delay_ms_nos(100);
	
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//唤醒MPU6050
	MPU_Set_Gyro_Fsr(3);					//陀螺仪传感器,±2000dps
	MPU_Set_Accel_Fsr(2);					//加速度传感器,±2g//±8g
	MPU_Set_Rate(1000);						//设置采样率50Hz
	MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//关闭所有中断
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C主模式关闭
	MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//关闭FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT引脚低电平有效
	res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
	if(res==MPU_ADDR)//器件ID正确
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//设置CLKSEL,PLL X轴为参考
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//加速度与陀螺仪都工作
		MPU_Set_Rate(1000);						//设置采样率为50Hz
 	}else return 1;
	return 0;
}
//设置MPU6050陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围
}
//设置MPU6050加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围
}
//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6;
	return MPU_Write_Byte(MPU_CFG_REG,data);//设置数字低通滤波器
}
//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~(Hz)
//返回值:0,设置成功
//    其他,设置失败
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}

//得到温度值
//返回值:温度值(扩大了100倍)
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
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码

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
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
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
//IIC连续写
//addr:器件地址
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i;
    IIC_Start();
	IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();
		return 1;
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
	for(i=0;i<len;i++)
	{
		IIC_Send_Byte(buf[i]);	//发送数据
		if(IIC_Wait_Ack())		//等待ACK
		{
			IIC_Stop();
			return 1;
		}
	}
    IIC_Stop();
	return 0;
}
//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
 	IIC_Start();
	IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();
		return 1;
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
    IIC_Start();
	IIC_Send_Byte((addr<<1)|1);//发送器件地址+读命令
    IIC_Wait_Ack();		//等待应答
	while(len)
	{
		if(len==1)*buf=IIC_Read_Byte(0);//读数据,发送nACK
		else *buf=IIC_Read_Byte(1);		//读数据,发送ACK
		len--;
		buf++;
	}
    IIC_Stop();	//产生一个停止条件
	return 0;
}
//IIC写一个字节
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Byte(u8 reg,u8 data)
{
    IIC_Start();
	IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();
		return 1;
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
	IIC_Send_Byte(data);//发送数据
	if(IIC_Wait_Ack())	//等待ACK
	{
		IIC_Stop();
		return 1;
	}
    IIC_Stop();
	return 0;
}
//IIC读一个字节
//reg:寄存器地址
//返回值:读到的数据
u8 MPU_Read_Byte(u8 reg)
{
	u8 res;
    IIC_Start();
	IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令
	IIC_Wait_Ack();		//等待应答
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
    IIC_Start();
	IIC_Send_Byte((MPU_ADDR<<1)|1);//发送器件地址+读命令
    IIC_Wait_Ack();		//等待应答
	res=IIC_Read_Byte(0);//读取数据,发送nACK
    IIC_Stop();			//产生一个停止条件
	return res;
}


//**********数据读取******************/
//数据读取用于解算使用
void MPU6050_Read_To_Use_(Origial_DATA*Origial_DATA__ACC_XYZ,Origial_DATA*Origial_DATA__GYRO_XYZ,OFFSET*offset)
{
	uint8_t Mpu6050_Data_Buf[14]={0};//传感器原始数据接收缓冲
	int32_t G_X=0,G_Y=0,G_Z=0,A_X=0,A_Y=0,A_Z=0;
	int32_t temp=0;
	uint8_t res;
	
	MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG,14,Mpu6050_Data_Buf);//模拟IIC读取数据

	
	 A_X=(((int16_t)Mpu6050_Data_Buf[0]<<8)|Mpu6050_Data_Buf[1])-offset->AX_Offset;
	 A_Y=(((int16_t)Mpu6050_Data_Buf[2]<<8)|Mpu6050_Data_Buf[3])-offset->AY_Offset;
	 A_Z=(((int16_t)Mpu6050_Data_Buf[4]<<8)|Mpu6050_Data_Buf[5])+offset->AZ_Offset;
	
	// Temperature=	(((int16_t)Mpu6050_Data_Buf[6]<<8)|Mpu6050_Data_Buf[7]);//内部温度ADC
	 G_X=(((int16_t)Mpu6050_Data_Buf[8]<<8)|Mpu6050_Data_Buf[9])-offset->GX_Offset;
	 G_Y=(((int16_t)Mpu6050_Data_Buf[10]<<8)|Mpu6050_Data_Buf[11])-offset->GY_Offset;
	 G_Z=(((int16_t)Mpu6050_Data_Buf[12]<<8)|Mpu6050_Data_Buf[13])-offset->GZ_Offset;
//	 u1_printf("%d\t%d\t%d\t%d\t%d\t%d\r\n",A_X,A_Y,A_Z,G_X,G_Y,G_Z);
	 /***数据限幅***/
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
	
	#if IMU_CHIP_ROTATION==0  //0：以MPU6050+X轴为机头前方
	

	#elif IMU_CHIP_ROTATION==1//1：以MPU6050-X轴为机头前方
	A_X=-A_X;
	A_Y=-A_Y;
	
	G_X=-G_X;
	G_Y=-G_Y;
	
	#elif IMU_CHIP_ROTATION==2//2：以MPU6050+Y轴为机头前方
	temp=A_X;
	A_X = A_Y;
	A_Y =-temp;
	
	temp=G_X;
	G_X = G_Y;
	G_Y = -temp;
	
	#elif IMU_CHIP_ROTATION==3//3：以MPU6050-Y轴为机头前方
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

//**********数据读取******************/
//数据读取用于零偏计算
void MPU6050_Read_To_Calculate(Origial_DATA *Origial_DATA__ACC_XYZ,Origial_DATA *Origial_DATA__GYRO_XYZ)
{
	uint8_t Mpu6050_Data_Buf[14]={0};//传感器原始数据接收缓冲
	int32_t G_X=0,G_Y=0,G_Z=0,A_X=0,A_Y=0,A_Z=0;
	int32_t temp=0;
	MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG,14,Mpu6050_Data_Buf);//模拟IIC读取数据
	
	 A_X=	(((int16_t)Mpu6050_Data_Buf[0]<<8)|Mpu6050_Data_Buf[1]);
	 A_Y=	(((int16_t)Mpu6050_Data_Buf[2]<<8)|Mpu6050_Data_Buf[3]);
	 A_Z=	(((int16_t)Mpu6050_Data_Buf[4]<<8)|Mpu6050_Data_Buf[5]);
	// Temperature=	(((int16_t)Mpu6050_Data_Buf[6]<<8)|Mpu6050_Data_Buf[7]);//内部温度ADC
	 G_X=	(((int16_t)Mpu6050_Data_Buf[8]<<8)|Mpu6050_Data_Buf[9]);
	 G_Y=	(((int16_t)Mpu6050_Data_Buf[10]<<8)|Mpu6050_Data_Buf[11]);
	 G_Z=	(((int16_t)Mpu6050_Data_Buf[12]<<8)|Mpu6050_Data_Buf[13]);

		#if IMU_CHIP_ROTATION==0  //0：以MPU6050+X轴为机头前方
	

	#elif IMU_CHIP_ROTATION==1//1：以MPU6050-X轴为机头前方
	A_X=-A_X;
	A_Y=-A_Y;
	
	G_X=-G_X;
	G_Y=-G_Y;
	
	#elif IMU_CHIP_ROTATION==2//2：以MPU6050+Y轴为机头前方
	temp=A_X;
	A_X = A_Y;
	A_Y =-temp;
	
	temp=G_X;
	G_X = G_Y;
	G_Y = -temp;
	
	#elif IMU_CHIP_ROTATION==3//3：以MPU6050-Y轴为机头前方
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

/************数据标定**************/
//功能：对加计、陀螺仪数据进行标定
//输入：OFF_SET *ACC_OFF,OFF_SET *GYRO_OFF
//输出：无
Origial_DATA Sensor_Acc,Sensor_Gyro;//传感器原始数据

const unsigned char offsetlength = 100;  //计算零偏所测量的数据次数

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
// 	STMFLASH_Write(Read_Addr,(u16*)off_set,6);//数据保存
	delay_ms(20);
//	LED1_OFF();
//	printf("offset:%d\t %d\t %d\t %d\t %d\t %d\r\n",off_set[0],off_set[1],off_set[2],off_set[3],off_set[4],off_set[5]);
	
}

/******************/
//陀螺仪数据滑动滤波处理 加计数据Kalman滤波
//输出：滑动滤波后除以转换比例得到的加速度(g/m^2)，角速度数据(rad/s)
//输入：原始数据 标定得到的零偏
//滤波深度不宜过大，否则加大数据延时，降低动态效应
const float acc_denominator = 417.95f; //加计灵敏度 4096/9.8=417.95 广州地区重力加速度9.788m/s^2
const float gyro_denominator = 1877.2f; //陀螺仪灵敏度 转换为rad/S
const unsigned char accfilter_Num = 6; //平滑滤波深度
const unsigned char gyrofilter_Num = 6; //平滑滤波深度
//const unsigned char offsetlength = 100;  //计算零偏所测量的数据次数
const float gyroscale=gyro_denominator*gyrofilter_Num;
kalman_struct KalmanfilterAccx,KalmanfilterAccy,KalmanfilterAccz;
void MPU6050_Data_Read_Analys_Kalman(Data_To_Imu *GYRO,Data_To_Imu *Acc)
{
	u8 j=0;
	float a=0.3f,b=0.7f;
	static u8 counter=0;//计数变量
	float temp4=0,temp5=0,temp6=0;
	static float ACC_TEMP[3]={0};
	static int16_t BUF4[gyrofilter_Num]={0},BUF5[gyrofilter_Num]={0},BUF6[gyrofilter_Num]={0};

     MPU6050_Read_To_Use_(&Sensor_Acc,&Sensor_Gyro,&OffSet);//读取数据，标定

/*******************卡尔曼滤波*****************************/
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
//双滑动滤波处理
//输出：滑动滤波后除以转换比例得到的加速度(g/m^2)，角速度数据(rad/s)
//输入：原始数据 标定得到的零偏
//滤波深度不宜过大，否则加大数据延时，降低动态效应
void MPU6050_Data_Read_Analys(Data_To_Imu *GYRO,Data_To_Imu *Acc)
{
	u8 i=0,j=0;
	static u8 count=0,counter=0;//计数变量
	int32_t temp1=0,temp2=0,temp3=0,temp4=0,temp5=0,temp6=0;
	static int16_t BUF1[accfilter_Num]={0},BUF2[accfilter_Num]={0},BUF3[accfilter_Num]={0};
	static int16_t BUF4[gyrofilter_Num]={0},BUF5[gyrofilter_Num]={0},BUF6[gyrofilter_Num]={0};
	
     MPU6050_Read_To_Use_(&Sensor_Acc,&Sensor_Gyro,&OffSet);//读取数据，标定

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

/*数据交换*/
//陀螺仪测得的角速度数据在imuupdate函数里面会改变值，如果采用地址传递改变参数值的话
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
float Kp = KP;//用于决定ACC HMC数据对GYRO补偿的大小   决定加速度计/磁力计收敛速度              
float Ki = 0.001f; //用于消除陀螺仪漂移 

static float exInt = 0.0f, eyInt = 0.0f, ezInt=0.0f;//积分环节变量
float q[4]={1.0f,0.0f,0.0f,0.0f};//全局四元数
//float gyro_yaw_scale=0.9;

extern Data_To_Imu ACC,VELOCITY;	
EULLA Eulla/*,LastEulla*/;//互补滤波后的欧拉角结构体
INERIAL Inerialacc;
/**************更新欧拉角****************/
//引自:S.O.H. Madgwick 代码 On September 10th,2010
//功能:MPU6050数据互补滤波更新欧拉角数据
//输入：角速度(rad/s)、加速度(m/s^2)、电子罗盘数据(°).
//输出：四元数、欧拉角更新!
//返回：无
void imu_update(Data_To_Imu*velocity,Data_To_Imu*acc,EULLA*eulla,float dt)
{
  float vx=0.0f, vy=0.0f, vz=0.0f;//当前的欧拉角(四元数)的机体坐标参照系上，换算出来的重力单位向量
  float ex=0.0f, ey=0.0f, ez=0.0f;//陀螺仪积分后的姿态和计算出来的姿态误差
  //double t[3][3]; //存放方向余弦矩阵数组
  float norm=0.0f;//模向量变量
	//double yaw_mag;//磁力计解算出的偏航
  float gyro_vel_z=velocity->Z;
/*************为后面数据计算准备*********/
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
		Kp=0;//不采用加速度计补偿矫正陀螺仪积分算出的角度
	}
	norm=Invsqrt(acc->X*acc->X + acc->Y*acc->Y + acc->Z*acc->Z);//快速求方根倒数
//	u1_printf("4\r\n");
	Inerialacc.acclength=acc->Z*norm;//规范化后的acc->Z
	if(Inerialacc.acclength<0.05f)
	{
		Inerialacc.acccorr=0;
	}
	else
	{
		Inerialacc.acccorr=sinf(acosf(Inerialacc.acccorr));
	}
	  acc->X = acc->X * norm;
	  acc->Y = acc->Y * norm;//把加计三位向量转换为单位向量
	  acc->Z = acc->Z * norm;//基于机体坐标所测得的重力向量
				
	  vx = 2*(q1q3 - q0q2);	//陀螺仪积分后推算的姿态											
	  vy = 2*(q0q1 + q2q3);//当前的欧拉角(四元数)的机体坐标参照系上，换算出来的重力单位向量
	  vz = q0q0 - q1q1 - q2q2 + q3q3 ;//刚好是方向余弦中第三行的三个元素
//	  vz = -0.5f+q0q0 + q3q3 ;//刚好是方向余弦中第三行的三个元素
	
	  ex = (acc->Y*vz - acc->Z*vy) ; //陀螺仪积分后的推算姿态和加计测量后的姿态之间的误差                          					
	  ey = (acc->Z*vx - acc->X*vz) ;
	  ez = (acc->X*vy - acc->Y*vx) ;
	//向量之间的误差，可以用向量叉积(也叫向量外积，叉乘)来表示，ex,ey,ez就是两个重力向量vxyz 和acc->x y z的叉乘。
	//这个叉积向量仍旧是位于机体坐标系上的，而陀螺仪积分误差也是基于机体坐标系，而且叉乘的大小与陀螺仪积分误差成
	//正比，正好拿来纠正陀螺仪。
	/*********误差积分********/		
	  exInt = exInt + ex * Ki;							 
	  eyInt = eyInt + ey * Ki;
	  ezInt = ezInt + ez * Ki;
/*********用叉积误差来做PI修正陀螺仪零偏********/
  velocity->X = velocity->X + Kp*ex + exInt;//P、I控制					   							
  velocity->Y = velocity->Y + Kp*ey + eyInt;
  velocity->Z = velocity->Z + Kp*ez + ezInt;//由于Z轴转动无法通过加计测量，所以不需要积分环节
													//这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减
 		   							
	/********龙格库塔法更新四元数，halfT为采样周期的一半**********/				   
	// integrate quaternion rate and normalise //四元数更新方程
	q[0] = q[0] + (-q[1]*velocity->X - q[2]*velocity->Y - q[3]*velocity->Z)*dt/2;
	q[1] = q[1] + ( q[0]*velocity->X + q[2]*velocity->Z - q[3]*velocity->Y)*dt/2;
	q[2] = q[2] + ( q[0]*velocity->Y - q[1]*velocity->Z + q[3]*velocity->X)*dt/2;
	q[3] = q[3] + ( q[0]*velocity->Z + q[1]*velocity->Y - q[2]*velocity->X)*dt/2;

	norm = Invsqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);//快速求方根倒数
	q[0] = q[0] * norm;//更新四元数
	q[1] = q[1] * norm;
	q[2] = q[2] * norm;
	q[3] = q[3] * norm;
	
/***********再次更新数据*************/
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
	
/*************更新方向余弦矩阵***********/
//     t[1][2]=2.0*(q2q3+q0q1); //y 
//     t[2][2]=q0q0-q1q1-q2q2+q3q3; //z
//     t[0][2]=2.0*(q1q3-q0q2); 	//x
//     t[0][1]=2.0*(q1q2+q0q3);
//     t[0][0]=q0q0+q1q1-q2q2-q3q3;

//	eulla->yaw = atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3]* q[3] + 1)* 57.3; // 绕 z轴旋转
	eulla->yaw  += gyro_vel_z*dt*57.3f;
 	eulla->pitch = sinf(-2 * q[1] * q[3] + 2 * q[0]* q[2])* 57.3; //绕 y轴旋转 -90°~+90°
 	eulla->roll  = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2]* q[2] + 1) * 57.3; // 绕 x轴旋转 -180°~+180°
	
//	delay_ms(5);
//u1_printf("5\r\n");
}





/*AcczWithoutG_Get*/
/*计算除去重力加速度的加速度*/
//参数：q[]全局四元数。当前四轴姿态的表示方法的一种
//			acc测量得到的机体加速度数据
//返回值：除去重力加速度的加速度大小
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
	imu_update(&VELOCITY,&ACC,&Eulla,dt);//姿态解算
	
	AcczWithoutG_Get(q,ACC);
	
}


void Imu_data_Prepare(void)
{
				#if USE_KALMAN
			{
				MPU6050_Data_Read_Analys_Kalman(&velocity,&acc);//kalman加窗口滑动滤波
				
			}
			#else
			{
//				MPU6050_Data_Read_Analys_LPF(&velocity,&acc);//一阶低通加窗口滑动	
				MPU6050_Data_Read_Analys(&velocity,&acc);		
				
				
			}
			#endif
			
			MPU6050_Data_Exchage(&acc,&velocity,&ACC,&VELOCITY);
//			u1_printf("%7.3f\t %7.3f\t %7.3f\t %7.3f\t %7.3f\t %7.3f\r\n", ACC.Y, ACC.X, ACC.Z, VELOCITY.X, VELOCITY.Y, VELOCITY.Z);
}




