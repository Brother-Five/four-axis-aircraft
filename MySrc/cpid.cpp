#include "cpid.h"
#include "bsp.h"
#include "filter.h"
static float TRI(float X,float DOWN,float MID,float UP);
static float LAD_DOWN(float X,float MID,float UP);
static float LAD_UP(float X,float DOWN,float MID);

PIDBase::PIDBase(float m_Kp, float m_Ki, float m_Kd, float m_Target,
float m_Rpi, float m_Rpa, float m_Rii, float m_Ria, float m_Rdi, float m_Rda, float m_Ti, float m_Ta,float m_DeadZone){
	Kp       = m_Kp;
	Ki       = m_Ki;
	Kd       = m_Kd;
	Target   = m_Target;
	Rpi      = m_Rpi;
	Rii      = m_Rii;
	Rdi      = m_Rdi;
	Rpa      = m_Rpa;
	Ria      = m_Ria;
	Rda      = m_Rda;
	Ti       = m_Ti;
	Ta       = m_Ta;
	DeadZone = m_DeadZone;
	Out      = 0;
	MODE     = 0;

}
void PIDBase::SetTarget(float target){
	Target = Constrain(target,Ti,Ta);
	// Target = target;
}
void PIDBase::SetCurrent(float current){
	Current = current;
}

void PIDBase::SetPID(float m_Kp, float m_Ki, float m_Kd){
	Kp = m_Kp;
	Ki = m_Ki;
	Kd = m_Kd;
	// printf("P:%5.2f\tI:%5.2f\tD:%5.2f\r\n",this->Kp,this->Ki,this->Kd);//调试用
}

void PIDBase::SetTargetMaxMin( float m_Ti, float m_Ta){
	Ti = m_Ti;
	Ta = m_Ta;
	// printf("Ti:%5.2f\tTa:%5.2f\r\n",this->Ti,this->Ta);//调试用
}
void PIDBase::SetPMaxMin(float m_Rpi, float m_Rpa){
	Rpi = m_Rpi;
	Rpa = m_Rpa;
	// printf("Rpi:%5.2f\tRpa:%5.2f\r\n",this->Rpi,this->Rpa);//调试用
}

void PIDBase::SetIMaxMin(float m_Rii, float m_Ria){
	Rii = m_Rii;
	Ria = m_Ria;
	// printf("Rii:%5.2f\tRia:%5.2f\r\n",this->Rii,this->Ria);//调试用
}
void PIDBase::SetDMaxMin(float m_Rdi, float m_Rda){
	Rdi = m_Rdi;
	Rda = m_Rda;
	// printf("Rdi:%5.2f\tRda:%5.2f\r\n",this->Rdi,this->Rda);//调试用
}

void PIDBase::SetDeadZone(float m_DeadZone){
 	DeadZone = m_DeadZone;
 	// printf("DeadZone:%5.2f\tD_dt_time:%d\r\n",this->DeadZone,this->D_dt_time);//调试用
}


unsigned char PIDTimer::UpdataTimeStamp(void)
{
	u32 now_time;

	//系统时间的获取
	if(last_time == 0)
	{
		last_time = micros();
		return 1;
	}
	now_time = micros();

	if(now_time < last_time)
	{
	  	dt = (float)(now_time + (0xFFFFFFFF - last_time) );
	}
	else
	{
		dt = (float)(now_time - last_time);

	}

  	last_time = now_time ;

	dt /= 1000000.0f;

	return 0;

}


void CPID::AdjustPID(void)
{
	float now_error;

	if(UpdataTimeStamp()) return;


	now_error = Target - Current;
	now_error = Slider_Filter(filterstruct.filterbuff[0],&filterstruct.num[0],5,now_error);
	

	//P
	PTerm = Kp * now_error;
	PTerm = Constrain(PTerm, Rpi, Rpa);

	//I

	ITerm +=  Ki * now_error * dt;

	ITerm = Constrain(ITerm, Rii, Ria);



	Omega = Slider_Filter(filterstruct.filterbuff[1],&filterstruct.num[1],5,Omega);
	DTerm =   Kd * (Omega - Omega_Offset);
	Out = PTerm + ITerm + DTerm;
	Out = Slider_Filter(filterstruct.filterbuff[2],&filterstruct.num[2],5,Out);//滤波，输出更加平滑

	if(myabs(now_error) <= DeadZone)
	{
		counter++;
		if(counter >= 1024)
			counter = 1024;
	}
	else
	{
		counter--;
		if(counter <= 0)
			counter = 0;
	}
	if(counter >= 1000)
	{
		Out = 0;
		ITerm = 0;
	}
}


void SCPID::AdjustPID(void)
{
	if(UpdataTimeStamp()) return;

	last_error = now_error;
	now_error = Target - Current;
	now_error = Slider_Filter(filterstruct.filterbuff[0],&filterstruct.num[0],5,now_error);

	//P
	PTerm = Kp * now_error;
	PTerm = Constrain(PTerm, Rpi, Rpa);

	//I
	ITerm +=  Ki * now_error * dt;
	ITerm = Constrain(ITerm, Rii, Ria);

	//D
	DTerm =   Kd * (now_error - last_error)/dt;
	DTerm = Constrain(DTerm, Rdi, Rda);
	DTerm = Slider_Filter(filterstruct.filterbuff[1],&filterstruct.num[1],5,DTerm);//滤波，输出更加平滑

	Out = PTerm + ITerm + DTerm;
	Out = Slider_Filter(filterstruct.filterbuff[2],&filterstruct.num[2],5,Out);//滤波，输出更加平滑
	
	if(myabs(now_error) <= DeadZone)
	{
		counter++;
		if(counter >= 1024)
			counter = 1024;
	}
	else
	{
		counter--;
		if(counter <= 0)
			counter = 0;
	}
	if(counter >= 1000)
	{
		Out = 0;
		ITerm = 0;
	}
}





void FUZZYPID::FuzzyPID(float error,float errorC)
{
	float Ux_S,Ux_M,Ux_B;
	float kp,ki,kd;
	Ux_S  = LAD_DOWN(myabs(error),this->NS_PS,this->NM_PM);
	Ux_M  = TRI(myabs(error),this->NS_PS,this->NM_PM,this->NB_PB);
	Ux_B  = LAD_UP(myabs(error),this->NM_PM,this->NB_PB);

	// Ux_S  = LAD_DOWN(myabs(error),this->NS_PS,this->NM_PM);
	// Ux_M  = TRI(myabs(error),this->NS_PS,this->NM_PM,this->NB_PB);
	// Ux_B  = LAD_UP(myabs(error),this->NM_PM,this->NB_PB);

	kp = (Ux_S * this->NS_PS_kp + Ux_M * this->NM_PM_kp + Ux_B * this->NB_PB_kp)/(Ux_S + Ux_M + Ux_B);
	ki = (Ux_S * this->NS_PS_ki + Ux_M * this->NM_PM_ki + Ux_B * this->NB_PB_ki)/(Ux_S + Ux_M + Ux_B);
	kd = (Ux_S * this->NS_PS_kd + Ux_M * this->NM_PM_kd + Ux_B * this->NB_PB_kd)/(Ux_S + Ux_M + Ux_B);

//	kp = CLimitSlider_Filter(Kpfilter,5,kp,10000);//滤波
//	ki = CLimitSlider_Filter(Kifilter,5,ki,10000);//滤波
//	kd = CLimitSlider_Filter(Kdfilter,5,kd,10000);//滤波

//	kp = Constrain(kp, 0.0f, this->NB_PB_kp);
//	ki = Constrain(ki, 0.0f, this->NB_PB_ki);
//	kd = Constrain(kd, 0.0f, this->NB_PB_kd);
//	if(myabs(error) <= NS_PS)
//	{
//		kd = NS_PS_kd*errorC;
//		kp = NS_PS_kp/errorC;
//	}
	SetPID(kp,ki,kd);
}


FUZZYPID::FUZZYPID(float m_Kp, float m_Ki, float m_Kd, float m_Target,\
		 float m_Rpi, float m_Rpa, float m_Rii, float m_Ria,\
		 float m_Rdi, float m_Rda, float m_Ti, float m_Ta, float m_DeadZone,\
		 float m_NB_PB, float m_NM_PM, float m_NS_PS,\
		 float m_NB_PB_kp,float m_NM_PM_kp, float m_NS_PS_kp,\
		 float m_NB_PB_ki,float m_NM_PM_ki, float m_NS_PS_ki,\
		 float m_NB_PB_kd,float m_NM_PM_kd, float m_NS_PS_kd)
		:PIDBase(m_Kp,m_Ki,m_Kd,m_Target,\
				m_Rpi,m_Rpa,m_Rii,m_Ria,\
				m_Rdi,m_Rda,m_Ti,m_Ta,m_DeadZone),
		NB_PB (m_NB_PB),
		NM_PM (m_NM_PM),
		NS_PS (m_NS_PS),
		NB_PB_kp (m_NB_PB_kp),
		NM_PM_kp (m_NM_PM_kp),
		NS_PS_kp (m_NS_PS_kp),
		NB_PB_ki (m_NB_PB_ki),
		NM_PM_ki (m_NM_PM_ki),
		NS_PS_ki (m_NS_PS_ki),
		NB_PB_kd (m_NB_PB_kd),
		NM_PM_kd (m_NM_PM_kd),
		NS_PS_kd (m_NS_PS_kd)
		{
			Kec = 0.00001f;
		};


void FUZZYPID::AdjustPID(void)
{
	if(UpdataTimeStamp()) return;

	now_error = Target - Current;
	now_error = Slider_Filter(filterstruct.filterbuff[0],&filterstruct.num[0],5,now_error);
	Omega = Slider_Filter(filterstruct.filterbuff[1],&filterstruct.num[1],5,Omega);

	errorC = Kec*Omega/dt;
	FuzzyPID(now_error,errorC);

	//P
	PTerm = Kp * now_error;
	PTerm = Constrain(PTerm, Rpi, Rpa);

	//I

	ITerm +=  Ki * now_error * dt;
	ITerm = Constrain(ITerm, Rii, Ria);

	//D
	DTerm =   Kd * (Omega - Omega_Offset);

	Out = PTerm + ITerm + DTerm;
	Out = Slider_Filter(filterstruct.filterbuff[2],&filterstruct.num[2],5,Out);//滤波，输出更加平滑
	
	if(myabs(now_error) <= DeadZone)
	{
		counter++;
		if(counter >= 1024)
			counter = 1024;
	}
	else
	{
		counter--;
		if(counter <= 0)
			counter = 0;
	}
	if(counter >= 1000)
	{
		Out = 0;
		ITerm = 0;
	}
}


void FUZZYSPID::AdjustPID(void)
{
	if(UpdataTimeStamp()) return;

	last_error = now_error;
	now_error = Target - Current;
	now_error = Slider_Filter(filterstruct.filterbuff[0],&filterstruct.num[0],5,now_error);

	errorC = Kec*Omega/dt;
	FuzzyPID(now_error,errorC);

	//P
	PTerm = Kp * now_error;
	PTerm = Constrain(PTerm, Rpi, Rpa);

	//I

	ITerm +=  Ki * now_error * dt;
	ITerm = Constrain(ITerm, Rii, Ria);

	//D
	DTerm =   Kd * (now_error - last_error)/dt;
	DTerm = Constrain(DTerm, Rdi, Rda);
	DTerm = Slider_Filter(filterstruct.filterbuff[1],&filterstruct.num[1],5,DTerm);//滤波，输出更加平滑
	
	Out = PTerm + ITerm + DTerm;
	Out = Slider_Filter(filterstruct.filterbuff[2],&filterstruct.num[2],5,Out);//滤波，输出更加平滑
	
	if(myabs(now_error) <= DeadZone)
	{
		counter++;
		if(counter >= 1024)
			counter = 1024;
	}
	else
	{
		counter--;
		if(counter <= 0)
			counter = 0;
	}
	if(counter >= 1000)
	{
		Out = 0;
		ITerm = 0;
	}
}

float TRI(float X,float DOWN,float MID,float UP)
{
	if( X >= DOWN && X <= UP)
		return ( 1 - myabs(X - MID)/( MID - DOWN) );
	else
		return 0;
}
float LAD_DOWN(float X,float MID,float UP)
{
	if(	X <= MID)
		return 1.0f;
	else if( X <= UP )
		return (X - UP) / (MID - UP);
	else
		return 0;
}

float LAD_UP(float X,float DOWN,float MID)
{
	if(	X >= MID)
		return 1.0f;
	else if( X >= DOWN )
		return (X - DOWN) / (MID - DOWN);
	else
		return 0;
}


TARGET Target;
RC Rcdata;

PID Pitchpid,Rollpid,Yawpid/*,Heightpid*/;
PID Gyropitchpid,Gyrorollpid,Gyroyawpid;
void ControlTask_Init(void)
{
		Rcdata_Init(&Rcdata);//
    PID_InitPITCH(&Pitchpid, PMAX,INTMAX, DMAX);//
    PID_InitROLL(&Rollpid,  PMAX,INTMAX, DMAX);//
    PID_InitYAW (&Yawpid,   PMAX,INTMAX, DMAX);//PID
	
	PID_InitPITCH_VEL(&Gyropitchpid,PMAX,INTMAX,DMAX);//PID
	PID_InitROLL_VEL(&Gyrorollpid, PMAX,INTMAX,DMAX);
	PID_InitYAW_VEL	(&Gyroyawpid,  PMAX,INTMAX,DMAX);	
	
	Target_parameter_init(&Target);
}

void Rcdata_Init(RC *rc)
{
	rc->pitch=0.0f;
	rc->roll =0.0f;
	rc->yaw  =0.0f;
	rc->thottle=0.0f;
//	rc->height=0.0f;
	
	rc->Jindou=FALSE;//
	rc->Lock=FALSE;//
	rc->Selft_Test=FALSE;//
	rc->batvol=TRUE;//
	
}
void Target_parameter_init(TARGET *target)
{
	target->pitch=0.0f;
	target->roll=0.0f;
	target->yaw=0.0f;
	target->thottle=0.0f;
//	target->height=0.0f;
	
	target->Jindou=0;
	target->Lock=1;//
	target->Selft_Test=0;//
	target->batvol=0;
	target->signal=GETSIGNAL;//
}

void PID_InitPITCH(PID *pid,float pmax,float intmax,float dmax)
{
	pid->error=0;
	pid->lasterror=0;	
	pid->kp=0.5f;//0.1
	pid->ki=0.1f;//0.1
	pid->kd=0.0f;//0.0
//	pid->Pmax=pmax;
	pid->Intmax=intmax;//
	pid->Intx=0;//
//	pid->Dmax=dmax;
//	pid->Pcontrol=0;
//	pid->Icontrol=0;
//	pid->Dcontrol=0;
	pid->controutput=0;
//	pid->lastoutput=0;
}

void PID_InitROLL(PID *pid,float pmax,float intmax,float dmax)
{
	pid->error=0;
	pid->lasterror=0;	
	pid->kp=0.5f;//0.1
	pid->ki=0.1f;//0.1
	pid->kd=0.0f;//0.0
//	pid->Pmax=pmax;
	pid->Intmax=intmax;//
	pid->Intx=0;//
//	pid->Dmax=dmax;
//	pid->Pcontrol=0;
//	pid->Icontrol=0;
//	pid->Dcontrol=0;
	pid->controutput=0;
//	pid->lastoutput=0;
}

void PID_InitYAW(PID *pid,float pmax,float intmax,float dmax)
{
	pid->error=0;
	pid->lasterror=0;	
	pid->kp=0.0f;//10.0f
	pid->ki=0.0f;//0.0
	pid->kd=0.0f;//0.0
//	pid->Pmax=pmax;
	pid->Intmax=intmax;//
	pid->Intx=0;//
//	pid->Dmax=dmax;
//	pid->Pcontrol=0;
//	pid->Icontrol=0;
//	pid->Dcontrol=0;
	pid->controutput=0;
//	pid->lastoutput=0;
}

void PID_InitPITCH_VEL(PID *pid,float pmax,float intmax,float dmax)
{
	pid->error=0;
	pid->lasterror=0;	
	pid->kp=1.0f;//1.5
	pid->ki=0.0f;
	pid->kd=4.0f;//4.1
//	pid->Pmax=pmax;
	pid->Intmax=intmax;//
	pid->Intx=0;//»ý·ÖÏî
//	pid->Dmax=dmax;
//	pid->Pcontrol=0;
//	pid->Icontrol=0;
//	pid->Dcontrol=0;
	pid->controutput=0;
//	pid->lastoutput=0;
}

void PID_InitROLL_VEL(PID *pid,float pmax,float intmax,float dmax)
{
	pid->error=0;
	pid->lasterror=0;	
	pid->kp=1.0f;//1.5
	pid->ki=0.0f;
	pid->kd=4.0f;//4.0
//	pid->Pmax=pmax;
	pid->Intmax=intmax;//
	pid->Intx=0;//
//	pid->Dmax=dmax;
//	pid->Pcontrol=0;
//	pid->Icontrol=0;
//	pid->Dcontrol=0;
	pid->controutput=0;
//	pid->lastoutput=0;
}


void PID_InitYAW_VEL(PID *pid,float pmax,float intmax,float dmax)
{
	pid->error=0;
	pid->lasterror=0;	
	pid->kp=0.0f;//
	pid->ki=0.0f;
	pid->kd=0.0f;
//	pid->Pmax=pmax;
	pid->Intmax=intmax;//
	pid->Intx=0;//
//	pid->Dmax=dmax;
//	pid->Pcontrol=0;
//	pid->Icontrol=0;
//	pid->Dcontrol=0;
	pid->controutput=0;
//	pid->lastoutput=0;
}
/**********************Ë«»·PID**************************/
void Pidcontrol_Altitude_VEL(TARGET *TARGET,EULLA *cureulla,Data_To_Imu*velocity,float ctrout[],float dt)
{		
/***************************Íâ»·error************************************/
		Pitchpid.error=TARGET->pitch -cureulla->pitch;
		Rollpid.error =TARGET->roll  -cureulla->roll;
		Yawpid.error  =TARGET->yaw  -cureulla->yaw;//
//		u1_printf("%7.3f\t%7.3f\t%7.3f\r\n",Pitchpid.error,Rollpid.error,Yawpid.error );
//		Yawpid.error  =TARGET->yaw  -velocity->Z*Radian_to_Angle;//½ÇËÙ¶È
/***********************integer and limit**********************************/	
//		Pitchpid.Intx=Pitchpid.error-2*Pitchpid.lasterror+Pitchpid.lasterror2;
	Pitchpid.Intx+=Pitchpid.error*dt;
	if(Pitchpid.Intx >  Pitchpid.Intmax)	Pitchpid.Intx= Pitchpid.Intmax;
	if(Pitchpid.Intx <- Pitchpid.Intmax)	Pitchpid.Intx=-Pitchpid.Intmax;
//		Rollpid.Intx = Rollpid.error-2*Rollpid.lasterror+Rollpid.lasterror2;
	Rollpid.Intx+=Rollpid.error*dt;
	if(Rollpid.Intx >  Rollpid.Intmax)	Rollpid.Intx= Rollpid.Intmax;
	if(Rollpid.Intx <- Rollpid.Intmax)	Rollpid.Intx=-Rollpid.Intmax;
//		Yawpid.Intx=Yawpid.error-2*Yawpid.lasterror+Yawpid.lasterror2;
	Yawpid.Intx+=Yawpid.error*dt;
	if(Yawpid.Intx >  Yawpid.Intmax)	Yawpid.Intx= Yawpid.Intmax;
	if(Yawpid.Intx <- Yawpid.Intmax)	Yawpid.Intx=-Yawpid.Intmax;
	
/*************************Íâ»· p iÏî*********************************************/	
	  Pitchpid.controutput=Pitchpid.kp*Pitchpid.error+Pitchpid.ki *Pitchpid.Intx+Pitchpid.kd*(Pitchpid.error-Pitchpid.lasterror);
	  Rollpid .controutput=Rollpid .kp*Rollpid .error+Rollpid .ki *Rollpid.Intx+Rollpid .kd*(Rollpid.error-Rollpid.lasterror);
	  Yawpid  .controutput=Yawpid  .kp*Yawpid  .error+Yawpid  .ki *Yawpid.Intx+Yawpid  .kd*(Yawpid.error-Yawpid.lasterror);
//		u1_printf("%7.3f\t%7.3f\t%7.3f\r\n",Pitchpid.controutput,Rollpid.controutput,Yawpid.controutput );
//		delay_ms(10);
/*************************Íâ»·IntxÏî*********************************************/
		Pitchpid.lasterror2=Pitchpid.lasterror;
		Pitchpid.lasterror=Pitchpid.error;
		Rollpid.lasterror2=Rollpid.lasterror;
		Rollpid.lasterror=Rollpid.error;
		Yawpid.lasterror2=Yawpid.lasterror;
		Yawpid.lasterror=Yawpid.error;
/**********************************limit****************************************/
		if(Pitchpid.controutput> 1.5f)	Pitchpid.controutput= 1.5f;
		if(Pitchpid.controutput<-1.5f)	Pitchpid.controutput=-1.5f;
		if(Rollpid.controutput>  1.5f)	Rollpid.controutput= 1.5f;
		if(Rollpid.controutput< -1.5f)	Rollpid.controutput=-1.5f;
		if(Yawpid.controutput> 1.0f)	  Yawpid.controutput= 1.0f;
		if(Yawpid.controutput<-1.0f)	  Yawpid.controutput=-1.0f;		
/**********************************output****************************************/		
    ctrout[0]=Pitchpid.controutput;
		ctrout[1]=Rollpid.controutput;
		ctrout[2]=Yawpid.controutput;
//		u1_printf("%7.3f\t%7.3f\t%7.3f\r\n",Pitchpid.controutput,Rollpid.controutput,Yawpid.controutput);
////		printf("ok");
//		delay_ms(10);
///*************************ÄÚ»· error*********************************************/
//		Gyropitchpid.error=Pitchpid.controutput-velocity->Y;
//		Gyrorollpid .error=Rollpid.controutput-velocity->X;
//		Gyroyawpid.error=Yawpid.controutput-velocity->Z;
///*************************ÄÚ»· p dÏî*********************************************/
//		Gyropitchpid.controutput=Gyropitchpid.kp*Gyropitchpid.error+Gyropitchpid.kd*(Gyropitchpid.error-Gyropitchpid.lasterror);
//		Gyrorollpid .controutput=Gyrorollpid .kp*Gyrorollpid .error+Gyrorollpid .kd*(Gyrorollpid .error-Gyrorollpid .lasterror);
//		Gyroyawpid  .controutput=Gyroyawpid  .kp*Gyroyawpid  .error+Gyroyawpid  .kd*(Gyroyawpid  .error-Gyroyawpid  .lasterror);		
///**********************************limit****************************************/
//		if(Gyropitchpid.controutput> 1.5f)	Gyropitchpid.controutput= 1.5f;
//		if(Gyropitchpid.controutput<-1.5f)	Gyropitchpid.controutput=-1.5f;
//		if(Gyrorollpid.controutput>  1.5f)	Gyrorollpid.controutput= 1.5f;
//		if(Gyrorollpid.controutput< -1.5f)	Gyrorollpid.controutput=-1.5f;
//		if(Gyroyawpid.controutput> 1.0f)	Gyroyawpid.controutput= 1.0f;
//		if(Gyroyawpid.controutput<-1.0f)	Gyroyawpid.controutput=-1.0f;		
///************************************exchange****************************************/	
//		ctrout[0]=Gyropitchpid.controutput;
//		ctrout[1]=Gyrorollpid.controutput;
//		ctrout[2]=Gyroyawpid.controutput;
////		u1_printf("%7.3f\t%7.3f\t%7.3f\r\n",Gyropitchpid.controutput,Gyrorollpid.controutput,Gyroyawpid.controutput );
////		delay_ms(10);		
///***********************************value saved*******************************************/
//	Gyropitchpid.lasterror=Gyropitchpid.error;
//	Gyrorollpid.lasterror =Gyrorollpid.error;
//	Gyroyawpid.lasterror  =Gyroyawpid.error;
	
}


//	temp[0] = ( now_error)/dt;
//	temp[1] = ( now_error);
		//D

//	if( myabs(now_error)>= 2)
//	{
//		if(iforad == D_dt_time -1)
//		{
//			Derror = now_error - prev_error ;

//			Derror = CLimitSlider_Filter(PitchEfilter,5,Derror,10000);//滤波，微分毛躁太多
//			DTerm =   Kd * ( Derror)/D_dt;
////			DTerm = KalmanFilter(DTerm,1,1);
//			DTerm = CLimitSlider_Filter(PitchDfilter,5,DTerm,10000);//滤波，微分毛躁太多
//			DTerm = Constrain(DTerm, Rdi, Rda);
//			//ANO_Data_Send(0xf2,D_dt*1000000.0f);
//			prev_error = now_error;
//			D_dt = 0;
//			iforad = 0;
//		}
//		else iforad++;

//	}
//	if( now_error == last_error && ITerm != 0)
//		i_error++;
//	else
//		i_error = 0;
//	if(i_error == 150)
//		ITerm = 0;
