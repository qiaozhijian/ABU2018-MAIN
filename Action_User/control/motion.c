#include "task.h"
#include "steer.h"
#include "can.h"
#include "shoot.h"
#include "stm32f4xx_it.h"
#include "timer.h"
#include "iwdg.h"
#include "robot.h"
#include "motion.h"
#include "adc.h"
#include "gpio.h"
#include "dma.h"

#if  GAS_CONTROL == 1           
#define GAS_ADC_ZERO          363
#else
#define GAS_ADC_ZERO          380
#endif 

extern Robot_t gRobot;

void PitchAngleMotion(float angle)
{
  /*控制的为-20°-10°实际给电机的是30°-0°*/
  if(angle>30.f)//????????????????????????????????
    angle=0.f;//?????????????????????????
  else if(angle<-10.f)
    angle=-10.f;
	
	angle=PITCH_COMPENSATE - angle;
	
  PosCrl(CAN2, PITCH_MOTOR_ID,ABSOLUTE_MODE,PITCH_ANGLE_TO_CODE(angle));
}

void CourseAngleMotion(float angle)
{
  if(angle<0.f)
    angle=0.f;
  else if(angle>COURCE_COMPENSATE&&angle<210.f)
    angle=190.f;
	else if(angle>=210.f&&angle<=260.f)
    angle=260.f;
	else if(angle>360.f)
    angle=350.f;
	
	if(angle-COURCE_COMPENSATE<0.f){
		angle=angle-COURCE_COMPENSATE;
	}else {
	  angle=(angle-COURCE_COMPENSATE)-360.f ;
	}		
	
	
  PosCrl(CAN2, COURCE_MOTOR_ID,ABSOLUTE_MODE,COURSE_ANGLE_TO_CODE(angle));
}

void GasMotion(float value)
{
	float Kp=0.03f,Ki=0.02f;
	float gasControl = 0.f;
	float error = value - gRobot.gasValue;
  if(fabs(error)<0.01&&fabs(error)>0.003&&gRobot.sDta.robocon2018==GOLD_BALL){
    gRobot.gasPidIntegral += Ki * error;	
		gasControl = Kp * error + gRobot.gasPidIntegral;
		gRobot.gasControl = gasControl;
	}
	else{
		gRobot.gasPidIntegral =0.f ;
		gRobot.gasControl = 0.f;
	}
	
	#ifndef  GAS_CONTOL_BY_PWM
	uint32_t data[2]={0x00005045,0x00000000};
	union
	{
		float dataFloat;
		uint32_t data32;
	}sendData;
	
	sendData.dataFloat = value /*+ gasControl*/;
	data[1] = sendData.data32 ;
  CAN_TxMsg(CAN2,SEND_TO_GASSENSOR,(uint8_t*)(&data),8);
	#else
	value=value /*+ gasControl*/;
  GasControlByPWM(value);
	#endif
}

void GasEnable(void)
{
	#ifndef  GAS_CONTOL_BY_PWM
	uint32_t data[2]={0x00004145,1};
  CAN_TxMsg(CAN2,SEND_TO_GASSENSOR,(uint8_t*)(&data),8);	
	#endif
}

void GasDisable(void){
	#ifndef  GAS_CONTOL_BY_PWM
	uint32_t data[2]={0x00008368,1};
	CAN_TxMsg(CAN2,SEND_TO_GASSENSOR,(uint8_t*)(&data),8);
	#endif
}


//开电打气
void GasIF(void){
	
	uint32_t send[2]={0x00004649,1};
	
	CAN_TxMsg(CAN2,SEND_TO_GASSENSOR,(uint8_t*)(&send),8);	

}




/* 动作执行函数
* 
* 持球的舵机一与舵机二
*	控制摄像头转向的舵机三
*
*/

void MotionExecute(void)
{
	/*如果持球舵机没有到位*/
	if(!(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_1_SUCCESS)
		||!(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_2_SUCCESS))
	{
		HoldBallPosCrlSeparate(gRobot.sDta.holdBallAimAngle[0],gRobot.sDta.holdBallAimAngle[1]);
	}
	
	if(!(gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS))
	{
		CourseAngleMotion(gRobot.sDta.courseAimAngle);
	}
	
	if(!(gRobot.sDta.AT_motionFlag&AT_PITCH_SUCCESS))
	{
		PitchAngleMotion(gRobot.sDta.pitchAimAngle);
	}
	
	if(!(gRobot.sDta.AT_motionFlag&AT_GAS_SUCCESS))
	{
		GasMotion(gRobot.sDta.gasAimValue);
	}
	
}


/* 动作状态问询函数
* 
* 持球的舵机一与舵机二
*	控制摄像头转向的舵机三
*	控制发射架航向角的电机
* 控制发射架俯仰角的电机
* 控制气压的气阀板
*
*/
void MotionRead(void)
{
	/*读取气压值（adc）*/
	#ifdef  GAS_CONTOL_BY_PWM
	GasRead();
	#endif
	/*读取俯仰角*/
	ReadActualPos(CAN2,PITCH_MOTOR_ID);
  /*将读俯仰角姿态的标志位归0*/
	SetMotionFlag(AT_PITCH_READ);
	/*读取航向角角*//*将计算速度时间清空从这个时候开始计时*/
	ReadActualPos(CAN2,COURCE_MOTOR_ID);
	/*将读航向角姿态的标志位归0*/
	SetMotionFlag(AT_COURSE_READ);
	/*读取上电机航向角角*/
	ReadActualPos(CAN2,UP_STEER_MOTOR_ID);	
	/*读取上电机标志位归0*/
	SetMotionFlag(AT_HOLD_BALL_1_RESPONSE);
	/*读取下电机航向角角*/
	ReadActualPos(CAN2,DOWN_STEER_MOTOR_ID);
	/*读取上电机标志位归0*/
	SetMotionFlag(AT_HOLD_BALL_2_RESPONSE);
}


/* 动作状态更新函数
* 
* 持球的舵机一与舵机二
*	控制摄像头转向的舵机三
*	控制发射架航向角的电机
* 控制发射架俯仰角的电机
* 控制气压的气阀板
*
*/
void MotionStatusUpdate(void)
{
	/*判断航向角是否到位*/
	if(fabs(gRobot.sDta.courseAimAngle-gRobot.courseAngle)<0.1f)
	{
		SetMotionFlag(AT_COURSE_SUCCESS);
	}
	else
	{
		SetMotionFlag(~AT_COURSE_SUCCESS);
	}
	
	/*判断俯仰角是否到位*/
	if(fabs(gRobot.sDta.pitchAimAngle-gRobot.pitchAngle)<0.1f)
	{
		SetMotionFlag(AT_PITCH_SUCCESS);
	}
	else
	{
		SetMotionFlag(~AT_PITCH_SUCCESS);
	}
	
	
	//气压判断边界
	float gasBoundary=0.005f;//彩球气压边界
	if(gRobot.sDta.WhichGoldBall!=0){
		gasBoundary=0.003f;//金球气压边界
	}
	if(fabs(gRobot.sDta.gasAimValue-gRobot.gasValue)< gasBoundary)
	{
		SetMotionFlag(AT_GAS_SUCCESS);
		gRobot.gasPidIntegral =0.f ;
		gRobot.gasControl = 0.f;
	}
	else
	{
		SetMotionFlag(~AT_GAS_SUCCESS);
	}

	
	
	/*判断持球舵机一是否到位*/
	if(fabs(gRobot.sDta.holdBallAimAngle[0]-gRobot.holdBallAngle[0])<1.5f)
	{
		SetMotionFlag(AT_HOLD_BALL_1_SUCCESS);
	}	
	else
	{
		SetMotionFlag(~AT_HOLD_BALL_1_SUCCESS);
	}
	
	/*判断持球舵机二是否到位*/
	if(fabs(gRobot.sDta.holdBallAimAngle[1]-gRobot.holdBallAngle[1])<1.5f)
	{
		SetMotionFlag(AT_HOLD_BALL_2_SUCCESS);	
	}
	else
	{
		SetMotionFlag(~AT_HOLD_BALL_2_SUCCESS);
	}
	/*接球的时候进行接球微调*/
//	SmallChangeGetBall();
	
	/*等到进入射球进程的时候进行一次计算微调航向*/
//  SmallChange();
	
	
}


void GasRead(void){
  uint16_t adc = Get_Adc_Average(11,15);
	gRobot.gasAdc = adc;
//  #if  GAS_CONTROL == 1       
//  gRobot.gasValue=((adc-GAS_ADC_ZERO))*1.145f/2048.f;
//	#else
	gRobot.gasValue=((adc-GAS_ADC_ZERO))*1.145f/2048.f;
//	#endif
}


void SmallChangeGetBall(void){
	float countAngle=0.f;
	//定义是否进行计算了的变量
	int whetherCount=0;
	

	if(gRobot.sDta.AT_motionFlag&AT_GET_PPS_PROBLEM ){
		
		USART_OUTByDMA("AT_GET_PPS_PROBLEM");
		return;
	}else if((gRobot.sDta.robocon2018==COLORFUL_BALL_1||gRobot.sDta.robocon2018==COLORFUL_BALL_2\
	||gRobot.sDta.robocon2018==GOLD_BALL)&&(PE_FOR_THE_BALL)){
	  USART_OUTByDMA("PE NO Change upsteer");
		return;
	}
	
	switch(gRobot.sDta.robocon2018){
		case COLORFUL_BALL_1:
			if(gRobot.sDta.process!=TO_GET_BALL_1){
				return;
			}
			
			if(fabs(gRobot.angle)>0.3f && gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_1_SUCCESS ){
				countAngle = gRobot.sDta.holdBallAimAngle[0]-gRobot.angle;

				whetherCount=1;
				//第一个彩球的调节范围
			}else {
				 whetherCount=0;
			}
		break;
		
		case COLORFUL_BALL_2:
			if(gRobot.sDta.process!=TO_GET_BALL_2){
				return;
			}
			
			if(fabs(gRobot.angle)>0.3f && gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_1_SUCCESS ){
				
			  countAngle = gRobot.sDta.holdBallAimAngle[0]-gRobot.angle;

				whetherCount=1;
				//第一个彩球的调节范围
			}else {
				 whetherCount=0;
			}
		break;
		
	}	
	
	//如果计算了判断计算值是否与给定的值超过了courseChangeDifference,超过了则微调
	if(whetherCount){
		/*防止计算的值超过限定角度*/
		if(fabs(gRobot.angle)>6.f||fabs(countAngle - GetGetBallUpSteerAngle(gRobot.sDta.process)) > 6.f){
			USART_OUTByDMA("countAngle OUT OF RANGE");
			USART_OUTByDMAF(countAngle);
			return;
		}
		
		if(fabs(countAngle - gRobot.sDta.holdBallAimAngle[0])> 0.3f){
			  
				SetMotionFlag(~AT_HOLD_BALL_1_SUCCESS);
				USART_OUTByDMA("upsteerAngle need change=");
				USART_OUTByDMAF(countAngle);
//			  gRobot.sDta.holdBallAimAngle[0]= countAngle;
		}else {
				USART_OUTByDMA("upsteerAngle OK\t");
				USART_OUTByDMAF(gRobot.sDta.holdBallAimAngle[0]);
		}
  }
	
	
	
}
