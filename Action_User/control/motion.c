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
	#ifndef  GAS_CONTOL_BY_PWM
	uint32_t data[2]={0x00005045,0x00000000};
	union
	{
		float dataFloat;
		uint32_t data32;
	}sendData;
	
	sendData.dataFloat = value;
	
	data[1] = sendData.data32;
  CAN_TxMsg(CAN2,SEND_TO_GASSENSOR,(uint8_t*)(&data),8);
	#else
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
	
	/*等到进入射球进程的时候进行一次计算微调航向*/
  SmallChange();
	
}


void GasRead(void){
  int adc = Get_Adc_Average(10,15);
  gRobot.gasValue=((float)(adc-370))*2.f/4095.f*1.145f;
}

