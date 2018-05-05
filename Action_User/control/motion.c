#include "task.h"
#include "steer.h"
#include "can.h"
#include "shoot.h"
#include "stm32f4xx_it.h"
#include "timer.h"
#include "iwdg.h"
#include "robot.h"
#include "motion.h"

extern Robot_t gRobot;

void PitchAngleMotion(float angle)
{
  /*控制的为-20°-10°实际给电机的是30°-0°*/
  if(angle>30.f)
    angle=0.f;
  else if(angle<-10.f)
    angle=-10.f;
	
	angle=PITCH_COMPENSATE - angle;
	
  PosCrl(CAN2, PITCH_MOTOR_ID,ABSOLUTE_MODE,PITCH_ANGLE_TO_CODE(angle));
}

void CourseAngleMotion(float angle)
{
	angle=angle-COURCE_COMPENSATE;
  if(angle<0.f)
    angle=0.f;
  else if(angle>150.f)
    angle=150.f;
	
	
  PosCrl(CAN2, COURCE_MOTOR_ID,ABSOLUTE_MODE,COURSE_ANGLE_TO_CODE(angle));
}

void GasMotion(float value)
{
	uint32_t data[2]={0x00005045,0x00000000};
	union
	{
		float dataFloat;
		uint32_t data32;
	}sendData;
	
	sendData.dataFloat = value;
	
	data[1] = sendData.data32;
  CAN_TxMsg(CAN2,SEND_TO_GASSENSOR,(uint8_t*)(&data),8);
}

void GasEnable(void)
{
	uint32_t data[2]={0x00004145,1};
  CAN_TxMsg(CAN2,SEND_TO_GASSENSOR,(uint8_t*)(&data),8);	
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
	/*读取俯仰角*/
	ReadActualPos(CAN2,PITCH_MOTOR_ID);
  /*将读俯仰角姿态的标志位归0*/
	SetMotionFlag(~AT_PITCH_READ_SUCCESS);
	/*读取航向角角*//*将计算速度时间清空从这个时候开始计时*/
	ReadActualPos(CAN2,COURCE_MOTOR_ID);
	gRobot.robotVel.countCourseTime=0;
	/*读取上电机航向角角*/
	ReadActualPos(CAN2,UP_STEER_MOTOR_ID);
	gRobot.robotVel.countSteerTime=0;
	/*读取下电机航向角角*/
	ReadActualPos(CAN2,DOWN_STEER_MOTOR_ID);
  /*将读航向角姿态的标志位归0*/
	SetMotionFlag(~AT_COURSE_READ_SUCCESS);
	/*读取航向速度*/
	ReadActualVel(CAN2,COURCE_MOTOR_ID);
	/*读取上舵机速度*/
	ReadActualVel(CAN2,UP_STEER_MOTOR_ID);
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
	if(fabs(gRobot.sDta.pitchAimAngle-gRobot.pitchAngle)<0.01f)
	{
		SetMotionFlag(AT_PITCH_SUCCESS);
	}
	else
	{
		SetMotionFlag(~AT_PITCH_SUCCESS);
	}
	
	
	if(fabs(gRobot.sDta.gasAimValue-gRobot.gasValue)<0.0025f)
	{
		SetMotionFlag(AT_GAS_SUCCESS);
	}
	else
	{
		SetMotionFlag(~AT_GAS_SUCCESS);
	}
	
	
	/*判断持球舵机一是否到位*/
	if(fabs(gRobot.sDta.holdBallAimAngle[0]-gRobot.holdBallAngle[0])<3.f)
	{
		SetMotionFlag(AT_HOLD_BALL_1_SUCCESS);
	}	
	else
	{
		SetMotionFlag(~AT_HOLD_BALL_1_SUCCESS);
	}
	
	/*判断持球舵机二是否到位*/
	if(fabs(gRobot.sDta.holdBallAimAngle[1]-gRobot.holdBallAngle[1])<3.f)
	{
		SetMotionFlag(AT_HOLD_BALL_2_SUCCESS);	
	}
	else
	{
		SetMotionFlag(~AT_HOLD_BALL_2_SUCCESS);
	}
	
	/*等到进入射球进程的时候进行一次计算微调航向*/
  //SmallChange();


}

