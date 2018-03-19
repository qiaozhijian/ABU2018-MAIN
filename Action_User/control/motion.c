#include "task.h"
#include "steer.h"
#include "can.h"
#include "shoot.h"
#include "stm32f4xx_it.h"
#include "timer.h"
#include "iwdg.h"
#include "robot.h"

extern Robot_t gRobot;

void PitchAngleMotion(float angle)
{
  /*控制的为-20°-10°实际给电机的是30°-0°*/
  if(angle>20.f)
    angle=20.f;
  else if(angle<-20.f)
    angle=-20.f;
	
	angle=20.f-angle;
	
  PosCrl(CAN2, 5,ABSOLUTE_MODE,PITCH_ANGLE_TO_CODE(angle));
}

void CourseAngleMotion(float angle)
{
  if(angle<0.f)
    angle=0.f;
  else if(angle>190.f)
    angle=190.f;
	
	angle=angle+3.f;
	
  PosCrl(CAN2, 6,ABSOLUTE_MODE,COURSE_ANGLE_TO_CODE(angle));
}

void GasMotion(float value)
{
  CAN_TxMsg(CAN2,SEND_TO_GASSENSOR,(uint8_t*)(&value),4);
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
	ReadActualPos(CAN2,5);
  /*将读俯仰角姿态的标志位归0*/
	SetMotionFlag(~AT_PITCH_READ_SUCCESS);
	/*读取航向角角*/
	ReadActualPos(CAN2,6);
	/*读取上电机航向角角*/
	ReadActualPos(CAN2,7);
	/*读取下电机航向角角*/
	ReadActualPos(CAN2,8);
  /*将读航向角姿态的标志位归0*/
	SetMotionFlag(~AT_COURSE_READ_SUCCESS);
	/*向舵机发送指令，从串口中断读取的状态是否发生错误，在MotionStatusUpdate（）对外发数*/
	ReadSteerErrorAll();
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
	
	
	if(fabs(gRobot.sDta.gasAimValue-gRobot.gasValue)<0.002f)
	{
		SetMotionFlag(AT_GAS_SUCCESS);
	}
	else
	{
		SetMotionFlag(~AT_GAS_SUCCESS);
	}
	
	/*等到进入射球进程的时候进行一次计算微调航向*/
  SmallChange();
	
//	/*判断持球舵机一是否到位*/
//	if(fabs(gRobot.sDta.holdBallAimAngle-gRobot.holdBallAngle[0])<0.5f)
//	{
//		SetMotionFlag(AT_HOLD_BALL_1_SUCCESS);
//		/*转到一定小角度就转不动了，干脆就不转了*/
//		if(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_2_SUCCESS)
//			gRobot.sDta.holdBallAimAngle=gRobot.holdBallAngle[0];
//	}	
//	else
//	{
//		SetMotionFlag(~AT_HOLD_BALL_1_SUCCESS);
//	}
//	
//	/*判断持球舵机二是否到位*/
//	if(fabs(gRobot.sDta.holdBallAimAngle-gRobot.holdBallAngle[1])<0.5f)
//	{
//		SetMotionFlag(AT_HOLD_BALL_2_SUCCESS);
//		/*转到一定小角度就转不动了，干脆就不转了*/
//		if(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_2_SUCCESS)
//			gRobot.sDta.holdBallAimAngle=gRobot.holdBallAngle[1];
//	}
//	else
//	{
//		SetMotionFlag(~AT_HOLD_BALL_2_SUCCESS);
//	}


}
