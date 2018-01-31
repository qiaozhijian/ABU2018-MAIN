#include "task.h"
#include "steer.h"
#include "can.h"

extern Robot_t gRobot;

/* 动作执行函数
* 
* 持球的舵机一与舵机二
*	控制摄像头转向的舵机三
*
*/
void MotionExecute(void)
{
	/*如果持球舵机没有到位*/
	if(!(gRobot.AT_motionFlag&AT_HOLD_BALL1_SUCCESS)
		||!(gRobot.AT_motionFlag&AT_HOLD_BALL2_SUCCESS))
	{
		HoldBallPosCrl(gRobot.holdBallAimAngle,2000);
	}
	
	if(!(gRobot.AT_motionFlag&AT_COURSE_SUCCESS))
	{
		CourseAngleMotion(gRobot.courseAimAngle);
	}
	
	if(!(gRobot.AT_motionFlag&AT_PITCH_SUCCESS))
	{
		PitchAngleMotion(gRobot.pitchAimAngle);
	}
	
	if(!(gRobot.AT_motionFlag&AT_GAS_SUCCESS))
	{
		//PitchAngleMotion(gRobot.pitchAimAngle);
	}
	
	
	//CameraAlign();
	
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
	static char count=0;
	count++;
	/*读取俯仰角*/
	ReadActualPos(CAN2,5);
  /*将读俯仰角姿态的标志位归0*/
	SetMotionFlag(~AT_PITCH_READ_SUCCESS);
	/*读取航向角角*/
	ReadActualPos(CAN2,6);
  /*将读航向角姿态的标志位归0*/
	SetMotionFlag(~AT_COURSE_READ_SUCCESS);
	/*读取舵机状态*/
	
	ReadHoldBallSteerPos();
	/*像平板发送气压值*/
	//if(gRobot.isOpenGasReturn&&count==3)
//	if(count==3)
	{
		count=0;
		USART_BLE_SEND(gRobot.gasValue);
	}
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
	if(abs(gRobot.cameraAimAngle-gRobot.cameraAngle)<1.f)
	{
		SetMotionFlag(AT_CAMERA_ROTATE_SUCCESS);
	}
	else
	{
		SetMotionFlag(~AT_CAMERA_ROTATE_SUCCESS);
	}
	
	
	if(abs(gRobot.courseAimAngle-gRobot.courseAngle)<1.f)
	{
		SetMotionFlag(AT_COURSE_SUCCESS);
	}
	else
	{
		SetMotionFlag(~AT_COURSE_SUCCESS);
	}
	
	if(abs(gRobot.pitchAimAngle-gRobot.pitchAngle)<1.f)
	{
		SetMotionFlag(AT_PITCH_SUCCESS);
	}
	else
	{
		SetMotionFlag(~AT_PITCH_SUCCESS);
	}
	
	if(abs(gRobot.holdBallAimAngle-gRobot.holdBallAngle[0])<1.f)
	{
		SetMotionFlag(AT_HOLD_BALL1_SUCCESS);
	}
	else
	{
		SetMotionFlag(~AT_HOLD_BALL1_SUCCESS);
	}
	
	if(abs(gRobot.holdBallAimAngle-gRobot.holdBallAngle[1])<1.f)
	{
		SetMotionFlag(AT_HOLD_BALL2_SUCCESS);
	}
	else
	{
		SetMotionFlag(~AT_HOLD_BALL2_SUCCESS);
	}
	
	if(abs(gRobot.gasAimValue-gRobot.gasValue)<0.01f)
	{
		SetMotionFlag(AT_GAS_SUCCESS);
	}
	else
	{
		SetMotionFlag(~AT_GAS_SUCCESS);
	}
	
}

