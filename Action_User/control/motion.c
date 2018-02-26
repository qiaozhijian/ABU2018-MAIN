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
  if(angle>10.f)
    angle=10.f;
  else if(angle<-20.f)
    angle=-20.f;
	
	angle=10.f-angle;
	
  PosCrl(CAN2, 5,ABSOLUTE_MODE,PITCH_ANGLE_TO_CODE(angle));
}

void CourseAngleMotion(float angle)
{
  if(angle<0.f)
    angle=0.f;
  else if(angle>190.f)
    angle=190.f;
	
	angle=angle+10.6f;
	
  PosCrl(CAN2, 6,ABSOLUTE_MODE,COURSE_ANGLE_TO_CODE(angle));
}

void GasMotion(float value)
{
  CAN_TxMsg(CAN2,SEND_TO_GASSENSOR,(uint8_t*)(&value),4);
}

//摄像头。c
void TalkToCamera(uint32_t command)
{
	int times=0;
	switch(command)
	{
		case CAMERA_START:
			/*如果与摄像头通信标志位没有置一，500us发一次数据*/
			while(!(gRobot.sDta.AT_motionFlag&AT_CAMERA_TALK_SUCCESS))
			{
				USART_OUT(CAMERA_USART,"AT\r\n");
				Delay_ms(3);
				times++;
				IWDG_Feed();
//				if(times>100)
//				{
////					USART_OUT(DEBUG_USART,"Camera dead\r\n");
////					break;
//				}
			}
			times=0;
			/*清空标志位*/
			SetMotionFlag(~AT_CAMERA_TALK_SUCCESS);
			break;
		case CAMERA_SHUT_ALL:
			/*如果与摄像头通信标志位没有置一，500us发一次数据*/
			while(!(gRobot.sDta.AT_motionFlag&AT_CAMERA_TALK_SUCCESS))
			{
				USART_OUT(CAMERA_USART,"AT+%d\r\n",CAMERA_SHUT_ALL);
				Delay_ms(3);
				times++;
				IWDG_Feed();
				if(times>100)
				{
//					USART_OUT(DEBUG_USART,"Camera dead\r\n");
//					break;
				}
			}
			times=0;
			/*清空标志位*/
			SetMotionFlag(~AT_CAMERA_TALK_SUCCESS);
			break;
		case CAMERA_OPEN_NEAR:
			/*如果与摄像头通信标志位没有置一，500us发一次数据*/
			while(!(gRobot.sDta.AT_motionFlag&AT_CAMERA_TALK_SUCCESS))
			{
				USART_OUT(CAMERA_USART,"AT+%d\r\n",CAMERA_OPEN_NEAR);
				Delay_ms(3);
				times++;
				IWDG_Feed();
				if(times>100)
				{
//					USART_OUT(DEBUG_USART,"Camera dead\r\n");
//					break;
				}
			}
			times=0;
			/*清空标志位*/
			SetMotionFlag(~AT_CAMERA_TALK_SUCCESS);
			break;
		case CAMERA_OPEN_FAR:
			/*如果与摄像头通信标志位没有置一，500us发一次数据*/
			while(!(gRobot.sDta.AT_motionFlag&AT_CAMERA_TALK_SUCCESS))
			{
				USART_OUT(CAMERA_USART,"AT+%d\r\n",CAMERA_OPEN_FAR);
				Delay_ms(3);
				times++;
				IWDG_Feed();
				if(times>100)
				{
//					USART_OUT(DEBUG_USART,"Camera dead\r\n");
//					break;
				}
			}
			times=0;
			/*清空标志位*/
			SetMotionFlag(~AT_CAMERA_TALK_SUCCESS);
			break;
	}
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
		#ifdef TEST
		HoldBallPosCrl(gRobot.sDta.holdBallAimAngle[0],2000);
		#else
		HoldBallPosCrl(gRobot.sDta.holdBallAimAngle,2000);
		#endif
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
	
	//CameraSteerPosCrl(gRobot.sDta.cameraAimAngle);

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
	/*向舵机发送指令，从串口中断读取的状态是否发生错误，在MotionStatusUpdate（）对外发数*/
	ReadSteerErrorAll();
	/*像平板发送气压值*/
	//if(gRobot.isOpenGasReturn&&count==3)
	if(count==3)
	{
		count=0;
	//	USART_BLE_SEND(gRobot.gasValue);
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
	/*判断航向角是否到位*/
	if(fabs(gRobot.sDta.courseAimAngle-gRobot.courseAngle)<0.01f)
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
	
	
	if(fabs(gRobot.sDta.gasAimValue-gRobot.gasValue)<0.001f)
	{
		SetMotionFlag(AT_GAS_SUCCESS);
	}
	else
	{
		SetMotionFlag(~AT_GAS_SUCCESS);
	}
	
	#ifndef TEST
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
	#endif
	
//	/*判断相机转台舵机是否到位*/
//	if(fabs(gRobot.sDta.cameraAimAngle-gRobot.cameraAngle)<0.5f)
//	{
//		SetMotionFlag(AT_CAMERA_RESPONSE_SUCCESS);
//	}
//	else
//	{
//		SetMotionFlag(~AT_CAMERA_RESPONSE_SUCCESS);
//	}

	SteerResponseError(HOLD_BALL_1,gRobot.holdBall1Error);
	SteerResponseError(HOLD_BALL_2,gRobot.holdBall2Error);
	SteerResponseError(CAMERA_STEER,gRobot.cameraSteerError);
	
}
