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
  /*���Ƶ�Ϊ-20��-10��ʵ�ʸ��������30��-0��*/
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

//����ͷ��c
void TalkToCamera(uint32_t command)
{
	int times=0;
	switch(command)
	{
		case CAMERA_START:
			/*���������ͷͨ�ű�־λû����һ��500us��һ������*/
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
			/*��ձ�־λ*/
			SetMotionFlag(~AT_CAMERA_TALK_SUCCESS);
			break;
		case CAMERA_SHUT_ALL:
			/*���������ͷͨ�ű�־λû����һ��500us��һ������*/
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
			/*��ձ�־λ*/
			SetMotionFlag(~AT_CAMERA_TALK_SUCCESS);
			break;
		case CAMERA_OPEN_NEAR:
			/*���������ͷͨ�ű�־λû����һ��500us��һ������*/
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
			/*��ձ�־λ*/
			SetMotionFlag(~AT_CAMERA_TALK_SUCCESS);
			break;
		case CAMERA_OPEN_FAR:
			/*���������ͷͨ�ű�־λû����һ��500us��һ������*/
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
			/*��ձ�־λ*/
			SetMotionFlag(~AT_CAMERA_TALK_SUCCESS);
			break;
	}
}

/* ����ִ�к���
* 
* ����Ķ��һ������
*	��������ͷת��Ķ����
*
*/

void MotionExecute(void)
{
	/*���������û�е�λ*/
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

/* ����״̬��ѯ����
* 
* ����Ķ��һ������
*	��������ͷת��Ķ����
*	���Ʒ���ܺ���ǵĵ��
* ���Ʒ���ܸ����ǵĵ��
* ������ѹ��������
*
*/
void MotionRead(void)
{
	static char count=0;
	count++;
	/*��ȡ������*/
	ReadActualPos(CAN2,5);
  /*������������̬�ı�־λ��0*/
	SetMotionFlag(~AT_PITCH_READ_SUCCESS);
	/*��ȡ����ǽ�*/
	ReadActualPos(CAN2,6);
  /*�����������̬�ı�־λ��0*/
	SetMotionFlag(~AT_COURSE_READ_SUCCESS);
	/*��������ָ��Ӵ����ж϶�ȡ��״̬�Ƿ���������MotionStatusUpdate�������ⷢ��*/
	ReadSteerErrorAll();
	/*��ƽ�巢����ѹֵ*/
	//if(gRobot.isOpenGasReturn&&count==3)
	if(count==3)
	{
		count=0;
	//	USART_BLE_SEND(gRobot.gasValue);
	}
}


/* ����״̬���º���
* 
* ����Ķ��һ������
*	��������ͷת��Ķ����
*	���Ʒ���ܺ���ǵĵ��
* ���Ʒ���ܸ����ǵĵ��
* ������ѹ��������
*
*/
void MotionStatusUpdate(void)
{
	/*�жϺ�����Ƿ�λ*/
	if(fabs(gRobot.sDta.courseAimAngle-gRobot.courseAngle)<0.01f)
	{
		SetMotionFlag(AT_COURSE_SUCCESS);
	}
	else
	{
		SetMotionFlag(~AT_COURSE_SUCCESS);
	}
	
	/*�жϸ������Ƿ�λ*/
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
//	/*�жϳ�����һ�Ƿ�λ*/
//	if(fabs(gRobot.sDta.holdBallAimAngle-gRobot.holdBallAngle[0])<0.5f)
//	{
//		SetMotionFlag(AT_HOLD_BALL_1_SUCCESS);
//		/*ת��һ��С�ǶȾ�ת�����ˣ��ɴ�Ͳ�ת��*/
//		if(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_2_SUCCESS)
//			gRobot.sDta.holdBallAimAngle=gRobot.holdBallAngle[0];
//	}	
//	else
//	{
//		SetMotionFlag(~AT_HOLD_BALL_1_SUCCESS);
//	}
//	
//	/*�жϳ��������Ƿ�λ*/
//	if(fabs(gRobot.sDta.holdBallAimAngle-gRobot.holdBallAngle[1])<0.5f)
//	{
//		SetMotionFlag(AT_HOLD_BALL_2_SUCCESS);
//		/*ת��һ��С�ǶȾ�ת�����ˣ��ɴ�Ͳ�ת��*/
//		if(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_2_SUCCESS)
//			gRobot.sDta.holdBallAimAngle=gRobot.holdBallAngle[1];
//	}
//	else
//	{
//		SetMotionFlag(~AT_HOLD_BALL_2_SUCCESS);
//	}
	#endif
	
//	/*�ж����ת̨����Ƿ�λ*/
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
