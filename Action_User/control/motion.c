#include "task.h"
#include "steer.h"
#include "can.h"
#include "shoot.h"
extern Robot_t gRobot;

/* ����ִ�к���
* 
* ����Ķ��һ������
*	��������ͷת��Ķ����
*
*/

void MotionExecute(void)
{
	/*���������û�е�λ*/
	if(!(gRobot.AT_motionFlag&AT_HOLD_BALL1_SUCCESS)
		||!(gRobot.AT_motionFlag&AT_HOLD_BALL2_SUCCESS))
	{
		#ifdef TEST
		HoldBallPosCrl(gRobot.holdBallAimAngle[0],2000);
		#else
		HoldBallPosCrl(gRobot.holdBallAimAngle,2000);
		#endif
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
		GasMotion(gRobot.gasAimValue);
	}
	
	//CameraSteerPosCrl(gRobot.cameraAimAngle);

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
	/*��ȡ���״̬*/
	
	ReadHoldBallSteerPos();
	/*��ƽ�巢����ѹֵ*/
	//if(gRobot.isOpenGasReturn&&count==3)
//	if(count==3)
	{
		count=0;
		USART_BLE_SEND(gRobot.gasValue);
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
	if(abs(gRobot.courseAimAngle-gRobot.courseAngle)<1.f)
	{
		SetMotionFlag(AT_COURSE_SUCCESS);
	}
	else
	{
		SetMotionFlag(~AT_COURSE_SUCCESS);
	}
	
	/*�жϸ������Ƿ�λ*/
	if(abs(gRobot.pitchAimAngle-gRobot.pitchAngle)<1.f)
	{
		SetMotionFlag(AT_PITCH_SUCCESS);
	}
	else
	{
		SetMotionFlag(~AT_PITCH_SUCCESS);
	}
	
	#ifndef TEST
	/*�жϳ�����һ�Ƿ�λ*/
	if(abs(gRobot.holdBallAimAngle-gRobot.holdBallAngle[0])<0.5f)
	{
		SetMotionFlag(AT_HOLD_BALL1_SUCCESS);
		/*ת��һ��С�ǶȾ�ת�����ˣ��ɴ�Ͳ�ת��*/
		if(gRobot.AT_motionFlag&AT_HOLD_BALL2_SUCCESS)
			gRobot.holdBallAimAngle=gRobot.holdBallAngle[0];
	}	
	else
	{
		SetMotionFlag(~AT_HOLD_BALL1_SUCCESS);
	}
	
	/*�жϳ��������Ƿ�λ*/
	if(abs(gRobot.holdBallAimAngle-gRobot.holdBallAngle[1])<0.5f)
	{
		SetMotionFlag(AT_HOLD_BALL2_SUCCESS);
		/*ת��һ��С�ǶȾ�ת�����ˣ��ɴ�Ͳ�ת��*/
		if(gRobot.AT_motionFlag&AT_HOLD_BALL2_SUCCESS)
			gRobot.holdBallAimAngle=gRobot.holdBallAngle[1];
	}
	else
	{
		SetMotionFlag(~AT_HOLD_BALL2_SUCCESS);
	}
	#endif
	
	/*�ж����ת̨����Ƿ�λ*/
	if(abs(gRobot.cameraAimAngle-gRobot.cameraAngle)<0.5f)
	{
		SetMotionFlag(AT_CAMERA_ROTATE_SUCCESS);
	}
	else
	{
		SetMotionFlag(~AT_CAMERA_ROTATE_SUCCESS);
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
