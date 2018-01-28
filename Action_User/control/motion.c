#include "task.h"
#include "steer.h"
#include "can.h"

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
		HoldBallPosCrl(gRobot.holdBallAimAngle,2000);
	}
	
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
 // USART_BLE_SEND(gRobot.gasValue);
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
	if(abs(gRobot.cameraAimAngle-gRobot.cameraAngle)<1.f)
	{
		SetMotionFlag(AT_CAMERA_SUCCESS);
	}
	else
	{
		SetMotionFlag(~AT_CAMERA_SUCCESS);
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
	
}

