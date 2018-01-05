#include "shoot.h"
#include "task.h"
#include "timer.h"
#include "steer.h"


extern Robot_t gRobot;


void PitchAngleMotion(float angle)
{
	if(angle>60.f)
		angle=60.f;
	else if(angle<0.f)
		angle=0.f;
	
	PosCrl(CAN2, 5,ABSOLUTE_MODE,(angle*28.0f*19.2f*8192.f/360.f));
}

void CourseAngleMotion(float angle)
{
	if(angle>0.f)
		angle=0.f;
	else if(angle<-180.f)
		angle=-180.f;
	PosCrl(CAN2, 6,ABSOLUTE_MODE,((angle-5.6f)*10.0f*19.2f*8192.f/360.f));
}

void ShootBall(void)
{				
	/*���Ӵ�*/
	GasValveControl(GASVALVE_BOARD_ID , CLAW_ID , CLAW_OPEN);
	/*�����ʵ���ʱ��֤���Ӻ��򲻸���*/
	Delay_ms(300);
	/*�������������״�*/
	GasValveControl(GASVALVE_BOARD_ID , SHOOT_BIG_ID , 1);
}

void ShootReset(void)
{
	/*��λ*/
	GasValveControl(GASVALVE_BOARD_ID , SHOOT_SMALL_ID , 0);
	GasValveControl(GASVALVE_BOARD_ID , SHOOT_BIG_ID , 0);
}

void PrepareGetBall(int index)
{
	switch(index)
	{
		case BALL_1:
			/*���ø����Ƕ�*/
			PitchAngleMotion(26.6);
			/*���ú���Ƕ�*/
			CourseAngleMotion(0);
			/*�ر��·���λצ*/
			GasValveControl(GASVALVE_BOARD_ID , CLAW_ID , CLAW_SHUT);
			/*���ת��*/
			ROBS_PosCrl(90, 91, 1000);
			break;
		case BALL_2:
			/*���ø����Ƕ�*/
			PitchAngleMotion(27.1f);
			/*���ú���Ƕ�*/
			CourseAngleMotion(0.f);
			/*�ر��·���λצ*/
			GasValveControl(GASVALVE_BOARD_ID , CLAW_ID , CLAW_SHUT);
			/*���ת��*/
			ROBS_PosCrl(-90, -90, 1000);
			break;
		case BALL_3:
			/*���ø����Ƕ�*/
			PitchAngleMotion(30.1f);
			/*���ú���Ƕ�*/
			CourseAngleMotion(0.f);
			/*�ر��·���λצ*/
			GasValveControl(GASVALVE_BOARD_ID , CLAW_ID , CLAW_SHUT);
			/*���ת��*/
			ROBS_PosCrl(90, 90, 1000);
			break;
		default:
			USART_OUT(DEBUG_USART,"PrepareGetBall error\r\n");
			break;
	}	
}

void PrepareShootBall(int index)
{
	switch(index)
	{
		case BALL_1:
			/*���ø����Ƕ�*/
			PitchAngleMotion(10.2f);
			/*���ú���Ƕ�*/
			CourseAngleMotion(-76.9f);
			/*�������һ�*/
			Delay_ms(500);
			/*��ǰ�򿪷���װ��С����*/
			GasValveControl(GASVALVE_BOARD_ID , SHOOT_SMALL_ID , 1);
			/*���ת��*/
			ROBS_PosCrl(0, 0, 2000);
			break;
		case BALL_2:
			/*���ø����Ƕ�*/
			PitchAngleMotion(30.1f);
			/*���ú���Ƕ�*/
			CourseAngleMotion(-79.5f);
			/*�������һ�*/
			Delay_ms(500);
			/*��ǰ�򿪷���װ��С����*/
			GasValveControl(GASVALVE_BOARD_ID , SHOOT_SMALL_ID , 1);
			/*���ת��*/
			ROBS_PosCrl(0, 0, 2000);
			break;
		case BALL_3:
			/*���ø����Ƕ�*/
			PitchAngleMotion(32.5f);
			/*���ú���Ƕ�*/
			CourseAngleMotion(-90.f);
			/*�������һ�*/
			Delay_ms(500);
			/*��ǰ�򿪷���װ��С����*/
			GasValveControl(GASVALVE_BOARD_ID , SHOOT_SMALL_ID , 1);
			/*���ת��*/
			ROBS_PosCrl(0, 0, 2000);
			break;
		default:
			USART_OUT(DEBUG_USART,"PrepareShootBall error\r\n");
			break;
	}	
}
