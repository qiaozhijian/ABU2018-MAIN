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
	gRobot.motorPara_t.pitchAimPos=(int)(angle*28.0f*19.2f*8192.f/360.f);
	PosCrl(CAN2, 5,ABSOLUTE_MODE,(angle*28.0f*19.2f*8192.f/360.f));
}

void CourseAngleMotion(float angle)
{
	if(angle>0.f)
		angle=0.f;
	else if(angle<-180.f)
		angle=-180.f;
	gRobot.motorPara_t.courseAimPos=(int)((angle-5.6f)*10.0f*19.2f*8192.f/360.f);
	PosCrl(CAN2, 6,ABSOLUTE_MODE,((angle-5.6f)*10.0f*19.2f*8192.f/360.f));
}

void GasMotion(float value)
{
	gRobot.motorPara_t.gasAimValue=value;
	CAN_TxMsg(CAN2,SEND_TO_GASSENSOR,(uint8_t*)(&value),4);
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

void prepareMotionParaInit(void)
{
	/*׼��ȥ�õ�һ���������*/
	gRobot.prepareMotion.PrepareGetBall1.courseAngle=0.f;
	gRobot.prepareMotion.PrepareGetBall1.pitchAngle=26.6f;
	gRobot.prepareMotion.PrepareGetBall1.steerAngle=90.f;
	gRobot.prepareMotion.PrepareGetBall1.steerSpeed=1000;
	gRobot.prepareMotion.PrepareGetBall1.gasAim=0.45f;
	
	/*׼��ȥ�õڶ����������*/
	gRobot.prepareMotion.PrepareGetBall2.courseAngle=0.f;
	gRobot.prepareMotion.PrepareGetBall2.pitchAngle=27.1f;
	gRobot.prepareMotion.PrepareGetBall2.steerAngle=-90.f;
	gRobot.prepareMotion.PrepareGetBall2.steerSpeed=1000;
	gRobot.prepareMotion.PrepareGetBall2.gasAim=0.45f;
	
	/*׼��ȥ�õ������������*/
	gRobot.prepareMotion.PrepareGetBall3.courseAngle=30.1f;
	gRobot.prepareMotion.PrepareGetBall3.pitchAngle=0.f;
	gRobot.prepareMotion.PrepareGetBall3.steerAngle=90.f;
	gRobot.prepareMotion.PrepareGetBall3.steerSpeed=1000;
	gRobot.prepareMotion.PrepareGetBall3.gasAim=0.45f;
	
	/*׼�����һ���������*/
	gRobot.prepareMotion.PrepareShootBall1.courseAngle=-76.9f;
	gRobot.prepareMotion.PrepareShootBall1.pitchAngle=10.2f;
	gRobot.prepareMotion.PrepareShootBall1.steerAngle=0.f;
	gRobot.prepareMotion.PrepareShootBall1.steerSpeed=2000;
	gRobot.prepareMotion.PrepareShootBall1.gasAim=0.45f;
	
	/*׼����ڶ����������*/
	gRobot.prepareMotion.PrepareShootBall2.courseAngle=-79.5f;
	gRobot.prepareMotion.PrepareShootBall2.pitchAngle=30.1f;
	gRobot.prepareMotion.PrepareShootBall1.steerAngle=0.f;
	gRobot.prepareMotion.PrepareShootBall1.steerSpeed=2000;
	gRobot.prepareMotion.PrepareShootBall2.gasAim=0.45f;
	
	/*׼����������������*/
	gRobot.prepareMotion.PrepareShootBall3.courseAngle=-90.f;
	gRobot.prepareMotion.PrepareShootBall3.pitchAngle=32.5f;
	gRobot.prepareMotion.PrepareShootBall1.steerAngle=0.f;
	gRobot.prepareMotion.PrepareShootBall1.steerSpeed=2000;
	gRobot.prepareMotion.PrepareShootBall3.gasAim=0.45f;
	
}

void PrepareGetBallMotion(shootPara_t PrepareGetBall_t)
{
		//������ѹ
		GasMotion(PrepareGetBall_t.gasAim);
		/*���ø����Ƕ�*/
		PitchAngleMotion(PrepareGetBall_t.pitchAngle);
		/*���ú���Ƕ�*/
		CourseAngleMotion(PrepareGetBall_t.courseAngle);
		/*�ر��·���λצ*/
		GasValveControl(GASVALVE_BOARD_ID , CLAW_ID , CLAW_SHUT);
		/*���ת��*/
		ROBS_PosCrl(PrepareGetBall_t.steerAngle, PrepareGetBall_t.steerAngle, PrepareGetBall_t.steerSpeed);
}

void PrepareGetBall(int index)
{
	switch(index)
	{
		case BALL_1:
			//����׼���õ���Ĳ���
			PrepareGetBallMotion(gRobot.prepareMotion.PrepareGetBall1);
			break;
		case BALL_2:
			//����׼���õ���Ĳ���
			PrepareGetBallMotion(gRobot.prepareMotion.PrepareGetBall2);
			break;
		case BALL_3:
			//����׼���õ���Ĳ���
			PrepareGetBallMotion(gRobot.prepareMotion.PrepareGetBall3);
			break;
		default:
			USART_OUT(DEBUG_USART,"PrepareGetBall error\r\n");
			break;
	}	
}

void PrepareShootBallMotion(shootPara_t PrepareShootBall_t)
{
	//������ѹ
	GasMotion(PrepareShootBall_t.gasAim);
	/*���ø����Ƕ�*/
	PitchAngleMotion(PrepareShootBall_t.pitchAngle);
	/*���ú���Ƕ�*/
	CourseAngleMotion(PrepareShootBall_t.courseAngle);
	/*�������һ�*/
	Delay_ms(500);
	/*��ǰ�򿪷���װ��С����*/
	GasValveControl(GASVALVE_BOARD_ID , SHOOT_SMALL_ID , 1);
	/*���ת��*/
	ROBS_PosCrl(PrepareShootBall_t.steerAngle, PrepareShootBall_t.steerAngle, PrepareShootBall_t.steerSpeed);
}

void PrepareShootBall(int index)
{
	switch(index)
	{
		case BALL_1:
			//����׼������Ĳ���
			PrepareShootBallMotion(gRobot.prepareMotion.PrepareGetBall1);
			break;
		case BALL_2:
			//����׼������Ĳ���
			PrepareShootBallMotion(gRobot.prepareMotion.PrepareGetBall2);
			break;
		case BALL_3:
			//����׼������Ĳ���
			PrepareShootBallMotion(gRobot.prepareMotion.PrepareGetBall3);
			break;
		default:
			USART_OUT(DEBUG_USART,"PrepareShootBall error\r\n");
			break;
	}	
}
