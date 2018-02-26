#include "shoot.h"
#include "task.h"
#include "timer.h"
#include "steer.h"
#include "motion.h"
#include "includes.h"
#include "process.h"

extern OS_EVENT *PeriodSem;


extern Robot_t gRobot;

/*���������Ͷ��Ĳ���*/
motionPara_t PrepareCompete;
motionPara_t PrepareGetBall1;
motionPara_t PrepareGetBall2;
motionPara_t PrepareGetBall3;
motionPara_t PrepareShootBall1;
motionPara_t PrepareShootBall2;
motionPara_t PrepareShootBall3;


void ShootBall(void)
{				
  CPU_INT08U  os_err;
  os_err = os_err;
	
  /*�����ʵ���ʱ��֤���Ӻ��򲻸���*/
  Delay_ms(2000);
	ShootLedOn();
  /*���Ӵ�*/
  ClawOpen();
  Delay_ms(300);
  /*�������������״�*/
  ShootBigOpen();
}

void ShootReset(void)
{
	ShootLedOff();
  /*��λ*/
  ShootSmallShut();
  ShootBigShut();
}

void prepareMotionParaInit(void)
{
	/*׼��������*/
  PrepareCompete.courseAngle=90.0f;
  PrepareCompete.pitchAngle=-20.0f;
  PrepareCompete.steerAngle=-90.f;
  PrepareCompete.steerSpeed=2000;
  PrepareCompete.gasAim=0.555f;
	
  /*׼��ȥ�õ�һ���������*/ 
  PrepareGetBall1.courseAngle=63.5f;
  PrepareGetBall1.pitchAngle=3.7f;
  PrepareGetBall1.steerAngle=-65.4f;
  PrepareGetBall1.steerSpeed=2000;
  PrepareGetBall1.gasAim=0.555f;
  
  /*׼�����һ���������*/
  PrepareShootBall1.courseAngle=172.9f;
  PrepareShootBall1.pitchAngle=7.1f;
  PrepareShootBall1.steerAngle=0.f;
  PrepareShootBall1.steerSpeed=2000;
  PrepareShootBall1.gasAim=0.555f;
	
  /*׼��ȥ�õڶ����������*/
  PrepareGetBall2.courseAngle=90.f;
  PrepareGetBall2.pitchAngle=3.1f;
  PrepareGetBall2.steerAngle=90.f;
  PrepareGetBall2.steerSpeed=2000;
  PrepareGetBall2.gasAim=0.555f;
  
  /*׼����ڶ����������*/
  PrepareShootBall2.courseAngle=174.9f;
  PrepareShootBall2.pitchAngle=3.6f;
  PrepareShootBall2.steerAngle=0.f;
  PrepareShootBall2.steerSpeed=2000;
  PrepareShootBall2.gasAim=0.555f;
  
  /*׼��ȥ�õ������������*/
  PrepareGetBall3.courseAngle=0.0f;
  PrepareGetBall3.pitchAngle=0.0f;
  PrepareGetBall3.steerAngle=0.f;
  PrepareGetBall3.steerSpeed=2000;
  PrepareGetBall3.gasAim=0.585f;
  
  /*׼����������������*/
  PrepareShootBall3.courseAngle=181.4f;
  PrepareShootBall3.pitchAngle=-1.4f;
  PrepareShootBall3.steerAngle=0.f;
  PrepareShootBall3.steerSpeed=2000;
  PrepareShootBall3.gasAim=0.585f;
  
}
//
void PrepareGetBallMotion(motionPara_t PrepareGetBall_t)
{
	/*����Ŀ������������ں����и���,���׳��ֵ������µķ��գ�*/
	gRobot.sDta.courseAimAngle=PrepareGetBall_t.courseAngle;
	gRobot.sDta.pitchAimAngle=PrepareGetBall_t.pitchAngle;
	gRobot.sDta.gasAimValue=PrepareGetBall_t.gasAim;
	#ifdef TEST
	gRobot.sDta.holdBallAimAngle[0]=gRobot.sDta.holdBallAimAngle[1]=PrepareGetBall_t.steerAngle;
	#else
	gRobot.sDta.holdBallAimAngle=PrepareGetBall_t.steerAngle;
	#endif
	
  //������ѹ
  GasMotion(PrepareGetBall_t.gasAim);
  /*���ø����Ƕ�*/
  PitchAngleMotion(PrepareGetBall_t.pitchAngle);
  /*���ú���Ƕ�*/
  CourseAngleMotion(PrepareGetBall_t.courseAngle);
  /*�ر��·���λצ*/
  ClawShut();
  /*���ת��*/
	#ifdef TEST
  HoldBallPosCrl(PrepareGetBall_t.steerAngle, PrepareGetBall_t.steerSpeed);
	#else
  HoldBallPosCrl(PrepareGetBall_t.steerAngle, PrepareGetBall_t.steerSpeed);
	#endif
}
void PrepareGetBall(int index)
{
  switch(index)
  {
  case READY:
		PrepareWork();
    //����׼�����Ĳ���
    PrepareGetBallMotion(PrepareCompete);
    break;
  case BALL_1:
    //����׼���õ���Ĳ���
    PrepareGetBallMotion(PrepareGetBall1);
    break;
  case BALL_2:
    //����׼���õ���Ĳ���
    PrepareGetBallMotion(PrepareGetBall2);
    break;
  case BALL_3:
    //����׼���õ���Ĳ���
    PrepareGetBallMotion(PrepareGetBall3);
    break;
  default:
    USART_OUT(DEBUG_USART,"PrepareGetBall error\r\n");
    break;
  }	
}

void PrepareShootBallMotion(motionPara_t PrepareShootBall_t)
{
  CPU_INT08U  os_err;
  os_err = os_err;
	
	/*����Ŀ������������ں����и���,���׳��ֵ������µķ��գ�*/
	gRobot.sDta.courseAimAngle=PrepareShootBall_t.courseAngle;
	gRobot.sDta.pitchAimAngle=PrepareShootBall_t.pitchAngle;
	gRobot.sDta.gasAimValue=PrepareShootBall_t.gasAim;
	#ifdef TEST
	gRobot.sDta.holdBallAimAngle[0]=gRobot.sDta.holdBallAimAngle[1]=PrepareShootBall_t.steerAngle;
	#else
	gRobot.sDta.holdBallAimAngle=PrepareShootBall_t.steerAngle;
	#endif
	
  //������ѹ
  GasMotion(PrepareShootBall_t.gasAim);
  /*���ø����Ƕ�*/
  PitchAngleMotion(PrepareShootBall_t.pitchAngle);
  /*���ú���Ƕ�*/
  CourseAngleMotion(PrepareShootBall_t.courseAngle);
  /*�������һ�*/
  Delay_ms(500);
  /*��ǰ�򿪷���װ��С����*/
  ShootSmallOpen();
  /*���ת��*/
	#ifdef TEST
  HoldBallPosCrl( PrepareShootBall_t.steerAngle, PrepareShootBall_t.steerSpeed);
	#else
  HoldBallPosCrl( PrepareShootBall_t.steerAngle, PrepareShootBall_t.steerSpeed);
	#endif
}
void PrepareShootBall(int index)
{
  switch(index)
  {
  case BALL_1:
    //����׼������Ĳ���
    PrepareShootBallMotion(PrepareShootBall1);
    break;
  case BALL_2:
    //����׼������Ĳ���
    PrepareShootBallMotion(PrepareShootBall2);
    break;
  case BALL_3:
    //����׼������Ĳ���
    PrepareShootBallMotion(PrepareShootBall3);
    break;
  default:
    USART_OUT(DEBUG_USART,"PrepareShootBall error\r\n");
    break;
  }	
}


/*��ʼ������ ���½�����ת*/
void PrepareWork(void)
{
	int cnt=0;
	#ifndef TEST
		/*����Ŀ������������ں����и���,���׳��ֵ������µķ��գ�*/
	gRobot.sDta.courseAimAngle=PrepareCompete.courseAngle;
	gRobot.sDta.pitchAimAngle=PrepareCompete.pitchAngle;
	gRobot.sDta.gasAimValue=PrepareCompete.gasAim;
	gRobot.sDta.holdBallAimAngle=PrepareCompete.steerAngle;
	
  /*�ر��·���λצ*/
  ClawShut();
  //������ѹ
  GasMotion(PrepareCompete.gasAim);
  /*���ת��*/
	while(1)
	{
		Delay_ms(5);
		cnt++;
		//ȷ��һ��ת����
		if(cnt>50)
			break;
		HoldBallPosCrl(PrepareCompete.steerAngle, PrepareCompete.steerSpeed);
	}
	/*���ø����Ƕ�*/
	PitchAngleMotion(PrepareCompete.pitchAngle);
	while(1)
	{
		Delay_ms(5);
		/*��ȡ������*/
		ReadActualPos(CAN2,5);
			/*�жϸ������Ƿ�λ*/
		if(fabs(gRobot.sDta.pitchAimAngle-gRobot.pitchAngle)<0.01f)
		{
			SetMotionFlag(AT_PITCH_SUCCESS);
			break;
		}
	}
	Delay_ms(2000);
  /*���ú���Ƕ�*/
  CourseAngleMotion(PrepareCompete.courseAngle);
	/*�ȴ���ת��״̬���*/
  while(1)
	{
		Delay_ms(5);
		/*��ȡ������*/
		ReadActualPos(CAN2,6);
			/*�жϸ������Ƿ�λ*/
		if(fabs(gRobot.sDta.courseAimAngle-gRobot.courseAngle)<0.01f)
		{
			SetMotionFlag(AT_COURSE_SUCCESS);
			break;
		}
	}
	
	
	#endif
}
