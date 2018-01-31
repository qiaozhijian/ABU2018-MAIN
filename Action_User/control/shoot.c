#include "shoot.h"
#include "task.h"
#include "timer.h"
#include "steer.h"


extern Robot_t gRobot;

/*���������Ͷ��Ĳ���*/
motionPara_t PrepareCompete;
motionPara_t PrepareGetBall1;
motionPara_t PrepareGetBall2;
motionPara_t PrepareGetBall3;
motionPara_t PrepareShootBall1;
motionPara_t PrepareShootBall2;
motionPara_t PrepareShootBall3;

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

void ShootBall(void)
{				
  /*���Ӵ�*/
  ClawOpen();
  /*�����ʵ���ʱ��֤���Ӻ��򲻸���*/
  Delay_ms(1000);
  /*�������������״�*/
  ShootBigOpen();
}

void ShootReset(void)
{
  /*��λ*/
  ShootSmallShut();
  ShootBigShut();
}

void prepareMotionParaInit(void)
{
	/*׼��������*/
  PrepareCompete.courseAngle=0.0f;
  PrepareCompete.pitchAngle=-20.0f;
  PrepareCompete.steerAngle=-85.f;
  PrepareCompete.steerSpeed=2000;
  PrepareCompete.gasAim=0.55f;
  /*׼��ȥ�õ�һ���������*/
  PrepareGetBall1.courseAngle=0.0f;
  PrepareGetBall1.pitchAngle=0.0f;
  PrepareGetBall1.steerAngle=0.f;
  PrepareGetBall1.steerSpeed=2000;
  PrepareGetBall1.gasAim=0.55f;
  
  /*׼��ȥ�õڶ����������*/
  PrepareGetBall2.courseAngle=90.f;
  PrepareGetBall2.pitchAngle=3.1f;
  PrepareGetBall2.steerAngle=85.f;
  PrepareGetBall2.steerSpeed=2000;
  PrepareGetBall2.gasAim=0.55f;
  
  /*׼��ȥ�õ������������*/
  PrepareGetBall3.courseAngle=0.0f;
  PrepareGetBall3.pitchAngle=0.0f;
  PrepareGetBall3.steerAngle=0.f;
  PrepareGetBall3.steerSpeed=2000;
  PrepareGetBall3.gasAim=0.6f;
  
  /*׼�����һ���������*/
  PrepareShootBall1.courseAngle=180.0f;
  PrepareShootBall1.pitchAngle=0.0f;
  PrepareShootBall1.steerAngle=0.f;
  PrepareShootBall1.steerSpeed=2000;
  PrepareShootBall1.gasAim=0.55f;
  
  /*׼����ڶ����������*/
  PrepareShootBall2.courseAngle=180.0f;
  PrepareShootBall2.pitchAngle=0.0f;
  PrepareShootBall2.steerAngle=0.f;
  PrepareShootBall2.steerSpeed=2000;
  PrepareShootBall2.gasAim=0.55f;
  
  /*׼����������������*/
  PrepareShootBall3.courseAngle=180.0f;
  PrepareShootBall3.pitchAngle=-4.8f;
  PrepareShootBall3.steerAngle=0.f;
  PrepareShootBall3.steerSpeed=2000;
  PrepareShootBall3.gasAim=0.56f;
  
}
void PrepareGetBallMotion(motionPara_t PrepareGetBall_t)
{
	
	/*����Ŀ������������ں����и���,���׳��ֵ������µķ��գ�*/
	gRobot.courseAimAngle=PrepareGetBall_t.courseAngle;
	gRobot.pitchAimAngle=PrepareGetBall_t.pitchAngle;
	gRobot.gasAimValue=PrepareGetBall_t.gasAim;
	#ifdef TEST
	gRobot.holdBallAimAngle[0]=gRobot.holdBallAimAngle[1]=PrepareGetBall_t.steerAngle;
	#else
	gRobot.holdBallAimAngle=PrepareGetBall_t.steerAngle;
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
	
	/*����Ŀ������������ں����и���,���׳��ֵ������µķ��գ�*/
	gRobot.courseAimAngle=PrepareShootBall_t.courseAngle;
	gRobot.pitchAimAngle=PrepareShootBall_t.pitchAngle;
	gRobot.gasAimValue=PrepareShootBall_t.gasAim;
	#ifdef TEST
	gRobot.holdBallAimAngle[0]=gRobot.holdBallAimAngle[1]=PrepareShootBall_t.steerAngle;
	#else
	gRobot.holdBallAimAngle=PrepareShootBall_t.steerAngle;
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
