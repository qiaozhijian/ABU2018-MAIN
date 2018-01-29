#include "shoot.h"
#include "task.h"
#include "timer.h"
#include "steer.h"


extern Robot_t gRobot;

/*���������Ͷ��Ĳ���*/
motionPara_t PrepareGetBall1;
motionPara_t PrepareGetBall2;
motionPara_t PrepareGetBall3;
motionPara_t PrepareShootBall1;
motionPara_t PrepareShootBall2;
motionPara_t PrepareShootBall3;

void PitchAngleMotion(float angle)
{
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
  else if(angle>180.f)
    angle=180.f;
	
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
  Delay_ms(300);
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
  /*׼��ȥ�õ�һ���������*/
  PrepareGetBall1.courseAngle=0.0f;
  PrepareGetBall1.pitchAngle=0.0f;
  PrepareGetBall1.steerAngle=0.f;
  PrepareGetBall1.steerSpeed=1000;
  PrepareGetBall1.gasAim=0.45f;
  
  /*׼��ȥ�õڶ����������*/
  PrepareGetBall2.courseAngle=90.f;
  PrepareGetBall2.pitchAngle=0.0f;
  PrepareGetBall2.steerAngle=90.f;
  PrepareGetBall2.steerSpeed=1000;
  PrepareGetBall2.gasAim=0.45f;
  
  /*׼��ȥ�õ������������*/
  PrepareGetBall3.courseAngle=0.0f;
  PrepareGetBall3.pitchAngle=0.0f;
  PrepareGetBall3.steerAngle=0.f;
  PrepareGetBall3.steerSpeed=1000;
  PrepareGetBall3.gasAim=0.45f;
  
  /*׼�����һ���������*/
  PrepareShootBall1.courseAngle=180.0f;
  PrepareShootBall1.pitchAngle=0.0f;
  PrepareShootBall1.steerAngle=0.f;
  PrepareShootBall1.steerSpeed=2000;
  PrepareShootBall1.gasAim=0.45f;
  
  /*׼����ڶ����������*/
  PrepareShootBall2.courseAngle=180.0f;
  PrepareShootBall2.pitchAngle=0.0f;
  PrepareShootBall2.steerAngle=0.f;
  PrepareShootBall2.steerSpeed=2000;
  PrepareShootBall2.gasAim=0.45f;
  
  /*׼����������������*/
  PrepareShootBall3.courseAngle=180.0f;
  PrepareShootBall3.pitchAngle=0.0f;
  PrepareShootBall3.steerAngle=0.f;
  PrepareShootBall3.steerSpeed=2000;
  PrepareShootBall3.gasAim=0.45f;
  
}

void PrepareGetBallMotion(motionPara_t PrepareGetBall_t)
{
  //������ѹ
  GasMotion(PrepareGetBall_t.gasAim);
  /*���ø����Ƕ�*/
  PitchAngleMotion(PrepareGetBall_t.pitchAngle);
  /*���ú���Ƕ�*/
  CourseAngleMotion(PrepareGetBall_t.courseAngle);
  /*�ر��·���λצ*/
  ClawShut();
  /*���ת��*/
  HoldBallPosCrl(PrepareGetBall_t.steerAngle, PrepareGetBall_t.steerSpeed);
}

void PrepareGetBall(int index)
{
  switch(index)
  {
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
  HoldBallPosCrl( PrepareShootBall_t.steerAngle, PrepareShootBall_t.steerSpeed);
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
