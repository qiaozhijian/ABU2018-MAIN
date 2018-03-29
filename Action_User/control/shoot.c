#include "shoot.h"
#include "task.h"
#include "timer.h"
#include "steer.h"
#include "motion.h"
#include "includes.h"
#include "process.h"
#include "robot.h"
#include "dma.h"

extern OS_EVENT *PeriodSem;


extern Robot_t gRobot;

/*���������Ͷ��Ĳ���*/
motionPara_t PrepareCompete;
motionPara_t PrepareGetBall1;
motionPara_t PrepareGetBall2;
motionPara_t PrepareGetBall3;
motionPara_t PrepareGetBall4;
motionPara_t PrepareShootBall1;
motionPara_t PrepareShootBall2;
motionPara_t PrepareShootBall3;


void ShootBall(void)
{	
	/*��Ϊ���ĸ������Ѿ��õ�ʱ�Ѿ�����Ͷ������㣬�����ڵ��ڹ�����һ����������������Ȼ��ͷ����ˣ�
	����Ȼ���˲ʱ�ٶȣ�����Ͷ��*/
	Delay_ms(150);
  /*����צ�ֱ�������̧*/
  LowerClawStairOff();
	ShootLedOn();
  /*���Ӵ�*/
  ClawOpen();
  Delay_ms(500);
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
  PrepareCompete.pitchAngle=19.0f;
  PrepareCompete.upSteerAngle=-90.f;
	PrepareCompete.downSteerAngle=-90.f;
  PrepareCompete.steerSpeed=2000;
  PrepareCompete.gasAim=0.470f;
	
  /*׼��ȥ�õ�һ���������*/ 
  PrepareGetBall1.courseAngle=43.0f;
  PrepareGetBall1.pitchAngle=-4.5f;
  PrepareGetBall1.upSteerAngle=-43.0f;
	PrepareGetBall1.downSteerAngle=-44.0f;
  PrepareGetBall1.steerSpeed=2000;
  PrepareGetBall1.gasAim=0.470f;
  
  /*׼�����һ���������*/
  PrepareShootBall1.courseAngle=173.f;
  PrepareShootBall1.pitchAngle=5.8f;
  PrepareShootBall1.upSteerAngle=0.f;
	PrepareShootBall1.downSteerAngle=0.f;
  PrepareShootBall1.steerSpeed=2000;
  PrepareShootBall1.gasAim=0.470f;
	
  /*׼��ȥ�õڶ����������*/
  PrepareGetBall2.courseAngle=90.f;
  PrepareGetBall2.pitchAngle=-4.5f;
  PrepareGetBall2.upSteerAngle=90.f; 
	PrepareGetBall2.downSteerAngle=90.f;
  PrepareGetBall2.steerSpeed=2000;
  PrepareGetBall2.gasAim=0.470;
  
  /*׼����ڶ����������*/
  PrepareShootBall2.courseAngle=174.5f;
  PrepareShootBall2.pitchAngle=2.3f;
  PrepareShootBall2.upSteerAngle=0.0f;
	PrepareShootBall2.downSteerAngle=0.0f;
  PrepareShootBall2.steerSpeed=2000;
  PrepareShootBall2.gasAim=0.470f;
  
  /*׼��ȥ�õ������������*/
  PrepareGetBall3.courseAngle=88.5f;
  PrepareGetBall3.pitchAngle=3.0f;
  PrepareGetBall3.upSteerAngle=-87.f;
	PrepareGetBall3.downSteerAngle=-90.f;
  PrepareGetBall3.steerSpeed=2000;
  PrepareGetBall3.gasAim=0.470f;
  
  /*׼����������������*/
  PrepareShootBall3.courseAngle=179.9f;
  PrepareShootBall3.pitchAngle=-1.f;
	PrepareShootBall3.upSteerAngle=0.0f;
  PrepareShootBall3.downSteerAngle=0.0f;
  PrepareShootBall3.steerSpeed=2000;
  PrepareShootBall3.gasAim=0.470f;
	
	/*׼�����ĸ���Ĳ���*/
	PrepareGetBall4.courseAngle=80.f;
	PrepareGetBall4.pitchAngle = 2.f;
	PrepareGetBall4.upSteerAngle = 60.f;
	PrepareGetBall4.downSteerAngle = 60.f;
	PrepareGetBall4.steerSpeed = 2000;
	PrepareGetBall4.gasAim = 0.470f;
  
}
//
void PrepareGetBallMotion(motionPara_t PrepareGetBall_t)
{
	/*����Ŀ������������ں����и���,���׳��ֵ������µķ��գ�*/
	gRobot.sDta.courseAimAngle=PrepareGetBall_t.courseAngle;
	gRobot.sDta.pitchAimAngle=PrepareGetBall_t.pitchAngle;
	gRobot.sDta.gasAimValue=PrepareGetBall_t.gasAim;
//	gRobot.sDta.holdBallAimAngle[0]=gRobot.sDta.holdBallAimAngle[1]=PrepareGetBall_t.steerAngle;
	gRobot.sDta.holdBallAimAngle[0]=PrepareGetBall_t.upSteerAngle;
	gRobot.sDta.holdBallAimAngle[1]=PrepareGetBall_t.downSteerAngle;

  //������ѹ
  GasMotion(PrepareGetBall_t.gasAim);
  /*���ø����Ƕ�*/
  PitchAngleMotion(PrepareGetBall_t.pitchAngle);
  /*���ú���Ƕ�*/
  CourseAngleMotion(PrepareGetBall_t.courseAngle);
  /*�ر��·���λצ*/
  ClawShut();
  /*���ת��*/
  HoldBallPosCrlSeparate(PrepareGetBall_t.upSteerAngle, PrepareGetBall_t.downSteerAngle);
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
		
		case BALL_4:
			//����׼���õ���Ĳ���
			PrepareGetBallMotion(PrepareGetBall4);
			break;
		
		default:
			USART_OUT(DEBUG_USART,"PrepareGetBall error\r\n");
			break;
  }	
}

void PrepareShootBallMotion(motionPara_t PrepareShootBall_t)
{
	/*����Ŀ������������ں����и���,���׳��ֵ������µķ��գ�*/
	gRobot.sDta.courseAimAngle=PrepareShootBall_t.courseAngle;
	gRobot.sDta.pitchAimAngle=PrepareShootBall_t.pitchAngle;
	gRobot.sDta.gasAimValue=PrepareShootBall_t.gasAim;
	
	gRobot.sDta.holdBallAimAngle[0]=PrepareShootBall_t.upSteerAngle;
	gRobot.sDta.holdBallAimAngle[1]=PrepareShootBall_t.downSteerAngle;
	
  //������ѹ
  GasMotion(PrepareShootBall_t.gasAim);
  /*���ø����Ƕ�*/
  PitchAngleMotion(PrepareShootBall_t.pitchAngle);
  /*���ú���Ƕ�*/
  CourseAngleMotion(PrepareShootBall_t.courseAngle);
//  /*�������һ�*/
//  Delay_ms(500);
  /*��ǰ�򿪷���װ��С����*/
  ShootSmallOpen();
  /*���ת��*/
  HoldBallPosCrlSeparate( PrepareShootBall_t.upSteerAngle, PrepareShootBall_t.downSteerAngle);
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
	case BALL_4:
    //����׼������Ĳ���
    PrepareShootBallMotion(PrepareShootBall3);
    break;
  default:
    USART_OUT(DEBUG_USART,"PrepareShootBall error\r\n");
    break;
  }	
}

/*����΢��*/
void SmallChange(void){
	//�����Ƿ���м����˵ı���
	int whetherCount=0;
	switch(gRobot.sDta.robocon2018){
		case COLORFUL_BALL_1:
			if(gRobot.sDta.process!=TO_THROW_BALL_1){
				return;
			}
			
			if((fabs(gRobot.posX-4450.f)>5.f || fabs(gRobot.posY-2140.f)>5.f || fabs(gRobot.angle)>1.f )&& gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS ){
				 gRobot.sDta.courseAimAngle = RAD_TO_ANGLE(asinf(445.f / sqrtf((525.f - gRobot.posX)*(525.f - gRobot.posX) + (3235.f - gRobot.posY)*(3235.f - gRobot.posY))))  \
							- RAD_TO_ANGLE(atan2((COLOR_BALL_FRAME_POSX - gRobot.posX) , (COLOR_BALL_FRAME_POSY - (gRobot.posY-ROBOT_CENTER_TO_COURCE)))) + 90.f;
				 /*atan((525.f - gRobot.posX) / (3235.f - (gRobot.posY-15)))*/
				 gRobot.sDta.courseAimAngle=gRobot.sDta.courseAimAngle-gRobot.angle;
				 whetherCount=1;
			}else {
				 whetherCount=0;
			}
		break;
		
		case COLORFUL_BALL_2:
			if(gRobot.sDta.process!=TO_THROW_BALL_2){
				return;
			}
		
			if((fabs(gRobot.posX-6565.f)>5.f || fabs(gRobot.posY-2180.f)>5.f || fabs(gRobot.angle)>1.f)&& gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS ){
				 gRobot.sDta.courseAimAngle = RAD_TO_ANGLE(asinf(445.f / sqrtf((525.f - gRobot.posX)*(525.f - gRobot.posX) + (3235.f - gRobot.posY)*(3235.f - gRobot.posY))))  \
							- RAD_TO_ANGLE(atan2((COLOR_BALL_FRAME_POSX - gRobot.posX) , (COLOR_BALL_FRAME_POSY - (gRobot.posY-15)))) + 90.f;
				/*atan((525.f - gRobot.posX) / (3235.f - (gRobot.posY-15)))*/
				 gRobot.sDta.courseAimAngle=gRobot.sDta.courseAimAngle-gRobot.angle;
				 whetherCount=1;
			}else {
				 whetherCount=0;
			}
		break;
		
		case GOLD_BALL:
			if(gRobot.sDta.process!=TO_THROW_BALL_3){
				return;
			}
			
			if((fabs(gRobot.posX-6080.f)>5.f || fabs(gRobot.posY-6030.f)>5.f || fabs(gRobot.angle)>1.f)&& gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS ){
				 gRobot.sDta.courseAimAngle = RAD_TO_ANGLE(asinf(445.f / sqrtf((GOLD_BALL_FRAME_POSX - gRobot.posX)*(GOLD_BALL_FRAME_POSX - gRobot.posX) + (GOLD_BALL_FRAME_POSY - gRobot.posY)*(GOLD_BALL_FRAME_POSY - gRobot.posY))))  \
						- RAD_TO_ANGLE(atan2((GOLD_BALL_FRAME_POSX - gRobot.posX) , (GOLD_BALL_FRAME_POSY - (gRobot.posY-ROBOT_CENTER_TO_COURCE)))) + 90.f;
				/*atan((GOLD_BALL_FRAME_POSX - gRobot.posX) / (GOLD_BALL_FRAME_POSY - (gRobot.posY-ROBOT_CENTER_TO_COURCE)))*/ 
				gRobot.sDta.courseAimAngle=gRobot.sDta.courseAimAngle-gRobot.angle;
				whetherCount=1;
			}else {
				 whetherCount=0;
			}
		break;
	}	
	
	
	
	//����������жϼ���ֵ�Ƿ��������ֵ������0.2��������΢��
	if(whetherCount){
		/*��ֹ�����ֵ�����޶��Ƕ�*/
		if(gRobot.sDta.courseAimAngle>189.f){
			gRobot.sDta.courseAimAngle=189.f;
			USART_OUTByDMA("courseAngle OUT OF RANGE");
		}else if(gRobot.sDta.courseAimAngle<0.f){
			gRobot.sDta.courseAimAngle=0.f;
		  USART_OUTByDMA("courseAngle OUT OF RANGE");
		}
		
		if(fabs(gRobot.sDta.courseAimAngle-gRobot.courseAngle)>0.35f){
				SetMotionFlag(~AT_COURSE_SUCCESS);
			
				USART_OUTByDMA("courseAngle need change=");
				USART_OUTByDMAF(gRobot.sDta.courseAimAngle);
//				USART_OUTByDMA("\r\n");
		}else {
				USART_OUTByDMA("courseAngle OK\t");
				USART_OUTByDMAF(gRobot.sDta.courseAimAngle);
//				USART_OUTByDMA("\r\n");
		}
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
	gRobot.sDta.holdBallAimAngle[0]=PrepareCompete.upSteerAngle;
	gRobot.sDta.holdBallAimAngle[1]=PrepareCompete.downSteerAngle;
	
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
		HoldBallPosCrlSeparate(PrepareCompete.upSteerAngle, PrepareCompete.downSteerAngle);
	}
	/*���ø����Ƕ�*/
	PitchAngleMotion(PrepareCompete.pitchAngle);
	while(1)
	{
		Delay_ms(5);
		/*��ȡ������*/
		ReadActualPos(CAN2,5);
			/*�жϸ������Ƿ�λ*/
		if(fabs(gRobot.sDta.pitchAimAngle-gRobot.pitchAngle)<0.1f)
		{
			SetMotionFlag(AT_PITCH_SUCCESS);
			break;
		}
	}
	Delay_ms(2000);
  /*���ú���Ƕ�*/
  CourseAngleMotion(PrepareCompete.courseAngle);
	/*�ȴ���ת��״̬���*/
	cnt=0;
  while(1)
	{
		Delay_ms(5);
			/*��ȡ�����*/
		ReadActualPos(CAN2,6);
			/*�жϺ�����Ƿ�λ*/
		if(fabs(gRobot.sDta.courseAimAngle-gRobot.courseAngle)<0.1f)
		{
			SetMotionFlag(AT_COURSE_SUCCESS);
			break;
		}else if(fabs(gRobot.sDta.courseAimAngle-gRobot.courseAngle)<0.2f)
		{
			cnt++;
			if(cnt>50)
				break;
		}
	}
	
		#endif

}
