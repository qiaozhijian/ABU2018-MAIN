#include "shoot.h"
#include "task.h"
#include "timer.h"
#include "steer.h"
#include "motion.h"
#include "includes.h"
#include "process.h"
#include "robot.h"
#include "dma.h"
#include "gpio.h"
extern OS_EVENT *PeriodSem;


extern Robot_t gRobot;

/*���������Ͷ��Ĳ���*/
motionPara_t PrepareCompete;
motionPara_t PrepareGetBall1;
motionPara_t PrepareGetBall2;
motionPara_t PrepareGetBall3Wait;
motionPara_t PrepareGetBall3;
motionPara_t PrepareGetBall4;
motionPara_t PrepareShootBall1;
motionPara_t PrepareShootBall2;
motionPara_t PrepareShootBall3;
motionPara_t PrepareShootBall4;

void ShootBall(void)
{	
	/*��Ϊ���ĸ������Ѿ��õ�ʱ�Ѿ�����Ͷ������㣬�����ڵ��ڹ�����һ����������������Ȼ��ͷ����ˣ�
	����Ȼ���˲ʱ�ٶȣ�����Ͷ��*/
//	if(gRobot.sDta.robocon2018==COLORFUL_BALL_1){
//		Delay_ms(450);
//	}else if(gRobot.sDta.robocon2018==COLORFUL_BALL_2){
//		Delay_ms(250);
//	}else if(gRobot.sDta.robocon2018==GOLD_BALL){
//		Delay_ms(850);
//	}
	ShootLedOn();
	Delay_ms(10);
  /*���Ӵ�*/
  ClawOpen();
  Delay_ms(50);
  /*�������������״�*/
  ShootBigOpen();
}

void ShootReset(void)
{
	/*�ر��·���λצ*/
  ClawShut();
	ShootLedOff();
  /*��λ*/
  ShootSmallShut();
  ShootBigShut();
	Delay_ms(250);
}

void prepareMotionParaInit(void)
{
	/*׼��������*/
  PrepareCompete.courseAngle=90.0f;
  PrepareCompete.pitchAngle=29.0f;
  PrepareCompete.upSteerAngle=-96.f;
	PrepareCompete.downSteerAngle=-96.f;
  PrepareCompete.gasAim=0.555f;
	
  /*׼��ȥ�õ�һ���������*/ 
  PrepareGetBall1.courseAngle=47.5f;
  PrepareGetBall1.pitchAngle=-1.5f;
  PrepareGetBall1.upSteerAngle=-43.0f;
	PrepareGetBall1.downSteerAngle=-54.0f;
  PrepareGetBall1.gasAim=0.555f;
  
  /*׼�����һ���������*/
  PrepareShootBall1.courseAngle=170.f;
  PrepareShootBall1.pitchAngle=12.0f;
  PrepareShootBall1.upSteerAngle=0.f;
	PrepareShootBall1.downSteerAngle=0.f;
  PrepareShootBall1.gasAim=0.555f;
	
  /*׼��ȥ�õڶ����������*/
  PrepareGetBall2.courseAngle=91.5f;
  PrepareGetBall2.pitchAngle=0.0f;
  PrepareGetBall2.upSteerAngle=90.f; 
	PrepareGetBall2.downSteerAngle=89.f;
  PrepareGetBall2.gasAim=0.555f;
  
  /*׼����ڶ����������*/
  PrepareShootBall2.courseAngle=175.5f;
  PrepareShootBall2.pitchAngle=10.7f;
  PrepareShootBall2.upSteerAngle=0.0f;
	PrepareShootBall2.downSteerAngle=0.0f;
  PrepareShootBall2.gasAim=0.555f;
  
  /*׼���ȴ��õ������������*/
  PrepareGetBall3Wait.courseAngle=120.f;
  PrepareGetBall3Wait.pitchAngle=-2.5f;
  PrepareGetBall3Wait.upSteerAngle=-45.f;
	PrepareGetBall3Wait.downSteerAngle=-59.f;
  PrepareGetBall3Wait.gasAim=0.520f;
	/*��ȡ��������Ĳ���*/
	PrepareGetBall3.courseAngle=93.f;
  PrepareGetBall3.pitchAngle=-2.5f;
  PrepareGetBall3.upSteerAngle=-45.f;
	PrepareGetBall3.downSteerAngle=-59.f;
  PrepareGetBall3.gasAim=0.520f;
  
  /*׼����������������*/
  PrepareShootBall3.courseAngle=180.5f;
  PrepareShootBall3.pitchAngle=1.2f;
	PrepareShootBall3.upSteerAngle=0.0f;
  PrepareShootBall3.downSteerAngle=0.0f;
  PrepareShootBall3.gasAim=0.520f;
	
	/*׼�����ĸ���Ĳ���*/
	PrepareGetBall4.courseAngle=91.5f;
	PrepareGetBall4.pitchAngle = -2.5f;
	PrepareGetBall4.upSteerAngle = -55.f;
	PrepareGetBall4.downSteerAngle = -64.f;
	PrepareGetBall4.gasAim = 0.520f;
	
	/*׼������ĸ��������*/
	PrepareShootBall4.courseAngle=180.f;
  PrepareShootBall4.pitchAngle=4.7f;
	PrepareShootBall4.upSteerAngle=0.0f;
  PrepareShootBall4.downSteerAngle=0.0f;
  PrepareShootBall4.gasAim=0.520f;
  
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
  /*���ת��*/
  HoldBallPosCrlSeparate(PrepareGetBall_t.upSteerAngle, PrepareGetBall_t.downSteerAngle);
}
void PrepareGetBall(int index)
{
  switch(index)
  {
		case READY:
			//����׼�����Ĳ���
			PrepareWork();
		break;
		
		case BALL_1:
			//����׼���õ���Ĳ���
			PrepareGetBallMotion(PrepareGetBall1);
			break;
		
		case BALL_2:
			//����׼���õ���Ĳ���
			PrepareGetBallMotion(PrepareGetBall2);
			break;
		
		case BALL_3_WAIT:
			//����׼���õ���Ĳ���
			PrepareGetBallMotion(PrepareGetBall3Wait);
		break;
		
		case BALL_3:
			//�����ȡ����Ĳ���
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
    PrepareShootBallMotion(PrepareShootBall4);
    break;
  default:
    USART_OUT(DEBUG_USART,"PrepareShootBall error\r\n");
    break;
  }	
}

/*����΢��*/
void SmallChange(void){
	float countAngle=0.f;
	//�����Ƿ���м����˵ı���
	int whetherCount=0;
	switch(gRobot.sDta.robocon2018){
		case COLORFUL_BALL_1:
			if(gRobot.sDta.process!=TO_THROW_BALL_1){
				return;
			}
			
			if((fabs(gRobot.posX-TZ_1_X)>25.f || fabs(gRobot.posY-TZ_1_Y)>25.f || fabs(gRobot.angle)>1.f )&& gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS ){
				 
				countAngle = RAD_TO_ANGLE(asinf(445.f / sqrtf((525.f - gRobot.posX)*(525.f - gRobot.posX) + (3235.f - gRobot.posY)*(3235.f - gRobot.posY))))  \
							- RAD_TO_ANGLE(atan2((COLOR_BALL_FRAME_POSX - gRobot.posX) , (COLOR_BALL_FRAME_POSY - (gRobot.posY-ROBOT_CENTER_TO_COURCE)))) + 90.f;
				 /*atan((525.f - gRobot.posX) / (3235.f - (gRobot.posY-15)))*/
				 
				 whetherCount=1;
			}else {
				 whetherCount=0;
			}
		break;
		
		case COLORFUL_BALL_2:
			if(gRobot.sDta.process!=TO_THROW_BALL_2){
				return;
			}
		
			if((fabs(gRobot.posX-TZ_2_X)>25.f || fabs(gRobot.posY-TZ_2_Y)>25.f || fabs(gRobot.angle)>1.f)&& gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS ){
				 
				 countAngle = RAD_TO_ANGLE(asinf(445.f / sqrtf((525.f - gRobot.posX)*(525.f - gRobot.posX) + (3235.f - gRobot.posY)*(3235.f - gRobot.posY))))  \
							- RAD_TO_ANGLE(atan2((COLOR_BALL_FRAME_POSX - gRobot.posX) , (COLOR_BALL_FRAME_POSY - (gRobot.posY-15)))) + 90.f;
				/*atan((525.f - gRobot.posX) / (3235.f - (gRobot.posY-15)))*/
				 whetherCount=1;
			}else {
				 whetherCount=0;
			}
		break;
		
		case GOLD_BALL:
			if(gRobot.sDta.process!=TO_THROW_BALL_3){
				return;
			}
			
			if((fabs(gRobot.posX-TZ_3_X)>25.f || fabs(gRobot.posY-TZ_3_Y)>25.f || fabs(gRobot.angle)>1.f)&& gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS ){
				 countAngle = RAD_TO_ANGLE(asinf(445.f / sqrtf((GOLD_BALL_FRAME_POSX - gRobot.posX)*(GOLD_BALL_FRAME_POSX - gRobot.posX) + (GOLD_BALL_FRAME_POSY - gRobot.posY)*(GOLD_BALL_FRAME_POSY - gRobot.posY))))  \
						- RAD_TO_ANGLE(atan2((GOLD_BALL_FRAME_POSX - gRobot.posX) , (GOLD_BALL_FRAME_POSY - (gRobot.posY-ROBOT_CENTER_TO_COURCE)))) + 90.f;
				/*atan((GOLD_BALL_FRAME_POSX - gRobot.posX) / (GOLD_BALL_FRAME_POSY - (gRobot.posY-ROBOT_CENTER_TO_COURCE)))*/ 
				
				whetherCount=1;
			}else {
				 whetherCount=0;
			}
		break;
	}	
	
	
	
	//����������жϼ���ֵ�Ƿ��������ֵ������0.2��������΢��
	if(whetherCount){
	
		gRobot.sDta.courseAimAngle=countAngle-gRobot.angle;
		/*��ֹ�����ֵ�����޶��Ƕ�*/
		if(gRobot.sDta.courseAimAngle>189.f){
			gRobot.sDta.courseAimAngle=189.f;
			USART_OUTByDMA("courseAngle OUT OF RANGE");
		}else if(gRobot.sDta.courseAimAngle<0.f){
			gRobot.sDta.courseAimAngle=0.f;
		  USART_OUTByDMA("courseAngle OUT OF RANGE");
		}
		
		if(fabs(gRobot.sDta.courseAimAngle-gRobot.courseAngle)>0.3f){
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
	static int prepareWorkStep=1;
	int cnt=0;
		/*����Ŀ������������ں����и���,���׳��ֵ������µķ��գ�*/
	gRobot.sDta.courseAimAngle=PrepareCompete.courseAngle;
	gRobot.sDta.pitchAimAngle=PrepareCompete.pitchAngle;
	gRobot.sDta.gasAimValue=PrepareCompete.gasAim;
	gRobot.sDta.holdBallAimAngle[0]=PrepareCompete.upSteerAngle;
	gRobot.sDta.holdBallAimAngle[1]=PrepareCompete.downSteerAngle;
	
	 /*�ر��·���λצ*/
  ClawShut();
	
	Delay_ms(200);

		
	while(1){
		Delay_ms(5);
		switch(prepareWorkStep){
			case 1:
				/*���ת��*/
	      HoldBallPosCrlSeparate(PrepareCompete.upSteerAngle, PrepareCompete.downSteerAngle);
				cnt++;
				/*��ȡ���µ���Ƕ�*/
				ReadActualPos(CAN2,7);
				ReadActualPos(CAN2,8);
				/*�жϸ������Ƿ�λ*/
				if(fabs(gRobot.sDta.holdBallAimAngle[0]-gRobot.holdBallAngle[0])<5.f&&fabs(gRobot.sDta.holdBallAimAngle[1]-gRobot.holdBallAngle[1])<5.f)
				{
					SetMotionFlag(AT_PITCH_SUCCESS);
					prepareWorkStep=2;
				}
				/*3sצ�Ӳ���λ*/
				if(cnt>600){
						cnt=0;
						BEEP_ON;
						USART_OUTByDMA("Steer Not Ok You need reset");
						/*���������ѹ�����*/
						while(1){
							Delay_ms(5);
							cnt++;
							/*�Ƚ�����������̫����*/
							if(cnt>=600){
								BEEP_OFF;
							}
							cnt%=600;
							USART_OUTByDMAF(gRobot.gasValue);
							if(gRobot.gasValue>0.6f){
									ShootLedOn();
									if(cnt<300){
										BEEP_ON;
									}
									else{
										BEEP_OFF;
									}
							}
							else
							{
								 ShootLedOff();
							}
						}
					}
			break;
			
			case 2:
				/*֮ǰ�������ֻ�ǿ�С�磬��ʱ��������ģʽ�ٸ������巢ָ��*/
				GasMotion(PrepareCompete.gasAim);
	      GasEnable();
				prepareWorkStep=3;
			  cnt=0;
			break;
			
			case 3:
				/*���ø����Ƕ�*/
	      PitchAngleMotion(PrepareCompete.pitchAngle);
				/*��ȡ������*/
		    ReadActualPos(CAN2,5);
				cnt++;
        /*�жϸ������Ƿ�λ*/
				if(fabs(gRobot.sDta.pitchAimAngle-gRobot.pitchAngle)<0.3f)
				{
					SetMotionFlag(AT_PITCH_SUCCESS);
					prepareWorkStep=4;
					cnt =0;
				}
				if(cnt>450){
					BEEP_ON;
					USART_OUTByDMA("Pitch Not Ok You need reset\t");
					prepareWorkStep=4;
					cnt =0;
				}
			break;
				
			case 4:
				/*���ú���Ƕ�*/
        CourseAngleMotion(PrepareCompete.courseAngle);
				/*��ȡ�����*/
				ReadActualPos(CAN2,6);
				cnt++;
        /*�жϺ�����Ƿ�λ*/
				if(fabs(gRobot.sDta.courseAimAngle-gRobot.courseAngle)<0.1f)
				{
					SetMotionFlag(AT_COURSE_SUCCESS);
					return;
				}
		
		    if(cnt>450){
			    BEEP_ON;
			    USART_OUTByDMA("Course Not Ok You need reset\t");
			    return;
		    }
		  break;
		}
	}

}
