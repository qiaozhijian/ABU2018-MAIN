#include "task.h"
#include "timer.h"
#include "gpio.h"
#include "process.h"
#include "stm32f4xx_it.h"
#include "steer.h"
#include "customer.h"
#include "motion.h"
#include "includes.h"
#include "robot.h"
#include "gpio.h"
#include "dma.h"

extern Robot_t gRobot;
extern int flagggg;

void SelfTest(void)
{
	AT_CMD_Handle();
	USART_BT_SendGas(gRobot.gasValue);
	MotionRead();
	static int step=100;
	static int count=0;
	switch(step)
	{
		case 0:
			ShootSmallOpen();
			//���λ�û�
			PosLoopCfg(CAN2, 5, 100000, 100000,100000);
			//���λ�û�
			PosLoopCfg(CAN2, 6, 100000, 100000,100000);
			step++;
			break;
		case 1:
			CourseAngleMotion(0.f);
			PitchAngleMotion(0.f);
			step++;
			break;
		case 2:
			if(PrepareForTheBall())
			{
				step++;
				Delay_ms(1000);
				CourseAngleMotion(180.f);
				Delay_ms(1000);
			}
			break;
		case 3:
			//��С����
			ReadActualPos(CAN2,6);
			if(gRobot.courseAngle>160.f)
			{
				//��С����
				step++;
			}
			break;
		case 4:
			if(gRobot.courseAngle>179.f)
			{
				ShootBall();
				Delay_ms(1000);
				ShootReset();
				step=0;
			}
			break;
			
		case 10:
			if(PrepareForTheBall())
			{
				count++;
			}else
				count=0;
			if(count>200)
			{
				Delay_ms(2000);
				count=0;
			}
		break;
	}
}
extern motionPara_t PrepareShootBall1;
/*���Ͷ�����һ������*/
void FightForBall1(void)
{
  switch(gRobot.sDta.process)
  {
    /*ȥȡ��һ����*/
		case TO_GET_BALL_1:
			//����Ƿ�ɨ��
				switch(gRobot.getBallStep.colorBall1){
				//��һ���Թ�����ɨ��
						case 0:
								if(PrepareForTheBall()){
									MotionCardCMDSend(NOTIFY_MOTIONCARD_GOT_BALL1);
									gRobot.raceTime.colorBall1WaitTime=gRobot.raceTime.roboconTime;
									gRobot.getBallStep.colorBall1++;
								}
						break;
					
						case 1:
							if(PE_FOR_THE_BALL){
								GasMotion(PrepareShootBall1.gasAim);
								/*���ú���Ƕ�*/
								CourseAngleMotion(PrepareShootBall1.courseAngle);
								/*��ǰ�򿪷���װ��С����*/
								ShootSmallOpen();
								/*���ת��*/
								HoldBallPosCrlSeparate( PrepareShootBall1.upSteerAngle, PrepareShootBall1.downSteerAngle);
								/*�������һ��Ҫ�ȵ��ȸ������ָ��ת����У���Ϊ����������ʱ*/
//								LedBallInto();
								gRobot.getBallStep.colorBall1++;
							}
						break;

						case 2:
							  Delay_ms(25);
								USART_OUTByDMA("IntoTheArea\t");
								//TalkToCamera(CAMERA_OPEN_NEAR);
								PrepareShootBall(BALL_1);
								gRobot.sDta.process=TO_THE_AREA_1;
						break;
			}
			break;
			
    /*��һ����ȡ����ϣ�ȥͶ����һ*/
		case TO_THE_AREA_1:
			/*��ǰ����*/
			if(gRobot.posY>2000.f){
				ShootLedOn();
		  }
			if(gRobot.sDta.AT_motionFlag&AT_REACH_FIRST_PLACE){
				gRobot.sDta.process=TO_THROW_BALL_1;
				//��λ֮����һ���ƿ������������޷�����΢�����Ȱ���λ��Ϊ0���¸����ڽ���SmallChange()���
				SetMotionFlag(~AT_COURSE_SUCCESS);
			}
			StartCount();
			//��CAN�жϵ��ж�ȡ���ƿ����������ݣ�����ָ��λ����gRobot.sDta.process��ΪΪTO_THROW_BALL_1
			break;
			
    /*����Ͷ����һ������*/
		case TO_THROW_BALL_1:
			/*��絽λ*/
			if(/*��������λ*/
			   		(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_1_SUCCESS)
						/*��������λ*/
						&&(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_2_SUCCESS)
							/*������λ��*/
							&&(gRobot.sDta.AT_motionFlag&AT_PITCH_SUCCESS)
								/*����λ*/
								&&(gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS)/*&&(gRobot.posY>2000.f*/)
									  /*��ѹ��λ*/
//										&&(gRobot.sDta.AT_motionFlag&AT_GAS_SUCCESS))
			{
				USART_OUTByDMA("\r\nball1_wait_time\t");
				USART_OUTByDMAF(returnEndUs()/1000.f);
				USART_OUTByDMA("\r\n");

				/*����*/
				ShootBall();
				
				/*����ʱʹ�������ִ�е�λ*/
				Delay_ms(125);
				
				/*֪ͨ���ƿ�*/
				MotionCardCMDSend(NOTIFY_MOTIONCARD_SHOT_BALL1);
				
				/*����ӽӵ��������ʱ��*/
				gRobot.raceTime.colorBall1ThrowTime=gRobot.raceTime.roboconTime - gRobot.raceTime.colorBall1WaitTime;
				gRobot.raceTime.colorBall1Time = gRobot.raceTime.colorBall1WaitTime + gRobot.raceTime.colorBall1ThrowTime ;
				
				//��ǰ������ѹ
				GasMotion(GetPrepareShootGoldBallGasAim(BALL_2));
				/*���������λ*/
				ShootReset();
				
				/*׼�������*/
				PrepareGetBall(BALL_2);
				/*������һ״̬*/
				
				gRobot.sDta.process=TO_GET_BALL_2;
				gRobot.sDta.robocon2018=COLORFUL_BALL_2;
				SetMotionFlag(AT_IS_SEND_DEBUG_DATA);
				SetMotionFlag(~AT_REACH_FIRST_PLACE);
			}
			else
			{
//				SetMotionFlag(~AT_IS_SEND_DEBUG_DATA);		
				if(!PE_FOR_THE_BALL)
					USART_OUTByDMA("!PE1 ");
				if(!(gRobot.sDta.AT_motionFlag&AT_PITCH_SUCCESS))
				{
					USART_OUTByDMA("!PITCH1 ");
					USART_OUTByDMAF(gRobot.pitchAngle);
				}
				if(!(gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS))
				{
					USART_OUTByDMA("!COURSE1 ");
					USART_OUTByDMAF(gRobot.courseAngle);
				}
				if(!(gRobot.sDta.AT_motionFlag&AT_GAS_SUCCESS))
				{
					USART_OUTByDMA("!GAS1 ");
					USART_OUTByDMAF(gRobot.gasValue);
				}
			}
			break;
		}
}

/*���Ͷ������������*/
void FightForBall2(void)
{
  switch(gRobot.sDta.process)
  {
    /*ȥȡ�ڶ�����*/
		case TO_GET_BALL_2:
			//����Ƿ�ɨ��
			switch(gRobot.getBallStep.colorBall2){
					//��һ���Թ�����ɨ��
				case 0:
					if(PrepareForTheBall()){
						ExtendCarOn();
						MotionCardCMDSend(NOTIFY_MOTIONCARD_GOT_BALL2);
						gRobot.raceTime.colorBall2WaitTime = gRobot.raceTime.roboconTime  - gRobot.raceTime.colorBall1Time;
						gRobot.getBallStep.colorBall2++;
					}
				break;
						
				case 1:
						Delay_ms(100);
						/*�Ͳ���1���һ��*/
						PrepareShootBall(BALL_2);
						gRobot.getBallStep.colorBall2++;
				break;

				case 2:
					if(gRobot.posX>5500.f)
					{	
						USART_OUTByDMA("IntoTheArea\t");
						gRobot.sDta.process=TO_THE_AREA_2;
					}
					break;
				}
		break;
				
			/*�ڶ�����ȡ����ϣ�ȥͶ������*/
		case TO_THE_AREA_2:
			/*��ǰ����*/
			if(gRobot.posY>2000.f){
				ShootLedOn();
		  }
			if(gRobot.sDta.AT_motionFlag&AT_REACH_SECOND_PLACE){
				gRobot.sDta.process=TO_THROW_BALL_2;
				SetMotionFlag(~AT_COURSE_SUCCESS);
			}
			StartCount();
//			if(!PrepareForTheBall())
//			{
//				MotionCardCMDSend(NOTIFY_MOTIONCARD_LOSE_BALL2);
//			}
			break;
			
			/*����Ͷ������������*/
		case TO_THROW_BALL_2:
			if(/*��������λ*/
						(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_1_SUCCESS)
							/*��������λ*/
							&&(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_2_SUCCESS)
								/*������λ��*/
								&&(gRobot.sDta.AT_motionFlag&AT_PITCH_SUCCESS)
									/*����λ*/
									&&(gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS)/*&&(gRobot.posY>2000.f)*/)
											/*��ѹ��λ*/
//											&&(gRobot.sDta.AT_motionFlag&AT_GAS_SUCCESS))
			{
				USART_OUTByDMA("\r\nball2_wait_time\t");
				USART_OUTByDMAF(returnEndUs()/1000.f);
				USART_OUTByDMA("\r\n");
				
				/*����*/
				ShootBall();
				
				/*����ʱʹ�������ִ�е�λ*/
				Delay_ms(125);
				
				MotionCardCMDSend(NOTIFY_MOTIONCARD_SHOT_BALL2);
				
				/*����Ͷ��ʱ��*/
				gRobot.raceTime.colorBall2ThrowTime=gRobot.raceTime.roboconTime - gRobot.raceTime.colorBall1Time - gRobot.raceTime.colorBall2WaitTime;
				gRobot.raceTime.colorBall2Time=gRobot.raceTime.colorBall2ThrowTime + gRobot.raceTime.colorBall2WaitTime;
				
				//��ǰ������ѹ
				if(gRobot.sDta.AT_motionFlag&AT_RESET_USE_GOLD_STANDYBY){
						GasMotion(GetPrepareShootGoldBallGasAim(BALL_3_BACKUP));
				}
				else {
						GasMotion(GetPrepareShootGoldBallGasAim(BALL_3));
				}
				/*���������λ*/
				ShootReset();
				
				/*�������ٶȼ�С*/
				if(gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS){
					PosLoopCfg(CAN2, COURCE_MOTOR_ID, 8000000, 8000000,6250000);
				}
				
				/*׼��������*/
				PrepareGetBall(BALL_3_WAIT);
				
				gRobot.sDta.process=TO_GET_BALL_3;
								
				gRobot.sDta.robocon2018=GOLD_BALL;
				SetMotionFlag(AT_IS_SEND_DEBUG_DATA);
				SetMotionFlag(~AT_REACH_SECOND_PLACE);
			}
			else
			{
//				SetMotionFlag(~AT_IS_SEND_DEBUG_DATA);
//				USART_OUTByDMAF(gRobot.posX);
//				USART_OUTByDMAF(gRobot.posY);
//				USART_OUTByDMAF(gRobot.angle);
//				USART_OUTByDMAF(gRobot.robotVel.readCourseVel);
//				USART_OUTByDMAF(gRobot.robotVel.readSteerVel[0]);

				if(!PE_FOR_THE_BALL)
					USART_OUTByDMA("!PE2 ");
				
		//			if(!(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_1_SUCCESS))
		//				USART_OUTByDMA("!HB12\t");
		//			if(!(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_2_SUCCESS))
		//				USART_OUTByDMA("!HB22\t");
				if(!(gRobot.sDta.AT_motionFlag&AT_PITCH_SUCCESS))
				{
					USART_OUTByDMA("!PITCH2 ");
					USART_OUTByDMAF(gRobot.pitchAngle);
				}
				if(!(gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS))
				{
					USART_OUTByDMA("!COURSE2 ");
					USART_OUTByDMAF(gRobot.courseAngle);
				}
				if(!(gRobot.sDta.AT_motionFlag&AT_GAS_SUCCESS))
				{
					USART_OUTByDMA("!GAS2 ");
					USART_OUTByDMAF(gRobot.gasValue);
				}
				//USART_OUTByDMA("\r\n");
			}
			break;
  }
  
}
extern motionPara_t PrepareGetBall4;
static int shootTime=0;
/*���Ͷ����������*/
void FightForGoldBall(void)
{
	if((gRobot.sDta.process==TO_GET_BALL_3||gRobot.sDta.process==TO_THE_AREA_3)&&(gRobot.sDta.AT_motionFlag&AT_GAS_SUCCESS)){
		 gRobot.getBallStep.gasSatisfyCnt++;
		 if(gRobot.getBallStep.gasSatisfyCnt>10){
			 gRobot.getBallStep.gasSatisfyCnt=0;
			 if(gRobot.getBallStep.gasSatisfyFlag==0){
					GasDisable();
				  gRobot.getBallStep.gasSatisfyFlag=1;
				  USART_OUTByDMA("GasDisable");
			 }
		 }
	}else if(gRobot.getBallStep.gasSatisfyFlag==1 &&(gRobot.sDta.AT_motionFlag&AT_GAS_SUCCESS)==0){
		 gRobot.getBallStep.gasSatisfyFlag=0;
		 GasMotion(gRobot.sDta.gasAimValue);
		 GasEnable();
		 USART_OUTByDMA("GasEnable");
	}
	
  switch(gRobot.sDta.process)
  {
    /*ȥȡ��������*/
  case TO_GET_BALL_3:
		switch(gRobot.getBallStep.goldBall)
		{
			case 0:
				if((gRobot.robotVel.countVel<300.f
						&&fabs(gRobot.posX-HANDOVER_3_X)<50.f
							&&fabs(gRobot.posY-HANDOVER_3_Y)<50.f)||gRobot.sDta.AT_motionFlag&AT_GET_MOTIONCARD_GET_GOLDBALL_AREA){
								
								if(gRobot.sDta.AT_motionFlag&AT_GET_MOTIONCARD_GET_GOLDBALL_AREA){
									ShootLedOn();
									SetMotionFlag(~AT_GET_MOTIONCARD_GET_GOLDBALL_AREA);
								}
								gRobot.getBallStep.goldBall++;
								GoldBallGraspStairTwoOn();
							  gRobot.sDta.WhichGoldBall=BALL_3;
				}
			break;
							
				
		  case 1:
				//GoldRackInto()�ڲ�����ѭ����һֱ�ڼ�����ܹ�磬�������Ҫ��������
				if(GoldRackInto()){
					ShootLedOff();
					gRobot.raceTime.goldBallWaitTime=gRobot.raceTime.roboconTime - gRobot.raceTime.colorBall1Time - gRobot.raceTime.colorBall2Time;
					MotionCardCMDSend(NOTIFY_MOTIONCARD_GOT_BALL3);
					gRobot.getBallStep.goldBall++;
				}
			break;
				
			case 2:
				if(gRobot.posY>1500.f){
					//��ȡ����һ
					PrepareGetBall(BALL_3);
					gRobot.getBallStep.goldBall++;
				}
			break;
				
			case 3:
				if((gRobot.sDta.AT_motionFlag&AT_PITCH_SUCCESS)
							&&(gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS))
				{
					gRobot.getBallStep.goldBall++;
					Delay_ms(150);
				}
			break;
				
			case 4:
				if((gRobot.sDta.AT_motionFlag&AT_PITCH_SUCCESS)
							&&(gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS))
				{
					gRobot.sDta.courseAimAngle = 179.9f;
					CourseAngleMotion(gRobot.sDta.courseAimAngle);					
					gRobot.getBallStep.goldBall++;
				}
			break;
				
				
//			case 5:
//				if((gRobot.courseAngle-114.f)>0.f)
//				{
//					PrepareShootBall(BALL_3);
//					LedBallInto();
//					//���ú���ת����צ�Ӷ����Ķ���
//					isGetBall++;
//				}
//			break;
				
			
			case 5:
				if(fabs(gRobot.courseAngle - gRobot.sDta.courseAimAngle)<45.f){
					GoldBallGraspStairTwoOff();
					/*����ת����λֱ�ӿ�ʼ׼���������*/
					if(gRobot.sDta.AT_motionFlag&AT_RESET_USE_GOLD_STANDYBY)
						PrepareShootBall(BALL_3_BACKUP);
					else
						PrepareShootBall(BALL_3);
					gRobot.sDta.process=TO_THE_AREA_3;
					USART_OUTByDMA("PrepareShoot ");
					gRobot.getBallStep.goldBall=6;			
				}
			break;
				
				
				
			//��ȥ�ڶ�����
			case 11:
					gRobot.sDta.WhichGoldBall=BALL_4;
				  Delay_ms(100);
//        if((gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_1_SUCCESS)
//					/*��������λ*/&&(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_2_SUCCESS))
//				{
			    PrepareGetBall(BALL_4);
				  gRobot.getBallStep.goldBall = 12;
//				}
			break;
				
			case 12:
				if((gRobot.sDta.AT_motionFlag&AT_PITCH_SUCCESS)
								&&(gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS)){
					Delay_ms(150);
					gRobot.getBallStep.goldBall=13;
				}
			break;
				
			case 13:
				if(PrepareForTheBall()){
					gRobot.sDta.courseAimAngle = 179.9f;
					CourseAngleMotion(gRobot.sDta.courseAimAngle);
					gRobot.getBallStep.goldBall=15;
				}
			break;
				
//			case 14:
//				if((gRobot.courseAngle-114.f)>0.f)
//				{
//					PrepareShootBall(BALL_4);
//					LedBallInto();
//					//���ú���ת����צ�Ӷ����Ķ���
//					isGetBall++;
//				}
//			break;
				
			case 15:
				if(fabs(gRobot.sDta.courseAimAngle-gRobot.courseAngle)<45.f){
					if(gRobot.sDta.AT_motionFlag&AT_RESET_USE_GOLD_STANDYBY)
						PrepareShootBall(BALL_4_BACKUP);
					else
						PrepareShootBall(BALL_4);
					gRobot.sDta.process=TO_THROW_BALL_3;
					gRobot.getBallStep.goldBall=15;
				}
			break;
				
		}
    break;
		
    /*��������ȡ����ϣ�ȥͶ������*/
  case TO_THE_AREA_3:
		if(gRobot.posY>5530.f){
			ShootLedOn();
		}
		if(gRobot.sDta.AT_motionFlag&AT_REACH_THIRD_PLACE){
			gRobot.sDta.process=TO_THROW_BALL_3;
			SetMotionFlag(~AT_COURSE_SUCCESS);
		}
		//��緢�ֶ�����ʱ��Ӧ��֪ͨ���ƿ�����ͬʱ�Լ�Ӧ�ð�gRobot.sDta.process��λȡ�������
    if(!PrepareForTheBall())
    {
      MotionCardCMDSend(NOTIFY_MOTIONCARD_LOSE_BALL3);
    }
    break;
		
    /*����Ͷ������������*/
  case TO_THROW_BALL_3:
		USART_OUTByDMA("SHOOTTIME=%d",shootTime);
    if(
				/*��������λ*/
		    (gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_1_SUCCESS)
					/*��������λ*/
					&&(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_2_SUCCESS)
						/*������λ��*/
						&&(gRobot.sDta.AT_motionFlag&AT_PITCH_SUCCESS)
							/*����λ*/
							&&(gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS)
									/*��ѹ��λ*/
									&&(gRobot.sDta.AT_motionFlag&AT_GAS_SUCCESS))
    {
			//��һ����
			if(shootTime==0){
				if(PE_FOR_THE_BALL)
					Delay_ms(GOLD_BALL_DELAY);
			}//�ڶ�����
			else{
				if(PE_FOR_THE_BALL)
					Delay_ms(GOLD_BALL_DELAY);
			}
      /*����*/
//			ShootBall();
			
			GasDisable();
			ShootBigOpen();
	    ShootLedOn();
	    Delay_ms(50);
		  ClawOpen();

      shootTime++;
      /*����ʱʹ�������ִ�е�λ*/
			if(shootTime==0){
			  Delay_ms(200);
			}
			else{
				Delay_ms(300);
			}
      
			gRobot.raceTime.goldBallThrowTime=gRobot.raceTime.roboconTime - gRobot.raceTime.colorBall1Time - gRobot.raceTime.colorBall2Time - gRobot.raceTime.goldBallWaitTime;
			gRobot.raceTime.goldBallTime=gRobot.raceTime.goldBallWaitTime + gRobot.raceTime.goldBallThrowTime;
			
			//��ǰ������ѹ
			if(gRobot.sDta.AT_motionFlag&AT_RESET_USE_GOLD_STANDYBY&&shootTime==1){
				 GasMotion(GetPrepareShootGoldBallGasAim(BALL_4_BACKUP));
			}
			else {
				 GasMotion(GetPrepareShootGoldBallGasAim(BALL_4));
			}
      /*���������λ*/
      ShootReset();
      			
			if(shootTime<2){
				gRobot.getBallStep.goldBall=11;
				gRobot.sDta.process=TO_GET_BALL_3;
				gRobot.sDta.holdBallAimAngle[0]=PrepareGetBall4.upSteerAngle;
			  gRobot.sDta.holdBallAimAngle[1]=PrepareGetBall4.downSteerAngle;
			}
			else{//�����򶼴�����
				gRobot.sDta.process=END_COMPETE;
				gRobot.sDta.AT_motionFlag=0;
				shootTime=0;
				SetMotionFlag(AT_RESET_USE_GOLD_STANDYBY);
				gRobot.sDta.WhichGoldBall=0;
			}
			
			SetMotionFlag(AT_IS_SEND_DEBUG_DATA);
    }
		else
		{
//			SetMotionFlag(~AT_IS_SEND_DEBUG_DATA);
//			USART_OUTByDMAF(gRobot.posX);
//			USART_OUTByDMAF(gRobot.posY);
//			USART_OUTByDMAF(gRobot.angle);
			if(!PE_FOR_THE_BALL)
				USART_OUTByDMA("!PE3 ");
			if(!(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_1_SUCCESS))
				USART_OUTByDMA("!HB13\t");
			if(!(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_2_SUCCESS))
				USART_OUTByDMA("!HB23\t");
			if(!(gRobot.sDta.AT_motionFlag&AT_PITCH_SUCCESS))
			{
				USART_OUTByDMA("!PITCH3 ");
				USART_OUTByDMAF(gRobot.pitchAngle);
			}
			if(!(gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS))
			{
				USART_OUTByDMA("!COURSE3 ");
				USART_OUTByDMAF(gRobot.courseAngle);
			}
			if(!(gRobot.sDta.AT_motionFlag&AT_GAS_SUCCESS))
			{
				USART_OUTByDMA("!GAS3 ");
				USART_OUTByDMAF(gRobot.gasValue);
			}
			//USART_OUTByDMA("\r\n");
		}
    break;
  }
}

void SetShootTimeZero(void)
{
	shootTime=0;
}

//motion��c
void MotionStatus(void)
{
	/*���ض��һ��״̬*/
  USART_OUTByDMA("steer 1 aimAngle ");
	USART_OUTByDMAF(gRobot.sDta.holdBallAimAngle[0]);
  USART_OUTByDMA("realpos ");
	USART_OUTByDMAF(gRobot.holdBallAngle[0]);
	USART_OUTByDMA("\r\n");
	
	/*���ض������״̬*/
  USART_OUTByDMA("steer 2 aimAngle ");
	USART_OUTByDMAF(gRobot.sDta.holdBallAimAngle[1]);
  USART_OUTByDMA("realpos ");
	USART_OUTByDMAF(gRobot.holdBallAngle[1]);
	USART_OUTByDMA("\r\n");
	
	/*���ض������״̬*/
  USART_OUTByDMA("steer 3 aimAngle ");
	USART_OUTByDMAF(gRobot.sDta.cameraAimAngle);
  USART_OUTByDMA("realpos ");
	USART_OUTByDMAF(gRobot.cameraAngle);
	USART_OUTByDMA("\r\n");
	
	/*���غ���ǵ�״̬*/
  USART_OUTByDMA("course aimAngle ");
	USART_OUTByDMAF(gRobot.sDta.courseAimAngle);
  USART_OUTByDMA("realpos ");
	USART_OUTByDMAF(gRobot.courseAngle);
	USART_OUTByDMA("\r\n");
	
	/*���ظ����ǵ�״̬*/
  USART_OUTByDMA("course aimAngle ");
	USART_OUTByDMAF(gRobot.sDta.pitchAimAngle);
  USART_OUTByDMA("realpos ");
	USART_OUTByDMAF(gRobot.pitchAngle);
	USART_OUTByDMA("\r\n");
  
  USART_OUTByDMA("gasValue aim ");
  USART_OUTByDMAF(gRobot.sDta.gasAimValue);
  USART_OUTByDMA("\treal ");
  USART_OUTByDMAF(gRobot.gasValue);
  USART_OUTByDMA("\r\n");
}

void processReponse(void)
{
	
  switch(gRobot.sDta.robocon2018)
  {
    case ROBOT_PREPARE:
      USART_OUTByDMA("ROBOT_PREPARE\t");
		break;
	
		case ROBOT_START:
			USART_OUTByDMA("ROBOT_START\t");
		break;
	}
  switch(gRobot.sDta.process)
  {
  case TO_START:
    USART_OUTByDMA("TO_START\t");
    
    break;
  case TO_GET_BALL_1:
    USART_OUTByDMA("TO_GET_BALL_1\t");
    
    break;
  case TO_THE_AREA_1:
    USART_OUTByDMA("TO_THE_AREA_1\t");
    
    break;
  case TO_THROW_BALL_1:
    USART_OUTByDMA("TO_THROW_BALL_1\t");
    
    break;
  case TO_GET_BALL_2:
    USART_OUTByDMA("TO_GET_BALL_2\t");
    
    break;
  case TO_THE_AREA_2:
    USART_OUTByDMA("TO_THE_AREA_2\t");
    
    break;
  case TO_THROW_BALL_2:
    USART_OUTByDMA("TO_THROW_BALL_2\t");
    
    break;
  case TO_GET_BALL_3:
    USART_OUTByDMA("TO_GET_BALL_3\t");
    
    break;
  case TO_THE_AREA_3:
    USART_OUTByDMA("TO_THE_AREA_3\t");
    
    break;
  case TO_THROW_BALL_3:
    USART_OUTByDMA("TO_THROW_BALL_3\t");
    
    break;
  case END_COMPETE:
    USART_OUTByDMA("END_COMPETE\t");
    
    break;
  }
}


//�����ˡ�c
void StatusReport(void)
{
  processReponse();
  ErrorReport();
  MotionStatus();
}

/*
*�������Լ���Ƶ���ʱ���к��������ڼ�����Ƿ�����ִ�е�λ
*֮�󲻲��ô˷�ʽ
*����ÿ�����ڶ��ȷ���ת����ٷ���ȡ����ķ�ʽ
*��Ȼ���˼��Σ���һ��Ҳ��ռ̫��ʱ�䣬���ǿ���ʡ�鷳
*����ô��ο϶�����ȥ�ˣ�����������⣬�ǿ϶��Ƕ������������߶���
*/

void DelayMsRun(char task,short ms)
{
  int index=0;
  gRobot.delayTask|=task;
  while(task!=1)
  {
    task>>=1;
    index++;
  }
  gRobot.delayTaskMs[index]=ms;
}
void DelayStop(char task)
{
  int index=0;
  gRobot.delayTask&=~task;
  
  while(task!=1)
  {
    task>>=1;
    index++;
  }
  gRobot.delayTaskMs[index]=0;

}


/*״̬������*/
#define TO_START													1
#define TO_GET_BALL_1											2
#define TO_THE_AREA_1											3
#define TO_THROW_BALL_1										4
#define TO_GET_BALL_2											5
#define TO_THE_AREA_2											6
#define TO_THROW_BALL_2										7
#define TO_GET_BALL_3											8
#define TO_THE_AREA_3											9
#define TO_THROW_BALL_3									  10
#define END_COMPETE												100
extern Robot_t gRobot;
void processReport(void)
{
  static uint8_t processLast=99;
  
  if(gRobot.sDta.process==processLast)
    return;
  
  switch(gRobot.sDta.process)
  {
  case TO_START:
    USART_OUTByDMA("TO_START\t");
    break;
  case TO_GET_BALL_1:
    USART_OUTByDMA("TO_GET_BALL_1\t");
    USART_OUTByDMA("\r\n");
    break;
  case TO_THE_AREA_1:
    USART_OUTByDMA("TO_THE_AREA_1\t");
    USART_OUTByDMA("\r\n");
    break;
  case TO_THROW_BALL_1:
    USART_OUTByDMA("TO_THROW_BALL_1\t");
    USART_OUTByDMA("\r\n");
    break;
  case TO_GET_BALL_2:
    USART_OUTByDMA("TO_GET_BALL_2\t");
    USART_OUTByDMA("\r\n");
    break;
  case TO_THE_AREA_2:
    USART_OUTByDMA("TO_THE_AREA_2\t");
    USART_OUTByDMA("\r\n");
    break;
  case TO_THROW_BALL_2:
    USART_OUTByDMA("TO_THROW_BALL_2\t");
    USART_OUTByDMA("\r\n");
    break;
  case TO_GET_BALL_3:
    USART_OUTByDMA("TO_GET_BALL_3\t");
    USART_OUTByDMA("\r\n");
    break;
  case TO_THE_AREA_3:
    USART_OUTByDMA("TO_THE_AREA_3\t");
    USART_OUTByDMA("\r\n");
    break;
  case TO_THROW_BALL_3:
    USART_OUTByDMA("TO_THROW_BALL_3\t");
    USART_OUTByDMA("\r\n");
    break;
  case END_COMPETE:
    USART_OUTByDMA("END_COMPETE\t");
    USART_OUTByDMA("\r\n");
    break;
  }
  processLast=gRobot.sDta.process;
}




//void DelayTaskRun(void)
//{
//  static int delayMs[DELAY_TASK_NUM]={0};
//  
//  /*�����1��λ����û�е���*/
//  if(gRobot.delayTask&DELAY_HOLD_BALL_1_CHECK_POS)
//  {
//    ReadSteer1Pos();
//    delayMs[0]++;
//    if(delayMs[0]*PERIOD_COUNTER>gRobot.delayTaskMs[0])
//    {
//      DelayStop(DELAY_HOLD_BALL_1_CHECK_POS);
//      //4096/360=11.377
//      if(fabs(gRobot.steerAimPos[0][1]-gRobot.steerPos[0])>2*11.377f)
//      {
//        Steer1HoldBallPosCrl(gRobot.steerAimPos[0][0],2000);
//        //USART_OUTByDMA("DELAY_HOLD_BALL_1_CHECK_POS FAIL\r\n");
//        ErrorRecord(HOLD_BALL_1_ROTATE_FAIL);
//      }
//      else
//      {
//        USART_OUTByDMA("DELAY_HOLD_BALL_1_CHECK_POS SUCCESS\r\n");
//      }
//      delayMs[0]=0;
//    }
//    else{
//      //			USART_OUTByDMAF(fabs(gRobot.steerAimPos[0][1]-gRobot.steerPos[0]));
//    }
//  }
//  
//  /*�����2��λ����û�е���*/
//  if(gRobot.delayTask&DELAY_HOLD_BALL_2_CHECK_POS)
//  {
//    ReadSteer2Pos();
//    delayMs[1]++;
//    if(delayMs[1]*PERIOD_COUNTER>gRobot.delayTaskMs[1])
//    {
//      DelayStop(DELAY_HOLD_BALL_2_CHECK_POS);
//      //4096/360=11.377
//      if(fabs(gRobot.steerAimPos[1][1]-gRobot.steerPos[1])>2*11.377f)
//      {
//        Steer2HoldBallPosCrl(gRobot.steerAimPos[1][0],2000);
//        //USART_OUTByDMA("DELAY_HOLD_BALL_2_CHECK_POS FAIL\r\n");
//        ErrorRecord(HOLD_BALL_2_ROTATE_FAIL);
//      }
//      else
//      {
//        USART_OUTByDMA("DELAY_HOLD_BALL_2_CHECK_POS SUCCESS\r\n");
//      }
//      delayMs[1]=0;
//    }
//    else{
//      //			USART_OUTByDMAF(fabs(gRobot.steerAimPos[1][1]-gRobot.steerPos[1]));
//      //			USART_OUTByDMA("\r\n");
//    }
//  }
//}




static int ShootLEDShineOnce=1;
void ShootLEDShine(void){
	ShootLedOn();
	Delay_ms(500);
	ShootLedOff();
	Delay_ms(500);
	ShootLedOn();
	Delay_ms(500);
	ShootLedOff();
	Delay_ms(500);
}
void RobotSelfTest(void){
	static int selfTestStep=0;
	static int GasTestTime=0;
	static int gasTestStep=1;
	MotionCardCMDSend(NOTIFY_MOTIONCARD_SELFTEST);
	
	USART_OUTByDMA("P\t");
	USART_OUTByDMAF(gRobot.posX);
	USART_OUTByDMAF(gRobot.posY);
  USART_OUTByDMAF(gRobot.angle);
	USART_OUTByDMA("MV ");
	USART_OUTByDMAF(gRobot.courseAngle);
	USART_OUTByDMAF(gRobot.holdBallAngle[0]);
	USART_OUTByDMAF(gRobot.holdBallAngle[1]);
	USART_OUTByDMAF(gRobot.pitchAngle);
	USART_OUTByDMAF(gRobot.gasValue);
	
	switch(selfTestStep){
		//�Զ������Ӽ��
		case 0:
			if(ShootLEDShineOnce){
				ShootLEDShineOnce=0;
				ShootLEDShine();
				MotionCardCMDSend(NOTIFY_MOTIONCARD_SELFTEST_THE_WHEEL);
			}
			USART_OUTByDMA("WHEEL_TEST\r\n");
			if(gRobot.sDta.AT_motionFlag&AT_THE_WHEEL_SELFTEST_OVER){
				selfTestStep++;
				ShootLEDShineOnce=1;
				USART_OUTByDMA("WHEEL_TEST_OVER\r\n");
			}
		break;
			
		case 1:
			if(ShootLEDShineOnce){
				ShootLEDShineOnce=0;
				ShootLEDShine();
				MotionCardCMDSend(NOTIFY_MOTIONCARD_SELFTEST_THE_DUCT);
			}
			USART_OUTByDMA("DUCT_TEST\r\n");
			if(gRobot.sDta.AT_motionFlag&AT_THE_DUCT_SELFTEST_OVER){
				selfTestStep++;
				ShootLEDShineOnce=1;
				USART_OUTByDMA("THE_DUCT_SELFTEST_OVER\r\n");
			}
		break;
			
		//�Ե����������Լ�
		case 2:
			if(ShootLEDShineOnce){
				ShootLEDShineOnce=0;
				ShootLEDShine();
			}
			USART_OUTByDMA("STEER_MOTION_TEST\r\n");
			//�����ǵ�-15��
			PitchAngleMotion(-15.f);
			gRobot.sDta.pitchAimAngle=-15.f;
			Delay_ms(1500);
			//�����ǵ�20��
			gRobot.sDta.pitchAimAngle=15.f;
			PitchAngleMotion(15.f);
			Delay_ms(1500);
			gRobot.sDta.pitchAimAngle=0.f;
			PitchAngleMotion(0.f);
			Delay_ms(1500);
		
			gRobot.sDta.courseAimAngle=0.f;//��ֹ�������£�MotionExceute�����ִ��
			CourseAngleMotion(0.f);
			Delay_ms(2000);
			gRobot.sDta.courseAimAngle=180.f;//��ֹ�������£�MotionExceute�����ִ��
			CourseAngleMotion(90.f);
			Delay_ms(2000);
			gRobot.sDta.courseAimAngle=0.f;
			CourseAngleMotion(0.f);
			Delay_ms(2500);
		
		  //�������������0
			gRobot.sDta.holdBallAimAngle[0]=gRobot.sDta.holdBallAimAngle[1]=0.f;
			HoldBallPosCrlSeparate(0.f,0.f);
			Delay_ms(2000);
			gRobot.sDta.holdBallAimAngle[0]=gRobot.sDta.holdBallAimAngle[1]=90.f;
			HoldBallPosCrlSeparate(90.f,90.f);
			Delay_ms(2000);
			gRobot.sDta.holdBallAimAngle[0]=gRobot.sDta.holdBallAimAngle[1]=0.f;
			HoldBallPosCrlSeparate(0.f,0.f);
			Delay_ms(2000);
			gRobot.sDta.holdBallAimAngle[0]=gRobot.sDta.holdBallAimAngle[1]=-90.f;
			HoldBallPosCrlSeparate(-90.f,-90.f);
			Delay_ms(2000);
			gRobot.sDta.holdBallAimAngle[0]=gRobot.sDta.holdBallAimAngle[1]=0.f;
			HoldBallPosCrlSeparate(0.f,0.f);
			Delay_ms(2000);
			ShootLEDShineOnce=1;
			selfTestStep++;
		break;
			
			
		//��ѹ��⣬˳���ȷ���
		case 3:
			if(ShootLEDShineOnce){
				ShootLEDShineOnce=0;
				ShootLEDShine();
			}
			USART_OUTByDMA("GAS_TEST\r\n");
			GasTestTime++;
			USART_OUTByDMA("gasValue\t");
			USART_OUTByDMAF(gRobot.gasValue);
			switch(gasTestStep){
				case 1:
					gRobot.sDta.gasAimValue = 0.250f;
				  GasMotion(0.250);
					GasEnable();
					if(fabs(gRobot.sDta.gasAimValue-gRobot.gasValue)<0.02){
				    gasTestStep++;
						GasTestTime=0;
						BEEP_OFF;
			    }
					if(GasTestTime>1000){
						BEEP_ON;
					}
			  break;
					
				case 2:
					gRobot.sDta.gasAimValue = 0.300;
				  GasMotion(0.300);
					BEEP_OFF;
					if(fabs(gRobot.sDta.gasAimValue-gRobot.gasValue)<0.02){
				    ShootLEDShineOnce=1;
				    selfTestStep++;
						BEEP_OFF;
			    }
					if(GasTestTime>1000){
						BEEP_ON;
					}
			  break;
				
			}
		break;
			
		//�Ը����������Լ�
		case 4:
			if(ShootLEDShineOnce){
				ShootLEDShineOnce=0;
				ShootLEDShine();
			}
			USART_OUTByDMA("GAS_BAORD_TEST\r\n");
			//צ���ſ�
			ClawOpen();
			Delay_ms(1000);
			ClawShut();
			Delay_ms(1000);
		
		  //������������
			ClawOpen();
			ShootSmallOpen();
			Delay_ms(1000);
      ShootBigOpen();
			Delay_ms(2000);	
			ShootSmallShut();
      ShootBigShut();
			ClawShut();
			Delay_ms(1000);

		
			//�����ץȡ����
			GoldBallGraspStairTwoOn();
			Delay_ms(1500);
			//�����ץȡ����
			GoldBallGraspStairTwoOff();
		  Delay_ms(1500);
			GoldBallGraspStairTwoOn();
			Delay_ms(1500);
			
			ExtendCarOn();
			Delay_ms(1500);
			ExtendCarOff();
			Delay_ms(1500);
			
		  selfTestStep++;
			ShootLEDShineOnce=1;
		break;
		
		
		//������
		case 5:
			if(ShootLEDShineOnce){
				ShootLEDShineOnce=0;
				ShootLEDShine();
				MotionCardCMDSend(NOTIFY_MOTIONCARD_SELFTEST_THE_LASER);
			}
			USART_OUTByDMA("LASER_TEST\r\n");
			if(PE_FOR_THE_BALL||PE_CHECK_GOLD){
				BEEP_ON;
				return;
			}else if(PE_FOR_THE_BALL==0&&PE_CHECK_GOLD==0){
				BEEP_OFF;
			}
			USART_OUTByDMA("A%d\t B%d\t",gRobot.laser[0],gRobot.laser[1]);
			USART_OUTByDMA("P ");
		  USART_OUTByDMAF(gRobot.posX);
	    USART_OUTByDMAF(gRobot.posY);
		  USART_OUTByDMAF(gRobot.angle);
			USART_OUTByDMA("\r\n");
			if(gRobot.laser[0]>20&&gRobot.laser[0]<300){
				BEEP_ON;
				Delay_ms(gRobot.laser[0]/10*5);
				BEEP_OFF;
				Delay_ms(gRobot.laser[0]/10*5);
			}
			if(gRobot.laser[1]>20&&gRobot.laser[1]<300){
				BEEP_ON;
				Delay_ms(gRobot.laser[1]/10*5);
				BEEP_OFF;
				Delay_ms(gRobot.laser[1]/10*5);
			}
			
			
		break;
		
	}
}


void GetRack3Ball(void){
	static 	int delayGetRack3BallTime=0;
  static  uint8_t getRack3BallStep=0;
	switch(getRack3BallStep){
		case 0:
			if(gRobot.posX>100.f){
		    PrepareGetBall(BALL_RACK3);
			}
			if(gRobot.posX>3500.f)
			{
				delayGetRack3BallTime++;
			}
			if(delayGetRack3BallTime>800){
					delayGetRack3BallTime=0;
				  getRack3BallStep=1;
				  MotionCardCMDSend(NOTIFY_MOTIONCARD_SHOT_BALL1);
			}
		break;
			
		case 1:
			if(PrepareForTheBall()){
					PrepareShootBall(BALL_1);
				  getRack3BallStep=2;
			}
		break;
			
	  case 2:
			if(gRobot.sDta.AT_motionFlag&AT_REACH_FIRST_PLACE){
				ShootLedOn();
				getRack3BallStep=3;
				SetMotionFlag(~AT_REACH_FIRST_PLACE);
			}
		break;
			
		case 3:
			if(/*��������λ*/
			   		(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_1_SUCCESS)
						/*��������λ*/
						&&(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_2_SUCCESS)
							/*������λ��*/
							&&(gRobot.sDta.AT_motionFlag&AT_PITCH_SUCCESS)
								/*����λ*/
								&&(gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS)/*&&(gRobot.posY>2000.f*/)
									  /*��ѹ��λ*/
//										&&(gRobot.sDta.AT_motionFlag&AT_GAS_SUCCESS))
			{
				/*����*/
				ShootBall();
				
				/*����ʱʹ�������ִ�е�λ*/
				Delay_ms(125);
				
				//��ǰ������ѹ
				GasMotion(GetPrepareShootGoldBallGasAim(BALL_2));
				/*���������λ*/
				ShootReset();
				
				/*׼�������*/
				PrepareGetBall(BALL_2);
				/*������һ״̬*/
				
				gRobot.sDta.process=TO_GET_BALL_2;
				gRobot.sDta.robocon2018=COLORFUL_BALL_2;
				SetMotionFlag(AT_IS_SEND_DEBUG_DATA);
				SetMotionFlag(~AT_REACH_FIRST_PLACE);
			}
			else
			{
//				SetMotionFlag(~AT_IS_SEND_DEBUG_DATA);		
				if(!PE_FOR_THE_BALL)
					USART_OUTByDMA("!PE1 ");
				if(!(gRobot.sDta.AT_motionFlag&AT_PITCH_SUCCESS))
				{
					USART_OUTByDMA("!PITCH1 ");
					USART_OUTByDMAF(gRobot.pitchAngle);
				}
				if(!(gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS))
				{
					USART_OUTByDMA("!COURSE1 ");
					USART_OUTByDMAF(gRobot.courseAngle);
				}
				if(!(gRobot.sDta.AT_motionFlag&AT_GAS_SUCCESS))
				{
					USART_OUTByDMA("!GAS1 ");
					USART_OUTByDMAF(gRobot.gasValue);
				}
			}
		break;
	}
		
		
		
		
}
	
	
	
	

