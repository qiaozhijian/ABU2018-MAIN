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
			//电机位置环
			PosLoopCfg(CAN2, 5, 100000, 100000,100000);
			//电机位置环
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
			//开小气阀
			ReadActualPos(CAN2,6);
			if(gRobot.courseAngle>160.f)
			{
				//关小气阀
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

/*完成投射彩球一的任务*/
void FightForBall1(void)
{
	static int getBallStep=0;
  switch(gRobot.sDta.process)
  {
    /*去取第一个球*/
		case TO_GET_BALL_1:
			//光电是否被扫到
				switch(getBallStep){
				//第一步对光电进行扫描
						case 0:
							if(PrepareForTheBall()){
//								Delay_ms(1000);
								MotionCardCMDSend(NOTIFY_MOTIONCARD_GOT_BALL1);
								gRobot.raceTime.colorBall1WaitTime=gRobot.raceTime.roboconTime;
								getBallStep++;
							}
						break;
					
						case 1:
							if(PE_FOR_THE_BALL){
								PrepareShootBall(BALL_1);
								/*这个动作一定要等到先给电机发指令转后进行，因为函数内有延时*/
								LedBallInto();
								getBallStep++;
							}
						break;

						case 2:
								USART_OUTByDMA("IntoTheArea\t");
								//TalkToCamera(CAMERA_OPEN_NEAR);
								
								gRobot.sDta.process=TO_THE_AREA_1;
						break;
			}
			break;
			
    /*第一个球取球完毕，去投射区一*/
		case TO_THE_AREA_1:
			if(gRobot.sDta.AT_motionFlag&AT_REACH_FIRST_PLACE||(gRobot.posY >=2130.f))
				gRobot.sDta.process=TO_THROW_BALL_1;
			//在CAN中断当中读取控制卡发来的数据，到达指定位置让gRobot.sDta.process变为为TO_THROW_BALL_1
			break;
			
    /*到达投射区一，射球*/
		case TO_THROW_BALL_1:
			/*光电到位*/
			if(gRobot.robotVel.countVel<200.f
					 &&PE_FOR_THE_BALL
				/*持球舵机到位*/
			   //		&&(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_1_SUCCESS)
						/*持球舵机到位*/
					//	&&(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_2_SUCCESS)
							/*俯仰到位，*/
							&&(gRobot.sDta.AT_motionFlag&AT_PITCH_SUCCESS)
								/*航向到位*/
								&&(gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS)/*&&(gRobot.posY>2000.f*/
									 &&(gRobot.posY>2100.f)
									  /*气压到位*/
										&&(gRobot.sDta.AT_motionFlag&AT_GAS_SUCCESS))
			{
				/*射球*/
				ShootBall();
				/*给延时使发射杆能执行到位*/
				Delay_ms(175);
				
				/*计算从接到球射球的时间*/
				gRobot.raceTime.colorBall1ThrowTime=gRobot.raceTime.roboconTime - gRobot.raceTime.colorBall1WaitTime;
				gRobot.raceTime.colorBall1Time = gRobot.raceTime.colorBall1WaitTime + gRobot.raceTime.colorBall1ThrowTime ;
				
				/*通知控制卡*/
				MotionCardCMDSend(NOTIFY_MOTIONCARD_SHOT_BALL1);
				
				/*射球机构复位*/
				ShootReset();
				
				/*准备接球二*/
				PrepareGetBall(BALL_2);
				/*进入下一状态*/
				gRobot.sDta.process=TO_GET_BALL_2;
				gRobot.sDta.robocon2018=COLORFUL_BALL_2;
				SetMotionFlag(AT_IS_SEND_DEBUG_DATA);
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
					USART_OUTByDMA("!PE1 ");
		//			if(!(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_1_SUCCESS))
		//				USART_OUTByDMA("!HB11\t");
		//			if(!(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_2_SUCCESS))
		//-USART_OUTByDMA("!HB21\t");
				if(gRobot.robotVel.countVel>150.f){
				  USART_OUTByDMA("RobotVel Large! ");
			  }
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
				//USART_OUTByDMA("\r\n");
			}
			break;
		}
}

/*完成投射彩球二的任务*/
void FightForBall2(void)
{
	static int getBallStep=0;
  switch(gRobot.sDta.process)
  {
    /*去取第二个球*/
		case TO_GET_BALL_2:
			//光电是否被扫到
			switch(getBallStep){
					//第一步对光电进行扫描
				case 0:
					if(PrepareForTheBall()){
						MotionCardCMDSend(NOTIFY_MOTIONCARD_GOT_BALL2);
						gRobot.raceTime.colorBall2WaitTime = gRobot.raceTime.roboconTime  - gRobot.raceTime.colorBall1Time;
						getBallStep++;
					}
				break;
						
				case 1:
					if(PrepareForTheBall()){
						/*和彩球1情况一样*/
						PrepareShootBall(BALL_2);
						LedBallInto();
						getBallStep++;

					}
				break;

				case 2:
					if(gRobot.posX>6000.f)
					{	
						USART_OUTByDMA("IntoTheArea\t");
						gRobot.sDta.process=TO_THE_AREA_2;
						
						
					}
					break;
				}
		break;
				
			/*第二个球取球完毕，去投射区二*/
		case TO_THE_AREA_2:
			if(gRobot.sDta.AT_motionFlag&AT_REACH_SECOND_PLACE||(gRobot.posY>=2130.f))
				gRobot.sDta.process=TO_THROW_BALL_2;
//			if(!PrepareForTheBall())
//			{
//				MotionCardCMDSend(NOTIFY_MOTIONCARD_LOSE_BALL2);
//			}
			break;
			
			/*到达投射区二，射球*/
		case TO_THROW_BALL_2:
			if(gRobot.robotVel.countVel<150.f
					 &&PE_FOR_THE_BALL
					/*持球舵机到位*/
			//			&&(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_1_SUCCESS)
							/*持球舵机到位*/
				//			&&(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_2_SUCCESS)
								/*俯仰到位，*/
								&&(gRobot.sDta.AT_motionFlag&AT_PITCH_SUCCESS)
									/*航向到位*/
									&&(gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS)/*&&(gRobot.posY>2000.f)*/
										&&fabs(gRobot.posY-TZ_2_Y)<80.f
											/*气压到位*/
											&&(gRobot.sDta.AT_motionFlag&AT_GAS_SUCCESS))
			{
				/*射球*/
				ShootBall();
				
				/*给延时使发射杆能执行到位*/
				Delay_ms(175);
				/*计算投球时间*/
				gRobot.raceTime.colorBall2ThrowTime=gRobot.raceTime.roboconTime - gRobot.raceTime.colorBall1Time - gRobot.raceTime.colorBall2WaitTime;
				gRobot.raceTime.colorBall2Time=gRobot.raceTime.colorBall2ThrowTime + gRobot.raceTime.colorBall2WaitTime;
				
				MotionCardCMDSend(NOTIFY_MOTIONCARD_SHOT_BALL2);
				
				/*射球机构复位*/
				ShootReset();
				
				/*金球航向速度减小*/
				if(gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS){
					PosLoopCfg(CAN2, COURCE_MOTOR_ID, 8000000, 8000000,800000);
				}
				
				/*准备接球三*/
				PrepareGetBall(BALL_3_WAIT);
				
				gRobot.sDta.process=TO_GET_BALL_3;
				
				gRobot.sDta.robocon2018=GOLD_BALL;
				SetMotionFlag(AT_IS_SEND_DEBUG_DATA);
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
				
				if(gRobot.robotVel.countVel>150.f){
				  USART_OUTByDMA("RobotVel Large! ");
			  }
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

/*完成投射金球的任务*/
void FightForGoldBall(void)
{
	static uint8_t isGetBall=0;
  switch(gRobot.sDta.process)
  {
    /*去取第三个球*/
  case TO_GET_BALL_3:
		switch(isGetBall)
		{
			case 0:
				if(gRobot.robotVel.countVel<50.f
						&&fabs(gRobot.posX-HANDOVER_3_X)<50.f
							&&fabs(gRobot.posY-HANDOVER_3_Y)<50.f){
								isGetBall++;
								GoldBallGraspStairTwoOn();
				}
			break;
							
				
		  case 1:
				if(GoldRackInto()){
					gRobot.raceTime.goldBallWaitTime=gRobot.raceTime.roboconTime - gRobot.raceTime.colorBall1Time - gRobot.raceTime.colorBall2Time;
					MotionCardCMDSend(NOTIFY_MOTIONCARD_GOT_BALL3);
					isGetBall++;
				}
			break;
				
			case 2:
				if(gRobot.posY>2100.f){
					//接取金球一
					PrepareGetBall(BALL_3);
					isGetBall++;
				}
			break;
				
			case 3:
				if((gRobot.sDta.AT_motionFlag&AT_PITCH_SUCCESS)
							&&(gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS)
					      &&PrepareForTheBall())
				{
					gRobot.sDta.courseAimAngle = 179.9f;
					isGetBall++;
				}
			break;
				
			case 4:
				if(fabs(gRobot.courseAngle - gRobot.sDta.courseAimAngle)<45.f){
					/*航向转到到位直接开始准备射球参数*/
					Delay_ms(200);
					PrepareShootBall(BALL_3);
					GoldBallGraspStairTwoOff();
					gRobot.sDta.process=TO_THE_AREA_3;
					USART_OUTByDMA("PrepareShoot ");
					isGetBall++;
				}
			break;
				
			//接去第二金球
			case 11:
				PrepareGetBall(BALL_4);
				isGetBall = 12;
			break;
				
			case 12:
				if((gRobot.sDta.AT_motionFlag&AT_PITCH_SUCCESS)
								&&(gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS)){
					Delay_ms(300);
					isGetBall=13;
				}
			break;
				
			case 13:
				if(PrepareForTheBall()){
					gRobot.sDta.courseAimAngle = 178.4f;
					isGetBall=14;
				}
			break;
				
			case 14:
				if(fabs(gRobot.sDta.courseAimAngle-gRobot.courseAngle)<45.f){
					PrepareShootBall(BALL_4);
					gRobot.sDta.process=TO_THROW_BALL_3;
					isGetBall=15;
				}
			break;
				
				
		}
    break;
		
    /*第三个球取球完毕，去投射区三*/
  case TO_THE_AREA_3:
		if(gRobot.sDta.AT_motionFlag&AT_REACH_THIRD_PLACE||(gRobot.posY>5850.f))/*射金球点6080 ， 6030*/
			gRobot.sDta.process=TO_THROW_BALL_3;
		//光电发现丢球这时候应该通知控制卡球丢了同时自己应该把gRobot.sDta.process归位取彩球进程
    if(!PrepareForTheBall())
    {
      MotionCardCMDSend(NOTIFY_MOTIONCARD_LOSE_BALL3);
    }
    break;
		
    /*到达投射区三，射球*/
  case TO_THROW_BALL_3:
    if(PE_FOR_THE_BALL
				&&gRobot.robotVel.countVel<100.f
				/*持球舵机到位*/
		//		&&(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_1_SUCCESS)
					/*持球舵机到位*/
		//			&&(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_2_SUCCESS)
						/*俯仰到位，*/
						&&(gRobot.sDta.AT_motionFlag&AT_PITCH_SUCCESS)
							/*航向到位*/
							&&(gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS)
								/*&&(gRobot.posY>5530.f)*/
								&&fabs(gRobot.posY-TZ_3_Y)<50.f
									/*气压到位*/
									&&(gRobot.sDta.AT_motionFlag&AT_GAS_SUCCESS))
    {
      /*射球*/
      ShootBall();
      
      /*给延时使发射杆能执行到位*/
      Delay_ms(175);
			
			gRobot.raceTime.goldBallThrowTime=gRobot.raceTime.roboconTime - gRobot.raceTime.colorBall1Time - gRobot.raceTime.colorBall2Time - gRobot.raceTime.goldBallWaitTime;
			gRobot.raceTime.goldBallTime=gRobot.raceTime.goldBallWaitTime + gRobot.raceTime.goldBallThrowTime;
			
      /*射球机构复位*/
      ShootReset();
      
			/*射完金球进程停止了，这时候看看需要再投掷，那就把 gRobot.sDta.process改为取金球同时更改第一个金球到的判读坐标条件*/
      gRobot.sDta.process=TO_GET_BALL_3;
			isGetBall=11;
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
//			if(!(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_1_SUCCESS))
//				USART_OUTByDMA("!HB13\t");
//			if(!(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_2_SUCCESS))
//				USART_OUTByDMA("!HB23\t");
			if(gRobot.robotVel.countVel>100.f){
				USART_OUTByDMA("RobotVel Large! ");
			}
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



//motion。c
void MotionStatus(void)
{
	/*返回舵机一的状态*/
  USART_OUTByDMA("steer 1 aimAngle ");
	USART_OUTByDMAF(gRobot.sDta.holdBallAimAngle[0]);
  USART_OUTByDMA("realpos ");
	USART_OUTByDMAF(gRobot.holdBallAngle[0]);
	USART_OUTByDMA("\r\n");
	
	/*返回舵机二的状态*/
  USART_OUTByDMA("steer 2 aimAngle ");
	USART_OUTByDMAF(gRobot.sDta.holdBallAimAngle[1]);
  USART_OUTByDMA("realpos ");
	USART_OUTByDMAF(gRobot.holdBallAngle[1]);
	USART_OUTByDMA("\r\n");
	
	/*返回舵机三的状态*/
  USART_OUTByDMA("steer 3 aimAngle ");
	USART_OUTByDMAF(gRobot.sDta.cameraAimAngle);
  USART_OUTByDMA("realpos ");
	USART_OUTByDMAF(gRobot.cameraAngle);
	USART_OUTByDMA("\r\n");
	
	/*返回航向角的状态*/
  USART_OUTByDMA("course aimAngle ");
	USART_OUTByDMAF(gRobot.sDta.courseAimAngle);
  USART_OUTByDMA("realpos ");
	USART_OUTByDMAF(gRobot.courseAngle);
	USART_OUTByDMA("\r\n");
	
	/*返回俯仰角的状态*/
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


//机器人。c
void StatusReport(void)
{
  processReponse();
  ErrorReport();
  MotionStatus();
}

/*
*以下是自己设计的延时进行函数，用于检测舵机是否真正执行到位
*之后不采用此方式
*采用每个周期都先发旋转命令，再发读取命令的方式
*虽然多了几次，但一是也不占太多时间，二是可以省麻烦
*发那么多次肯定发过去了，如果还有问题，那肯定是舵机有问题或者线断了
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


/*状态量解释*/
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
//  /*检测舵机1的位置有没有到达*/
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
//  /*检测舵机2的位置有没有到达*/
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
	
	switch(selfTestStep){
		//自动车轮子检测
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
			
		//对电机，舵机的自检
		case 2:
			if(ShootLEDShineOnce){
				ShootLEDShineOnce=0;
				ShootLEDShine();
			}
			USART_OUTByDMA("STEER_MOTION_TEST\r\n");
			//俯仰角到-15度
			PitchAngleMotion(-15.f);
			gRobot.sDta.pitchAimAngle=-15.f;
			Delay_ms(1500);
			//俯仰角到20度
			gRobot.sDta.pitchAimAngle=15.f;
			PitchAngleMotion(15.f);
			Delay_ms(1500);
			gRobot.sDta.pitchAimAngle=0.f;
			PitchAngleMotion(0.f);
			Delay_ms(1500);
		
			gRobot.sDta.courseAimAngle=0.f;//防止参数更新，MotionExceute里面会执行
			CourseAngleMotion(0.f);
			Delay_ms(2000);
			gRobot.sDta.courseAimAngle=180.f;//防止参数更新，MotionExceute里面会执行
			CourseAngleMotion(90.f);
			Delay_ms(2000);
			gRobot.sDta.courseAimAngle=0.f;
			CourseAngleMotion(0.f);
			Delay_ms(2500);
		
		  //上下两个舵机到0
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
			
			
		//气压检测，顺便先放气
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
			    }
					if(GasTestTime>1000){
						BEEP_ON;
					}
			  break;
				
			}
		break;
			
		//对各个气阀的自检
		case 4:
			if(ShootLEDShineOnce){
				ShootLEDShineOnce=0;
				ShootLEDShine();
			}
			USART_OUTByDMA("GAS_BAORD_TEST\r\n");
			//爪子张开
			ClawOpen();
			Delay_ms(1000);
			ClawShut();
			Delay_ms(1000);
		
		  //射球两个气阀
			ClawOpen();
			ShootSmallOpen();
			Delay_ms(1000);
      ShootBigOpen();
			Delay_ms(2000);	
			ShootSmallShut();
      ShootBigShut();
			ClawShut();
			Delay_ms(1000);

		
			//金球架抓取气阀
			GoldBallGraspStairTwoOn();
			Delay_ms(1500);
			//金球架抓取气阀
			GoldBallGraspStairTwoOff();
		  Delay_ms(1500);

		  selfTestStep++;
			ShootLEDShineOnce=1;
		break;
		
		
		//激光检测
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
