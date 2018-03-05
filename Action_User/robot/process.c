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

extern Robot_t gRobot;
extern int flagggg;

void SelfTest(void)
{
	AT_CMD_Handle();
	USART_BLE_SEND(gRobot.gasValue);
	static int step=100;
	static int count=0;
	switch(step)
	{
		case 10:
		if(PrepareForTheBall())
		{
			count++;
		}else
			count=0;
		if(count>200)
		{
			BoostPolePush();
			Delay_ms(2000);
			BoostPoleReturn();
			count=0;
		}
	
		
//			SetResponseStair(0x01);
//	HoldSteer1PosCrl(0.f,2000);		
//	OpenSteerTorque(0xfe);
//  SteerPosCrlBy485(0xfe,(int)(180.f*11.378f));
//		HoldSteer2PosCrl(30.f,2000);
		
//		if(flagggg==1)
//		{
//			BoostPolePush();
//			step++;
//		}
//		
			break;
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
	}
}

/*完成投射彩球一的任务*/
void FightForBall1(void)
{
	static float PE_GotX=0.f;
  switch(gRobot.sDta.process)
  {
    /*去取第一个球*/
  case TO_GET_BALL_1:
		//光电是否被扫到
    if(PrepareForTheBall()&&PE_GotX==0.f)
    {	
			PE_GotX=gRobot.posX;
    }
		if((gRobot.posX>4054.f&&PE_GotX<3754.f)||(gRobot.posY>2000.f))
		{
			USART_OUT(DEBUG_USART,"IntoTheArea\r\n");
      MotionCardCMDSend(NOTIFY_MOTIONCARD_GOT_BALL1);
			//TalkToCamera(CAMERA_OPEN_NEAR);
      PrepareShootBall(BALL_1);
      gRobot.sDta.process=TO_THE_AREA_1;
		}
    break;
    /*第一个球取球完毕，去投射区一*/
  case TO_THE_AREA_1:
		if(gRobot.sDta.AT_motionFlag&AT_REACH_FIRST_PLACE)
			gRobot.sDta.process=TO_THROW_BALL_1;
		//在CAN中断当中读取控制卡发来的数据，到达指定位置让gRobot.sDta.process变为为TO_THROW_BALL_1
    break;
    /*到达投射区一，射球*/
  case TO_THROW_BALL_1:
		/*光电到位*/
    if(PrepareForTheBall()
				/*持球舵机到位*/
		//		&&(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_1_SUCCESS)
					/*持球舵机到位*/
				//	&&(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_2_SUCCESS)
						/*俯仰到位，*/
						&&(gRobot.sDta.AT_motionFlag&AT_PITCH_SUCCESS)
							/*航向到位*/
							&&(gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS)&&(gRobot.posY>1800.f)
								/*气压到位*/
								&&(gRobot.sDta.AT_motionFlag&AT_GAS_SUCCESS))
    {
      /*射球*/
      ShootBall();
      /*给延时使发射杆能执行到位*/
      Delay_ms(500);
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
			SetMotionFlag(~AT_IS_SEND_DEBUG_DATA);
			if(!PE_FOR_THE_BALL)
				USART_OUT(DEBUG_USART,"!PE1\t");
//			if(!(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_1_SUCCESS))
//				USART_OUT(DEBUG_USART,"!HB11\t");
//			if(!(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_2_SUCCESS))
//				USART_OUT(DEBUG_USART,"!HB21\t");
			if(!(gRobot.sDta.AT_motionFlag&AT_PITCH_SUCCESS))
			{
				USART_OUT(DEBUG_USART,"!PITCH1\t");
				USART_OUT_F(gRobot.pitchAngle);
			}
			if(!(gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS))
			{
				USART_OUT(DEBUG_USART,"!COURSE1\t");
				USART_OUT_F(gRobot.courseAngle);
			}
			if(!(gRobot.sDta.AT_motionFlag&AT_GAS_SUCCESS))
			{
				USART_OUT(DEBUG_USART,"!GAS1\t");
				USART_OUT_F(gRobot.gasValue);
			}
			USART_Enter();
		}
    break;
  }
}

/*完成投射彩球二的任务*/
void FightForBall2(void)
{
	static float PE_GotX=0.f;
  switch(gRobot.sDta.process)
  {
    /*去取第二个球*/
  case TO_GET_BALL_2:
		//光电是否被扫到
    if(PrepareForTheBall()&&gRobot.posY<1600.f&&PE_GotX==0.f)
    {	
			PE_GotX=gRobot.posX;
    }
		if(fabs(gRobot.posX-PE_GotX)>200.f&&gRobot.posX>6000.f)
    {	
      /*扫到光电后，为了更稳地接到球而给的延时*/
      gRobot.sDta.process=TO_THE_AREA_2;
      
      MotionCardCMDSend(NOTIFY_MOTIONCARD_GOT_BALL2);
      
      PrepareShootBall(BALL_2);
      
    }
    break;
    /*第二个球取球完毕，去投射区二*/
  case TO_THE_AREA_2:
		if(gRobot.sDta.AT_motionFlag&AT_REACH_SECOND_PLACE)
			gRobot.sDta.process=TO_THROW_BALL_2;
    if(!PrepareForTheBall())
    {
      MotionCardCMDSend(NOTIFY_MOTIONCARD_LOSE_BALL2);
    }
    break;
    /*到达投射区二，射球*/
  case TO_THROW_BALL_2:
		/*光电到位*/
    if(PrepareForTheBall()
				/*持球舵机到位*/
	//			&&(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_1_SUCCESS)
					/*持球舵机到位*/
		//			&&(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_2_SUCCESS)
						/*俯仰到位，*/
						&&(gRobot.sDta.AT_motionFlag&AT_PITCH_SUCCESS)
							/*航向到位*/
							&&(gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS)&&(gRobot.posY>1800.f)
								/*气压到位*/
								&&(gRobot.sDta.AT_motionFlag&AT_GAS_SUCCESS))
    {
      /*射球*/
      ShootBall();
      
      /*给延时使发射杆能执行到位*/
      Delay_ms(500);
      MotionCardCMDSend(NOTIFY_MOTIONCARD_SHOT_BALL2);
      
      /*射球机构复位*/
      ShootReset();
      
      /*准备接球三*/
      PrepareGetBall(BALL_3);
      
      gRobot.sDta.process=TO_GET_BALL_3;
      gRobot.sDta.robocon2018=GOLD_BALL;
			SetMotionFlag(AT_IS_SEND_DEBUG_DATA);
    }
		else
		{
			SetMotionFlag(~AT_IS_SEND_DEBUG_DATA);
			if(!PE_FOR_THE_BALL)
				USART_OUT(DEBUG_USART,"!PE2\t");
//			if(!(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_1_SUCCESS))
//				USART_OUT(DEBUG_USART,"!HB12\t");
//			if(!(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_2_SUCCESS))
//				USART_OUT(DEBUG_USART,"!HB22\t");
			if(!(gRobot.sDta.AT_motionFlag&AT_PITCH_SUCCESS))
			{
				USART_OUT(DEBUG_USART,"!PITCH2\t");
				USART_OUT_F(gRobot.pitchAngle);
			}
			if(!(gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS))
			{
				USART_OUT(DEBUG_USART,"!COURSE2\t");
				USART_OUT_F(gRobot.courseAngle);
			}
			if(!(gRobot.sDta.AT_motionFlag&AT_GAS_SUCCESS))
			{
				USART_OUT(DEBUG_USART,"!GAS2\t");
				USART_OUT_F(gRobot.gasValue);
			}
			USART_Enter();
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
				if(PrepareForTheBall()&&gRobot.posY<1600.f)
					isGetBall++;
				break;
			case 1:
				Delay_ms(3000);
				BoostPolePush();
				isGetBall++;
				break;
			case 2:
				if(gRobot.posY>2500.f)
				{
					//TalkToCamera(CAMERA_OPEN_FAR);
					
					//这之后应该向金球架抓取气阀发数抓取金球架
					gRobot.sDta.process=TO_THE_AREA_3;
					
					MotionCardCMDSend(NOTIFY_MOTIONCARD_GOT_BALL3);
					
					PrepareShootBall(BALL_3);
				}
				break;
		}
    break;
    /*第三个球取球完毕，去投射区三*/
  case TO_THE_AREA_3:
		if(gRobot.sDta.AT_motionFlag&AT_REACH_THIRD_PLACE)
			gRobot.sDta.process=TO_THROW_BALL_3;
		//在y大于2500的时候将助推气阀归位
		if(gRobot.posY>2500.f) BoostPoleReturn();
		//光电发现丢球这时候应该通知控制卡球丢了同时自己应该把gRobot.sDta.process归位取彩球进程
    if(!PrepareForTheBall())
    {
      MotionCardCMDSend(NOTIFY_MOTIONCARD_LOSE_BALL3);
    }
    break;
    /*到达投射区三，射球*/
  case TO_THROW_BALL_3:
		/*光电到位*/
    if(PrepareForTheBall()
				/*持球舵机到位*/
		//		&&(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_1_SUCCESS)
					/*持球舵机到位*/
		//			&&(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_2_SUCCESS)
						/*俯仰到位，*/
						&&(gRobot.sDta.AT_motionFlag&AT_PITCH_SUCCESS)
							/*航向到位*/
							&&(gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS)&&(gRobot.posY>1800.f)
								/*气压到位*/
								&&(gRobot.sDta.AT_motionFlag&AT_GAS_SUCCESS))
    {
      /*射球*/
      ShootBall();
      
      /*给延时使发射杆能执行到位*/
      Delay_ms(500);
      /*射球机构复位*/
      ShootReset();
      
			/*射完金球进程停止了，这时候看看需要再投掷，那就把 gRobot.sDta.process改为取金球同时更改第一个金球到的判读坐标条件*/
      gRobot.sDta.process=END_COMPETE;
			SetMotionFlag(AT_IS_SEND_DEBUG_DATA);
    }
		else
		{
			SetMotionFlag(~AT_IS_SEND_DEBUG_DATA);
			if(!PE_FOR_THE_BALL)
				USART_OUT(DEBUG_USART,"!PE3\t");
//			if(!(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_1_SUCCESS))
//				USART_OUT(DEBUG_USART,"!HB13\t");
//			if(!(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_2_SUCCESS))
//				USART_OUT(DEBUG_USART,"!HB23\t");
			if(!(gRobot.sDta.AT_motionFlag&AT_PITCH_SUCCESS))
			{
				USART_OUT(DEBUG_USART,"!PITCH3\t");
				USART_OUT_F(gRobot.pitchAngle);
			}
			if(!(gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS))
			{
				USART_OUT(DEBUG_USART,"!COURSE3\t");
				USART_OUT_F(gRobot.courseAngle);
			}
			if(!(gRobot.sDta.AT_motionFlag&AT_GAS_SUCCESS))
			{
				USART_OUT(DEBUG_USART,"!GAS3\t");
				USART_OUT_F(gRobot.gasValue);
			}
			USART_Enter();
		}
    break;
  }
}



//motion。c
void MotionStatus(void)
{
	/*返回舵机一的状态*/
  USART_OUT(DEBUG_USART,"steer 1 aimAngle ");
	USART_OUT_F(gRobot.sDta.holdBallAimAngle[0]);
  USART_OUT(DEBUG_USART,"realpos ");
	USART_OUT_F(gRobot.holdBallAngle[0]);
	USART_Enter();
	
	/*返回舵机二的状态*/
  USART_OUT(DEBUG_USART,"steer 2 aimAngle ");
	USART_OUT_F(gRobot.sDta.holdBallAimAngle[1]);
  USART_OUT(DEBUG_USART,"realpos ");
	USART_OUT_F(gRobot.holdBallAngle[1]);
	USART_Enter();
	
	/*返回舵机三的状态*/
  USART_OUT(DEBUG_USART,"steer 3 aimAngle ");
	USART_OUT_F(gRobot.sDta.cameraAimAngle);
  USART_OUT(DEBUG_USART,"realpos ");
	USART_OUT_F(gRobot.cameraAngle);
	USART_Enter();
	
	/*返回航向角的状态*/
  USART_OUT(DEBUG_USART,"course aimAngle ");
	USART_OUT_F(gRobot.sDta.courseAimAngle);
  USART_OUT(DEBUG_USART,"realpos ");
	USART_OUT_F(gRobot.courseAngle);
	USART_Enter();
	
	/*返回俯仰角的状态*/
  USART_OUT(DEBUG_USART,"course aimAngle ");
	USART_OUT_F(gRobot.sDta.pitchAimAngle);
  USART_OUT(DEBUG_USART,"realpos ");
	USART_OUT_F(gRobot.pitchAngle);
	USART_Enter();
  
  USART_OUT(DEBUG_USART,"gasValue aim ");
  USART_OUT_F(gRobot.sDta.gasAimValue);
  USART_OUT(DEBUG_USART,"\treal ");
  USART_OUT_F(gRobot.gasValue);
  USART_OUT(DEBUG_USART,"\r\n");
}

void processReponse(void)
{
	
  switch(gRobot.sDta.robocon2018)
  {
  case ROBOT_PREPARE:
    USART_OUT(DEBUG_USART,"ROBOT_PREPARE\t");
	
  case ROBOT_START:
    USART_OUT(DEBUG_USART,"ROBOT_START\t");
	}
  switch(gRobot.sDta.process)
  {
  case TO_START:
    USART_OUT(DEBUG_USART,"TO_START\t");
    
    break;
  case TO_GET_BALL_1:
    USART_OUT(DEBUG_USART,"TO_GET_BALL_1\t");
    
    break;
  case TO_THE_AREA_1:
    USART_OUT(DEBUG_USART,"TO_THE_AREA_1\t");
    
    break;
  case TO_THROW_BALL_1:
    USART_OUT(DEBUG_USART,"TO_THROW_BALL_1\t");
    
    break;
  case TO_GET_BALL_2:
    USART_OUT(DEBUG_USART,"TO_GET_BALL_2\t");
    
    break;
  case TO_THE_AREA_2:
    USART_OUT(DEBUG_USART,"TO_THE_AREA_2\t");
    
    break;
  case TO_THROW_BALL_2:
    USART_OUT(DEBUG_USART,"TO_THROW_BALL_2\t");
    
    break;
  case TO_GET_BALL_3:
    USART_OUT(DEBUG_USART,"TO_GET_BALL_3\t");
    
    break;
  case TO_THE_AREA_3:
    USART_OUT(DEBUG_USART,"TO_THE_AREA_3\t");
    
    break;
  case TO_THROW_BALL_3:
    USART_OUT(DEBUG_USART,"TO_THROW_BALL_3\t");
    
    break;
  case END_COMPETE:
    USART_OUT(DEBUG_USART,"END_COMPETE\t");
    
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
    USART_OUT(DEBUG_USART,"TO_START\t");
    break;
  case TO_GET_BALL_1:
    USART_OUT(DEBUG_USART,"TO_GET_BALL_1\t");
    USART_Enter();
    break;
  case TO_THE_AREA_1:
    USART_OUT(DEBUG_USART,"TO_THE_AREA_1\t");
    USART_Enter();
    break;
  case TO_THROW_BALL_1:
    USART_OUT(DEBUG_USART,"TO_THROW_BALL_1\t");
    USART_Enter();
    break;
  case TO_GET_BALL_2:
    USART_OUT(DEBUG_USART,"TO_GET_BALL_2\t");
    USART_Enter();
    break;
  case TO_THE_AREA_2:
    USART_OUT(DEBUG_USART,"TO_THE_AREA_2\t");
    USART_Enter();
    break;
  case TO_THROW_BALL_2:
    USART_OUT(DEBUG_USART,"TO_THROW_BALL_2\t");
    USART_Enter();
    break;
  case TO_GET_BALL_3:
    USART_OUT(DEBUG_USART,"TO_GET_BALL_3\t");
    USART_Enter();
    break;
  case TO_THE_AREA_3:
    USART_OUT(DEBUG_USART,"TO_THE_AREA_3\t");
    USART_Enter();
    break;
  case TO_THROW_BALL_3:
    USART_OUT(DEBUG_USART,"TO_THROW_BALL_3\t");
    USART_Enter();
    break;
  case END_COMPETE:
    USART_OUT(DEBUG_USART,"END_COMPETE\t");
    USART_Enter();
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
//        //USART_OUT(DEBUG_USART,"DELAY_HOLD_BALL_1_CHECK_POS FAIL\r\n");
//        ErrorRecord(HOLD_BALL_1_ROTATE_FAIL);
//      }
//      else
//      {
//        USART_OUT(DEBUG_USART,"DELAY_HOLD_BALL_1_CHECK_POS SUCCESS\r\n");
//      }
//      delayMs[0]=0;
//    }
//    else{
//      //			USART_OUT_F(fabs(gRobot.steerAimPos[0][1]-gRobot.steerPos[0]));
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
//        //USART_OUT(DEBUG_USART,"DELAY_HOLD_BALL_2_CHECK_POS FAIL\r\n");
//        ErrorRecord(HOLD_BALL_2_ROTATE_FAIL);
//      }
//      else
//      {
//        USART_OUT(DEBUG_USART,"DELAY_HOLD_BALL_2_CHECK_POS SUCCESS\r\n");
//      }
//      delayMs[1]=0;
//    }
//    else{
//      //			USART_OUT_F(fabs(gRobot.steerAimPos[1][1]-gRobot.steerPos[1]));
//      //			USART_Enter();
//    }
//  }
//}

void RobotSelfTest(void){
	
	MotionCardCMDSend(NOTIFY_MOTIONCARD_SELFTEST);
	static int selfTestStep=0;
	ShootLedOn();
	Delay_ms(500);
	ShootLedOff();
	Delay_ms(500);
	ShootLedOn();
	Delay_ms(500);
	ShootLedOff();
	switch(selfTestStep){
		//对电机，舵机的自检
		case 0:
			//俯仰角到-15度
			PitchAngleMotion(-15.f);
			Delay_ms(1500);
			//俯仰角到20度
			PitchAngleMotion(15.f);
			Delay_ms(1500);
			PitchAngleMotion(0.f);
			Delay_ms(1500);
		
			//航向90度
			CourseAngleMotion(90.f);
			Delay_ms(1500);
			CourseAngleMotion(180.f);
			Delay_ms(1500);
			CourseAngleMotion(0.f);
			Delay_ms(2500);
		
		  //上下两个舵机到0
			HoldBallPosCrlSeparate(0.f,0.f,1000);
			Delay_ms(1500);
			HoldBallPosCrlSeparate(90.f,90.f,1000);
			Delay_ms(1500);
			HoldBallPosCrlSeparate(-90.f,-90.f,1000);
			
			selfTestStep++;
		break;
		
		//对各个气阀的自检
		case 1:
			//爪子张开
			ClawOpen();
			Delay_ms(1000);
			ClawShut();
			Delay_ms(1000);
		
		  //射球两个气阀
			ShootSmallOpen();
      ShootBigOpen();
			Delay_ms(1000);
			ShootSmallShut();
      ShootBigShut();
			Delay_ms(1000);
		
			//助推车的气阀
			BoostPolePush();
			Delay_ms(1000);
			//助推车的气阀
			BoostPoleReturn();
		
			//金球架抓取气阀
			GoldBallGraspStairOneOn();
			GoldBallGraspStairTwoOn();
			Delay_ms(1500);
			//金球架抓取气阀
			GoldBallGraspStairOneOff();
			GoldBallGraspStairTwoOff();
		  Delay_ms(1500);
		  selfTestStep++;
		break;
		
		//气压检测
		case 2:
			GasMotion(0.500);
			USART_OUT(DEBUG_USART,"gasValue\t");
			USART_OUT_F(gRobot.gasValue);
			USART_Enter();
			Delay_ms(3000);
			USART_OUT(DEBUG_USART,"gasValue\t");
			USART_OUT_F(gRobot.gasValue);
			USART_Enter();
			GasMotion(0.430);
			Delay_ms(3000);
			USART_OUT(DEBUG_USART,"gasValue\t");
			USART_OUT_F(gRobot.gasValue);
			USART_Enter();
			
			selfTestStep++;
		break;
		
		//自动车轮子检测
		case 3:
			MotionCardCMDSend(NOTIFY_MOTIONCARD_SELFTEST_THE_WHEEL);
			if(){
				selfTestStep++;
			}
		break;
		
		case 4:
			selfTestStep++;
		break;
		
		
	}
	
}