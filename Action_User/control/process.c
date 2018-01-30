#include "task.h"
#include "timer.h"
#include "gpio.h"
#include "process.h"
#include "stm32f4xx_it.h"
#include "steer.h"

extern Robot_t gRobot;

void SelfTest(void)
{
	static int step=0;
	switch(step)
	{
		case 0:
			BEEP_ON;
			break;
	}
}

/*完成投射彩球一的任务*/
void FightForBall1(void)
{
  
  switch(gRobot.process)
  {
    /*去取第一个球*/
  case TO_GET_BALL_1:
    
    if(PE_FOR_THE_BALL)
    {	
      Delay_ms(500);
      
      MotionCardCMDSend(NOTIFY_MOTIONCARD_GOT_BALL1);
      
			TalkToCamera(CAMERA_OPEN_NEAR);
			
      PrepareShootBall(BALL_1);
      
      gRobot.process=TO_THE_AREA_1;
      
    }
    break;
    /*第一个球取球完毕，去投射区一*/
  case TO_THE_AREA_1:
    if(!PE_FOR_THE_BALL)
    {
      MotionCardCMDSend(NOTIFY_MOTIONCARD_LOSE_BALL1);
    }
    break;
    /*到达投射区一，射球*/
  case TO_THROW_BALL_1:
		/*光电到位*/
    if(PE_FOR_THE_BALL
				/*持球舵机到位*/
				&&(gRobot.AT_motionFlag&AT_HOLD_BALL1_SUCCESS)
					/*持球舵机到位*/
					&&(gRobot.AT_motionFlag&AT_HOLD_BALL2_SUCCESS)
						/*俯仰到位，*/
						&&(gRobot.AT_motionFlag&AT_PITCH_SUCCESS)
							/*航向到位*/
							&&(gRobot.AT_motionFlag&AT_COURSE_SUCCESS))
    {
      /*射球*/
      ShootBall();
      /*给延时使发射杆能执行到位*/
      Delay_ms(150);
      /*通知控制卡*/
      MotionCardCMDSend(NOTIFY_MOTIONCARD_SHOT_BALL1);
      /*射球机构复位*/
      ShootReset();
      /*准备接球二*/
      PrepareGetBall(BALL_2);
      /*进入下一状态*/
      gRobot.process=TO_GET_BALL_2;
      gRobot.robocon2018=COLORFUL_BALL_2;
    }
		else
		{
			USART_OUT(DEBUG_USART,"THROW1\t");
			if(!PE_FOR_THE_BALL)
				USART_OUT(DEBUG_USART,"!PE\t");
			if(!(gRobot.AT_motionFlag&AT_HOLD_BALL1_SUCCESS))
				USART_OUT(DEBUG_USART,"!HB1\t");
			if(!(gRobot.AT_motionFlag&AT_HOLD_BALL2_SUCCESS))
				USART_OUT(DEBUG_USART,"!HB2\t");
			if(!(gRobot.AT_motionFlag&AT_PITCH_SUCCESS))
				USART_OUT(DEBUG_USART,"!PITCH\t");
			if(!(gRobot.AT_motionFlag&AT_COURSE_SUCCESS))
				USART_OUT(DEBUG_USART,"!COURSE\t");
			USART_Enter();
		}
    break;
  }
}

/*完成投射彩球二的任务*/
void FightForBall2(void)
{
  switch(gRobot.process)
  {
    /*去取第二个球*/
  case TO_GET_BALL_2:
    if(PE_FOR_THE_BALL)
    {	
      /*扫到光电后，为了更稳地接到球而给的延时*/
      Delay_ms(500);
      
      gRobot.process=TO_THE_AREA_2;
      
      MotionCardCMDSend(NOTIFY_MOTIONCARD_GOT_BALL2);
      
      PrepareShootBall(BALL_2);
      
    }
    break;
    /*第二个球取球完毕，去投射区二*/
  case TO_THE_AREA_2:
    if(!PE_FOR_THE_BALL)
    {
      MotionCardCMDSend(NOTIFY_MOTIONCARD_LOSE_BALL2);
    }
    break;
    /*到达投射区二，射球*/
  case TO_THROW_BALL_2:
		/*光电到位*/
    if(PE_FOR_THE_BALL
				/*持球舵机到位*/
				&&(gRobot.AT_motionFlag&AT_HOLD_BALL1_SUCCESS)
					/*持球舵机到位*/
					&&(gRobot.AT_motionFlag&AT_HOLD_BALL2_SUCCESS)
						/*俯仰到位，*/
						&&(gRobot.AT_motionFlag&AT_PITCH_SUCCESS)
							/*航向到位*/
							&&(gRobot.AT_motionFlag&AT_COURSE_SUCCESS))
    {
      /*射球*/
      ShootBall();
      
      /*给延时使发射杆能执行到位*/
      Delay_ms(150);
      
      MotionCardCMDSend(NOTIFY_MOTIONCARD_SHOT_BALL2);
      
      /*射球机构复位*/
      ShootReset();
      
      /*准备接球三*/
      PrepareGetBall(BALL_3);
      
      gRobot.process=TO_GET_BALL_3;
      gRobot.robocon2018=GOLD_BALL;
    }
		else
		{
			USART_OUT(DEBUG_USART,"THROW2\t");
			if(!PE_FOR_THE_BALL)
				USART_OUT(DEBUG_USART,"!PE\t");
			if(!(gRobot.AT_motionFlag&AT_HOLD_BALL1_SUCCESS))
				USART_OUT(DEBUG_USART,"!HB1\t");
			if(!(gRobot.AT_motionFlag&AT_HOLD_BALL2_SUCCESS))
				USART_OUT(DEBUG_USART,"!HB2\t");
			if(!(gRobot.AT_motionFlag&AT_PITCH_SUCCESS))
				USART_OUT(DEBUG_USART,"!PITCH\t");
			if(!(gRobot.AT_motionFlag&AT_COURSE_SUCCESS))
				USART_OUT(DEBUG_USART,"!COURSE\t");
			USART_Enter();
		}
    break;
  }
  
}

/*完成投射金球的任务*/
void FightForGoldBall(void)
{
  switch(gRobot.process)
  {
    /*去取第三个球*/
  case TO_GET_BALL_3:
    if(PE_FOR_THE_BALL)
    {	
      /*扫到光电后，为了更稳地接到球而给的延时*/
      Delay_ms(500);
      
			TalkToCamera(CAMERA_OPEN_FAR);
      
      gRobot.process=TO_THE_AREA_3;
      
      MotionCardCMDSend(NOTIFY_MOTIONCARD_GOT_BALL3);
      
      PrepareShootBall(BALL_3);
      
    }
    break;
    /*第三个球取球完毕，去投射区三*/
  case TO_THE_AREA_3:
    if(!PE_FOR_THE_BALL)
    {
      MotionCardCMDSend(NOTIFY_MOTIONCARD_LOSE_BALL3);
    }
    break;
    /*到达投射区三，射球*/
  case TO_THROW_BALL_3:
		/*光电到位*/
    if(PE_FOR_THE_BALL
				/*持球舵机到位*/
				&&(gRobot.AT_motionFlag&AT_HOLD_BALL1_SUCCESS)
					/*持球舵机到位*/
					&&(gRobot.AT_motionFlag&AT_HOLD_BALL2_SUCCESS)
						/*俯仰到位，*/
						&&(gRobot.AT_motionFlag&AT_PITCH_SUCCESS)
							/*航向到位*/
							&&(gRobot.AT_motionFlag&AT_COURSE_SUCCESS))
    {
      /*射球*/
      ShootBall();
      
      /*给延时使发射杆能执行到位*/
      Delay_ms(150);
      
      /*射球机构复位*/
      ShootReset();
      
      gRobot.process=END_COMPETE;
    }
		else
		{
			USART_OUT(DEBUG_USART,"THROW3\t");
			if(!PE_FOR_THE_BALL)
				USART_OUT(DEBUG_USART,"!PE\t");
			if(!(gRobot.AT_motionFlag&AT_HOLD_BALL1_SUCCESS))
				USART_OUT(DEBUG_USART,"!HB1\t");
			if(!(gRobot.AT_motionFlag&AT_HOLD_BALL2_SUCCESS))
				USART_OUT(DEBUG_USART,"!HB2\t");
			if(!(gRobot.AT_motionFlag&AT_PITCH_SUCCESS))
				USART_OUT(DEBUG_USART,"!PITCH\t");
			if(!(gRobot.AT_motionFlag&AT_COURSE_SUCCESS))
				USART_OUT(DEBUG_USART,"!COURSE\t");
			USART_Enter();
		}
    break;
  }
}


void MotionStatus(void)
{
	/*返回舵机一的状态*/
  USART_OUT(DEBUG_USART,"steer 1 aimAngle ");
	USART_OUT_F(gRobot.holdBallAimAngle);
  USART_OUT(DEBUG_USART,"realpos ");
	USART_OUT_F(gRobot.holdBallAngle[0]);
	USART_Enter();
	
	/*返回舵机二的状态*/
  USART_OUT(DEBUG_USART,"steer 2 aimAngle ");
	USART_OUT_F(gRobot.holdBallAimAngle);
  USART_OUT(DEBUG_USART,"realpos ");
	USART_OUT_F(gRobot.holdBallAngle[1]);
	USART_Enter();
	
	/*返回舵机三的状态*/
  USART_OUT(DEBUG_USART,"steer 3 aimAngle ");
	USART_OUT_F(gRobot.cameraAimAngle);
  USART_OUT(DEBUG_USART,"realpos ");
	USART_OUT_F(gRobot.cameraAngle);
	USART_Enter();
	
	/*返回航向角的状态*/
  USART_OUT(DEBUG_USART,"course aimAngle ");
	USART_OUT_F(gRobot.courseAimAngle);
  USART_OUT(DEBUG_USART,"realpos ");
	USART_OUT_F(gRobot.courseAngle);
	USART_Enter();
	
	/*返回俯仰角的状态*/
  USART_OUT(DEBUG_USART,"course aimAngle ");
	USART_OUT_F(gRobot.pitchAimAngle);
  USART_OUT(DEBUG_USART,"realpos ");
	USART_OUT_F(gRobot.pitchAngle);
	USART_Enter();
  
  USART_OUT(DEBUG_USART,"gasValue aim ");
  USART_OUT_F(gRobot.gasAimValue);
  USART_OUT(DEBUG_USART,"\treal ");
  USART_OUT_F(gRobot.gasValue);
  USART_OUT(DEBUG_USART,"\r\n");
}

void processReponse(void)
{
  switch(gRobot.process)
  {
  case TO_START:
    USART_OUT(DEBUG_USART,"TO_START");
    USART_Enter();
    break;
  case TO_GET_BALL_1:
    USART_OUT(DEBUG_USART,"TO_GET_BALL_1");
    USART_Enter();
    break;
  case TO_THE_AREA_1:
    USART_OUT(DEBUG_USART,"TO_THE_AREA_1");
    USART_Enter();
    break;
  case TO_THROW_BALL_1:
    USART_OUT(DEBUG_USART,"TO_THROW_BALL_1");
    USART_Enter();
    break;
  case TO_GET_BALL_2:
    USART_OUT(DEBUG_USART,"TO_GET_BALL_2");
    USART_Enter();
    break;
  case TO_THE_AREA_2:
    USART_OUT(DEBUG_USART,"TO_THE_AREA_2");
    USART_Enter();
    break;
  case TO_THROW_BALL_2:
    USART_OUT(DEBUG_USART,"TO_THROW_BALL_2");
    USART_Enter();
    break;
  case TO_GET_BALL_3:
    USART_OUT(DEBUG_USART,"TO_GET_BALL_3");
    USART_Enter();
    break;
  case TO_THE_AREA_3:
    USART_OUT(DEBUG_USART,"TO_THE_AREA_3");
    USART_Enter();
    break;
  case TO_THROW_BALL_3:
    USART_OUT(DEBUG_USART,"TO_THROW_BALL_3");
    USART_Enter();
    break;
  case END_COMPETE:
    USART_OUT(DEBUG_USART,"END_COMPETE");
    USART_Enter();
    break;
  }
}

void SteerErrorReport(void)
{
  for(int i=0;i<gRobot.errorTime;i++)
  {
    switch(gRobot.error[i][0])
    {
    case HOLD_BALL1_ENABLE_FAIL:
      USART_OUT(DEBUG_USART,"HOLD_BALL1_ENABLE_FAIL\t");
      break;
    case HOLD_BALL1_ROTATE_FAIL:
      USART_OUT(DEBUG_USART,"HOLD_BALL1_ROTATE_FAIL\t");
      break;
    case HOLD_BALL2_ENABLE_FAIL:
      USART_OUT(DEBUG_USART,"HOLD_BALL2_ENABLE_FAIL\t");
      break;
    case HOLD_BALL2_ROTATE_FAIL:
      USART_OUT(DEBUG_USART,"HOLD_BALL2_ROTATE_FAIL\t");
      break;
    case HOLD_BALL1_ROTATE_SEND_FAIL:
      USART_OUT(DEBUG_USART,"HOLD_BALL1_ROTATE_SEND_FAIL\t");
      break;
    case HOLD_BALL2_ROTATE_SEND_FAIL:
      USART_OUT(DEBUG_USART,"HOLD_BALL2_ROTATE_SEND_FAIL\t");
      break;
    case CAN1_FAIL:
      USART_OUT(DEBUG_USART,"CAN1_FAIL\t");
      break;
    case CAN2_FAIL:
      USART_OUT(DEBUG_USART,"CAN2_FAIL\t");
      break;
    }
    switch(gRobot.error[i][1])
    {
    case TO_START:
      USART_OUT(DEBUG_USART,"TO_START");
      USART_Enter();
      break;
    case TO_GET_BALL_1:
      USART_OUT(DEBUG_USART,"TO_GET_BALL_1");
      USART_Enter();
      break;
    case TO_THE_AREA_1:
      USART_OUT(DEBUG_USART,"TO_THE_AREA_1");
      USART_Enter();
      break;
    case TO_THROW_BALL_1:
      USART_OUT(DEBUG_USART,"TO_THROW_BALL_1");
      USART_Enter();
      break;
    case TO_GET_BALL_2:
      USART_OUT(DEBUG_USART,"TO_GET_BALL_2");
      USART_Enter();
      break;
    case TO_THE_AREA_2:
      USART_OUT(DEBUG_USART,"TO_THE_AREA_2");
      USART_Enter();
      break;
    case TO_THROW_BALL_2:
      USART_OUT(DEBUG_USART,"TO_THROW_BALL_2");
      USART_Enter();
      break;
    case TO_GET_BALL_3:
      USART_OUT(DEBUG_USART,"TO_GET_BALL_3");
      USART_Enter();
      break;
    case TO_THE_AREA_3:
      USART_OUT(DEBUG_USART,"TO_THE_AREA_3");
      USART_Enter();
      break;
    case TO_THROW_BALL_3:
      USART_OUT(DEBUG_USART,"TO_THROW_BALL_3");
      USART_Enter();
      break;
    case END_COMPETE:
      USART_OUT(DEBUG_USART,"END_COMPETE");
      USART_Enter();
      break;
    default:
      USART_OUT(DEBUG_USART,"%d",gRobot.error[i][1]);
      USART_Enter();
      break;
    }
  }
}

void SteerErrorRecord(char type)
{
  int i=0;
  while(gRobot.error[i][0]!=type||gRobot.error[i][1]!=gRobot.process)
  {
    i++;
    if(i==STEER_ERROR_TIME)
    {
      gRobot.error[gRobot.errorTime][0]=type;
      if(gRobot.error[gRobot.errorTime][0]==CAN1_FAIL)
        gRobot.error[gRobot.errorTime][1]=CAN_GetLastErrorCode(CAN1);
      else if(gRobot.error[gRobot.errorTime][0]==CAN2_FAIL)
        gRobot.error[gRobot.errorTime][1]=CAN_GetLastErrorCode(CAN2);
      else
        gRobot.error[gRobot.errorTime][1]=gRobot.process;
      gRobot.errorTime++;
      break;
    }
  }
}

void StatusReport(void)
{
  processReponse();
  SteerErrorReport();
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
  
  if(gRobot.process==processLast)
    return;
  
  switch(gRobot.process)
  {
  case TO_START:
    USART_OUT(DEBUG_USART,"TO_START");
    USART_Enter();
    break;
  case TO_GET_BALL_1:
    USART_OUT(DEBUG_USART,"TO_GET_BALL_1");
    USART_Enter();
    break;
  case TO_THE_AREA_1:
    USART_OUT(DEBUG_USART,"TO_THE_AREA_1");
    USART_Enter();
    break;
  case TO_THROW_BALL_1:
    USART_OUT(DEBUG_USART,"TO_THROW_BALL_1");
    USART_Enter();
    break;
  case TO_GET_BALL_2:
    USART_OUT(DEBUG_USART,"TO_GET_BALL_2");
    USART_Enter();
    break;
  case TO_THE_AREA_2:
    USART_OUT(DEBUG_USART,"TO_THE_AREA_2");
    USART_Enter();
    break;
  case TO_THROW_BALL_2:
    USART_OUT(DEBUG_USART,"TO_THROW_BALL_2");
    USART_Enter();
    break;
  case TO_GET_BALL_3:
    USART_OUT(DEBUG_USART,"TO_GET_BALL_3");
    USART_Enter();
    break;
  case TO_THE_AREA_3:
    USART_OUT(DEBUG_USART,"TO_THE_AREA_3");
    USART_Enter();
    break;
  case TO_THROW_BALL_3:
    USART_OUT(DEBUG_USART,"TO_THROW_BALL_3");
    USART_Enter();
    break;
  case END_COMPETE:
    USART_OUT(DEBUG_USART,"END_COMPETE");
    USART_Enter();
    break;
  }
  processLast=gRobot.process;
}
//void DelayTaskRun(void)
//{
//  static int delayMs[DELAY_TASK_NUM]={0};
//  
//  /*检测舵机1的位置有没有到达*/
//  if(gRobot.delayTask&DELAY_HOLD_BALL1_CHECK_POS)
//  {
//    ReadSteer1Pos();
//    delayMs[0]++;
//    if(delayMs[0]*PERIOD_COUNTER>gRobot.delayTaskMs[0])
//    {
//      DelayStop(DELAY_HOLD_BALL1_CHECK_POS);
//      //4096/360=11.377
//      if(abs(gRobot.steerAimPos[0][1]-gRobot.steerPos[0])>2*11.377f)
//      {
//        Steer1HoldBallPosCrl(gRobot.steerAimPos[0][0],2000);
//        //USART_OUT(DEBUG_USART,"DELAY_HOLD_BALL1_CHECK_POS FAIL\r\n");
//        SteerErrorRecord(HOLD_BALL1_ROTATE_FAIL);
//      }
//      else
//      {
//        USART_OUT(DEBUG_USART,"DELAY_HOLD_BALL1_CHECK_POS SUCCESS\r\n");
//      }
//      delayMs[0]=0;
//    }
//    else{
//      //			USART_OUT_F(abs(gRobot.steerAimPos[0][1]-gRobot.steerPos[0]));
//    }
//  }
//  
//  /*检测舵机2的位置有没有到达*/
//  if(gRobot.delayTask&DELAY_HOLD_BALL2_CHECK_POS)
//  {
//    ReadSteer2Pos();
//    delayMs[1]++;
//    if(delayMs[1]*PERIOD_COUNTER>gRobot.delayTaskMs[1])
//    {
//      DelayStop(DELAY_HOLD_BALL2_CHECK_POS);
//      //4096/360=11.377
//      if(abs(gRobot.steerAimPos[1][1]-gRobot.steerPos[1])>2*11.377f)
//      {
//        Steer2HoldBallPosCrl(gRobot.steerAimPos[1][0],2000);
//        //USART_OUT(DEBUG_USART,"DELAY_HOLD_BALL2_CHECK_POS FAIL\r\n");
//        SteerErrorRecord(HOLD_BALL2_ROTATE_FAIL);
//      }
//      else
//      {
//        USART_OUT(DEBUG_USART,"DELAY_HOLD_BALL2_CHECK_POS SUCCESS\r\n");
//      }
//      delayMs[1]=0;
//    }
//    else{
//      //			USART_OUT_F(abs(gRobot.steerAimPos[1][1]-gRobot.steerPos[1]));
//      //			USART_Enter();
//    }
//  }
//}

