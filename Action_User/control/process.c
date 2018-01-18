#include "task.h"
#include "timer.h"
#include "gpio.h"
#include "process.h"
#include "stm32f4xx_it.h"
#include "steer.h"

extern Robot_t gRobot;

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
    if(PE_FOR_THE_BALL)
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
    if(PE_FOR_THE_BALL)
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
    if(PE_FOR_THE_BALL)
    {
      /*射球*/
      ShootBall();
      
      /*给延时使发射杆能执行到位*/
      Delay_ms(150);
      
      /*射球机构复位*/
      ShootReset();
      
      gRobot.process=END_COMPETE;
    }
    break;
  }
}

void DelayMsRun(char task,short ms)
{
	int index;
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
	int index;
	gRobot.delayTask&=~task;
	while(task!=1)
	{
		task>>=1;
		index++;
	}
	gRobot.delayTaskMs[index]=0;
}

void DelayTaskRun(void)
{
	static int delayMs[DELAY_TASK_NUM]={0};
	
	/*检测舵机1的位置有没有到达*/
	if(gRobot.delayTask&DELAY_STEER1_CHECK_POS)
	{
		delayMs[0]++;
		if(delayMs[0]*PERIOD_COUNTER>gRobot.delayTaskMs[0])
		{
			//4096/360=11.377
			if(abs(gRobot.steer_t.steerAimPos[0][1]-gRobot.steer_t.steerPos[0])>80*11.377f)
			{
				Steer1ROBS_PosCrl(gRobot.steer_t.steerAimPos[0][0],2000);
				USART_OUT_ONCE("DELAY_STEER1_CHECK_POS FAIL\r\n");
				SetMotionFlag(STEER1_ROTATE_FAIL);
			}
			else
			{
				USART_OUT_ONCE("DELAY_STEER1_CHECK_POS SUCCESS\r\n");
			}
			DelayStop(DELAY_STEER1_CHECK_POS);
			delayMs[0]=0;
		}
	}
	
	/*检测舵机2的位置有没有到达*/
	if(gRobot.delayTask&DELAY_STEER2_CHECK_POS)
	{
		delayMs[1]++;
		if(delayMs[1]*PERIOD_COUNTER>gRobot.delayTaskMs[1])
		{
			//4096/360=11.377
			if(abs(gRobot.steer_t.steerAimPos[1][1]-gRobot.steer_t.steerPos[1])>80*11.377f)
			{
				Steer1ROBS_PosCrl(gRobot.steer_t.steerAimPos[1][0],2000);
				USART_OUT_ONCE("DELAY_STEER2_CHECK_POS FAIL\r\n");
				SetMotionFlag(STEER2_ROTATE_FAIL);
			}
			else
			{
				USART_OUT_ONCE("DELAY_STEER2_CHECK_POS SUCCESS\r\n");
			}
			DelayStop(DELAY_STEER2_CHECK_POS);
			delayMs[1]=0;
		}
	}
	
	
}

void MotionStatus(void)
{
	ReadROBSAngle();
	USART_OUT(DEBUG_USART,"steer 1 aimpos %d\trealpos %d\r\n",gRobot.steer_t.steerAimPos[0],gRobot.steer_t.steerPos[0]);
	USART_OUT(DEBUG_USART,"steer 2 aimpos %d\trealpos %d\r\n",gRobot.steer_t.steerAimPos[1],gRobot.steer_t.steerPos[1]);
	
	while(!gRobot.motorPara_t.pitchReadSuccess)
		ReadActualPos(CAN2,5);
	gRobot.motorPara_t.pitchReadSuccess=0;
	
	while(!gRobot.motorPara_t.courseReadSuccess)
		ReadActualPos(CAN2,6);
	gRobot.motorPara_t.courseReadSuccess=0;
	
	USART_OUT(DEBUG_USART,"course aimpos %d\trealpos %d\r\n",gRobot.motorPara_t.courseAimPos,gRobot.motorPara_t.coursePos);
	USART_OUT(DEBUG_USART,"pitch aimpos %d\trealpos %d\r\n",gRobot.motorPara_t.pitchAimPos,gRobot.motorPara_t.pitchPos);
	
	USART_OUT(DEBUG_USART,"gasValue aim ");
	USART_OUT_F(gRobot.motorPara_t.gasAimValue);
	USART_OUT(DEBUG_USART,"\treal ");
	USART_OUT_F(gRobot.motorPara_t.gasValue);
	USART_OUT(DEBUG_USART,"\r\n");
}
void StatusReport(void)
{
	processReport();
	SteerErrorReport();
	MotionStatus();
}




