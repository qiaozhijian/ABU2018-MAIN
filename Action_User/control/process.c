#include "task.h"
#include "timer.h"
#include "gpio.h"
#include "process.h"
#include "stm32f4xx_it.h"
#include "steer.h"

extern Robot_t gRobot;

/*���Ͷ�����һ������*/
void FightForBall1(void)
{
  
  switch(gRobot.process)
  {
    /*ȥȡ��һ����*/
  case TO_GET_BALL_1:
    
    if(PE_FOR_THE_BALL)
    {	
      Delay_ms(500);
      
      MotionCardCMDSend(NOTIFY_MOTIONCARD_GOT_BALL1);
      
      PrepareShootBall(BALL_1);
      
      gRobot.process=TO_THE_AREA_1;
      
    }
    break;
    /*��һ����ȡ����ϣ�ȥͶ����һ*/
  case TO_THE_AREA_1:
    if(!PE_FOR_THE_BALL)
    {
      MotionCardCMDSend(NOTIFY_MOTIONCARD_LOSE_BALL1);
    }
    break;
    /*����Ͷ����һ������*/
  case TO_THROW_BALL_1:
    if(PE_FOR_THE_BALL)
    {
      /*����*/
      ShootBall();
      /*����ʱʹ�������ִ�е�λ*/
      Delay_ms(150);
      /*֪ͨ���ƿ�*/
      MotionCardCMDSend(NOTIFY_MOTIONCARD_SHOT_BALL1);
      /*���������λ*/
      ShootReset();
      /*׼�������*/
      PrepareGetBall(BALL_2);
      /*������һ״̬*/
      gRobot.process=TO_GET_BALL_2;
			gRobot.robocon2018=COLORFUL_BALL_2;
    }
    break;
  }
}

/*���Ͷ������������*/
void FightForBall2(void)
{
  switch(gRobot.process)
  {
    /*ȥȡ�ڶ�����*/
  case TO_GET_BALL_2:
    if(PE_FOR_THE_BALL)
    {	
      /*ɨ������Ϊ�˸��ȵؽӵ����������ʱ*/
      Delay_ms(500);
      
      gRobot.process=TO_THE_AREA_2;
      
      MotionCardCMDSend(NOTIFY_MOTIONCARD_GOT_BALL2);
      
      PrepareShootBall(BALL_2);
      
    }
    break;
    /*�ڶ�����ȡ����ϣ�ȥͶ������*/
  case TO_THE_AREA_2:
    if(!PE_FOR_THE_BALL)
    {
      MotionCardCMDSend(NOTIFY_MOTIONCARD_LOSE_BALL2);
    }
    break;
    /*����Ͷ������������*/
  case TO_THROW_BALL_2:
    if(PE_FOR_THE_BALL)
    {
      /*����*/
      ShootBall();
      
      /*����ʱʹ�������ִ�е�λ*/
      Delay_ms(150);
      
      MotionCardCMDSend(NOTIFY_MOTIONCARD_SHOT_BALL2);
      
      /*���������λ*/
      ShootReset();
      
      /*׼��������*/
      PrepareGetBall(BALL_3);
      
      gRobot.process=TO_GET_BALL_3;
			gRobot.robocon2018=GOLD_BALL;
    }
    break;
  }
  
}

/*���Ͷ����������*/
void FightForGoldBall(void)
{
  switch(gRobot.process)
  {
    /*ȥȡ��������*/
  case TO_GET_BALL_3:
    if(PE_FOR_THE_BALL)
    {	
      /*ɨ������Ϊ�˸��ȵؽӵ����������ʱ*/
      Delay_ms(500);
      
      gRobot.process=TO_THE_AREA_3;
      
      MotionCardCMDSend(NOTIFY_MOTIONCARD_GOT_BALL3);
      
      PrepareShootBall(BALL_3);
      
    }
    break;
    /*��������ȡ����ϣ�ȥͶ������*/
  case TO_THE_AREA_3:
    if(!PE_FOR_THE_BALL)
    {
      MotionCardCMDSend(NOTIFY_MOTIONCARD_LOSE_BALL3);
    }
    break;
    /*����Ͷ������������*/
  case TO_THROW_BALL_3:
    if(PE_FOR_THE_BALL)
    {
      /*����*/
      ShootBall();
      
      /*����ʱʹ�������ִ�е�λ*/
      Delay_ms(150);
      
      /*���������λ*/
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
	
	/*�����1��λ����û�е���*/
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
	
	/*�����2��λ����û�е���*/
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




