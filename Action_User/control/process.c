#include "task.h"
#include "timer.h"
#include "gpio.h"

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
