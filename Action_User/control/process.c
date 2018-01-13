#include "task.h"
#include "timer.h"
#include "gpio.h"

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
