#include "shoot.h"
#include "task.h"
#include "timer.h"
#include "steer.h"
#include "motion.h"
#include "includes.h"
#include "process.h"

extern OS_EVENT *PeriodSem;


extern Robot_t gRobot;

/*储存射球和投球的参数*/
motionPara_t PrepareCompete;
motionPara_t PrepareGetBall1;
motionPara_t PrepareGetBall2;
motionPara_t PrepareGetBall3;
motionPara_t PrepareShootBall1;
motionPara_t PrepareShootBall2;
motionPara_t PrepareShootBall3;


void ShootBall(void)
{				
  CPU_INT08U  os_err;
  os_err = os_err;
	
  /*进行适当延时保证夹子和球不干涉*/
  Delay_ms(2000);
	ShootLedOn();
  /*夹子打开*/
  ClawOpen();
  Delay_ms(300);
  /*把两个发射气缸打开*/
  ShootBigOpen();
}

void ShootReset(void)
{
	ShootLedOff();
  /*复位*/
  ShootSmallShut();
  ShootBigShut();
}

void prepareMotionParaInit(void)
{
	/*准备区动作*/
  PrepareCompete.courseAngle=90.0f;
  PrepareCompete.pitchAngle=-20.0f;
  PrepareCompete.steerAngle=-90.f;
  PrepareCompete.steerSpeed=2000;
  PrepareCompete.gasAim=0.555f;
	
  /*准备去拿第一个球的数据*/ 
  PrepareGetBall1.courseAngle=63.5f;
  PrepareGetBall1.pitchAngle=3.7f;
  PrepareGetBall1.steerAngle=-65.4f;
  PrepareGetBall1.steerSpeed=2000;
  PrepareGetBall1.gasAim=0.555f;
  
  /*准备射第一个球的数据*/
  PrepareShootBall1.courseAngle=172.9f;
  PrepareShootBall1.pitchAngle=7.1f;
  PrepareShootBall1.steerAngle=0.f;
  PrepareShootBall1.steerSpeed=2000;
  PrepareShootBall1.gasAim=0.555f;
	
  /*准备去拿第二个球的数据*/
  PrepareGetBall2.courseAngle=90.f;
  PrepareGetBall2.pitchAngle=3.1f;
  PrepareGetBall2.steerAngle=90.f;
  PrepareGetBall2.steerSpeed=2000;
  PrepareGetBall2.gasAim=0.555f;
  
  /*准备射第二个球的数据*/
  PrepareShootBall2.courseAngle=174.9f;
  PrepareShootBall2.pitchAngle=3.6f;
  PrepareShootBall2.steerAngle=0.f;
  PrepareShootBall2.steerSpeed=2000;
  PrepareShootBall2.gasAim=0.555f;
  
  /*准备去拿第三个球的数据*/
  PrepareGetBall3.courseAngle=0.0f;
  PrepareGetBall3.pitchAngle=0.0f;
  PrepareGetBall3.steerAngle=0.f;
  PrepareGetBall3.steerSpeed=2000;
  PrepareGetBall3.gasAim=0.585f;
  
  /*准备射第三个球的数据*/
  PrepareShootBall3.courseAngle=181.4f;
  PrepareShootBall3.pitchAngle=-1.4f;
  PrepareShootBall3.steerAngle=0.f;
  PrepareShootBall3.steerSpeed=2000;
  PrepareShootBall3.gasAim=0.585f;
  
}
//
void PrepareGetBallMotion(motionPara_t PrepareGetBall_t)
{
	/*更新目标参数（不能在函数中更新,容易出现迭代更新的风险）*/
	gRobot.sDta.courseAimAngle=PrepareGetBall_t.courseAngle;
	gRobot.sDta.pitchAimAngle=PrepareGetBall_t.pitchAngle;
	gRobot.sDta.gasAimValue=PrepareGetBall_t.gasAim;
	#ifdef TEST
	gRobot.sDta.holdBallAimAngle[0]=gRobot.sDta.holdBallAimAngle[1]=PrepareGetBall_t.steerAngle;
	#else
	gRobot.sDta.holdBallAimAngle=PrepareGetBall_t.steerAngle;
	#endif
	
  //设置气压
  GasMotion(PrepareGetBall_t.gasAim);
  /*设置俯仰角度*/
  PitchAngleMotion(PrepareGetBall_t.pitchAngle);
  /*设置航向角度*/
  CourseAngleMotion(PrepareGetBall_t.courseAngle);
  /*关闭下方限位爪*/
  ClawShut();
  /*舵机转向*/
	#ifdef TEST
  HoldBallPosCrl(PrepareGetBall_t.steerAngle, PrepareGetBall_t.steerSpeed);
	#else
  HoldBallPosCrl(PrepareGetBall_t.steerAngle, PrepareGetBall_t.steerSpeed);
	#endif
}
void PrepareGetBall(int index)
{
  switch(index)
  {
  case READY:
		PrepareWork();
    //传入准备区的参数
    PrepareGetBallMotion(PrepareCompete);
    break;
  case BALL_1:
    //传入准备得到球的参数
    PrepareGetBallMotion(PrepareGetBall1);
    break;
  case BALL_2:
    //传入准备得到球的参数
    PrepareGetBallMotion(PrepareGetBall2);
    break;
  case BALL_3:
    //传入准备得到球的参数
    PrepareGetBallMotion(PrepareGetBall3);
    break;
  default:
    USART_OUT(DEBUG_USART,"PrepareGetBall error\r\n");
    break;
  }	
}

void PrepareShootBallMotion(motionPara_t PrepareShootBall_t)
{
  CPU_INT08U  os_err;
  os_err = os_err;
	
	/*更新目标参数（不能在函数中更新,容易出现迭代更新的风险）*/
	gRobot.sDta.courseAimAngle=PrepareShootBall_t.courseAngle;
	gRobot.sDta.pitchAimAngle=PrepareShootBall_t.pitchAngle;
	gRobot.sDta.gasAimValue=PrepareShootBall_t.gasAim;
	#ifdef TEST
	gRobot.sDta.holdBallAimAngle[0]=gRobot.sDta.holdBallAimAngle[1]=PrepareShootBall_t.steerAngle;
	#else
	gRobot.sDta.holdBallAimAngle=PrepareShootBall_t.steerAngle;
	#endif
	
  //设置气压
  GasMotion(PrepareShootBall_t.gasAim);
  /*设置俯仰角度*/
  PitchAngleMotion(PrepareShootBall_t.pitchAngle);
  /*设置航向角度*/
  CourseAngleMotion(PrepareShootBall_t.courseAngle);
  /*避免球乱晃*/
  Delay_ms(500);
  /*提前打开发射装置小气缸*/
  ShootSmallOpen();
  /*舵机转向*/
	#ifdef TEST
  HoldBallPosCrl( PrepareShootBall_t.steerAngle, PrepareShootBall_t.steerSpeed);
	#else
  HoldBallPosCrl( PrepareShootBall_t.steerAngle, PrepareShootBall_t.steerSpeed);
	#endif
}
void PrepareShootBall(int index)
{
  switch(index)
  {
  case BALL_1:
    //传入准备射球的参数
    PrepareShootBallMotion(PrepareShootBall1);
    break;
  case BALL_2:
    //传入准备射球的参数
    PrepareShootBallMotion(PrepareShootBall2);
    break;
  case BALL_3:
    //传入准备射球的参数
    PrepareShootBallMotion(PrepareShootBall3);
    break;
  default:
    USART_OUT(DEBUG_USART,"PrepareShootBall error\r\n");
    break;
  }	
}


/*初始化动作 先下降再旋转*/
void PrepareWork(void)
{
	int cnt=0;
	#ifndef TEST
		/*更新目标参数（不能在函数中更新,容易出现迭代更新的风险）*/
	gRobot.sDta.courseAimAngle=PrepareCompete.courseAngle;
	gRobot.sDta.pitchAimAngle=PrepareCompete.pitchAngle;
	gRobot.sDta.gasAimValue=PrepareCompete.gasAim;
	gRobot.sDta.holdBallAimAngle=PrepareCompete.steerAngle;
	
  /*关闭下方限位爪*/
  ClawShut();
  //设置气压
  GasMotion(PrepareCompete.gasAim);
  /*舵机转向*/
	while(1)
	{
		Delay_ms(5);
		cnt++;
		//确保一定转向了
		if(cnt>50)
			break;
		HoldBallPosCrl(PrepareCompete.steerAngle, PrepareCompete.steerSpeed);
	}
	/*设置俯仰角度*/
	PitchAngleMotion(PrepareCompete.pitchAngle);
	while(1)
	{
		Delay_ms(5);
		/*读取俯仰角*/
		ReadActualPos(CAN2,5);
			/*判断俯仰角是否到位*/
		if(fabs(gRobot.sDta.pitchAimAngle-gRobot.pitchAngle)<0.01f)
		{
			SetMotionFlag(AT_PITCH_SUCCESS);
			break;
		}
	}
	Delay_ms(2000);
  /*设置航向角度*/
  CourseAngleMotion(PrepareCompete.courseAngle);
	/*等待慢转动状态完成*/
  while(1)
	{
		Delay_ms(5);
		/*读取俯仰角*/
		ReadActualPos(CAN2,6);
			/*判断俯仰角是否到位*/
		if(fabs(gRobot.sDta.courseAimAngle-gRobot.courseAngle)<0.01f)
		{
			SetMotionFlag(AT_COURSE_SUCCESS);
			break;
		}
	}
	
	
	#endif
}
