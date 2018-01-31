#include "shoot.h"
#include "task.h"
#include "timer.h"
#include "steer.h"


extern Robot_t gRobot;

/*储存射球和投球的参数*/
motionPara_t PrepareCompete;
motionPara_t PrepareGetBall1;
motionPara_t PrepareGetBall2;
motionPara_t PrepareGetBall3;
motionPara_t PrepareShootBall1;
motionPara_t PrepareShootBall2;
motionPara_t PrepareShootBall3;

void PitchAngleMotion(float angle)
{
  /*控制的为-20°-10°实际给电机的是30°-0°*/
  if(angle>10.f)
    angle=10.f;
  else if(angle<-20.f)
    angle=-20.f;
	
	angle=10.f-angle;
	
  PosCrl(CAN2, 5,ABSOLUTE_MODE,PITCH_ANGLE_TO_CODE(angle));
}

void CourseAngleMotion(float angle)
{
  if(angle<0.f)
    angle=0.f;
  else if(angle>190.f)
    angle=190.f;
	
	angle=angle+10.6f;
	
  PosCrl(CAN2, 6,ABSOLUTE_MODE,COURSE_ANGLE_TO_CODE(angle));
}

void GasMotion(float value)
{
  CAN_TxMsg(CAN2,SEND_TO_GASSENSOR,(uint8_t*)(&value),4);
}

void ShootBall(void)
{				
  /*夹子打开*/
  ClawOpen();
  /*进行适当延时保证夹子和球不干涉*/
  Delay_ms(1000);
  /*把两个发射气缸打开*/
  ShootBigOpen();
}

void ShootReset(void)
{
  /*复位*/
  ShootSmallShut();
  ShootBigShut();
}

void prepareMotionParaInit(void)
{
	/*准备区动作*/
  PrepareCompete.courseAngle=0.0f;
  PrepareCompete.pitchAngle=-20.0f;
  PrepareCompete.steerAngle=-85.f;
  PrepareCompete.steerSpeed=2000;
  PrepareCompete.gasAim=0.55f;
  /*准备去拿第一个球的数据*/
  PrepareGetBall1.courseAngle=0.0f;
  PrepareGetBall1.pitchAngle=0.0f;
  PrepareGetBall1.steerAngle=0.f;
  PrepareGetBall1.steerSpeed=2000;
  PrepareGetBall1.gasAim=0.55f;
  
  /*准备去拿第二个球的数据*/
  PrepareGetBall2.courseAngle=90.f;
  PrepareGetBall2.pitchAngle=3.1f;
  PrepareGetBall2.steerAngle=85.f;
  PrepareGetBall2.steerSpeed=2000;
  PrepareGetBall2.gasAim=0.55f;
  
  /*准备去拿第三个球的数据*/
  PrepareGetBall3.courseAngle=0.0f;
  PrepareGetBall3.pitchAngle=0.0f;
  PrepareGetBall3.steerAngle=0.f;
  PrepareGetBall3.steerSpeed=2000;
  PrepareGetBall3.gasAim=0.6f;
  
  /*准备射第一个球的数据*/
  PrepareShootBall1.courseAngle=180.0f;
  PrepareShootBall1.pitchAngle=0.0f;
  PrepareShootBall1.steerAngle=0.f;
  PrepareShootBall1.steerSpeed=2000;
  PrepareShootBall1.gasAim=0.55f;
  
  /*准备射第二个球的数据*/
  PrepareShootBall2.courseAngle=180.0f;
  PrepareShootBall2.pitchAngle=0.0f;
  PrepareShootBall2.steerAngle=0.f;
  PrepareShootBall2.steerSpeed=2000;
  PrepareShootBall2.gasAim=0.55f;
  
  /*准备射第三个球的数据*/
  PrepareShootBall3.courseAngle=180.0f;
  PrepareShootBall3.pitchAngle=-4.8f;
  PrepareShootBall3.steerAngle=0.f;
  PrepareShootBall3.steerSpeed=2000;
  PrepareShootBall3.gasAim=0.56f;
  
}
void PrepareGetBallMotion(motionPara_t PrepareGetBall_t)
{
	
	/*更新目标参数（不能在函数中更新,容易出现迭代更新的风险）*/
	gRobot.courseAimAngle=PrepareGetBall_t.courseAngle;
	gRobot.pitchAimAngle=PrepareGetBall_t.pitchAngle;
	gRobot.gasAimValue=PrepareGetBall_t.gasAim;
	#ifdef TEST
	gRobot.holdBallAimAngle[0]=gRobot.holdBallAimAngle[1]=PrepareGetBall_t.steerAngle;
	#else
	gRobot.holdBallAimAngle=PrepareGetBall_t.steerAngle;
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
	
	/*更新目标参数（不能在函数中更新,容易出现迭代更新的风险）*/
	gRobot.courseAimAngle=PrepareShootBall_t.courseAngle;
	gRobot.pitchAimAngle=PrepareShootBall_t.pitchAngle;
	gRobot.gasAimValue=PrepareShootBall_t.gasAim;
	#ifdef TEST
	gRobot.holdBallAimAngle[0]=gRobot.holdBallAimAngle[1]=PrepareShootBall_t.steerAngle;
	#else
	gRobot.holdBallAimAngle=PrepareShootBall_t.steerAngle;
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
