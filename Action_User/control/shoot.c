#include "shoot.h"
#include "task.h"
#include "timer.h"
#include "steer.h"
#include "motion.h"
#include "includes.h"
#include "process.h"
#include "robot.h"
#include "dma.h"

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
  /*将下爪手臂气缸上抬*/
  LowerClawStairOff();
	ShootLedOn();
  /*夹子打开*/
  ClawOpen();
  Delay_ms(200);
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
  PrepareCompete.pitchAngle=19.0f;
  PrepareCompete.upSteerAngle=-90.f;
	PrepareCompete.downSteerAngle=-90.f;
  PrepareCompete.steerSpeed=2000;
  PrepareCompete.gasAim=0.480f;
	
  /*准备去拿第一个球的数据*/ 
  PrepareGetBall1.courseAngle=64.0f;
  PrepareGetBall1.pitchAngle=0.5f;
  PrepareGetBall1.upSteerAngle=-62.0f;
	PrepareGetBall1.downSteerAngle=-65.0f;
  PrepareGetBall1.steerSpeed=2000;
  PrepareGetBall1.gasAim=0.480f;
  
  /*准备射第一个球的数据*/
  PrepareShootBall1.courseAngle=171.5f;
  PrepareShootBall1.pitchAngle=7.7f;
  PrepareShootBall1.upSteerAngle=0.f;
	PrepareShootBall1.downSteerAngle=0.f;
  PrepareShootBall1.steerSpeed=2000;
  PrepareShootBall1.gasAim=0.480f;
	
  /*准备去拿第二个球的数据*/
  PrepareGetBall2.courseAngle=90.5f;
  PrepareGetBall2.pitchAngle=0.8f;
  PrepareGetBall2.upSteerAngle=97.f; 
	PrepareGetBall2.downSteerAngle=97.f;
  PrepareGetBall2.steerSpeed=2000;
  PrepareGetBall2.gasAim=0.480;
  
  /*准备射第二个球的数据*/
  PrepareShootBall2.courseAngle=174.2f;
  PrepareShootBall2.pitchAngle=5.0f;
  PrepareShootBall2.upSteerAngle=0.0f;
	PrepareShootBall2.downSteerAngle=0.0f;
  PrepareShootBall2.steerSpeed=2000;
  PrepareShootBall2.gasAim=0.480f;
  
  /*准备去拿第三个球的数据*/
  PrepareGetBall3.courseAngle=88.5f;
  PrepareGetBall3.pitchAngle=3.0f;
  PrepareGetBall3.upSteerAngle=-87.f;
	PrepareGetBall3.downSteerAngle=-90.f;
  PrepareGetBall3.steerSpeed=2000;
  PrepareGetBall3.gasAim=0.480f;
  
  /*准备射第三个球的数据*/
  PrepareShootBall3.courseAngle=181.4f;
  PrepareShootBall3.pitchAngle=2.0f;
	PrepareShootBall3.upSteerAngle=0.0f;
  PrepareShootBall3.downSteerAngle=0.0f;
  PrepareShootBall3.steerSpeed=2000;
  PrepareShootBall3.gasAim=0.480f;
  
}
//
void PrepareGetBallMotion(motionPara_t PrepareGetBall_t)
{
	/*更新目标参数（不能在函数中更新,容易出现迭代更新的风险）*/
	gRobot.sDta.courseAimAngle=PrepareGetBall_t.courseAngle;
	gRobot.sDta.pitchAimAngle=PrepareGetBall_t.pitchAngle;
	gRobot.sDta.gasAimValue=PrepareGetBall_t.gasAim;
//	gRobot.sDta.holdBallAimAngle[0]=gRobot.sDta.holdBallAimAngle[1]=PrepareGetBall_t.steerAngle;
	gRobot.sDta.holdBallAimAngle[0]=PrepareGetBall_t.upSteerAngle;
	gRobot.sDta.holdBallAimAngle[1]=PrepareGetBall_t.downSteerAngle;

  //设置气压
  GasMotion(PrepareGetBall_t.gasAim);
  /*设置俯仰角度*/
  PitchAngleMotion(PrepareGetBall_t.pitchAngle);
  /*设置航向角度*/
  CourseAngleMotion(PrepareGetBall_t.courseAngle);
  /*关闭下方限位爪*/
  ClawShut();
  /*舵机转向*/
  HoldBallPosCrlSeparate(PrepareGetBall_t.upSteerAngle, PrepareGetBall_t.downSteerAngle);
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
	/*更新目标参数（不能在函数中更新,容易出现迭代更新的风险）*/
	gRobot.sDta.courseAimAngle=PrepareShootBall_t.courseAngle;
	gRobot.sDta.pitchAimAngle=PrepareShootBall_t.pitchAngle;
	gRobot.sDta.gasAimValue=PrepareShootBall_t.gasAim;
	
	gRobot.sDta.holdBallAimAngle[0]=PrepareShootBall_t.upSteerAngle;
	gRobot.sDta.holdBallAimAngle[1]=PrepareShootBall_t.downSteerAngle;
	
  //设置气压
  GasMotion(PrepareShootBall_t.gasAim);
  /*设置俯仰角度*/
  PitchAngleMotion(PrepareShootBall_t.pitchAngle);
  /*设置航向角度*/
  CourseAngleMotion(PrepareShootBall_t.courseAngle);
//  /*避免球乱晃*/
//  Delay_ms(500);
  /*提前打开发射装置小气缸*/
  ShootSmallOpen();
  /*舵机转向*/
  HoldBallPosCrlSeparate( PrepareShootBall_t.upSteerAngle, PrepareShootBall_t.downSteerAngle);
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

/*进行微调*/
void SmallChange(void){
	//定义是否进行计算了的变量
	int whetherCount=0;
	switch(gRobot.sDta.robocon2018){
		case COLORFUL_BALL_1:
			if(gRobot.sDta.process!=TO_THROW_BALL_1){
				return;
			}
			
			if((fabs(gRobot.posX-4450.f)>5.f || fabs(gRobot.posY-2140.f)>5.f )&& gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS ){
				 gRobot.sDta.courseAimAngle = RAD_TO_ANGLE(asinf(445.f / sqrtf((525.f - gRobot.posX)*(525.f - gRobot.posX) + (3235.f - gRobot.posY)*(3235.f - gRobot.posY))))  \
							- RAD_TO_ANGLE(atan2((COLOR_BALL_FRAME_POSX - gRobot.posX) , (COLOR_BALL_FRAME_POSY - (gRobot.posY-ROBOT_CENTER_TO_COURCE)))) + 90.f;
				 /*atan((525.f - gRobot.posX) / (3235.f - (gRobot.posY-15)))*/
				 whetherCount=1;
			}else {
				 whetherCount=0;
			}
		break;
		
		case COLORFUL_BALL_2:
			if(gRobot.sDta.process!=TO_THROW_BALL_2){
				return;
			}
		
			if((fabs(gRobot.posX-6565.f)>5.f || fabs(gRobot.posY-2180.f)>5.f )&& gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS ){
				 gRobot.sDta.courseAimAngle = RAD_TO_ANGLE(asinf(445.f / sqrtf((525.f - gRobot.posX)*(525.f - gRobot.posX) + (3235.f - gRobot.posY)*(3235.f - gRobot.posY))))  \
							- RAD_TO_ANGLE(atan2((COLOR_BALL_FRAME_POSX - gRobot.posX) , (COLOR_BALL_FRAME_POSY - (gRobot.posY-15)))) + 90.f;
				/*atan((525.f - gRobot.posX) / (3235.f - (gRobot.posY-15)))*/
				 whetherCount=1;
			}else {
				 whetherCount=0;
			}
		break;
		
		case GOLD_BALL:
			if(gRobot.sDta.process!=TO_THROW_BALL_3){
				return;
			}
			
			if((fabs(gRobot.posX-6080.f)>5.f || fabs(gRobot.posY-6030.f)>5.f )&& gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS ){
				 gRobot.sDta.courseAimAngle = RAD_TO_ANGLE(asinf(445.f / sqrtf((GOLD_BALL_FRAME_POSX - gRobot.posX)*(GOLD_BALL_FRAME_POSX - gRobot.posX) + (GOLD_BALL_FRAME_POSY - gRobot.posY)*(GOLD_BALL_FRAME_POSY - gRobot.posY))))  \
						- RAD_TO_ANGLE(atan2((GOLD_BALL_FRAME_POSX - gRobot.posX) , (GOLD_BALL_FRAME_POSY - (gRobot.posY-ROBOT_CENTER_TO_COURCE)))) + 90.f;
				/*atan((GOLD_BALL_FRAME_POSX - gRobot.posX) / (GOLD_BALL_FRAME_POSY - (gRobot.posY-ROBOT_CENTER_TO_COURCE)))*/ 
				whetherCount=1;
			}else {
				 whetherCount=0;
			}
		break;
	}	
	
	
	
	//如果计算了判断计算值是否与给定的值超过了0.2超过了则微调
	if(whetherCount){
		/*防止计算的值超过限定角度*/
		if(gRobot.sDta.courseAimAngle>198.f){
			gRobot.sDta.courseAimAngle=198.f;
			USART_OUTByDMA("courseAngle OUT OF RANGE");
		}else if(gRobot.sDta.courseAimAngle<0.f){
			gRobot.sDta.courseAimAngle=0.f;
		  USART_OUTByDMA("courseAngle OUT OF RANGE");
		}
		
		if(fabs(gRobot.sDta.courseAimAngle-gRobot.courseAngle)>0.35f){
				SetMotionFlag(~AT_COURSE_SUCCESS);
			
				USART_OUTByDMA("courseAngle need change=");
				USART_OUTByDMAF(gRobot.sDta.courseAimAngle);
				USART_OUTByDMA("\r\n");
		}else {
				USART_OUTByDMA("courseAngle OK\t");
				USART_OUTByDMAF(gRobot.sDta.courseAimAngle);
				USART_OUTByDMA("\r\n");
		}
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
	gRobot.sDta.holdBallAimAngle[0]=PrepareCompete.upSteerAngle;
	gRobot.sDta.holdBallAimAngle[1]=PrepareCompete.downSteerAngle;
	
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
		HoldBallPosCrlSeparate(PrepareCompete.upSteerAngle, PrepareCompete.downSteerAngle);
	}
	/*设置俯仰角度*/
	PitchAngleMotion(PrepareCompete.pitchAngle);
	while(1)
	{
		Delay_ms(5);
		/*读取俯仰角*/
		ReadActualPos(CAN2,5);
			/*判断俯仰角是否到位*/
		if(fabs(gRobot.sDta.pitchAimAngle-gRobot.pitchAngle)<0.1f)
		{
			SetMotionFlag(AT_PITCH_SUCCESS);
			break;
		}
	}
	Delay_ms(2000);
  /*设置航向角度*/
  CourseAngleMotion(PrepareCompete.courseAngle);
	/*等待慢转动状态完成*/
	cnt=0;
  while(1)
	{
		Delay_ms(5);
			/*读取航向角*/
		ReadActualPos(CAN2,6);
			/*判断航向角是否到位*/
		if(fabs(gRobot.sDta.courseAimAngle-gRobot.courseAngle)<0.1f)
		{
			SetMotionFlag(AT_COURSE_SUCCESS);
			break;
		}else if(fabs(gRobot.sDta.courseAimAngle-gRobot.courseAngle)<0.2f)
		{
			cnt++;
			if(cnt>50)
				break;
		}
	}
	
		#endif

}
