#include "shoot.h"
#include "task.h"
#include "timer.h"
#include "steer.h"
#include "motion.h"
#include "includes.h"
#include "process.h"
#include "robot.h"
#include "dma.h"
#include "gpio.h"
extern OS_EVENT *PeriodSem;


extern Robot_t gRobot;

/*储存射球和投球的参数*/
motionPara_t PrepareCompete;
motionPara_t PrepareGetBall1;
motionPara_t PrepareGetBall2;
motionPara_t PrepareGetBall3;
motionPara_t PrepareGetBall4;
motionPara_t PrepareShootBall1;
motionPara_t PrepareShootBall2;
motionPara_t PrepareShootBall3;
motionPara_t PrepareShootBall4;

void ShootBall(void)
{	
	/*因为第四个金球已经拿到时已经到了投掷金球点，可能在调节过程中一次性满足所有条件然后就发射了，
	航向等还有瞬时速度，干扰投球*/
	if(gRobot.sDta.robocon2018==COLORFUL_BALL_1&&gRobot.sDta.robocon2018==COLORFUL_BALL_2){
		Delay_ms(200);
	}else if(gRobot.sDta.robocon2018==GOLD_BALL){
		Delay_ms(1000);
	}
  /*将下爪手臂气缸上抬*/
  LowerClawStairOff();
	ShootLedOn();
  /*夹子打开*/
  ClawOpen();
  Delay_ms(400);
  /*把两个发射气缸打开*/
  ShootBigOpen();
}

void ShootReset(void)
{
	/*关闭下方限位爪*/
  ClawShut();
	ShootLedOff();
  /*复位*/
  ShootSmallShut();
  ShootBigShut();
	Delay_ms(250);
}

void prepareMotionParaInit(void)
{
	/*准备区动作*/
  PrepareCompete.courseAngle=90.0f;
  PrepareCompete.pitchAngle=19.0f;
  PrepareCompete.upSteerAngle=-90.f;
	PrepareCompete.downSteerAngle=-90.f;
  PrepareCompete.steerSpeed=2000;
  PrepareCompete.gasAim=0.440;
	
  /*准备去拿第一个球的数据*/ 
  PrepareGetBall1.courseAngle=43.0f;
  PrepareGetBall1.pitchAngle=-6.5f;
  PrepareGetBall1.upSteerAngle=-43.0f;
	PrepareGetBall1.downSteerAngle=-44.0f;
  PrepareGetBall1.steerSpeed=2000;
  PrepareGetBall1.gasAim=0.440;
  
  /*准备射第一个球的数据*/
  PrepareShootBall1.courseAngle=171.5f;
  PrepareShootBall1.pitchAngle=5.7f;
  PrepareShootBall1.upSteerAngle=0.f;
	PrepareShootBall1.downSteerAngle=0.f;
  PrepareShootBall1.steerSpeed=2000;
  PrepareShootBall1.gasAim=0.440;
	
  /*准备去拿第二个球的数据*/
  PrepareGetBall2.courseAngle=90.f;
  PrepareGetBall2.pitchAngle=-4.5f;
  PrepareGetBall2.upSteerAngle=90.f; 
	PrepareGetBall2.downSteerAngle=90.f;
  PrepareGetBall2.steerSpeed=2000;
  PrepareGetBall2.gasAim=0.440;
  
  /*准备射第二个球的数据*/
  PrepareShootBall2.courseAngle=174.2f;
  PrepareShootBall2.pitchAngle=2.f;
  PrepareShootBall2.upSteerAngle=0.0f;
	PrepareShootBall2.downSteerAngle=0.0f;
  PrepareShootBall2.steerSpeed=2000;
  PrepareShootBall2.gasAim=0.440;
  
  /*准备去拿第三个球的数据*/
  PrepareGetBall3.courseAngle=91.5f;
  PrepareGetBall3.pitchAngle=-6.2f;
  PrepareGetBall3.upSteerAngle=-90.f;
	PrepareGetBall3.downSteerAngle=-90.f;
  PrepareGetBall3.steerSpeed=2000;
  PrepareGetBall3.gasAim=0.440f;
  
  /*准备射第三个球的数据*/
  PrepareShootBall3.courseAngle=178.4f;
  PrepareShootBall3.pitchAngle=-2.f;
	PrepareShootBall3.upSteerAngle=0.0f;
  PrepareShootBall3.downSteerAngle=0.0f;
  PrepareShootBall3.steerSpeed=2000;
  PrepareShootBall3.gasAim=0.440;
	
	/*准备第四个球的参数*/
	PrepareGetBall4.courseAngle=93.5f;
	PrepareGetBall4.pitchAngle = -5.7f;
	PrepareGetBall4.upSteerAngle = -67.f;
	PrepareGetBall4.downSteerAngle = -65.f;
	PrepareGetBall4.steerSpeed = 2000;
	PrepareGetBall4.gasAim = 0.440;
	
	/*准备射第四个球的数据*/
	PrepareShootBall4.courseAngle=179.4f;
  PrepareShootBall4.pitchAngle=-2.f;
	PrepareShootBall4.upSteerAngle=0.0f;
  PrepareShootBall4.downSteerAngle=0.0f;
  PrepareShootBall4.steerSpeed=2000;
  PrepareShootBall4.gasAim=0.440;
  
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
  /*舵机转向*/
  HoldBallPosCrlSeparate(PrepareGetBall_t.upSteerAngle, PrepareGetBall_t.downSteerAngle);
}
void PrepareGetBall(int index)
{
  switch(index)
  {
		case READY:
			//传入准备区的参数
			PrepareWork();
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
		
		case BALL_4:
			//传入准备得到球的参数
			PrepareGetBallMotion(PrepareGetBall4);
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
	case BALL_4:
    //传入准备射球的参数
    PrepareShootBallMotion(PrepareShootBall4);
    break;
  default:
    USART_OUT(DEBUG_USART,"PrepareShootBall error\r\n");
    break;
  }	
}

/*进行微调*/
void SmallChange(void){
	float countAngle=0.f;
	//定义是否进行计算了的变量
	int whetherCount=0;
	switch(gRobot.sDta.robocon2018){
		case COLORFUL_BALL_1:
			if(gRobot.sDta.process!=TO_THROW_BALL_1){
				return;
			}
			
			if((fabs(gRobot.posX-4450.f)>50.f || fabs(gRobot.posY-2140.f)>50.f || fabs(gRobot.angle)>1.f )&& gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS ){
				 
				countAngle = RAD_TO_ANGLE(asinf(445.f / sqrtf((525.f - gRobot.posX)*(525.f - gRobot.posX) + (3235.f - gRobot.posY)*(3235.f - gRobot.posY))))  \
							- RAD_TO_ANGLE(atan2((COLOR_BALL_FRAME_POSX - gRobot.posX) , (COLOR_BALL_FRAME_POSY - (gRobot.posY-ROBOT_CENTER_TO_COURCE)))) + 90.f;
				 /*atan((525.f - gRobot.posX) / (3235.f - (gRobot.posY-15)))*/
				 
				 whetherCount=1;
			}else {
				 whetherCount=0;
			}
			whetherCount=0;
		break;
		
		case COLORFUL_BALL_2:
			if(gRobot.sDta.process!=TO_THROW_BALL_2){
				return;
			}
		
			if((fabs(gRobot.posX-6565.f)>50.f || fabs(gRobot.posY-2180.f)>50.f || fabs(gRobot.angle)>1.f)&& gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS ){
				 
				 countAngle = RAD_TO_ANGLE(asinf(445.f / sqrtf((525.f - gRobot.posX)*(525.f - gRobot.posX) + (3235.f - gRobot.posY)*(3235.f - gRobot.posY))))  \
							- RAD_TO_ANGLE(atan2((COLOR_BALL_FRAME_POSX - gRobot.posX) , (COLOR_BALL_FRAME_POSY - (gRobot.posY-15)))) + 90.f;
				/*atan((525.f - gRobot.posX) / (3235.f - (gRobot.posY-15)))*/
				 whetherCount=1;
			}else {
				 whetherCount=0;
			}
			whetherCount=0;
		break;
		
		case GOLD_BALL:
			if(gRobot.sDta.process!=TO_THROW_BALL_3){
				return;
			}
			
			if((fabs(gRobot.posX-6080.f)>50.f || fabs(gRobot.posY-6030.f)>50.f || fabs(gRobot.angle)>1.f)&& gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS ){
				 countAngle = RAD_TO_ANGLE(asinf(445.f / sqrtf((GOLD_BALL_FRAME_POSX - gRobot.posX)*(GOLD_BALL_FRAME_POSX - gRobot.posX) + (GOLD_BALL_FRAME_POSY - gRobot.posY)*(GOLD_BALL_FRAME_POSY - gRobot.posY))))  \
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
	
		gRobot.sDta.courseAimAngle=countAngle-gRobot.angle;
		/*防止计算的值超过限定角度*/
		if(gRobot.sDta.courseAimAngle>189.f){
			gRobot.sDta.courseAimAngle=189.f;
			USART_OUTByDMA("courseAngle OUT OF RANGE");
		}else if(gRobot.sDta.courseAimAngle<0.f){
			gRobot.sDta.courseAimAngle=0.f;
		  USART_OUTByDMA("courseAngle OUT OF RANGE");
		}
		
		if(fabs(gRobot.sDta.courseAimAngle-gRobot.courseAngle)>0.3f){
				SetMotionFlag(~AT_COURSE_SUCCESS);
			
				USART_OUTByDMA("courseAngle need change=");
				USART_OUTByDMAF(gRobot.sDta.courseAimAngle);
//				USART_OUTByDMA("\r\n");
		}else {
				USART_OUTByDMA("courseAngle OK\t");
				USART_OUTByDMAF(gRobot.sDta.courseAimAngle);
//				USART_OUTByDMA("\r\n");
		}
  }
	
}

/*初始化动作 先下降再旋转*/
void PrepareWork(void)
{
	int cnt=0;
		/*更新目标参数（不能在函数中更新,容易出现迭代更新的风险）*/
	gRobot.sDta.courseAimAngle=PrepareCompete.courseAngle;
	gRobot.sDta.pitchAimAngle=PrepareCompete.pitchAngle;
	gRobot.sDta.gasAimValue=PrepareCompete.gasAim;
	gRobot.sDta.holdBallAimAngle[0]=PrepareCompete.upSteerAngle;
	gRobot.sDta.holdBallAimAngle[1]=PrepareCompete.downSteerAngle;
	
	 /*关闭下方限位爪*/
  ClawShut();
	
	Delay_ms(200);
  //设置气压
  GasMotion(PrepareCompete.gasAim);
	
  /*舵机转向*/
	HoldBallPosCrlSeparate(PrepareCompete.upSteerAngle, PrepareCompete.downSteerAngle);
	cnt=0;
	while(1)
	{
		Delay_ms(5);
		cnt++;
		/*读取上下电机角度*/
		ReadActualPos(CAN2,7);
		ReadActualPos(CAN2,8);
			/*判断俯仰角是否到位*/
		if(fabs(gRobot.sDta.holdBallAimAngle[0]-gRobot.holdBallAngle[0])&&fabs(gRobot.sDta.holdBallAimAngle[1]-gRobot.holdBallAngle[1])<0.3f)
		{
			SetMotionFlag(AT_PITCH_SUCCESS);
			break;
		}
		if(cnt>450){
			BEEP_ON;
			USART_OUTByDMA("Steer Not Ok You need reset");
		}
	}
	
	cnt=0;
	/*设置俯仰角度*/
	PitchAngleMotion(PrepareCompete.pitchAngle);
	while(1)
	{
		Delay_ms(5);
		cnt++;
		/*读取俯仰角*/
		ReadActualPos(CAN2,5);
			/*判断俯仰角是否到位*/
		if(fabs(gRobot.sDta.pitchAimAngle-gRobot.pitchAngle)<0.3f)
		{
			SetMotionFlag(AT_PITCH_SUCCESS);
			break;
		}
		if(cnt>450){
			BEEP_ON;
			USART_OUTByDMA("Pitch Not Ok You need reset\t");
		}
	}
	
  /*设置航向角度*/
  CourseAngleMotion(PrepareCompete.courseAngle);
	/*等待慢转动状态完成*/
	cnt=0;
  while(1)
	{
		Delay_ms(5);
		cnt++;
		/*读取航向角*/
		ReadActualPos(CAN2,6);
		/*判断航向角是否到位*/
		if(fabs(gRobot.sDta.courseAimAngle-gRobot.courseAngle)<0.1f)
		{
			SetMotionFlag(AT_COURSE_SUCCESS);
			break;
		}
		
		if(cnt>450){
			BEEP_ON;
			USART_OUTByDMA("Course Not Ok You need reset\t");
		}
	}

}
