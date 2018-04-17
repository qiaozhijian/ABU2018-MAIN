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
motionPara_t PrepareGetBall3Wait;
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
//	if(gRobot.sDta.robocon2018==COLORFUL_BALL_1){
//		Delay_ms(450);
//	}else if(gRobot.sDta.robocon2018==COLORFUL_BALL_2){
//		Delay_ms(250);
//	}else if(gRobot.sDta.robocon2018==GOLD_BALL){
//		Delay_ms(850);
//	}
	ShootLedOn();
	Delay_ms(10);
  /*夹子打开*/
  ClawOpen();
  Delay_ms(50);
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
  PrepareCompete.gasAim=0.500;
	
  /*准备去拿第一个球的数据*/ 
  PrepareGetBall1.courseAngle=47.5f;
  PrepareGetBall1.pitchAngle=-1.5f;
  PrepareGetBall1.upSteerAngle=-43.0f;
	PrepareGetBall1.downSteerAngle=-54.0f;
  PrepareGetBall1.gasAim=0.500;
  
  /*准备射第一个球的数据*/
  PrepareShootBall1.courseAngle=170.f;
  PrepareShootBall1.pitchAngle=9.8f;
  PrepareShootBall1.upSteerAngle=0.f;
	PrepareShootBall1.downSteerAngle=0.f;
  PrepareShootBall1.gasAim=0.500;
	
  /*准备去拿第二个球的数据*/
  PrepareGetBall2.courseAngle=91.5f;
  PrepareGetBall2.pitchAngle=-0.5f;
  PrepareGetBall2.upSteerAngle=98.f; 
	PrepareGetBall2.downSteerAngle=89.f;
  PrepareGetBall2.gasAim=0.500;
  
  /*准备射第二个球的数据*/
  PrepareShootBall2.courseAngle=175.f;
  PrepareShootBall2.pitchAngle=10.f;
  PrepareShootBall2.upSteerAngle=0.0f;
	PrepareShootBall2.downSteerAngle=0.0f;
  PrepareShootBall2.gasAim=0.500;
  
  /*准备等待拿第三个球的数据*/
  PrepareGetBall3Wait.courseAngle=115.f;
  PrepareGetBall3Wait.pitchAngle=-4.f;
  PrepareGetBall3Wait.upSteerAngle=-42.f;
	PrepareGetBall3Wait.downSteerAngle=-59.f;
  PrepareGetBall3Wait.gasAim=0.450;
	/*接取第三个球的参数*/
	PrepareGetBall3.courseAngle=95.f;
  PrepareGetBall3.pitchAngle=-4.f;
  PrepareGetBall3.upSteerAngle=-42.f;
	PrepareGetBall3.downSteerAngle=-59.f;
  PrepareGetBall3.gasAim=0.450;
  
  /*准备射第三个球的数据*/
  PrepareShootBall3.courseAngle=181.5f;
  PrepareShootBall3.pitchAngle=-0.3f;
	PrepareShootBall3.upSteerAngle=0.0f;
  PrepareShootBall3.downSteerAngle=0.0f;
  PrepareShootBall3.gasAim=0.450;
	
	/*准备第四个球的参数*/
	PrepareGetBall4.courseAngle=150.f;
	PrepareGetBall4.pitchAngle = -5.5f;
	PrepareGetBall4.upSteerAngle = -72.f;
	PrepareGetBall4.downSteerAngle = -70.f;
	PrepareGetBall4.gasAim = 0.450;
	
	/*准备射第四个球的数据*/
	PrepareShootBall4.courseAngle=179.9f;
  PrepareShootBall4.pitchAngle=0.7f;
	PrepareShootBall4.upSteerAngle=0.0f;
  PrepareShootBall4.downSteerAngle=0.0f;
  PrepareShootBall4.gasAim=0.450;
  
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
		
		case BALL_3_WAIT:
			//传入准备得到球的参数
			PrepareGetBallMotion(PrepareGetBall3Wait);
		break;
		
		case BALL_3:
			//传入接取金球的参数
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
			
			if((fabs(gRobot.posX-TZ_1_X)>25.f || fabs(gRobot.posY-TZ_1_Y)>25.f || fabs(gRobot.angle)>1.f )&& gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS ){
				 
				countAngle = RAD_TO_ANGLE(asinf(445.f / sqrtf((525.f - gRobot.posX)*(525.f - gRobot.posX) + (3235.f - gRobot.posY)*(3235.f - gRobot.posY))))  \
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
		
			if((fabs(gRobot.posX-TZ_2_X)>25.f || fabs(gRobot.posY-TZ_2_Y)>25.f || fabs(gRobot.angle)>1.f)&& gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS ){
				 
				 countAngle = RAD_TO_ANGLE(asinf(445.f / sqrtf((525.f - gRobot.posX)*(525.f - gRobot.posX) + (3235.f - gRobot.posY)*(3235.f - gRobot.posY))))  \
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
			
			if((fabs(gRobot.posX-TZ_3_X)>25.f || fabs(gRobot.posY-TZ_3_Y)>25.f || fabs(gRobot.angle)>1.f)&& gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS ){
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
	static int prepareWorkStep=1;
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

		
	while(1){
		Delay_ms(5);
		switch(prepareWorkStep){
			case 1:
				/*舵机转向*/
	      HoldBallPosCrlSeparate(PrepareCompete.upSteerAngle, PrepareCompete.downSteerAngle);
				cnt++;
				/*读取上下电机角度*/
				ReadActualPos(CAN2,7);
				ReadActualPos(CAN2,8);
				/*判断俯仰角是否到位*/
				if(fabs(gRobot.sDta.holdBallAimAngle[0]-gRobot.holdBallAngle[0])<5.f&&fabs(gRobot.sDta.holdBallAimAngle[1]-gRobot.holdBallAngle[1])<5.f)
				{
					SetMotionFlag(AT_PITCH_SUCCESS);
					prepareWorkStep=2;
				}
				/*3s爪子不到位*/
				if(cnt>600){
						cnt=0;
						BEEP_ON;
						USART_OUTByDMA("Steer Not Ok You need reset");
						/*进入打气气压检测中*/
						while(1){
							Delay_ms(5);
							cnt++;
							/*先将蜂鸣器关了太吵了*/
							if(cnt>=600){
								BEEP_OFF;
							}
							cnt%=600;
							USART_OUTByDMAF(gRobot.gasValue);
							if(gRobot.gasValue>0.6f){
									ShootLedOn();
									if(cnt<300){
										BEEP_ON;
									}
									else{
										BEEP_OFF;
									}
							}
							else
							{
								 ShootLedOff();
							}
						}
					}
			break;
			
			case 2:
				/*之前打气检测只是开小电，这时候是正常模式再给气阀板发指令*/
				GasMotion(PrepareCompete.gasAim);
	      GasEnable();
				prepareWorkStep=3;
			  cnt=0;
			break;
			
			case 3:
				/*设置俯仰角度*/
	      PitchAngleMotion(PrepareCompete.pitchAngle);
				/*读取俯仰角*/
		    ReadActualPos(CAN2,5);
				cnt++;
        /*判断俯仰角是否到位*/
				if(fabs(gRobot.sDta.pitchAimAngle-gRobot.pitchAngle)<0.3f)
				{
					SetMotionFlag(AT_PITCH_SUCCESS);
					prepareWorkStep=4;
					cnt =0;
				}
				if(cnt>450){
					BEEP_ON;
					USART_OUTByDMA("Pitch Not Ok You need reset\t");
					prepareWorkStep=4;
					cnt =0;
				}
			break;
				
			case 4:
				/*设置航向角度*/
        CourseAngleMotion(PrepareCompete.courseAngle);
				/*读取航向角*/
				ReadActualPos(CAN2,6);
				cnt++;
        /*判断航向角是否到位*/
				if(fabs(gRobot.sDta.courseAimAngle-gRobot.courseAngle)<0.1f)
				{
					SetMotionFlag(AT_COURSE_SUCCESS);
					return;
				}
		
		    if(cnt>450){
			    BEEP_ON;
			    USART_OUTByDMA("Course Not Ok You need reset\t");
			    return;
		    }
		  break;
		}
	}

}
