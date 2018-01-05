#include "shoot.h"
#include "task.h"
#include "timer.h"
#include "steer.h"


extern Robot_t gRobot;


void PitchAngleMotion(float angle)
{
	if(angle>60.f)
		angle=60.f;
	else if(angle<0.f)
		angle=0.f;
	
	PosCrl(CAN2, 5,ABSOLUTE_MODE,(angle*28.0f*19.2f*8192.f/360.f));
}

void CourseAngleMotion(float angle)
{
	if(angle>0.f)
		angle=0.f;
	else if(angle<-180.f)
		angle=-180.f;
	PosCrl(CAN2, 6,ABSOLUTE_MODE,((angle-5.6f)*10.0f*19.2f*8192.f/360.f));
}

void ShootBall(void)
{				
	/*夹子打开*/
	GasValveControl(GASVALVE_BOARD_ID , CLAW_ID , CLAW_OPEN);
	/*进行适当延时保证夹子和球不干涉*/
	Delay_ms(300);
	/*把两个发射气缸打开*/
	GasValveControl(GASVALVE_BOARD_ID , SHOOT_BIG_ID , 1);
}

void ShootReset(void)
{
	/*复位*/
	GasValveControl(GASVALVE_BOARD_ID , SHOOT_SMALL_ID , 0);
	GasValveControl(GASVALVE_BOARD_ID , SHOOT_BIG_ID , 0);
}

void PrepareGetBall(int index)
{
	switch(index)
	{
		case BALL_1:
			/*设置俯仰角度*/
			PitchAngleMotion(26.6);
			/*设置航向角度*/
			CourseAngleMotion(0);
			/*关闭下方限位爪*/
			GasValveControl(GASVALVE_BOARD_ID , CLAW_ID , CLAW_SHUT);
			/*舵机转向*/
			ROBS_PosCrl(90, 91, 1000);
			break;
		case BALL_2:
			/*设置俯仰角度*/
			PitchAngleMotion(27.1f);
			/*设置航向角度*/
			CourseAngleMotion(0.f);
			/*关闭下方限位爪*/
			GasValveControl(GASVALVE_BOARD_ID , CLAW_ID , CLAW_SHUT);
			/*舵机转向*/
			ROBS_PosCrl(-90, -90, 1000);
			break;
		case BALL_3:
			/*设置俯仰角度*/
			PitchAngleMotion(30.1f);
			/*设置航向角度*/
			CourseAngleMotion(0.f);
			/*关闭下方限位爪*/
			GasValveControl(GASVALVE_BOARD_ID , CLAW_ID , CLAW_SHUT);
			/*舵机转向*/
			ROBS_PosCrl(90, 90, 1000);
			break;
		default:
			USART_OUT(DEBUG_USART,"PrepareGetBall error\r\n");
			break;
	}	
}

void PrepareShootBall(int index)
{
	switch(index)
	{
		case BALL_1:
			/*设置俯仰角度*/
			PitchAngleMotion(10.2f);
			/*设置航向角度*/
			CourseAngleMotion(-76.9f);
			/*避免球乱晃*/
			Delay_ms(500);
			/*提前打开发射装置小气缸*/
			GasValveControl(GASVALVE_BOARD_ID , SHOOT_SMALL_ID , 1);
			/*舵机转向*/
			ROBS_PosCrl(0, 0, 2000);
			break;
		case BALL_2:
			/*设置俯仰角度*/
			PitchAngleMotion(30.1f);
			/*设置航向角度*/
			CourseAngleMotion(-79.5f);
			/*避免球乱晃*/
			Delay_ms(500);
			/*提前打开发射装置小气缸*/
			GasValveControl(GASVALVE_BOARD_ID , SHOOT_SMALL_ID , 1);
			/*舵机转向*/
			ROBS_PosCrl(0, 0, 2000);
			break;
		case BALL_3:
			/*设置俯仰角度*/
			PitchAngleMotion(32.5f);
			/*设置航向角度*/
			CourseAngleMotion(-90.f);
			/*避免球乱晃*/
			Delay_ms(500);
			/*提前打开发射装置小气缸*/
			GasValveControl(GASVALVE_BOARD_ID , SHOOT_SMALL_ID , 1);
			/*舵机转向*/
			ROBS_PosCrl(0, 0, 2000);
			break;
		default:
			USART_OUT(DEBUG_USART,"PrepareShootBall error\r\n");
			break;
	}	
}
