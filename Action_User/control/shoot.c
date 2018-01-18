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
	gRobot.motorPara_t.pitchAimPos=(int)(angle*28.0f*19.2f*8192.f/360.f);
	PosCrl(CAN2, 5,ABSOLUTE_MODE,(angle*28.0f*19.2f*8192.f/360.f));
}

void CourseAngleMotion(float angle)
{
	if(angle>0.f)
		angle=0.f;
	else if(angle<-180.f)
		angle=-180.f;
	gRobot.motorPara_t.courseAimPos=(int)((angle-5.6f)*10.0f*19.2f*8192.f/360.f);
	PosCrl(CAN2, 6,ABSOLUTE_MODE,((angle-5.6f)*10.0f*19.2f*8192.f/360.f));
}

void GasMotion(float value)
{
	gRobot.motorPara_t.gasAimValue=value;
	CAN_TxMsg(CAN2,SEND_TO_GASSENSOR,(uint8_t*)(&value),4);
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

void prepareMotionParaInit(void)
{
	/*准备去拿第一个球的数据*/
	gRobot.prepareMotion.PrepareGetBall1.courseAngle=0.f;
	gRobot.prepareMotion.PrepareGetBall1.pitchAngle=26.6f;
	gRobot.prepareMotion.PrepareGetBall1.steerAngle=90.f;
	gRobot.prepareMotion.PrepareGetBall1.steerSpeed=1000;
	gRobot.prepareMotion.PrepareGetBall1.gasAim=0.45f;
	
	/*准备去拿第二个球的数据*/
	gRobot.prepareMotion.PrepareGetBall2.courseAngle=0.f;
	gRobot.prepareMotion.PrepareGetBall2.pitchAngle=27.1f;
	gRobot.prepareMotion.PrepareGetBall2.steerAngle=-90.f;
	gRobot.prepareMotion.PrepareGetBall2.steerSpeed=1000;
	gRobot.prepareMotion.PrepareGetBall2.gasAim=0.45f;
	
	/*准备去拿第三个球的数据*/
	gRobot.prepareMotion.PrepareGetBall3.courseAngle=30.1f;
	gRobot.prepareMotion.PrepareGetBall3.pitchAngle=0.f;
	gRobot.prepareMotion.PrepareGetBall3.steerAngle=90.f;
	gRobot.prepareMotion.PrepareGetBall3.steerSpeed=1000;
	gRobot.prepareMotion.PrepareGetBall3.gasAim=0.45f;
	
	/*准备射第一个球的数据*/
	gRobot.prepareMotion.PrepareShootBall1.courseAngle=-76.9f;
	gRobot.prepareMotion.PrepareShootBall1.pitchAngle=10.2f;
	gRobot.prepareMotion.PrepareShootBall1.steerAngle=0.f;
	gRobot.prepareMotion.PrepareShootBall1.steerSpeed=2000;
	gRobot.prepareMotion.PrepareShootBall1.gasAim=0.45f;
	
	/*准备射第二个球的数据*/
	gRobot.prepareMotion.PrepareShootBall2.courseAngle=-79.5f;
	gRobot.prepareMotion.PrepareShootBall2.pitchAngle=30.1f;
	gRobot.prepareMotion.PrepareShootBall1.steerAngle=0.f;
	gRobot.prepareMotion.PrepareShootBall1.steerSpeed=2000;
	gRobot.prepareMotion.PrepareShootBall2.gasAim=0.45f;
	
	/*准备射第三个球的数据*/
	gRobot.prepareMotion.PrepareShootBall3.courseAngle=-90.f;
	gRobot.prepareMotion.PrepareShootBall3.pitchAngle=32.5f;
	gRobot.prepareMotion.PrepareShootBall1.steerAngle=0.f;
	gRobot.prepareMotion.PrepareShootBall1.steerSpeed=2000;
	gRobot.prepareMotion.PrepareShootBall3.gasAim=0.45f;
	
}

void PrepareGetBallMotion(shootPara_t PrepareGetBall_t)
{
		//设置气压
		GasMotion(PrepareGetBall_t.gasAim);
		/*设置俯仰角度*/
		PitchAngleMotion(PrepareGetBall_t.pitchAngle);
		/*设置航向角度*/
		CourseAngleMotion(PrepareGetBall_t.courseAngle);
		/*关闭下方限位爪*/
		GasValveControl(GASVALVE_BOARD_ID , CLAW_ID , CLAW_SHUT);
		/*舵机转向*/
		ROBS_PosCrl(PrepareGetBall_t.steerAngle, PrepareGetBall_t.steerAngle, PrepareGetBall_t.steerSpeed);
}

void PrepareGetBall(int index)
{
	switch(index)
	{
		case BALL_1:
			//传入准备得到球的参数
			PrepareGetBallMotion(gRobot.prepareMotion.PrepareGetBall1);
			break;
		case BALL_2:
			//传入准备得到球的参数
			PrepareGetBallMotion(gRobot.prepareMotion.PrepareGetBall2);
			break;
		case BALL_3:
			//传入准备得到球的参数
			PrepareGetBallMotion(gRobot.prepareMotion.PrepareGetBall3);
			break;
		default:
			USART_OUT(DEBUG_USART,"PrepareGetBall error\r\n");
			break;
	}	
}

void PrepareShootBallMotion(shootPara_t PrepareShootBall_t)
{
	//设置气压
	GasMotion(PrepareShootBall_t.gasAim);
	/*设置俯仰角度*/
	PitchAngleMotion(PrepareShootBall_t.pitchAngle);
	/*设置航向角度*/
	CourseAngleMotion(PrepareShootBall_t.courseAngle);
	/*避免球乱晃*/
	Delay_ms(500);
	/*提前打开发射装置小气缸*/
	GasValveControl(GASVALVE_BOARD_ID , SHOOT_SMALL_ID , 1);
	/*舵机转向*/
	ROBS_PosCrl(PrepareShootBall_t.steerAngle, PrepareShootBall_t.steerAngle, PrepareShootBall_t.steerSpeed);
}

void PrepareShootBall(int index)
{
	switch(index)
	{
		case BALL_1:
			//传入准备射球的参数
			PrepareShootBallMotion(gRobot.prepareMotion.PrepareGetBall1);
			break;
		case BALL_2:
			//传入准备射球的参数
			PrepareShootBallMotion(gRobot.prepareMotion.PrepareGetBall2);
			break;
		case BALL_3:
			//传入准备射球的参数
			PrepareShootBallMotion(gRobot.prepareMotion.PrepareGetBall3);
			break;
		default:
			USART_OUT(DEBUG_USART,"PrepareShootBall error\r\n");
			break;
	}	
}
