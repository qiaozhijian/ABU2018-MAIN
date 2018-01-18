#ifndef __TASK_H
#define __TASK_H
#include "stdint.h"
#include "customer.h"
#include "string.h"
#include "stm32f4xx_usart.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "usart.h"
#include "gasvalveControl.h"
#include "task.h"
#include "elmo.h"
#include "shoot.h"
#include "arm_math.h"
/**************#define area**********/

#define PE_FOR_THE_BALL								(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0))							


//常量定义
#define CAR_L													0.5389f
/*轮子直径*/
#define WHEEL_D												0.1524f
#define SHOOT_ANGLE  (6)
#define COUNTS_PER_ROUND_REDUC  4096.f

/*轮子的角度（角度制）对应磁编码器的脉冲数*/
#define ANGLE_TO_CODE(angle) 																					((angle)*11.377778f)												//((angle)/360.f*COUNTS_PER_ROUND_REDUC)
/*磁编码器的脉冲数对应轮子的角度（角度制）*/
#define CODE_TO_ANGLE(code) 																					((code)*0.087890625f)											//((code) *360.f/COUNTS_PER_ROUND_REDUC)

/*以毫米每秒的速度转换为脉冲*/																					
#define MILLIMETER_TO_CODE(millimeter)   															((millimeter)*8.5551f)								//(ANGLE_TO_CODE(RAD_TO_ANGLE((centimeter)/1000.0/WHEEL_D*2.f)))
/*脉冲转换为以毫米每秒的速度*/
#define CODE_TO_MILLIMETER(code) 			  															((code)*0.11689f)												//(ANGLE_TO_RAD(CODE_TO_ANGLE((code)))*WHEEL_D/2.f*1000.0)

/*自转角速度（角度制）需要的脉冲量*///float精确度会不会小到使得出的调节量小到失真？
#define ROTATE_W_TO_CODE(rotateW)																			((rotateW)*89.58883f)										//MILLIMETER_TO_CODE((ANGLE_TO_RAD((rotateW))*CAR_L*1000))
/*脉冲量转换为对应的自转角速度（角度制）*/
#define CODE_TO_ROTATE_W(code)																				((code)*0.0111621f)										


/*角度制转换为弧度制*/
#define ANGLE_TO_RAD(angle) 																					((angle)*0.0174533f)
/*弧度制转换为角度制*/
#define RAD_TO_ANGLE(rad) 																						((rad)*57.29578f)

//状态量解释
#define AT_CLAW_STATUS_OPEN 										0x01u

#define AT_STEER_READY 													0x02u

#define AT_SHOOT_BIG_ENABLE 										0x04u

#define AT_SHOOT_SMALL_ENABLE 			  					0x08u

#define AT_STEER1_SUCCESS												0x10u

#define AT_STEER2_SUCCESS												0x20u

#define AT_STEER1_READ_SUCCESS									0x40u

#define AT_STEER2_READ_SUCCESS									0x80u

//状态量解释
#define CAN_CLAW_STATUS_OPEN 										0x01u

#define CAN_STEER_READY 												0x02u

#define CAN_SHOOT_BIG_ENABLE 										0x04u

#define CAN_SHOOT_SMALL_ENABLE 			  					0x08u


#define CLAW_OPEN 									  					(1)
#define CLAW_SHUT 									 			 			(0)


/*状态量解释*/
#define TO_START													1
#define TO_GET_BALL_1											2
#define TO_THE_AREA_1											3
#define TO_THROW_BALL_1										4
#define TO_GET_BALL_2											5
#define TO_THE_AREA_2											6
#define TO_THROW_BALL_2										7
#define TO_GET_BALL_3											8
#define TO_THE_AREA_3											9
#define TO_THROW_BALL_3									  10
#define END_COMPETE												100

/*比赛进程宏定义解释*/
#define ROBOT_START							0
#define COLORFUL_BALL_1					1
#define COLORFUL_BALL_2					2
#define GOLD_BALL								3

/*控制卡通信解释*/
//开始出发
#define NOTIFY_MOTIONCARD_START						1
//通知控制已经得到球一
#define NOTIFY_MOTIONCARD_GOT_BALL1				2
//通知控制卡已经完成射击球一
#define NOTIFY_MOTIONCARD_SHOT_BALL1			3
//通知控制已经得到球二
#define NOTIFY_MOTIONCARD_GOT_BALL2				4
//通知控制卡已经完成射击球二
#define NOTIFY_MOTIONCARD_SHOT_BALL2			5
//通知控制已经得到球三
#define NOTIFY_MOTIONCARD_GOT_BALL3				6
//通知控制卡球一掉了
#define NOTIFY_MOTIONCARD_LOSE_BALL1			7
//通知控制卡球二掉了
#define NOTIFY_MOTIONCARD_LOSE_BALL2			8
//通知控制卡球三掉了
#define NOTIFY_MOTIONCARD_LOSE_BALL3			9

/*球编号*/
#define BALL_1														1
#define BALL_2														2
#define BALL_3														3

#define DELAY_TASK_NUM										2
/*延时进行的任务*/
#define DELAY_STEER1_CHECK_POS										0x01
#define DELAY_STEER2_CHECK_POS										0x02



typedef struct{
	/*航向角*/
	float courseAngle;
	/*俯仰角*/
	float pitchAngle;
	/*舵机角*/
	float steerAngle;
	/*舵机旋转的速度*/
	int steerSpeed;
	/*气压值*/
	float gasAim;
	
}shootPara_t;


/**************typedef area**********/
typedef struct{
	
	/*激光的初始值*/
	float laserInit;
	
	/*激光测得的值*/
	float laser;
	
	/*关于控制命令执行的动作情况*/
	uint16_t AT_motionFlag; 
	
	/*记录此时处于哪个步骤*/
	uint8_t process;
	
	/*比赛进程*/
	uint8_t robocon2018;
	
	/*射击所应有的参数结构体*/
	struct{
		
		shootPara_t PrepareGetBall1;
		
		shootPara_t PrepareGetBall2;
		
		shootPara_t PrepareGetBall3;
		
		shootPara_t PrepareShootBall1;
		
		shootPara_t PrepareShootBall2;
		
		shootPara_t PrepareShootBall3;
		
	}prepareMotion;
	
	
	struct{
		
		/*舵机的目标位置*/
		int steerAimPos[2][2];
		/*舵机的位置*/
		int steerPos[2];
		/*错误记录，第一个是错误类型，第二个是发生的过程*/
		char error[10][2];
		/*错误发生的次数*/
		char errorTime;
	}steer_t;
	
	struct{
		
		int courseAimPos;
		int coursePos;
		char courseReadSuccess;
		
		int pitchAimPos;
		int pitchPos;
		char pitchReadSuccess;
		
		float gasValue;
		float gasAimValue;
	}motorPara_t;
	
	uint16_t delayTask;
	uint16_t delayTaskMs[DELAY_TASK_NUM];
	
}Robot_t ;




#endif

