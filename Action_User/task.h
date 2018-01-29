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
#include "elmo.h"
#include "shoot.h"
#include "arm_math.h"
/**************#define area**********/

//#define DEBUG
//#define TEST

//常量定义
//定位系统位于自动车水平方向的中间，垂直方向距墙800mm
//摄像头位于自动车偏右21.85mm，距墙172.44mm方向的地方
//二维码板子距墙505mm
#define CAMERA_TO_GYRO_X							21.85f
#define CAMERA_TO_GYRO_Y							627.56f
#define QUICK_MARK_X_1								5505.f
#define QUICK_MARK_X_2								6505.f
#define QUICK_MARK_Y									505.f


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

/*3508减速比3591/187=19.203208*/
#define PITCH_CODE_TO_ANGLE(code)													((code)/8738.1333f)
#define PITCH_ANGLE_TO_CODE(angle)												((int)((angle)*8738.1333f))//((angle)*20.0f*19.2f*8192.f/360.f)

#define COURSE_CODE_TO_ANGLE(code)												((code)/6116.693f)
#define COURSE_ANGLE_TO_CODE(angle)												((int)(((angle))*6116.693f))//((angle-5.6f)*14.0f*19.2f*8192.f/360.f)

#define HOLD_BALL1_CODE_TO_ANGLE(code)												 ((code)/11.378f) //(code)/4096.f*360.f
#define HOLD_BALL1_ANGLE_TO_CODE(angle)												((int)((180.f-(angle-3.3f))*11.378f))//(angle)*4096.f/360.f



#define HOLD_BALL2_CODE_TO_ANGLE(code)												((code)/11.378f) //(code)/4096.f*360.f
#define HOLD_BALL2_ANGLE_TO_CODE(angle)												((int)(((angle)+180.f)*11.378f))//(angle)*4096.f/360.f



#define CAMERA_CODE_TO_ANGLE(code)												((code)/11.378f) //(code)/4096.f*360.f
#define CAMERA_ANGLE_TO_CODE(angle)												((int)(((angle))*11.378f))//(angle)*4096.f/360.f

//状态量解释
#define AT_CLAW_STATUS_OPEN 										0x01u

#define AT_STEER_READY 													0x02u

#define AT_SHOOT_BIG_ENABLE 										0x04u

#define AT_SHOOT_SMALL_ENABLE 			  					0x08u

#define AT_HOLD_BALL1_SUCCESS										0x10u

#define AT_HOLD_BALL2_SUCCESS										0x20u

#define AT_HOLD_BALL1_READ_SUCCESS							0x40u

#define AT_HOLD_BALL2_READ_SUCCESS							0x80u

#define AT_CAMERA_READ_SUCCESS									0x100u

#define AT_COURSE_READ_SUCCESS									0x200u

#define AT_PITCH_READ_SUCCESS										0x400u

#define AT_COURSE_SUCCESS												0x800u

#define AT_PITCH_SUCCESS												0x1000u

#define AT_CAMERA_SUCCESS												0x2000u


//状态量解释
#define CAN_CLAW_STATUS_OPEN 										0x01u

#define CAN_STEER_READY 												0x02u

#define CAN_SHOOT_BIG_ENABLE 										0x04u

#define CAN_SHOOT_SMALL_ENABLE 			  					0x08u


/*摄像头命令集*/
#define CAMERA_SHUT_ALL													0
#define CAMERA_OPEN_NEAR												1
#define CAMERA_OPEN_FAR													2


/**错误发生情况**/
#define HOLD_BALL1_ENABLE_FAIL														1
#define HOLD_BALL1_ROTATE_FAIL														2
#define HOLD_BALL2_ENABLE_FAIL														3
#define HOLD_BALL2_ROTATE_FAIL														4
#define HOLD_BALL1_ROTATE_SEND_FAIL												5
#define HOLD_BALL2_ROTATE_SEND_FAIL												6
#define CAN1_FAIL																					7
#define CAN2_FAIL																					8

/*舵机号命名*/
#define HOLD_BALL1														1
#define HOLD_BALL2														2
#define CAMERA														3

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
//通知控制卡开始自检										
#define NOTIFY_MOTIONCARD_START_SELFTEST	10

/*已到达区域一，可以投球*/
#define GET_MOTIONCARD_REACH_AREA1				1
/*已到达区域二，可以投球*/
#define GET_MOTIONCARD_REACH_AREA2				2
/*已到达区域三，可以投球*/
#define GET_MOTIONCARD_REACH_AREA3				3
/*自检完成*/
#define GET_MOTIONCARD_SELFTEST_FINISH		4

/*球编号*/
#define BALL_1														1
#define BALL_2														2
#define BALL_3														3

#define DELAY_TASK_NUM										2
/*延时进行的任务*/
#define DELAY_HOLD_BALL1_CHECK_POS										0x01
#define DELAY_HOLD_BALL2_CHECK_POS										0x02

#define STEER_ERROR_TIME										10

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
  
}motionPara_t;


/**************typedef area**********/
typedef struct{
  
  /*激光的初始值*/
  float laserInit;
  
  /*激光测得的值*/
  float laser;
  
  /*关于控制命令执行的动作情况*/
  uint32_t AT_motionFlag; 
  
  /*记录此时处于哪个步骤*/
  uint8_t process;
  
  /*比赛进程*/
  uint8_t robocon2018;
  
  /*持球舵机的目标位置*/
  float holdBallAimAngle;
  /*持球舵机的位置*/
  float holdBallAngle[2];
  
  /*相机舵机的目标位置*/
  float cameraAimAngle;
  /*相机舵机的位置*/
  float cameraAngle;
  
  /*航向角*/
  float courseAimAngle;
  float courseAngle;
  
  /*横滚角*/
  float pitchAimAngle;
  float pitchAngle;
  
  /*气压*/
  float gasValue;
  float gasAimValue;
  
  /*延时类型*/
  uint16_t delayTask;
  /*延时需要的时间*/
  uint16_t delayTaskMs[DELAY_TASK_NUM];
  
  /*错误记录，第一个是错误类型，第二个是发生的过程*/
  char error[STEER_ERROR_TIME][2];
  /*错误发生的次数*/
  char errorTime;
  
  float angle;
  float posX;
  float posY;
  
}Robot_t ;




#endif

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
#define DEBUG


//常量定义
//定位系统位于自动车水平方向的中间，垂直方向距墙800mm
//摄像头位于自动车偏右21.85mm，距墙172.44mm方向的地方
//二维码板子距墙505mm

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
#define AT_CLAW_STATUS_OPEN 												0x01u

#define AT_STEER_READY 															0x02u

#define AT_SHOOT_BIG_ENABLE 												0x04u

#define AT_SHOOT_SMALL_ENABLE 			  							0x08u

#define AT_HOLD_BALL1_SUCCESS												0x10u

#define AT_HOLD_BALL2_SUCCESS												0x20u

#define AT_HOLD_BALL1_READ_SUCCESS									0x40u

#define AT_HOLD_BALL2_READ_SUCCESS									0x80u

//状态量解释
#define CAN_CLAW_STATUS_OPEN 										0x01u

#define CAN_STEER_READY 												0x02u

#define CAN_SHOOT_BIG_ENABLE 										0x04u

#define CAN_SHOOT_SMALL_ENABLE 			  					0x08u


#define CLAW_OPEN 									  					(1)
#define CLAW_SHUT 									 			 			(0)


/**错误发生情况**/
#define HOLD_BALL1_ENABLE_FAIL														1
#define HOLD_BALL1_ROTATE_FAIL														2
#define HOLD_BALL2_ENABLE_FAIL														3
#define HOLD_BALL2_ROTATE_FAIL														4
#define HOLD_BALL1_ROTATE_SEND_FAIL												5
#define HOLD_BALL2_ROTATE_SEND_FAIL												6
#define CAN1_FAIL																			7
#define CAN2_FAIL																			8




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
#define DELAY_HOLD_BALL1_CHECK_POS										0x01
#define DELAY_HOLD_BALL2_CHECK_POS										0x02

#define STEER_ERROR_TIME										10

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
}motionPara_t;


/**************typedef area**********/
typedef struct{
  
  /*比赛进程*/
  uint8_t robocon2018;
  
  /*记录此时处于哪个步骤*/
  uint8_t process;
  
  /*关于控制命令执行的动作情况*/
  uint16_t AT_motionFlag; 
  
  /*激光的初始值*/
  float laserInit;
  /*激光测得的值*/
  float laser;
  
  /*舵机的目标位置*/
  int steerAimAngle[2];
  /*舵机的位置*/
  int steerAngle[2];
  
  /*航向角*/
  float courseAimAngle;
  float courseAimAngle;
  char courseReadSuccess;
  
  /*横滚角*/
  int pitchAimAngle;
  int pitchAngle;
  char pitchReadSuccess;
  
  /*气压*/
  float gasValue;
  float gasAimValue;
  
  /*--------延时执行任务信息--------*/
  /*延时类型*/
  uint16_t delayTask;
  /*延时需要的时间*/
  uint16_t delayTaskMs[DELAY_TASK_NUM];
  
  /*--------错误记录信息--------*/
  /*错误记录，第一个是错误类型，第二个是发生的过程*/
  char error[STEER_ERROR_TIME][2];
  /*错误发生的次数*/
  char errorTime;
  
  /*--------姿态信息--------*/
  float angle;
  float posX;
  float posY;
  
}Robot_t ;




#endif

