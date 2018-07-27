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
/**************#define area**********/
#define TEST_GOLD
//#define TEST
#define GAS_CONTOL_BY_PWM  1
//	 DEBUG

//常量定义
//定位系统位于自动车水平方向的中间，垂直方向距墙mm505.f
//摄像头位于自动车偏右21.85mm，距墙172.44mm方向的地方
//二维码板子距墙505mm
#define CAMERA_TO_GYRO_X																							21.85f
#define CAMERA_TO_GYRO_Y																							332.56f
#define QUICK_MARK_X_1																								5505.f
#define QUICK_MARK_X_2																								6505.f
#define QUICK_MARK_Y																									800.f


#define CAR_L																													0.5389f
/*轮子直径*/
#define WHEEL_D																												0.1524f
#define SHOOT_ANGLE																									  (6)
#define COUNTS_PER_ROUND_REDUC 																		  	4096.f

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
/*定位系统与车中心的差*/
#define DISX_GYRO2CENTER (164.0f)
#define DISY_GYRO2CENTER (445.0f)
/*车中心与转盘中心的误差*/
#define CAR_CENTER_TO_COURCE_CENTER   (15.f)

/*3508减速比3591/187=19.203208  8192位总脉冲*/
#define PITCH_CODE_TO_ANGLE(code)																			((code)/8738.1333f)
#define PITCH_ANGLE_TO_CODE(angle)																		((int)((angle)*8738.1333f))//((angle)*20.0f*19.2f*8192.f/360.f)

#define COURSE_CODE_TO_ANGLE(code)																		((code)/6116.693f)
#define COURSE_ANGLE_TO_CODE(angle)																		((int)(((angle))*6116.693f))//((angle)*14.0f*19.2f*8192.f/360.f)

#define UPSTEER_CODE_TO_ANGLE(code)                       						((code)/1280.f)       
#define DOWN_STEER_CODE_TO_ANGLE(code)                    						((code)/1638.4f)

#define UP_STEER_COMPENSATE                             						  (116.f)   
#define DOWN_STEER_COMPENSATE																					(107.f)
#define PITCH_COMPENSATE																							(-10.f)
#define COURCE_COMPENSATE																						  (193.f)
//状态量解释
#define AT_CLAW_STATUS_OPEN 																					0x01u

#define AT_STEER_READY 																								0x02u

#define AT_SHOOT_BIG_ENABLE 																					0x04u

#define AT_SHOOT_SMALL_ENABLE 			  																0x08u

#define AT_HOLD_BALL_1_SUCCESS																				0x10u

#define AT_HOLD_BALL_2_SUCCESS																				0x20u

#define AT_HOLD_BALL_1_RESPONSE																				0x40u

#define AT_HOLD_BALL_2_RESPONSE																				0x80u

#define AT_COURSE_READ																								0x200u

#define AT_PITCH_READ																									0x400u

#define AT_COURSE_SUCCESS																							0x800u

#define AT_PITCH_SUCCESS																							0x1000u

#define AT_CAMERA_RESPONSE_SUCCESS																		0x2000u

#define AT_CAMERA_TALK_SUCCESS																				0x100u

#define AT_GAS_SUCCESS																								0x4000u

#define AT_PREPARE_READY																							0x8000u

#define AT_IS_SEND_DEBUG_DATA																					0x10000u

#define AT_REACH_FIRST_PLACE																					0x20000u

#define AT_REACH_SECOND_PLACE																					0x40000u

#define AT_REACH_THIRD_PLACE																					0x80000u

#define AT_THE_WHEEL_SELFTEST_OVER																		0x100000u

#define AT_THE_DUCT_SELFTEST_OVER																			0x200000u

#define AT_RESET_THE_ROBOT																						0x400000u
//判断是否投过金球
#define AT_RESET_SHOOT_GOLD																						0x800000u

#define AT_RESET_USE_GOLD_STANDYBY																		0x1000000u

#define AT_GET_MOTIONCARD_RESET_FINISH																0x2000000u

#define AT_GET_MOTIONCARD_GET_GOLDBALL_AREA         									0x4000000u

#define AT_GET_PPS_PROBLEM         									                  0x8000000u




//状态量解释
#define CAN_CLAW_STATUS_OPEN 																					0x01u

#define CAN_STEER_READY 																							0x02u

#define CAN_SHOOT_BIG_ENABLE 																					0x04u

#define CAN_SHOOT_SMALL_ENABLE 			  																0x08u


/**错误发生情况**/
//初始化时舵机使能有问题
#define HOLD_BALL_1_ENABLE_FAIL																				1
#define HOLD_BALL_2_ENABLE_FAIL																				2
//射球或者抓球时舵机没有转动到位
#define HOLD_BALL_1_ROTATE_FAIL																				3
#define HOLD_BALL_2_ROTATE_FAIL																				4
//射球或者抓球时转台没有转动到位
#define COURSE_ROTATE_FAIL																						5
//射球或者抓球时俯仰没有转动到位
#define PITCH_ROTATE_FAIL																							6
#define HOLD_BALL_1_ROTATE_FAIL																				3
#define HOLD_BALL_2_ROTATE_FAIL																				4
#define CAN1_FAIL																											5
#define CAN2_FAIL																											6

/*舵机号命名*/
#define HOLD_BALL_1																										1
#define HOLD_BALL_2																										2
#define CAMERA_STEER																									3

/*状态量解释*/
#define TO_START																											1
#define TO_GET_BALL_1																									2
#define TO_THE_AREA_1																									3
#define TO_THROW_BALL_1																								4
#define TO_GET_BALL_2																									5
#define TO_THE_AREA_2																									6
#define TO_THROW_BALL_2																								7
#define TO_GET_BALL_3																									8
#define TO_THE_AREA_3																									9
#define TO_THROW_BALL_3									  														10
#define END_COMPETE																										100

/*比赛进程宏定义解释*/
#define ROBOT_PREPARE																									0
#define ROBOT_START																										1
#define COLORFUL_BALL_1																								2
#define COLORFUL_BALL_2																								3
#define GOLD_BALL																											4
#define ROBOT_SELF_TEST      																					9
#define INTO_HARDFAULT																								10
/*重启准备完成*/
#define INTO_RESET_PREPARE    																				16
/*蓝牙调试自定义程序*/
#define ROBOT_CONTROL_BY_BT   																				11

#define RACK3_BALL                                                    18
/*控制卡通信解释*/
//开始出发
#define NOTIFY_MOTIONCARD_START																				1
//通知控制已经得到球一
#define NOTIFY_MOTIONCARD_GOT_BALL1																		2
//通知控制卡已经完成射击球一
#define NOTIFY_MOTIONCARD_SHOT_BALL1																	3
//通知控制已经得到球二
#define NOTIFY_MOTIONCARD_GOT_BALL2																		4
//通知控制卡已经完成射击球二
#define NOTIFY_MOTIONCARD_SHOT_BALL2																	5
//通知控制已经得到球三
#define NOTIFY_MOTIONCARD_GOT_BALL3																		6
//通知控制卡球一掉了
#define NOTIFY_MOTIONCARD_LOSE_BALL1																	7
//通知控制卡球二掉了
#define NOTIFY_MOTIONCARD_LOSE_BALL2																	8
//通知控制卡球三掉了
#define NOTIFY_MOTIONCARD_LOSE_BALL3																	9
//通知控制卡开始自检										
#define NOTIFY_MOTIONCARD_START_SELFTEST															10
//通知控制卡自动车主控准备完成									
#define NOTIFY_MOTIONCARD_PREPARE_FINISH															11

//通知控制卡进行自检
#define NOTIFY_MOTIONCARD_SELFTEST																		12
//通知控制卡进行轮子自检
#define NOTIFY_MOTIONCARD_SELFTEST_THE_WHEEL													13
//通知控制卡进行激光自检
#define NOTIFY_MOTIONCARD_SELFTEST_THE_LASER													14
//通知控制卡自检涵道
#define NOTIFY_MOTIONCARD_SELFTEST_THE_DUCT														33
//通知控制卡主控进入自检模式
#define NOTIFY_MOTIONCARD_INTO_BT_CTRL																84
//通知控制卡使能轮子
#define NOTIFY_MOTIONCARD_ENABLE_WHEEL																78
//通知控制卡失能轮子
#define NOTIFY_MOTIONCARD_DISABLE_WHEEL																70
//通知控制卡进入重启	
#define NOTIFY_MOTIONCARD_INTO_RESET																	97
//通知控制卡进入重启
#define NOTIFY_MOTIONCARD_RESET_GOLD																	98 
//通知控制卡进入重启
#define NOTIFY_MOTIONCARD_RESET_ALL																		99
//通知控制卡进入擦轮程序
#define NOTIFY_MOTIONCARD_WIPE_WHEEL																	38               
//通知控制卡进入金球走形测试测试
#define NOTIFY_MOTIONCARD_INTO_TEST_GOLD                              68
//通知控制卡进入去球架三模式
#define NOTIFY_MOTIONCARD_INTO_GET_RACK3BALL                          51                          
//通知控制卡红蓝场选择
#define NOTIFY_MOTIONCARD_CHOOSE_BLUE                                 63

#define NOTIFY_MOTIONCARD_CHOOSE_RED                                  64
/*已到达区域一，可以投球*/
#define GET_MOTIONCARD_REACH_AREA1																		1
/*已到达区域二，可以投球*/
#define GET_MOTIONCARD_REACH_AREA2																		2
/*已到达区域三，可以投球*/
#define GET_MOTIONCARD_REACH_AREA3																		3
/*自检完成*/
#define GET_MOTIONCARD_SELFTEST_FINISH																4
/*控制卡初始化完成*/
#define GET_MOTIONCARD_PREPARE_READY																	5
/*控制卡到达接金球位置*/
#define GET_MOTIONCARD_GET_GOLDBALL_AREA															8
/*控制卡告诉我涵道自检完成*/
#define GET_MOTIONCARD_DUCT_SELFTEST_OK																21
/*控制卡告诉我涵道自检完成*/
#define GET_MOTIONCARD_RESET_FINISH																		33


/*控制卡通知进入硬件中断*/
#define GET_MOTIONCARD_INTO_HARDFAULT																	44

#define GET_MOTIONCARD_SELFTEST_WHEEL_OVER 														66
	
#define GET_QIAO_ZHIJIAN_TEST          														    88

#define GET_PPS_PROBLEM                														    74


/*球编号*/
#define READY																													0
#define BALL_1																												1
#define BALL_2																												2
#define BALL_3																												3
#define BALL_4																												4
/*彩球12的备用*/
#define BALL_1_BACKUP																									5
#define BALL_2_BACKUP																									8
/*金球12的备用*/
#define BALL_3_BACKUP																									9
#define BALL_4_BACKUP																									10

#define BALL_3_WAIT																										6
/*球架3的球*/
#define BALL_RACK3																										16

#define DELAY_TASK_NUM																								2
/*延时进行的任务*/
#define DELAY_HOLD_BALL_1_CHECK_POS																		0x01
#define DELAY_HOLD_BALL_2_CHECK_POS																		0x02

#define ERROR_TIME																										10

#define COLOR_BALL_FRAME_POSX																				  (515.f)
#define COLOR_BALL_FRAME_POSY																					(3275.f)

#define GOLD_BALL_FRAME_POSX																					(-3220.f)
#define GOLD_BALL_FRAME_POSY																					(6535.f)

#define ROBOT_COURCE_CENTER_TO_ARM																		(445.f)
#define ROBOT_ARM_TO_MOTOR																						(509.26f)
#define ROBOT_ARM_TO_THROW_CENTER																			(596.88f)
#define ROBOT_CENTER_TO_COURCE																				(0.f)

#define TZ_1_X																												(4565.0f)
#define TZ_1_Y																												(2180.0f)

#define TZ_2_X																												(6565.0f)
#define TZ_2_Y																												(2230.0f)

#define TZ_3_X																												(6894.0f)
#define TZ_3_Y																												(6030.0f)

#define HANDOVER_3_X																									(6894.f)
#define HANDOVER_3_Y																									(935.f)

typedef struct{
  
	/*判断此次是否为看门狗复位*/
  uint32_t isReset;
  
  /*关于控制命令执行的动作情况*/
  uint32_t AT_motionFlag; 
  
  /*记录此时处于哪个步骤*/
  uint32_t process;
  
  /*比赛进程*/
  uint32_t robocon2018;
  
  /*持球舵机的目标位置*/
//	#ifndef TEST
//  float holdBallAimAngle;
//	#else
  float holdBallAimAngle[2];
//	#endif
  
  /*相机舵机的目标位置*/
  float cameraAimAngle;
  
  /*航向角*/
  float courseAimAngle;
  
  /*横滚角*/
  float pitchAimAngle;
  
  /*气压*/
  float gasAimValue;
	
	/*某一个金球*/
	int WhichGoldBall;
}DataSave_t;
/*计算机器人的速度结构体*/
typedef struct{
	float lastPosX[3];
  float lastPosY[3];
	uint32_t countTime;
	//计算车xy方向上的和和速度
	
	float countVel;
	float countXVel;
	float countYVel;
	
	float courseVel;
	float steerVel[2];
	float readCourseVel;
	float readSteerVel[2];
}RobotVel_t;
/*时间计算结构体*/
typedef struct{
	/*定时器里面计算的自增量*/
	uint32_t roboconCnt;
	/*定时器里面计算的总时间*/
	float roboconTime;
		
	float colorBall1Time;
	float colorBall2Time;
	float goldBallTime;
	
	float colorBall1WaitTime;
	float colorBall2WaitTime;
	float goldBallWaitTime;
	
	float colorBall1ThrowTime;
	float colorBall2ThrowTime;
	float goldBallThrowTime;
}RobotconTime_t;
typedef struct{
	int readMotorTime[4];
}ErrorTime_t;
typedef struct{
	int colorBall1;
	int colorBall2;
	int goldBall;
	uint8_t gasSatisfyFlag;
  uint8_t gasSatisfyCnt;
}GetBallStep_t;
/**************typedef area**********/
typedef struct{
  
  /*激光的初始值*/
  float laserInit;
  
  /*激光测得的值*/
  uint16_t laser[2];
  
  /*持球舵机的位置*/
  float holdBallAngle[2];
	
	/*舵机返回的一字节命令*/
	uint8_t steerByte;
  
  /*相机舵机的位置*/
  float cameraAngle;
  
  /*航向角*/
  float courseAngle;
  
  /*横滚角*/
  float pitchAngle;
  
  /*气压*/
  float gasValue;
  float gasAimValue;
	uint16_t gasAdc;
  
  /*延时类型*/
  uint32_t delayTask;
  /*延时需要的时间*/
  uint32_t delayTaskMs[DELAY_TASK_NUM];
	
  float angle;
  float posX;
  float posY;
	float speedX;
	float speedY;
	float AngularVelocity;
	float CarSpeed;
	int posSystemCode[2];
	float angleBais;
	float KalmanZ;
	
	float gasControl;
	uint32_t posSystemReady;
	
//	/*舵机错误指示*/
//	uint8_t holdBall1Error;
//	uint8_t holdBall2Error;
//	uint8_t cameraSteerError;
	
	/*系统复位有关变量*/
	uint32_t isOpenGasReturn;
	
	DataSave_t sDta;
	
	/*重启次数*/
	uint32_t resetTime;
	
	/*重启后可以选择性执行程序*/
	uint32_t resetFlag;
	
	
	/*计算机器人的速度结构体*/
	RobotVel_t robotVel;
  
	/*取球步骤*/
	GetBallStep_t getBallStep;
	/*比赛各个阶段进行的时间*/
	RobotconTime_t raceTime;
	
	ErrorTime_t  errorTime;
}Robot_t ;

#include "gasvalveControl.h"
#include "elmo.h"
#include "shoot.h"
#include "arm_math.h"
#include "errorReport.h"
#endif

