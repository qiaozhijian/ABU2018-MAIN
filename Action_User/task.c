#include "includes.h"
#include <app_cfg.h>
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "elmo.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_usart.h"
#include "gasvalveControl.h"
#include "dma.h"
#include "task.h"
#include "shoot.h"
#include "customer.h"
#include "adc.h"
#include "algorithm.h"
#include "steer.h"
/*
===============================================================
						信号量定义
===============================================================
*/
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;

//定义互斥型信号量用于管理CAN发送资源
OS_EVENT *CANSendMutex;

static  OS_STK  App_ConfigStk[Config_TASK_START_STK_SIZE];

static 	OS_STK  RobotTaskStk[SHOOT_TASK_STK_SIZE];

void App_Task()
{
	CPU_INT08U  os_err;
	os_err = os_err;		  /*防止警告...*/

	/*创建信号量*/
	PeriodSem				=	OSSemCreate(0);

	//创建互斥型信号量
	CANSendMutex			=   OSMutexCreate(9,&os_err);

	/*创建任务*/
	os_err = OSTaskCreate(	(void (*)(void *)) ConfigTask,				/*初始化任务*/
							(void		  * ) 0,
							(OS_STK		* )&App_ConfigStk[Config_TASK_START_STK_SIZE-1],
							(INT8U		   ) Config_TASK_START_PRIO);

	os_err = OSTaskCreate(	(void (*)(void *)) RobotTask,
							(void		  * ) 0,
							(OS_STK		* )&RobotTaskStk[SHOOT_TASK_STK_SIZE-1],
							(INT8U		   ) SHOOT_TASK_PRIO);

}
/*函数声明*/
void MotorInit(void);
void HardWareInit(void);
void statusInit(void);

/*全局变量的声明*/
Robot_t gRobot;

void ConfigTask(void)
{
	CPU_INT08U  os_err;
	os_err = os_err;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	HardWareInit();
	
	MotorInit();
	
	statusInit();
	
	gRobot.process=TO_START;
	gRobot.laserInit=(Get_Adc_Average(ADC_Channel_14,200));
	
	OSTaskSuspend(OS_PRIO_SELF);
}

void RobotTask(void)
{
	CPU_INT08U  os_err;
	os_err = os_err;
	OSSemSet(PeriodSem, 0, &os_err);
	while(1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		
		AT_CMD_Handle();
		
		switch(gRobot.process)
		{
			case TO_START:
				gRobot.laser=(Get_Adc_Average(ADC_Channel_14,100));
				if(gRobot.laser-gRobot.laserInit>20.f)
				{
					MotionCardCMDSend(1);
					
					gRobot.process=TO_GET_BALL_1;
				}
				USART_OUT(DEBUG_USART,"TO_START\r\n");
				break;
			case TO_GET_BALL_1:
				
				if(PE_FOR_THE_BALL)
				{	
					Delay_ms(500);
					MotionCardCMDSend(2);
					
					gRobot.process=TO_THE_AREA_1;
					
					PitchAngleMotion(10.2f);
					CourseAngleMotion(-76.9f);
					
					Delay_ms(500);
					ROBS_PosCrl(0, 0, 2000);
				
					
				}
				USART_OUT(DEBUG_USART,"TO_GET_BALL_1\r\n");
				break;
			case TO_THE_AREA_1:
				USART_OUT(DEBUG_USART,"TO_THE_AREA_1\r\n");
				break;
			case TO_THROW_BALL_1:
				/*扔球*/
				GasValveControl(GASVALVE_BOARD_ID , CLAW_ID , CLAW_OPEN);
				Delay_ms(300);
				GasValveControl(GASVALVE_BOARD_ID , SHOOT_SMALL_ID , 1);
				GasValveControl(GASVALVE_BOARD_ID , SHOOT_BIG_ID , 1);
				Delay_ms(150);
				/*复位*/
				GasValveControl(GASVALVE_BOARD_ID , SHOOT_SMALL_ID , 0);
				GasValveControl(GASVALVE_BOARD_ID , SHOOT_BIG_ID , 0);
				GasValveControl(GASVALVE_BOARD_ID , CLAW_ID , CLAW_SHUT);
				/*去接下一个球*/
				CourseAngleMotion(0.f);
				PitchAngleMotion(27.1f);
				ROBS_PosCrl(-90, -90, 1000);
				/*通知控制卡*/
				MotionCardCMDSend(3);
				/*进入下一状态*/
				gRobot.process=TO_GET_BALL_2;
				USART_OUT(DEBUG_USART,"TO_THROW_BALL_1\r\n");
				break;
			case TO_GET_BALL_2:
				if(PE_FOR_THE_BALL)
				{	
					
					PitchAngleMotion(30.1f);
					CourseAngleMotion(-79.5f);
					
					MotionCardCMDSend(4);
					gRobot.process=TO_THE_AREA_2;
					Delay_ms(500);
					ROBS_PosCrl(0, 0, 2000);
				
				}
				USART_OUT(DEBUG_USART,"TO_GET_BALL_2\r\n");
				break;
			case TO_THE_AREA_2:
				USART_OUT(DEBUG_USART,"TO_THE_AREA_2\r\n");
				
				break;
			case TO_THROW_BALL_2:
				GasValveControl(GASVALVE_BOARD_ID , CLAW_ID , CLAW_OPEN);
				Delay_ms(300);
				GasValveControl(GASVALVE_BOARD_ID , SHOOT_SMALL_ID , 1);
				GasValveControl(GASVALVE_BOARD_ID , SHOOT_BIG_ID , 1);
				Delay_ms(150);
				/*复位*/
				GasValveControl(GASVALVE_BOARD_ID , SHOOT_SMALL_ID , 0);
				GasValveControl(GASVALVE_BOARD_ID , SHOOT_BIG_ID , 0);
				GasValveControl(GASVALVE_BOARD_ID , CLAW_ID , CLAW_SHUT);
				/*去接下一个球*/
				CourseAngleMotion(0.f);
				PitchAngleMotion(30.1f);
				ROBS_PosCrl(90, 90, 1000);
				MotionCardCMDSend(5);
				gRobot.process=TO_GET_BALL_3;
				USART_OUT(DEBUG_USART,"TO_THROW_BALL_2\r\n");
				break;
			case TO_GET_BALL_3:
				if(PE_FOR_THE_BALL)
				{	
					
					PitchAngleMotion(32.5f);
					CourseAngleMotion(-90.f);
					
					Delay_ms(500);
					MotionCardCMDSend(6);
					gRobot.process=TO_THE_AREA_3;
					Delay_ms(500);
					ROBS_PosCrl(0, 0, 2000);
				
				}
				USART_OUT(DEBUG_USART,"TO_GET_BALL_2\r\n");
				break;
			case TO_THE_AREA_3:
				USART_OUT(DEBUG_USART,"TO_THE_AREA_2\r\n");
				
				break;
			case TO_THROW_BALL_3:
				GasValveControl(GASVALVE_BOARD_ID , CLAW_ID , CLAW_OPEN);
				Delay_ms(300);
				GasValveControl(GASVALVE_BOARD_ID , SHOOT_SMALL_ID , 1);
				GasValveControl(GASVALVE_BOARD_ID , SHOOT_BIG_ID , 1);
				Delay_ms(150);
				/*复位*/
				GasValveControl(GASVALVE_BOARD_ID , SHOOT_SMALL_ID , 0);
				GasValveControl(GASVALVE_BOARD_ID , SHOOT_BIG_ID , 0);
				GasValveControl(GASVALVE_BOARD_ID , CLAW_ID , CLAW_SHUT);
				/*去接下一个球*/
//				CourseAngleMotion(0.f);
//				PitchAngleMotion(30.1f);
//				ROBS_PosCrl(-90, -90, 1000);
//				MotionCardCMDSend(7);
				gRobot.process=15;
				USART_OUT(DEBUG_USART,"TO_THROW_BALL_2\r\n");
				break;
		}
	}
}

void HardWareInit(void){
	//定时器初始化
	TIM_Init(TIM2, 99, 839, 0, 0);   //1ms主定时器
	
	CAN_Config(CAN1, 500, GPIOB, GPIO_Pin_8, GPIO_Pin_9);
	
	CAN_Config(CAN2, 500, GPIOB, GPIO_Pin_5, GPIO_Pin_6);

	//舵机1串口初始化
	Steer1Init(1000000);
	//舵机2串口初始化
	Steer2Init(1000000);
	//控制蓝牙
	ControlBLE_Init(115200);
	//调试蓝牙
	DebugBLE_Init(921600);
	//激光初始化
	Laser_Init();
	//光电初始化
	PhotoelectricityInit();
	
	Delay_ms(1000);
	
	Enable_ROBS();//使能舵机
	
}
void MotorInit(void){
		//电机初始化及使能
		ElmoInit(CAN2);

		//电机位置环
		PosLoopCfg(CAN2, 5, 8000000, 8000000,1250000);
		//电机位置环
		PosLoopCfg(CAN2, 6, 8000000, 8000000,800000);

		MotorOn(CAN2,5); 
		MotorOn(CAN2,6); 
}

void MotorDisable(void){
	
	//电机初始化及使能
	ElmoInit(CAN2);
	/*从电机正面看过去，逆时针为正  */
	VelLoopCfg(CAN2,5,10000000,10000000);
	VelLoopCfg(CAN2,6,10000000,10000000);
	
	MotorOff(CAN2,ELMO_BROADCAST_ID); 

	//爪子状态控制
	//gasMotion();
}
void statusInit(void)
{
	/*运动控制状态初始化*/
	SetMotionFlag(~AT_CLAW_STATUS_OPEN);
	SetMotionFlag(AT_STEER_READY);
	SetMotionFlag(~AT_SHOOT_BIG_ENABLE);
	SetMotionFlag(~AT_SHOOT_BIG_ENABLE);
	
	Delay_ms(2000);
	Delay_ms(2000);
	Delay_ms(2000);
	/*接球角度*/
	PitchAngleMotion(26.6);
	CourseAngleMotion(0);
	GasValveControl(GASVALVE_BOARD_ID , CLAW_ID , CLAW_SHUT);
	ROBS_PosCrl(90, 91, 1000);
	
	
}


