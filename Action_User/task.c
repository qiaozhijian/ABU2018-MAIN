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

	int aaa;
void ConfigTask(void)
{
	CPU_INT08U  os_err;
	os_err = os_err;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	HardWareInit();
	
	MotorInit();
	
	statusInit();
	
	int laserInit = (Get_Adc_Average(ADC_Channel_14,200));
	
	USART_OUT(DEBUG_USART,"Init");
	USART_OUT_F(laserInit,1);
	USART_Enter(1);
	
	do{
		gRobot.laser=(Get_Adc_Average(ADC_Channel_14,100));
		USART_OUT_F(gRobot.laser,1);
		USART_Enter(1);
		Delay_ms(1);
	}
	while(gRobot.laser-laserInit<20);
	
	
	USART_OUT(DEBUG_USART,"ok\r\n");
	MotionCardCMDSend(1);
	USART_OUT(DEBUG_USART,"ok\r\n");
	
	OSTaskSuspend(OS_PRIO_SELF);
}
/*
0x30 1
0x30 2
0x40 1
*/
void SendStatus(void);
void RobotTask(void)
{
	CPU_INT08U  os_err;
	os_err = os_err;
	OSSemSet(PeriodSem, 0, &os_err);
	while(1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		
		AT_CMD_Handle();
		SendStatus();
		/*捡到球的时候*/
		if(!(gRobot.CAN_motionFlag&GET_THE_FIRST_BALL))
		{
			
			if(PE_FOR_THE_BALL)
			{	
				Delay_ms(500);
				MotionCardCMDSend(2);
				gRobot.CAN_motionFlag|=GET_THE_FIRST_BALL;
				
				PitchAngleMotion(10.2f);
				CourseAngleMotion(-76.9f);
				//CourseAngleMotion(-180.f);
				
				Delay_ms(500);
				ROBS_PosCrl(0, 0, 2000);
				USART_OUT(DEBUG_USART,"GET THE BALL");
				USART_OUT(DEBUG_USART,"\r\n");
			}
		}
		static int a;
		if(gRobot.CAN_motionFlag&READY_FIRST_BALL&&a==0)
		{
			a=1;
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
			MotionCardCMDSend(3);
		}
		
		if((gRobot.CAN_motionFlag&GET_THE_FIRST_BALL)&&(!(gRobot.CAN_motionFlag&GET_THE_SECOND_BALL))&&(gRobot.CAN_motionFlag&READY_FIRST_BALL))
		{
			if(PE_FOR_THE_BALL)
			{	
				gRobot.CAN_motionFlag|=GET_THE_SECOND_BALL;

				PitchAngleMotion(30.1f);
				CourseAngleMotion(-79.5f);
				
				Delay_ms(500);
				ROBS_PosCrl(0, 0, 2000);
			}
		}
		
		if(gRobot.CAN_motionFlag&READY_SECOND_BALL)
		{
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
			ROBS_PosCrl(-90, -90, 1000);
			MotionCardCMDSend(4);
			gRobot.CAN_motionFlag&=~READY_SECOND_BALL;
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
void SendStatus(void)
{
	if(gRobot.CAN_motionFlag&GET_THE_FIRST_BALL)
		USART_OUT(DEBUG_USART,"GET_THE_FIRST_BALL\t");
	if(gRobot.CAN_motionFlag&READY_FIRST_BALL)
		USART_OUT(DEBUG_USART,"READY_FIRST_BALL\t");
	if(gRobot.CAN_motionFlag&GET_THE_SECOND_BALL)
		USART_OUT(DEBUG_USART,"GET_THE_SECOND_BALL\t");
	if(gRobot.CAN_motionFlag&READY_SECOND_BALL)
		USART_OUT(DEBUG_USART,"READY_SECOND_BALL\t");
	
	USART_Enter(1);
}

