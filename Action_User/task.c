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
	
	//MotorInit();
	
	statusInit();
	
	int laserInit = (Get_Adc_Average(ADC_Channel_15,200));
//	int i=50;
//	while(i--)
//	{
//		laserInit = KalmanFilter(Get_Adc_Average(ADC_Channel_14,1));
//		Delay_ms(1);
//	}
	
//	do{
//		gRobot.laser=(Get_Adc_Average(ADC_Channel_15,200));
//		Delay_ms(1);
//	}
//	while(1);
	//while(gRobot.laser-laserInit<20);
	
	uint32_t value = 1;
	CAN_TxMsg(CAN2,SEND_TO_MOTIONCARD,(uint8_t*)(&value),4);
	
	OSTaskSuspend(OS_PRIO_SELF);
}
/*
0x30 1
0x30 2
0x40 1
*/
void RobotTask(void)
{
	CPU_INT08U  os_err;
	os_err = os_err;
	OSSemSet(PeriodSem, 0, &os_err);
	while(1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		
		AT_CMD_Handle();
		
		if(!(gRobot.progressCase&GET_THE_BALL))
		{
			if(PE_FOR_THE_BALL)
			{	
				uint32_t value = 2;
				CAN_TxMsg(CAN2,SEND_TO_MOTIONCARD,(uint8_t*)(&value),4);
				gRobot.progressCase|=GET_THE_BALL;
			}
		}
		
		if(gRobot.progressCase&READY_FIRST_BALL)
		{
			GasValveControl(GASVALVE_BOARD_ID , CLAW_ID , CLAW_OPEN);
			Delay_ms(10);
			GasValveControl(GASVALVE_BOARD_ID , SHOOT_SMALL_ID , 1);
			GasValveControl(GASVALVE_BOARD_ID , SHOOT_BIG_ID , 1);
			Delay_ms(500);
			GasValveControl(GASVALVE_BOARD_ID , SHOOT_SMALL_ID , 0);
			GasValveControl(GASVALVE_BOARD_ID , SHOOT_BIG_ID , 0);
			GasValveControl(GASVALVE_BOARD_ID , CLAW_ID , CLAW_SHUT);
			gRobot.progressCase&=~READY_FIRST_BALL;
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
		ElmoInit(CAN1);

		//电机位置环
		PosLoopCfg(CAN1, 1, 8000000, 8000000,1250000);

		MotorOn(CAN1,ELMO_BROADCAST_ID); 
		
		PosCrl(CAN1, 1,ABSOLUTE_MODE,0);
}

void MotorDisable(void){
	
	//电机初始化及使能
	ElmoInit(CAN1);
	/*从电机正面看过去，逆时针为正  */
	VelLoopCfg(CAN1,1,10000000,10000000);
	VelLoopCfg(CAN1,2,10000000,10000000);
	VelLoopCfg(CAN1,3,10000000,10000000);
	VelLoopCfg(CAN1,4,10000000,10000000);
	VelLoopCfg(CAN1,5,10000000,10000000);
	
	MotorOn(CAN1,ELMO_BROADCAST_ID); 

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
}
