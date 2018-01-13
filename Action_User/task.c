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
	
	gRobot.process=TO_START;
	gRobot.laserInit=(Get_Adc_Average(ADC_Channel_14,200));
	USART_OUT(DEBUG_USART,"laserInit %d\r\n",gRobot.laserInit);
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
		
		processReport();
		
		switch(gRobot.process)
		{
			/*出发*/
			case TO_START:
//				gRobot.laser=(Get_Adc_Average(ADC_Channel_14,100));
//				
//				if(gRobot.laser-gRobot.laserInit>20.f)
//				{
//					MotionCardCMDSend(NOTIFY_MOTIONCARD_START);
//					
//					gRobot.process=TO_GET_BALL_1;
//					
//				}
				break;
			/*去取第一个球*/
			case TO_GET_BALL_1:
				
				if(PE_FOR_THE_BALL)
				{	
					Delay_ms(500);
					
					MotionCardCMDSend(NOTIFY_MOTIONCARD_GOT_BALL1);
					
					PrepareShootBall(BALL_1);
					
					gRobot.process=TO_THE_AREA_1;
					
				}
				break;
			/*第一个球取球完毕，去投射区一*/
			case TO_THE_AREA_1:
				if(!PE_FOR_THE_BALL)
				{
					MotionCardCMDSend(NOTIFY_MOTIONCARD_LOSE_BALL1);
				}
				break;
			/*到达投射区一，射球*/
			case TO_THROW_BALL_1:
				if(PE_FOR_THE_BALL)
				{
				/*射球*/
				ShootBall();
				/*给延时使发射杆能执行到位*/
				Delay_ms(150);
				/*通知控制卡*/
				MotionCardCMDSend(NOTIFY_MOTIONCARD_SHOT_BALL1);
				/*射球机构复位*/
				ShootReset();
				/*准备接球二*/
				PrepareGetBall(BALL_2);
				/*进入下一状态*/
				gRobot.process=TO_GET_BALL_2;
				}
				break;
			/*去取第二个球*/
			case TO_GET_BALL_2:
				if(PE_FOR_THE_BALL)
				{	
					/*扫到光电后，为了更稳地接到球而给的延时*/
					Delay_ms(500);
					
					gRobot.process=TO_THE_AREA_2;
					
					MotionCardCMDSend(NOTIFY_MOTIONCARD_GOT_BALL2);
					
					PrepareShootBall(BALL_2);
					
				}
				break;
			/*第二个球取球完毕，去投射区二*/
			case TO_THE_AREA_2:
				if(!PE_FOR_THE_BALL)
				{
					MotionCardCMDSend(NOTIFY_MOTIONCARD_LOSE_BALL2);
				}
				break;
			/*到达投射区二，射球*/
			case TO_THROW_BALL_2:
				if(PE_FOR_THE_BALL)
				{
				/*射球*/
				ShootBall();
			
				/*给延时使发射杆能执行到位*/
				Delay_ms(150);
			
				MotionCardCMDSend(NOTIFY_MOTIONCARD_SHOT_BALL2);
			
				/*射球机构复位*/
				ShootReset();
			
				/*准备接球三*/
				PrepareGetBall(BALL_3);
			
				gRobot.process=TO_GET_BALL_3;
				}
				break;
			/*去取第三个球*/
			case TO_GET_BALL_3:
				if(PE_FOR_THE_BALL)
				{	
					/*扫到光电后，为了更稳地接到球而给的延时*/
					Delay_ms(500);
					
					gRobot.process=TO_THE_AREA_3;
					
					MotionCardCMDSend(NOTIFY_MOTIONCARD_GOT_BALL3);
					
					PrepareShootBall(BALL_3);
					
				}
				break;
			/*第三个球取球完毕，去投射区三*/
			case TO_THE_AREA_3:
				if(!PE_FOR_THE_BALL)
				{
					MotionCardCMDSend(NOTIFY_MOTIONCARD_LOSE_BALL3);
				}
				break;
			/*到达投射区三，射球*/
			case TO_THROW_BALL_3:
				if(PE_FOR_THE_BALL)
				{
				/*射球*/
				ShootBall();
			
				/*给延时使发射杆能执行到位*/
				Delay_ms(150);
			
				/*射球机构复位*/
				ShootReset();
			
				gRobot.process=END_COMPETE;
				}
				break;
		}
	}
}

void HardWareInit(void){
	//定时器初始化
	TIM_Init(TIM2, 99, 839, 0, 0);   //1ms主定时器
	
	CAN_Config(CAN1, 500, GPIOB, GPIO_Pin_8, GPIO_Pin_9);
	
	CAN_Config(CAN2, 500, GPIOB, GPIO_Pin_5, GPIO_Pin_6);

	/*初始化取球，射击参数结构体*/
	prepareMotionParaInit();
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
	
	PrepareGetBall(BALL_1);
	
}


