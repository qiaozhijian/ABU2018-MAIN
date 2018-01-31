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
#include "process.h"
#include "motion.h"
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

static 	OS_STK  RobotTaskStk[ROBOT_TASK_STK_SIZE];

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
                        (OS_STK		* )&RobotTaskStk[ROBOT_TASK_STK_SIZE-1],
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
  
  //给航向，俯仰电机上电初始化时间
  Delay_ms(100);
  
  HardWareInit();
	
  MotorInit();
	
  statusInit();
	
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
		#ifdef TEST
		SelfTest();
		#else		
    AT_CMD_Handle();
		
    processReport();
		
		/*运动状态标志位更新*/
		MotionStatusUpdate();
		
		/*运动参数执行*/
		MotionExecute();
		
		SteerPosCrl(gRobot.cameraAimAngle);
		
		/*运动状态更新*/
    MotionRead();
		
//		USART_SendData(USART6,41);
//		USART_OUT(USART6,"AT\r\n");
		USART_OUT(DEBUG_USART,"%d\t%d\t%d\t%d\t%d\t",PE_FOR_THE_BALL,gRobot.process,(int)(gRobot.courseAngle),(int)(gRobot.posX),(int)(gRobot.posY));
//		USART_OUT_F(gRobot.posX);
//		USART_OUT_F(gRobot.posY);
		USART_OUT_F(gRobot.angle);
		USART_Enter();
		
		#ifndef	DEBUG 
    switch(gRobot.robocon2018)
    {
    case ROBOT_START:
			if(gRobot.posX>100.f)
			{
				PrepareGetBall(BALL_1);			
			}
      if(gRobot.posX>4000.f)
		  {
				gRobot.process=TO_GET_BALL_1;
		  	gRobot.robocon2018=COLORFUL_BALL_1;
			}
      break;
    case COLORFUL_BALL_1: 
      /*完成彩球一的投射*/
      FightForBall1();
      break;
    case COLORFUL_BALL_2:
      /*完成彩球二的投射*/
      FightForBall2();
      break;
    case GOLD_BALL:
      /*完成金球的投射*/
      FightForGoldBall();
      break;
    }
		#endif
#endif
  } 
}

void HardWareInit(void){
  //定时器初始化
  TIM_Init(TIM2, 99, 839, 0, 0);   //1ms主定时器
  
  CAN_Config(CAN1, 500, GPIOB, GPIO_Pin_8, GPIO_Pin_9);
  
  CAN_Config(CAN2, 500, GPIOB, GPIO_Pin_5, GPIO_Pin_6);
  
  /*初始化取球，射击参数结构体*/
  prepareMotionParaInit();
	
	/*持球上舵机初始化*/
  Steer1Init(1000000);
	/*持球下舵机初始化*/
  Steer2Init(1000000);
  //摄像头转台初始化
  CameraSteerInit(1000000);
	
	/*与摄像头通信的串口初始化*/
	CameraInit(115200);
	
	/*接收定位系统数据的串口初始化*/
	GYRO_Init(921600);
	
  /*调试蓝牙*/
  DebugBLE_Init(921600);
	
  /*光电初始化*/
  PhotoelectricityInit();
  
  //蜂鸣器PE7
  GPIO_Init_Pins(GPIOE, GPIO_Pin_7, GPIO_Mode_OUT);
  
#ifndef	DEBUG 
  Delay_ms(3000);
  Enable_ROBS();//使能舵机
#endif
  
}
void MotorInit(void){
	#ifndef	DEBUG 
  //电机初始化及使能
  ElmoInit(CAN2);
  
  //电机位置环
  PosLoopCfg(CAN2, 5, 100000, 100000,100000);
  //电机位置环
  PosLoopCfg(CAN2, 6, 100000, 100000,100000);
  
  MotorOn(CAN2,5); 
  MotorOn(CAN2,6); 
	#endif
}	



//状态初始化，张开抓投球，等舵机到0的时候，闭合之后舵机才能转
void statusInit(void)
{	
  /*运动控制状态初始化*/
  SetMotionFlag(~AT_CLAW_STATUS_OPEN);
  SetMotionFlag(AT_STEER_READY);
  SetMotionFlag(~AT_SHOOT_BIG_ENABLE);
  SetMotionFlag(~AT_SHOOT_BIG_ENABLE);
  
	#ifndef	DEBUG 
	ClawShut();
	BoostPoleReturn();
	ShootSmallShut();
	ShootBigShut();
	ShootLedOff();
	GoldBallGraspStairOneOn();
	GoldBallGraspStairTwoOn();
	
	Delay_ms(3000);
  
  /*与上一次的调试数据区分开*/
  USART_Enter();
  USART_Enter();
  USART_Enter();
  USART_Enter();
  USART_Enter();
  USART_Enter();
  
  PrepareGetBall(READY);
  
	/*等待慢转动状态完成*/
  Delay_ms(5000);
	
	/*恢复快速转动状态*/
  PosLoopCfg(CAN2, 5, 8000000, 8000000,1250000);        
  PosLoopCfg(CAN2, 6, 8000000, 8000000,800000);
	#endif
	
  gRobot.robocon2018=ROBOT_START;
}	


