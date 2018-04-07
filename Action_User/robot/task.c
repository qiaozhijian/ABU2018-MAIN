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
#include "iwdg.h"
#include "DataRecover.h"
#include "debug.h"
#include "robot.h"
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
Robot_t gRobot={0};

void ConfigTask(void)
{
  CPU_INT08U  os_err;
  os_err = os_err;  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
// DebugBLE_Init(921600);
  /*dma初始化*/
  USARTDMASendInit(DEBUG_USART,USART1DMASendBuf,&DebugBLE_Init,921600);
	
	USART_OUTByDMA("START\r\n");
//  USART_OUTByDMA("START\r\n");

  #ifndef TEST
	USART_OUTByDMA("SoftWareReset\r\n");
//  USART_OUTByDMA("SoftWareReset\r\n");
  SoftWareReset();
  #endif
	
  
  HardWareInit();
  if(!gRobot.resetFlag)
  {
			//给航向，俯仰电机上电初始化时间
			Delay_ms(100);
			/*电机初始化*/
			MotorInit();
			if(gRobot.sDta.robocon2018!=ROBOT_SELF_TEST){
			/*状态初始化*/
			statusInit();
		}
  }
	
  #ifndef TEST
  IWDG_Init(1,50); // 11ms-11.2ms
  #endif
	
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
		/*清除信号量*/
		OSSemSet(PeriodSem, 0, &os_err);
		#ifdef DEBUG
			IWDG_Feed();
			debugFunction();
		#else
			#ifdef TEST
			  SelfTest();
			#else		
				/*喂狗，判断程序是否正常运行，另一处喂狗在延时函数里*/
				IWDG_Feed();
				
				/*调试数据发送*/
				DebugDataUSART_OUT();
				
				
				/*蓝牙命令处理&&和进入平板调试模式*/
				if(KEYSWITCH){
					KeySwitchIntoBTCtrl();
				}
				
				/*过程报告*/
				//processReport();
				
				/*运动状态标志位更新*/
				MotionStatusUpdate();
				
				/*运动参数执行*/
				MotionExecute();
				
				/*运动状态更新*/
				MotionRead();
				
				
				switch(gRobot.sDta.robocon2018)
				{
					case ROBOT_CONTROL_BY_BT:
						AT_CMD_Handle();
					break;
					
					case ROBOT_SELF_TEST:
						 RobotSelfTest();
					break;
					
					case ROBOT_PREPARE:
						if(gRobot.sDta.AT_motionFlag&AT_PREPARE_READY)
						{
							//灯亮两秒，蜂鸣器响两秒，表示准备完成
							BEEP_ON;
							ShootLedOn();
							MotionCardCMDSend(NOTIFY_MOTIONCARD_PREPARE_FINISH);
							Delay_ms(2000);
							ShootLedOff();
							BEEP_OFF;
							//收到控制卡发数然后将AT_PREPARE_READY标志位置为零
							SetMotionFlag(~AT_PREPARE_READY);
							gRobot.sDta.robocon2018=ROBOT_START;
						}
						break;
						
					case ROBOT_START:
						if(gRobot.posX>100.f)
						{
							PrepareGetBall(BALL_1);			
						}
						if(gRobot.posX>2000.f)
						{
							gRobot.sDta.process=TO_GET_BALL_1;
							gRobot.sDta.robocon2018=COLORFUL_BALL_1;
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
					
					case INTO_HARDFAULT:
						ShootLedOn();
						BEEP_ON;
					  USART_OUTByDMA("INTO_HARDFAULT!!!");
					break;
				}
				USART_OUTByDMA("\r\n");
			#endif
		#endif
  } 
}

void HardWareInit(void){
  USART_OUTByDMA("HardWareInit\r\n");
  //定时器初始化
  TIM_Init(TIM2, 99, 839, 0, 0);   //1ms主定时器
	
  CAN_Config(CAN1, 500, GPIOB, GPIO_Pin_8, GPIO_Pin_9);
  
  CAN_Config(CAN2, 500, GPIOB, GPIO_Pin_5, GPIO_Pin_6);
  
  /*初始化取球，射击参数结构体*/
  prepareMotionParaInit();
	
  //摄像头转台初始化
  SteerInit(1000000);
	
  Steer2Init(1000000);
	
  /*与摄像头通信的串口初始化*/
  CameraTalkInit(256000);
	
  /*接收定位系统数据的串口初始化*/
  GYRO_Init(921600);
	
  /*调试蓝牙*/
  ControlBLE_Init(921600);
	
  /*光电初始化*/
  PhotoelectricityInit();
	
	/*检测到金球架进来就推助推气阀的光电*/
  PhotoelectricityCheckGoldBallInit();
	
  //蜂鸣器PE7
  GPIO_Init_Pins(GPIOC, GPIO_Pin_3, GPIO_Mode_OUT);
	
  TIM_Init(TIM7,99,83,0,0);					//100us
  
}
void MotorInit(void){
	
  //电机初始化
  ElmoInit(CAN2);
  
  //电机位置环
  PosLoopCfg(CAN2, PITCH_MOTOR_ID, 600000, 600000,600000);
  //电机位置环
  PosLoopCfg(CAN2, COURCE_MOTOR_ID, 600000, 600000,600000);
  //电机位置环
  PosLoopCfg(CAN2, UP_STEER_MOTOR_ID, 10000000, 10000000,20000000);
		
	PosLoopCfg(CAN2, DOWN_STEER_MOTOR_ID , 6400000 , 6400000 , 12800000);
//	PosLoopCfg(CAN2, DOWN_STEER_MOTOR_ID, 10000000, 10000000,20000000);

	//电机使能
  MotorOn(CAN2,PITCH_MOTOR_ID); 
  MotorOn(CAN2,COURCE_MOTOR_ID); 
	MotorOn(CAN2,UP_STEER_MOTOR_ID);
	MotorOn(CAN2,DOWN_STEER_MOTOR_ID); 

}	



//状态初始化，张开抓投球，等舵机到0的时候，闭合之后舵机才能转
void statusInit(void)
{	
	USART_OUTByDMA("statusInit start\r\n");
  Delay_ms(1000);
	
  /*运动控制状态初始化*/
	
	/*射球完毕时的归位小气阀重置*/
	ShootSmallShut();
	/*射球时的助力大气阀重置*/
	ShootBigShut();
	ShootLedOff();
	/*金球架抓取二级气阀打开*/
	GoldBallGraspStairTwoOn();
	/*下爪手臂向上抬*/
	LowerClawStairOff();
	USART_OUTByDMA("statusInit step 1\r\n");
	Delay_ms(1500);
	
	#ifndef TEST
	USART_OUTByDMA("statusInit step 2\r\n");
  /*与上一次的调试数据区分开*/
  USART_OUTByDMA("\r\n");
  USART_OUTByDMA("\r\n");
  USART_OUTByDMA("\r\n");
  USART_OUTByDMA("\r\n");
  USART_OUTByDMA("\r\n");
  USART_OUTByDMA("\r\n");
	
	PrepareGetBall(READY);
	
	USART_OUTByDMA("statusInit step PrepareGetBall\r\n");
	#endif
	
	/*恢复快速转动状态*/
  PosLoopCfg(CAN2, PITCH_MOTOR_ID, 8000000, 8000000,1250000);        
  PosLoopCfg(CAN2, COURCE_MOTOR_ID, 8000000, 8000000,12500000);
	
	#ifdef TEST
		#ifndef DEBUG
		#endif
		BEEP_ON;
		ShootLedOn();
		Delay_ms(2000);
		ShootLedOff();
		BEEP_OFF;
	#endif
	
	SetMotionFlag(AT_IS_SEND_DEBUG_DATA);
	/*自检检测*/
	KeySwitchCheck();
	
	if(gRobot.sDta.robocon2018!=ROBOT_SELF_TEST){
	  /*准备工作完毕*/
		gRobot.sDta.robocon2018=ROBOT_PREPARE;
	}
	
	USART_OUTByDMA("statusInit step finish\r\n");
}	


