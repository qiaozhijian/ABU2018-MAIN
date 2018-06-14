/**
  ******************************************************************************
  * @file    gpio.c
  * @author  Calcus Lee
  * @version V1.0.1
  * @date    9-August-2013
  * @brief   functions of gpio
  ******************************************************************************
  */
  
#include "gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "gasvalveControl.h"
#include "timer.h"
#include "usart.h"
#include "task.h"
#include "dma.h"
#include "robot.h"
#include "motion.h"
#include "process.h"
/**
  * @brief  set the pins of a specific GPIO group to be input or output driver pin.
  * @param  GPIOx: where x can be A-I.
  * @param  GPIO_Pin: The specific pins you want to select in group GPIOX.
			This parameter can be combination of GPIO_Pin_x where x can be (0..15) @ref GPIO_pins_define
  * @param  GPIO_Mode. the value can be one of the following value
		    GPIO_Mode_IN   
		    GPIO_Mode_OUT 
		    GPIO_Mode_AF  
		    GPIO_Mode_AN
  * @retval None
  * @author Calcus Lee
**/
            
void GPIO_Init_Pins(GPIO_TypeDef * GPIOx,
					uint16_t GPIO_Pin,
					GPIOMode_TypeDef GPIO_Mode)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	  /* Enable GPIOx, clock */  
	switch((uint32_t)GPIOx)
	{
		case GPIOA_BASE: 
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
			break;
		
		case GPIOB_BASE: 
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
			break;
		
		case GPIOC_BASE: 
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
			break;

		case GPIOD_BASE: 
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
			break;

		case GPIOE_BASE: 
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
			break;

		case GPIOF_BASE: 
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
			break;

		case GPIOG_BASE: 
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
			break;

		case GPIOH_BASE: 
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
			break;

		case GPIOI_BASE: 
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
			break;
	
		default: 
			break;
	}
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode;

	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		
	GPIO_Init(GPIOx, &GPIO_InitStructure);	
}
//行程开关 PD7
void KeyInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);	
}

void KeyResetInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);	
}

void KeyIntoTestGoldeInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
}

//行程开关 PD7
void RaBSwitchInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);	
}
//LED
void LEDInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	RED_LIGHT_OFF;
	BLUE_LIGHT_ON;
}


//光电
void PhotoelectricityInit(void)        
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOB, GPIO_Pin_0);
}

//光电检测金球架进入，然后控制助推气阀推
void PhotoelectricityCheckGoldBallInit(void)        
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOD, GPIO_Pin_14);
}

extern Robot_t gRobot;
static int IsBallRack=0;
#define IS_A_BaLL_RACK 1
#define NOT_A_Ball_RACK  0
int GoldRackInto(void){	
	while(1){
		if(gRobot.sDta.AT_motionFlag&AT_GET_MOTIONCARD_GET_GOLDBALL_AREA){
				ShootLedOn();
				SetMotionFlag(~AT_GET_MOTIONCARD_GET_GOLDBALL_AREA);
	  }
		Delay_ms(2);
		USART_OUTByDMA("BallRack %d ",PE_CHECK_GOLD);
		
		/*进入重启*/
		if(KEY_RESET_SWITCH&&PE_CHECK_GOLD!=1){
			  Delay_ms(3);
				if(KeySwitchIntoReset()){
					break;
				}
		}
		
		if(PE_CHECK_GOLD){
			IsBallRack++;
		}else{
			IsBallRack=0;
		}
		if(IsBallRack>=3){
			IsBallRack=0;
			return IS_A_BaLL_RACK;
		}
	}
	return NOT_A_Ball_RACK;
}
/*光电25ms触发说明拿到球*/
#define IS_A_BaLL 1
#define NOT_Ball  0

int PrepareForTheBall(void){
	static int IsBall=0;
	if(PE_FOR_THE_BALL){
		IsBall++;
	}else{
		IsBall=0;
	}
	if(IsBall>=5){
		IsBall=0;
		return IS_A_BaLL;
	}
	return NOT_Ball;
}
extern Robot_t gRobot;
//行程开关计数进入自检，或者擦轮
void KeySwitchCheck(void){
	//自检行程开关触发时间
  int keySelfCheckTouchTime=0;
	//擦轮行程开关触发时间
	int keyWipeWheelTouchTime=0;
	int cntTime=6;
	while(cntTime--){
		if(KEYSWITCH){
			keySelfCheckTouchTime++;
		}
		else{
			keySelfCheckTouchTime=0;
		}
		if(keySelfCheckTouchTime>=3){
			gRobot.sDta.robocon2018=ROBOT_SELF_TEST;
			USART_OUTByDMA("In the RobotSelfTest\r\n");
			SetMotionFlag(~AT_IS_SEND_DEBUG_DATA);
		}
		
		if(KEY_RESET_SWITCH){
			keyWipeWheelTouchTime++;
		}
		else{
			keyWipeWheelTouchTime=0;
		}
		if(keyWipeWheelTouchTime>=3){
			PosLoopCfg(CAN2, COURCE_MOTOR_ID, 600000, 600000,600000);
			Delay_ms(1000);
			gRobot.sDta.robocon2018=ROBOT_CONTROL_BY_BT;
			//让航向换个角度
			gRobot.sDta.courseAimAngle=135.f;
			CourseAngleMotion(gRobot.sDta.courseAimAngle);
			MotionCardCMDSend(NOTIFY_MOTIONCARD_WIPE_WHEEL);
			USART_OUTByDMA("In the WipeWheel\r\n");
		}
		
		Delay_ms(500);
  }
	//
}
void KeySwitchIntoBTCtrl(void){
	static int keyIntoBTTouchTime = 0;
	if(KEYSWITCH){
	  keyIntoBTTouchTime++;
	}
	else{
	  keyIntoBTTouchTime=0;
	}
	if(keyIntoBTTouchTime>=650){
		keyIntoBTTouchTime=0;
		//通知控制卡进入平板控制模式
		MotionCardCMDSend(NOTIFY_MOTIONCARD_PREPARE_FINISH);
		Delay_ms(300);
		MotionCardCMDSend(NOTIFY_MOTIONCARD_INTO_BT_CTRL);
		gRobot.sDta.robocon2018=ROBOT_CONTROL_BY_BT;
		USART_OUTByDMA("In the RobotBTCTRL\r\n");
		SetMotionFlag(~AT_IS_SEND_DEBUG_DATA);
		BEEP_ON;
		for(int i=0;i<3;i++){
			ShootLedOn();
			Delay_ms(300);
			ShootLedOff();
			Delay_ms(300);
		}
		BEEP_OFF;

	}
}

int KeySwitchIntoReset(void){
	//重启进程一共需要按两次
	static int resetProgress=0;
	//按住行程开关的时间
	static int keyResetTouchTime=0;
	if(KEY_RESET_SWITCH){
	  keyResetTouchTime++;
	}
	else{
	  keyResetTouchTime=0;
	}
	
	switch(resetProgress){
		case 0:
			if(keyResetTouchTime>=100){
				resetProgress++;
				keyResetTouchTime=0;
				//通知控制卡准备重启
				MotionCardCMDSend(NOTIFY_MOTIONCARD_INTO_RESET);
				BEEP_ON;
				ShootLedOn();
				USART_OUTByDMA("In the RobotReset\r\n");
				
				ExtendCarOff();
				GoldBallGraspStairTwoOn();

				PrepareGetBall(READY);
				if(gRobot.sDta.AT_motionFlag&AT_RESET_SHOOT_GOLD)
					gRobot.sDta.gasAimValue= GetPrepareShootGoldBallGasAim();
				
				SetShootTimeZero();
				Delay_ms(1200);
		   	PosLoopCfg(CAN2, PITCH_MOTOR_ID, 8000000, 8000000,1250000);        
        PosLoopCfg(CAN2, COURCE_MOTOR_ID, 8000000, 8000000,12500000);
				BEEP_OFF;
				
				return 1;
	    }
		break;
		
		case 1:
			if(keyResetTouchTime>=100){
			  BEEP_ON;
				ShootLedOff();
				//重启步骤归零
		    resetProgress=0;
				//取球步骤归零
				gRobot.getBallStep.colorBall1=0;
				gRobot.getBallStep.colorBall2=0;
        gRobot.getBallStep.goldBall=0;
				//将某一个金球置位0,这个标志位控制气压满足范围
				gRobot.sDta.WhichGoldBall=0;
		    //将标志位全部清空,但是要判断是否在金球区重启
				if(gRobot.sDta.AT_motionFlag&AT_RESET_SHOOT_GOLD)
				{
					USART_OUTByDMA("\r\nset reset gold\t");
					gRobot.sDta.AT_motionFlag=0;
					SetMotionFlag(AT_RESET_SHOOT_GOLD);
				}else
				{
					USART_OUTByDMA("\r\nset unreset gold\t");
					gRobot.sDta.AT_motionFlag=0;
				}
				Delay_ms(1000);
				gRobot.sDta.robocon2018=INTO_RESET_PREPARE;
				SetMotionFlag(AT_IS_SEND_DEBUG_DATA);
				SetMotionFlag(AT_RESET_THE_ROBOT);
			  //告诉控制卡抱死重启模式选择
				if(gRobot.sDta.AT_motionFlag&AT_RESET_SHOOT_GOLD)
					MotionCardCMDSend(NOTIFY_MOTIONCARD_RESET_GOLD);
				else
					MotionCardCMDSend(NOTIFY_MOTIONCARD_RESET_ALL);
				SetMotionFlag(AT_IS_SEND_DEBUG_DATA);
				BEEP_OFF;
				Delay_ms(200);
				
				return 1;
			}
		break;
	}
	
	return 0;
}

int KeySwitchIntoTestGold(void){
	//按住行程开关的时间
	static int keyTestTouchTime=0;
	if(KEY_TEST_GOLD_SWITCH){
	  keyTestTouchTime++;
	}
	else{
	  keyTestTouchTime=0;
	}
	
	if(keyTestTouchTime>=600){
		keyTestTouchTime=0;
		
		//通知控制卡准备测试摩擦
		MotionCardCMDSend(NOTIFY_MOTIONCARD_INTO_TEST_GOLD);
		BEEP_ON;
		ShootLedOn();
		USART_OUTByDMA("In the TEST_GOLD\r\n");
		
		ExtendCarOn();
		Delay_ms(1500);
		BEEP_OFF;
		ShootLedOff();
		return 1;
	}
	 return 0;
}

