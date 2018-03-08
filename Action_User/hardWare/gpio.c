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

//LED
void LEDInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOE, GPIO_Pin_7);
}


//光电
void PhotoelectricityInit(void)        
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOB, GPIO_Pin_15);
}

//光电检测金球架进入，然后控制助推气阀推
void PhotoelectricityCheckGoldBallInit(void)        
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOE, GPIO_Pin_12);
}

static int IsBallRack=0;
#define IS_A_BaLL_RACK 1
#define NOT_A_Ball_RACK  0
int GoldRackInto(void){
	if(KEYSWITCH_CHECK_GOLD){
		IsBallRack++;
	}else{
		IsBallRack=0;
	}
	if(IsBallRack>=4){
		IsBallRack=0;
		return IS_A_BaLL_RACK;
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
//行程开关触发时间
static int keyOpenTime=0;
//行程开关计数进入自检
void KeySwitchCheck(void){
	static int cntTime=6;
	while(cntTime--){
		if(KEYSWITCH){
			keyOpenTime++;
		}
		else{
			keyOpenTime=0;
		}
		if(keyOpenTime>=3){
			keyOpenTime=0;
			gRobot.sDta.robocon2018=ROBOT_SELF_TEST;
			USART_OUTByDMA("In the RobotSelfTest\r\n");
		}
		Delay_ms(500);
  }
	//
}
