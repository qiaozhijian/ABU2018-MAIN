#ifndef __GPIO_H
#define __GPIO_H

#include "stm32f4xx_gpio.h"

#define KEYSWITCH		    	(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_7))

#define PE_CHECK_GOLD		    	(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_14))

#define BEEP_ON          		 GPIO_SetBits(GPIOC, GPIO_Pin_3)

#define BEEP_OFF         		 GPIO_ResetBits(GPIOC, GPIO_Pin_3)

#define PE_FOR_THE_BALL								(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0))	

#define KEY_RESET_SWITCH		    	(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3))

#define KEY_TEST_GOLD_SWITCH		    	(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4))

#define RED_BLUE_SWITCH           (GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_1))


  
#define RED_LIGHT_ON             GPIO_ResetBits(GPIOB , GPIO_Pin_10)
#define RED_LIGHT_OFF 					 GPIO_SetBits(GPIOB , GPIO_Pin_10)

#define BLUE_LIGHT_ON						 GPIO_ResetBits(GPIOB , GPIO_Pin_11)
#define BLUE_LIGHT_OFF					 GPIO_SetBits(GPIOB , GPIO_Pin_11)

void GPIO_Init_Pins(GPIO_TypeDef * GPIOx,uint16_t GPIO_Pin,GPIOMode_TypeDef GPIO_Mode);

void KeyInit(void);

void LEDInit(void);

void BeepInit(void);

void PhotoelectricityInit(void);

void PhotoelectricityCheckGoldBallInit(void);

int PrepareForTheBall(void);

int GoldRackInto(void);

void KeySwitchCheck(void);

void KeySwitchIntoBTCtrl(void);

void KeyResetInit(void);

void KeyIntoTestGoldeInit(void);

int KeySwitchIntoReset(void);

int KeySwitchIntoTestGold(void);

void RaBSwitchInit(void);
#endif
