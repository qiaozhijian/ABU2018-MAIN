#ifndef __GPIO_H
#define __GPIO_H

#include "stm32f4xx_gpio.h"

#define KEYSWITCH		    	(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2))

#define BEEP_ON          		 GPIO_SetBits(GPIOE, GPIO_Pin_7)

#define BEEP_OFF         		 GPIO_ResetBits(GPIOE, GPIO_Pin_7)

#define PE_FOR_THE_BALL								(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_12))		

void GPIO_Init_Pins(GPIO_TypeDef * GPIOx,uint16_t GPIO_Pin,GPIOMode_TypeDef GPIO_Mode);

void KeyInit(void);

void LEDInit(void);

void BeepInit(void);

void PhotoelectricityInit(void);

#endif
