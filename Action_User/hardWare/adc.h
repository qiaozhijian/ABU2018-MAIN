#ifndef __ADC_H
#define __ADC_H	
//#include "sys.h" 
#include "stdint.h"
#include "stm32f4xx_adc.h"

void AdcInit(void); 				
uint16_t  Get_Adc(uint8_t channel); 				
uint16_t Get_Adc_Average(uint8_t channel,uint8_t times);
#endif 
