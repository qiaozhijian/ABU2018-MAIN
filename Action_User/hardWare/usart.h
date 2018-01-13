#ifndef __USART_H
#define __USART_H

#include "stdint.h"
#include "stm32f4xx_usart.h"

#define DEBUG_USART		UART5


void ControlBLE_Init(uint32_t BaudRate);
void Steer1Init(uint32_t BaudRate);
void Steer2Init(uint32_t BaudRate);
void DebugBLE_Init(uint32_t BaudRate);

void USART_OUT(USART_TypeDef* USARTx, const char *Data,...);
char *itoa(int value, char *string, int radix);
void UART5_OUT(const uint8_t *Data, ...);
void USART_OUT_F(float value,int send);
void USART_Enter(int send);
void USART_BLE_SEND(float value);
void processReport(void);
void USART_OUT_ONCE(const char * s);
#endif

