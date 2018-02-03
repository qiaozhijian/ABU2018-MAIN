#ifndef __USART_H
#define __USART_H

#include "stdint.h"
#include "stm32f4xx_usart.h"

#define DEBUG_USART		USART1

#define RS485_TX_EN					GPIO_SetBits(GPIOC,GPIO_Pin_13);
#define RS485_RX_EN					GPIO_ResetBits(GPIOC,GPIO_Pin_13);

#define CAMERA_USART										USART6

/*…„œÒÕ∑√¸¡ÓºØ*/
#define CAMERA_START														3
#define CAMERA_SHUT_ALL													0
#define CAMERA_OPEN_NEAR												1
#define CAMERA_OPEN_FAR													2

void DebugBLE_Init(uint32_t BaudRate);
void Steer2Init(uint32_t BaudRate);
void SteerInit(uint32_t BaudRate);
void GYRO_Init(uint32_t BaudRate);
void ControlBLE_Init(uint32_t BaudRate);
void CameraTalkInit(uint32_t BaudRate);

void RS485_Send_Data(unsigned char *buf,unsigned char len);
void USART_OUT(USART_TypeDef* USARTx, const char *Data,...);
char *itoa(int value, char *string, int radix);
void USART_OUT_F(float value);
void USART_Enter(void);
void USART_BLE_SEND(float value);
void USART_OUT_ONCE(const char * s);
#endif

