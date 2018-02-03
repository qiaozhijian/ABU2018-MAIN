/**
  *******************************************************************************************************
  * @filE	dma.c
  * @author	ACTION_2017
  * @version	V
  * @date	2017/04/29
  * @brief	This file contains
  *
  *******************************************************************************************************
  * @attention
  *
  *
  *******************************************************************************************************
  */

/* Includes -------------------------------------------------------------------------------------------*/
#include <string.h>
#include "stm32f4xx_dma.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include "usart.h"
#include "dma.h"

#include "stm32f4xx_usart.h"

/* Private typedef ------------------------------------------------------------------------------------*/
/* Private define -------------------------------------------------------------------------------------*/
#define UART5_SEND_BUF_CAPACITY 400u
/* Private macro --------------------------------------------------------------------------------------*/
/* Private variables ----------------------------------------------------------------------------------*/
static uint16_t sendBufferCnt = 0u;
static uint8_t UART5SendBuf[UART5_SEND_BUF_CAPACITY] = {0};
static uint8_t UART5DMASendBuf[UART5_SEND_BUF_CAPACITY] = {0};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief
  * @note
  * @param
  * @retval None
  */
void UART5DMAInit(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef 	NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	//ControlBLE_Init(115200);

	DMA_DeInit(DMA1_Stream7);
	while (DMA_GetCmdStatus(DMA1_Stream7) != DISABLE){}

	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t)&(UART5->DR); // peripheral address, = & USART5->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)UART5DMASendBuf;	// memory address to save DMA data
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;				// data dirction:  memory to peripheral
	DMA_InitStructure.DMA_BufferSize = 0;					//the buffer size, in data unit
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//8 bit data
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;		//8 bit data  32??MCU?1?half-word?16 bits
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream7, &DMA_InitStructure);
//	DMA_Cmd(DMA1_Stream7, ENABLE);

	USART_DMACmd(UART5,USART_DMAReq_Tx,ENABLE);
	DMA_ITConfig(DMA1_Stream7,DMA_IT_TC,ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void DMA1_Stream7_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_Stream7, DMA_IT_TCIF7) != RESET)
	{
		DMA_Cmd(DMA1_Stream7, DISABLE);
		DMA_ClearITPendingBit(DMA1_Stream7, DMA_IT_TCIF7);
	}
}

/**
  * @brief	使能DMA向串口传输数据
  * @note	
  * @param	None
  * @retval	None
  */
void UART5_DMA_Send(void)
{
	uint64_t timeout = 0;
	while(DMA_GetCurrDataCounter(DMA1_Stream7) != 0u)
	{
		timeout++;
		if(timeout > 0x0fffffff)
		{
			USART_SendData(UART5,'D');
			USART_SendData(UART5,'M');
			USART_SendData(UART5,'A');
			USART_SendData(UART5,'C');
			USART_SendData(UART5,'O');
			USART_SendData(UART5,'U');
			USART_SendData(UART5,'N');
			USART_SendData(UART5,'T');
			USART_SendData(UART5,'E');
			USART_SendData(UART5,'R');	
			USART_SendData(UART5,'R');
			USART_SendData(UART5,'O');
			USART_SendData(UART5,'R');
			USART_SendData(UART5,'\r');
			USART_SendData(UART5,'\n');

		}
	}
	DMA_ClearFlag(DMA1_Stream7,DMA_FLAG_TCIF7);
	DMA_Cmd(DMA1_Stream7, DISABLE);
	memcpy(UART5DMASendBuf, UART5SendBuf, sendBufferCnt);
	timeout = 0;
	while (DMA_GetCmdStatus(DMA1_Stream7) != DISABLE)
	{
		timeout++;
		if(timeout > 0x0fffffff)
		{
			USART_SendData(UART5,'D');
			USART_SendData(UART5,'M');
			USART_SendData(UART5,'A');
			USART_SendData(UART5,'N');
			USART_SendData(UART5,'O');
			USART_SendData(UART5,'T');
			USART_SendData(UART5,'D');
			USART_SendData(UART5,'I');
			USART_SendData(UART5,'S');
			USART_SendData(UART5,'A');	
			USART_SendData(UART5,'B');
			USART_SendData(UART5,'L');
			USART_SendData(UART5,'E');
			USART_SendData(UART5,'\r');
			USART_SendData(UART5,'\n');

		}	
	}
	DMA_SetCurrDataCounter(DMA1_Stream7, sendBufferCnt);
	DMA_Cmd(DMA1_Stream7, ENABLE);
	sendBufferCnt = 0u;
}


/**
  * @brief	向UART5SendBuf缓冲区中写数据
  * @note	fix me 并不是环形数组  注意填入数据过多时可能导致Buffer溢出
  * @param	data: 需要传输的八位数据
  * @retval	None
  */
void UART5BufPut(uint8_t data)
{
//	uint16_t i = 0u;
	if(sendBufferCnt < UART5_SEND_BUF_CAPACITY)
	{
		UART5SendBuf[sendBufferCnt++] = data;
		if(DMA_GetCmdStatus(DMA1_Stream7) == DISABLE && DMA_GetCurrDataCounter(DMA1_Stream7) == 0u)
		{
			if(sendBufferCnt > 100u)
			{
				UART5_DMA_Send();
			}
		}
//		else if(sendBufferCnt > 144u)
//		{
//			while(DMA_GetCmdStatus(DMA1_Stream7) == ENABLE || DMA_GetCurrDataCounter(DMA1_Stream7) != 0u)
//			{
//				i++;
//				if(i == 0)
//					break;
//			}
//			UART5_DMA_Send();
//		}
	}
	else
	{
		USART_SendData(UART5,'D');
		USART_SendData(UART5,'M');
		USART_SendData(UART5,'A');
		USART_SendData(UART5,'O');
		USART_SendData(UART5,'V');
		USART_SendData(UART5,'E');
		USART_SendData(UART5,'R');
		USART_SendData(UART5,'F');
		USART_SendData(UART5,'L');
		USART_SendData(UART5,'O');	
		USART_SendData(UART5,'W');
		USART_SendData(UART5,'\r');
		USART_SendData(UART5,'\n');
		sendBufferCnt = 0;
	}
}

