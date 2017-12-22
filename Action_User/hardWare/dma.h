/**
  ******************************************************************************
  * @file    dma.h
  * @author  ACTION_2017
  * @version V0.0.0._alpha
  * @date    2017/04/29
  * @brief   This file contains all the functions prototypes for DMA
  *          
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DMA_H
#define __DMA_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

void UART5DMAInit(void);
/**
  * @brief	使能DMA向串口传输数据
  * @note	
  * @param	None
  * @retval	None
  */
void UART5_DMA_Send(void);
void DMA1_Stream7_IRQHandler(void);

/**
  * @brief	向UART5SendBuf缓冲区中写数据
  * @note	fix me 并不是环形数组  注意填入数据过多时可能导致Buffer溢出
  * @param	data: 需要传输的八位数据
  * @retval	None
  */
void UART5BufPut(uint8_t data);
	 
#ifdef __cplusplus
}
#endif

#endif /* __DMA_H */
