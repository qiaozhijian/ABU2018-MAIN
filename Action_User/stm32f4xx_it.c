/**
******************************************************************************
* @file    Project/STM32F4xx_StdPeriph_Template/stm32f4xx_it.c
* @author  MCD Application Team
* @version V1.0.1
* @date    13-April-2012
* @brief   Main Interrupt Service Routines.
*          This file provides template for all exceptions handler and
*          peripherals interrupt service routine.
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
*
* Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
* You may not use this file except in compliance with the License.
* You may obtain a copy of the License at:
*
*        http://www.st.com/software_license_agreement_liberty_v2
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
#include  <ucos_ii.h>
#include "app_cfg.h"
#include <math.h>
#include "usart.h"
#include "timer.h"
#include "can.h"
#include "gpio.h"
#include "elmo.h"
#include "task.h"

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/
extern Robot_t gRobot;
//用来处理CAN接收数据
union MSG
{
  uint8_t data8[8];
  int data32[2];
  float dataf[2];
}msg;

void CAN1_RX0_IRQHandler(void)
{
  OS_CPU_SR  cpu_sr;
  uint8_t buffer[8];
  uint32_t StdId=0;
  uint8_t canNodeId = 0;
  int32_t i = 0;
  
  OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
  OSIntNesting++;
  OS_EXIT_CRITICAL();
  CAN_RxMsg(CAN1, &StdId, buffer, 8);
  canNodeId = StdId - SDO_RESPONSE_COB_ID_BASE;
  
  if(canNodeId==5)     //get speed value
  {
    //fix me, if length not 8
    for(i = 0; i < 8; i++)
      msg.data8[i] = buffer[i];
    //位置
    if(msg.data32[0]==0x00005850)
    {
    }
    //速度
    if(msg.data32[0]==0x00005856)
    {
    }
  }
 
	if(StdId==GET_FROM_MOTIONCARD)     //get speed value
	{
		//fix me, if length not 8
		for(i = 0; i < 4; i++)
			msg.data8[i] = buffer[i];
		//位置
		if(msg.data32[0]==1)
			gRobot.CAN_motionFlag|=READY_FIRST_BALL;
		if(msg.data32[0]==2)
			gRobot.CAN_motionFlag|=READY_SECOND_BALL;
	}
  
  CAN_ClearFlag(CAN1, CAN_FLAG_EWG);
  CAN_ClearFlag(CAN1, CAN_FLAG_EPV);
  CAN_ClearFlag(CAN1, CAN_FLAG_BOF);
  CAN_ClearFlag(CAN1, CAN_FLAG_LEC);
  
  CAN_ClearFlag(CAN1, CAN_FLAG_FMP0);
  CAN_ClearFlag(CAN1, CAN_FLAG_FF0);
  CAN_ClearFlag(CAN1, CAN_FLAG_FOV0);
  CAN_ClearFlag(CAN1, CAN_FLAG_FMP1);
  CAN_ClearFlag(CAN1, CAN_FLAG_FF1);
  CAN_ClearFlag(CAN1, CAN_FLAG_FOV1);
  OSIntExit();
}

/**
* @brief  CAN1 SCE interrupt  handler
* @note
* @param  None
* @retval None
*/
void CAN1_SCE_IRQHandler(void)
{
  OS_CPU_SR  cpu_sr;
  OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
  OSIntNesting++;
  OS_EXIT_CRITICAL();
  UART5_OUT((uint8_t *)"CAN1 BUS OFF %d!!\r\n" ,CAN_GetLastErrorCode(CAN1));
  //	BEEP_ON;
  CAN_ClearFlag(CAN1, CAN_FLAG_BOF);
  OSIntExit();
}

union MSG4
{
	uint8_t data4[4];
	int data32;
	float dataf;
}msg4;

void CAN2_RX0_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr;
	uint8_t buffer[4];
	uint32_t StdId=0;
	uint8_t canNodeId = 0;
	int32_t i = 0;

	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	CAN_RxMsg(CAN2, &StdId, buffer, 4);
	canNodeId = StdId;

	if(StdId == 0x40 && buffer[0] == 1)
	{
		int jiang = 0;
	}
	
	
	if(canNodeId==GET_FROM_GASSENSOR)     //get speed value
	{
		//fix me, if length not 8
		for(i = 0; i < 4; i++)
			msg4.data4[i] = buffer[i];
		//位置
		if(msg4.dataf>1.0f)
			msg4.dataf=1.f;
		else if(msg4.dataf<0.f)
			msg4.dataf=0.f;
		USART_BLE_SEND(msg4.dataf);
	}

	CAN_ClearFlag(CAN2, CAN_FLAG_EWG);
	CAN_ClearFlag(CAN2, CAN_FLAG_EPV);
	CAN_ClearFlag(CAN2, CAN_FLAG_BOF);
	CAN_ClearFlag(CAN2, CAN_FLAG_LEC);

	CAN_ClearFlag(CAN2, CAN_FLAG_FMP0);
	CAN_ClearFlag(CAN2, CAN_FLAG_FF0);
	CAN_ClearFlag(CAN2, CAN_FLAG_FOV0);
	CAN_ClearFlag(CAN2, CAN_FLAG_FMP1);
	CAN_ClearFlag(CAN2, CAN_FLAG_FF1);
	CAN_ClearFlag(CAN2, CAN_FLAG_FOV1);
	OSIntExit();
}
/**
  * @brief  CAN2 SCE interrupt  handler
  * @note
  * @param  None
  * @retval None
  */
void CAN2_SCE_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
//	UART5_OUT((uint8_t *)"CAN2 BUS OFF %d!!\r\n" ,CAN_GetLastErrorCode(CAN2));
//	BEEP_ON;
	CAN_ClearFlag(CAN2, CAN_FLAG_BOF);
	OSIntExit();
}

/*************定时器2******start************/
//每1ms调用一次

extern  OS_EVENT 		*PeriodSem;

void TIM2_IRQHandler(void)
{
  
  //用来计数10次，产生10ms的定时器
  static uint8_t periodCounter = PERIOD_COUNTER;
  OS_CPU_SR  cpu_sr;
  OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
  OSIntNesting++;
  OS_EXIT_CRITICAL();
  
  
  if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
  {
    //实现10ms 发送1次信号量
    periodCounter--;
    if (periodCounter == 0)
    {		
      OSSemPost(PeriodSem);
      periodCounter = PERIOD_COUNTER;
    }
    
    
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  }
  OSIntExit();
}


void TIM1_UP_TIM10_IRQHandler(void)
{
  OS_CPU_SR  cpu_sr;
  OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
  OSIntNesting++;
  OS_EXIT_CRITICAL();
  if(TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
  {
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
  }
  OSIntExit();
}


void TIM8_UP_TIM13_IRQHandler(void)
{
  OS_CPU_SR  cpu_sr;
  OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
  OSIntNesting++;
  OS_EXIT_CRITICAL();
  if(TIM_GetITStatus(TIM8, TIM_IT_Update) == SET)
  {
    TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
  }
  OSIntExit();
}

void TIM5_IRQHandler(void)
{
  OS_CPU_SR  cpu_sr;
  OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
  OSIntNesting++;
  OS_EXIT_CRITICAL();
  if(TIM_GetITStatus(TIM5, TIM_IT_Update) == SET)
  {
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
  }
  OSIntExit();
}

void TIM3_IRQHandler(void)
{
  OS_CPU_SR  cpu_sr;
  OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
  OSIntNesting++;
  OS_EXIT_CRITICAL();
  if(TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
  }
  OSIntExit();
}

void TIM4_IRQHandler(void)
{
  OS_CPU_SR  cpu_sr;
  OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
  OSIntNesting++;
  OS_EXIT_CRITICAL();
  if(TIM_GetITStatus(TIM4, TIM_IT_Update)==SET)
  {
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
  }
  OSIntExit();
}

/**
* @brief   This function handles NMI exception.
* @param  None
* @retval None
*/
void NMI_Handler(void)
{
  while (1)
  {
    UART5_OUT((uint8_t *)"NMI exception !!!!!!!!!!!!!\r\n");
  }
}

/**
* @brief  This function handles Hard Fault exception.
* @param  None
* @retval None
*/
void HardFault_Handler(void)
{
  
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
    UART5_OUT((uint8_t *)"Hard Fault exception!!!!!!!!!!\r\n");
  }
}

/**
* @brief  This function handles Memory Manage exception.
* @param  None
* @retval None
*/
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
    UART5_OUT((uint8_t *)"Memory Manage exception occurs!!!!!!!!!\r\n");
  }
}

/**
* @brief  This function handles Bus Fault exception.
* @param  None
* @retval None
*/
void BusFault_Handler(void)
{
  
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
    UART5_OUT((uint8_t *)"Bus Fault exception!!!!!!!!\r\n");
  }
}

/**
* @brief  This function handles Usage Fault exception.
* @param  None
* @retval None
*/
void UsageFault_Handler(void)
{
  
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
    UART5_OUT((uint8_t *)"Usage Fault exception!!!!!!!!!\r\n");
  }
}

/**
* @brief  This function handles SVCall exception.
* @param  None
* @retval None
*/
void SVC_Handler(void)
{
}

/**
* @brief  This function handles Debug Monitor exception.
* @param  None
* @retval None
*/
void DebugMon_Handler(void)
{
}

