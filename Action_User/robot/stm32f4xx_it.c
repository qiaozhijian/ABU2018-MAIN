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
#include "DataRecover.h"

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
  int32_t i = 0;
  
  OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
  OSIntNesting++;
  OS_EXIT_CRITICAL();
  CAN_RxMsg(CAN1, &StdId, buffer, 8);
  
  if(StdId==GET_FROM_MOTIONCARD)     //get speed value
  {
    //fix me, if length not 8
    for(i = 0; i < 4; i++)
      msg.data8[i] = buffer[i];
    //位置
    if(msg.data32[0]==1&&gRobot.sDta.process==TO_THE_AREA_1)
		{
      gRobot.sDta.process=TO_THROW_BALL_1;
			
//      PrepareShootBall(BALL_1);
//			SetMotionFlag(~AT_GAS_SUCCESS);
		}
    if(msg.data32[0]==2&&gRobot.sDta.process==TO_THE_AREA_2)
      gRobot.sDta.process=TO_THROW_BALL_2;
    if(msg.data32[0]==3&&gRobot.sDta.process==TO_THE_AREA_3)
      gRobot.sDta.process=TO_THROW_BALL_3;
    if(msg.data32[0]==GET_MOTIONCARD_PREPARE_READY&&gRobot.sDta.process==ROBOT_PREPARE)
      SetMotionFlag(AT_PREPARE_READY);
    USART_OUT(DEBUG_USART,"GET_FROM_MOTIONCARD %d\r\n",msg.data32[0]);
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
  USART_OUT(DEBUG_USART,"CAN1 BUS OFF %d!!\r\n" ,CAN_GetLastErrorCode(CAN1));
  BEEP_ON;
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
  uint8_t buffer[8];
  uint32_t StdId=0;
  uint8_t canNodeId = 0;
  int32_t i = 0;
  
  OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
  OSIntNesting++;
  OS_EXIT_CRITICAL();
  CAN_RxMsg(CAN2, &StdId, buffer, 8);
  canNodeId = StdId;
  
  if((StdId - SDO_RESPONSE_COB_ID_BASE)==5)     //俯仰
  {
    //fix me, if length not 8
    for(i = 0; i < 8; i++)
      msg.data8[i] = buffer[i];
    //位置
    if(msg.data32[0]==0x00005850)
    {
      gRobot.pitchAngle = PITCH_CODE_TO_ANGLE(msg.data32[1]);
			gRobot.pitchAngle=10.f-gRobot.pitchAngle;
      SetMotionFlag(AT_PITCH_READ_SUCCESS);
    }
    //速度
    if(msg.data32[0]==0x00005856)
    {
      
    }
  }else if((StdId - SDO_RESPONSE_COB_ID_BASE)==6)     //航向
  {
    //fix me, if length not 8
    for(i = 0; i < 8; i++)
      msg.data8[i] = buffer[i];
    //位置
    if(msg.data32[0]==0x00005850)
    {
      gRobot.courseAngle = COURSE_CODE_TO_ANGLE(msg.data32[1]);
			gRobot.courseAngle=gRobot.courseAngle-10.6f;
      SetMotionFlag(AT_COURSE_READ_SUCCESS);
    }
    //速度
    if(msg.data32[0]==0x00005856)
    {
      
    }
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
    gRobot.gasValue=msg4.dataf;
  }
  else
  {
    msg4.dataf=msg4.dataf;
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
  USART_OUT(DEBUG_USART,"CAN2 BUS OFF %d!!\r\n" ,CAN_GetLastErrorCode(CAN2));
  BEEP_ON;
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

uint32_t startCnt=0;
uint32_t Cnt=0;

void TIM7_IRQHandler(void)
{ 
	OS_CPU_SR  cpu_sr;
  OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
  OSIntNesting++;
  OS_EXIT_CRITICAL();
  if(TIM_GetITStatus(TIM7, TIM_IT_Update)==SET)
  {	
		if(startCnt==1)
		{
			Cnt++;
			//USART_OUT(DEBUG_USART,"%d\r\n",Cnt*100);
		}
		//printf("%d\r\n",Cnt);
		//IWDG_Feed();
    TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	}
  OSIntExit();
}

void StartCount(void)
{
	//printf("%d\t",startCnt);
	startCnt=1;
	Cnt=0;
}

uint32_t returnEndUs(void)
{
	uint32_t	end;
	end=Cnt*100;
	Cnt=0;
	startCnt=0;
	USART_OUT(DEBUG_USART,"%d\r\n",end);
	return end;
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
    USART_OUT(DEBUG_USART,"NMI exception !!!!!!!!!!!!!\r\n");
  }
}
void Hex_To_Str(uint8_t * pHex,char * s,float num)
{
  char        hex[] = "0123456789ABCDEF";
  char        *pStr = s;
  for (uint8_t i = 0; i < (int)(num/2.f+0.5f); i++)//(int)(x+0.5f)是把x四舍五入的意思
  {
    
    /*
    1.*pStr++右结合,并且*索引的是没有++之前的地址
    2.f.移位不会改变指针指向的那个空间的值
    3.对指针指向空间的移位也不会改变指针的指向
    */
    if (((num<((int)(num / 2.f + 0.5f))*2.f)&&i>0)|| (num==((int)(num / 2.f + 0.5f)) * 2.f))
      *pStr++ = hex[*(pHex + (int)(num / 2.f + 0.5f) - i - 1) >> 4];
    *pStr++ = hex[*(pHex + (int)(num / 2.f + 0.5f) - i - 1) & 0x0F];
  }
}
/**
* @brief  This function handles Hard Fault exception.
* @param  None
* @retval None
*/
void HardFault_Handler(void)
{
  	if(gRobot.resetTime<=500)
	{
		/*确定要写的结构体*/
		FindResetTime();
		
		gRobot.sDta.isReset=1;
		
		WriteFlashData(gRobot,gRobot.resetTime);
	}
	/*屏蔽掉的只是为了返回进硬件中断的错误语句，但是比赛的时候不可能出现错误，只可能有静电等突发因素，所以不用返回了*/
//  static uint32_t r_sp ;
//  /*判断发生异常时使用MSP还是PSP*/
//  if(__get_PSP()!=0x00) //获取SP的值
//    r_sp = __get_PSP(); 
//  else
//    r_sp = __get_MSP(); 
//  /*因为经历中断函数入栈之后，堆栈指针会减小0x10，所以平移回来（可能不具有普遍性）*/
//  r_sp = r_sp+0x10;
//  /*串口发数通知*/
//  USART_OUT(DEBUG_USART,"\r\nHardFault");
//  char sPoint[2]={0};
//  USART_OUT(DEBUG_USART,"%s","0x");
//  /*获取出现异常时程序的地址*/
//  for(int i=3;i>=-28;i--){
//    Hex_To_Str((uint8_t*)(r_sp+i+28),sPoint,2);
//    USART_OUT(DEBUG_USART,"%s",sPoint);
//    if(i%4==0)
//      USART_Enter();
//  }
//  /*发送回车符*/
//  USART_Enter();
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
		//方便打断点，无意义
    gRobot.sDta.isReset=gRobot.sDta.isReset;
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
    USART_OUT(DEBUG_USART,"Memory Manage exception occurs!!!!!!!!!\r\n");
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
    USART_OUT(DEBUG_USART,"Bus Fault exception!!!!!!!!\r\n");
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
    USART_OUT(DEBUG_USART,"Usage Fault exception!!!!!!!!!\r\n");
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

