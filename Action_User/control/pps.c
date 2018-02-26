#include "pps.h"
#include "stdint.h"
#include "os_cpu.h"
#include "ucos_ii.h"
#include "stm32f4xx_usart.h"
#include "task.h"

extern Robot_t gRobot;



/*定位系统串口中断*/
void USART3_IRQHandler(void)
{
  static uint8_t ch;
  static union {
    uint8_t data[24];
    float ActVal[6];
  } posture;
  static uint8_t count = 0;
  static uint8_t i = 0;
  OS_CPU_SR  cpu_sr;
  OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
  OSIntNesting++;
  OS_EXIT_CRITICAL();
  
  if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
  {
    USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    ch = USART_ReceiveData(USART3);
    switch (count)
    {
    case 0:
      if (ch == 0x0d)
        count++;
      else if(ch=='O')
        count=5;
      else
        count = 0;
      break;
      
    case 1:
      if (ch == 0x0a)
      {
        i = 0;
        count++;
      }
      else if (ch == 0x0d)
        ;
      else
        count = 0;
      break;
      
    case 2:
      posture.data[i] = ch;
      i++;
      if (i >= 24)
      {
        i = 0;
        count++;
      }
      break;
      
    case 3:
      if (ch == 0x0a)
        count++;
      else
        count = 0;
      break;
      
    case 4:
      if (ch == 0x0d)
      {
        gRobot.angle=posture.ActVal[0] ;
        gRobot.posX = -posture.ActVal[3];
        gRobot.posY = posture.ActVal[4];	
				gRobot.posSystemReady=1;
      }
      count = 0;
      break;
    default:
      count = 0;
      break;
    }
  }
  else
  {
    USART_ReceiveData(USART3);
  }
  OSIntExit();
}

