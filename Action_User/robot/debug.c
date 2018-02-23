#include "task.h"
#include "debug.h"
#include "includes.h"
#include "process.h"
#include "gpio.h"

extern Robot_t gRobot;
void debugFunction(void)
{
	
	USART_OUT_F(gRobot.sDta.cameraAimAngle++);
	USART_OUT_F(gRobot.sDta.AT_motionFlag--);
	USART_OUT_F(gRobot.sDta.courseAimAngle++);
	USART_OUT_F(gRobot.sDta.gasAimValue++);
	USART_OUT_F(gRobot.sDta.holdBallAimAngle++);
	USART_OUT_F(gRobot.sDta.isReset);
	USART_OUT_F(gRobot.sDta.pitchAimAngle++);
	USART_OUT_F(gRobot.sDta.process++);
	USART_OUT_F(gRobot.sDta.robocon2018++);
	USART_Enter();
}



/*调试蓝牙中断*/
static char buffer[20];
static int bufferI=0;

void AT_DEBUG_Judge(void);
void BufferDebugInit(void);

/****************串口一接收中断****start****************/

void USART1_IRQHandler(void)
{
  static uint8_t data;
  OS_CPU_SR  cpu_sr;
  OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
  OSIntNesting++;
  OS_EXIT_CRITICAL();
  
  if(USART_GetITStatus(USART1,USART_IT_RXNE)==SET)
  {
    USART_ClearITPendingBit( USART1,USART_IT_RXNE);
    data=USART_ReceiveData(USART1);
    buffer[bufferI]=data;
    bufferI++;
    if(bufferI==20)
      bufferI=0;
    if(bufferI>1&&buffer[bufferI-1]=='\n'&&buffer[bufferI-2]=='\r'){
      AT_DEBUG_Judge();
    }else{
      if(buffer[0]!='A'){
        BufferDebugInit();
        //USART_OUT(USART1,"NOT START WITH 'A'\r\n");
      }
    }
  }else{
    data=USART_ReceiveData(USART1);
  }
  OSIntExit();
}



void AT_DEBUG_Judge(void){
	
  if((bufferI >= 9) && strncmp(buffer, "AT+report", 9)==0)//AT    
  {
    StatusReport();
  }
  else if((bufferI >= 4) && strncmp(buffer, "AT+PE", 5)==0)//AT    
  {
    USART_OUT(DEBUG_USART,"PE\t%d\r\n",PE_FOR_THE_BALL);
  }
  else if((bufferI >= 4) && strncmp(buffer, "AT+init",7 )==0)//AT    
  {
    PhotoelectricityInit();
    USART_OUT(DEBUG_USART,"OK\r\n");
  }
  else if((bufferI >= 12) && strncmp(buffer, "AT+HARDFAULT\r\n",12 )==0)
  {
		int a[2];
    USART_OUT(DEBUG_USART,"hardfault\r\n");
		for(int i=10000;i<80000;i++)
			a[i]=100;
		a[1]=a[0];
  }

	BufferDebugInit();
}

void BufferDebugInit(void){
  bufferI=0;
  for(int i=0;i<20;i++)
    buffer[i]=0;
}

