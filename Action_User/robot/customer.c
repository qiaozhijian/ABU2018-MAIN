#include "customer.h"
#include "string.h"
#include "stm32f4xx_usart.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "usart.h"
#include "gasvalveControl.h"
#include "task.h"
#include "elmo.h"
#include "shoot.h"
#include "steer.h"
#include  <includes.h>
#include "process.h"
#include "gpio.h"
#include "motion.h"
#include "timer.h"
#include "robot.h"
#include "dma.h"

#define YENUM					124
extern Robot_t gRobot;
static char buffer[YENUM];
static int bufferI=0;
static int atCommand=0;

void AT_CMD_Judge(void);
	
void BufferInit(void){
  bufferI=0;
	atCommand=0;
  for(int i=0;i<YENUM;i++)
    buffer[i]=0;
}

void UART4_IRQHandler(void)
{
  uint8_t data;
	static int updateKey=0;
	static int updateFlag=0;
	static int step=0;
  OS_CPU_SR  cpu_sr;
  OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
  OSIntNesting++;
  OS_EXIT_CRITICAL();
  if(USART_GetITStatus(UART4,USART_IT_RXNE)==SET)
  {
    USART_ClearITPendingBit( UART4,USART_IT_RXNE);
    data=USART_ReceiveData(UART4);
		
		switch(step)
		{
			case 0:
				if(data==0x59)
					step++;
				else
					step=0;
				break;
			case 1:
				if(data==0x49)
					step++;
				else
					step=0;
				break;
			case 2:
				if(data==0x53)
				{
					updateKey=1;
					updateFlag=0;
					BufferInit();
				}
				step=0;
				break;
		}
		
		if(updateFlag==1)
		{
			buffer[bufferI]=data;
			bufferI++;
			if(bufferI==YENUM)
			{
				AT_CMD_Judge();
				BufferInit();
				updateFlag=0;
			}
		}
		
		if(updateKey==1)
		{
			updateKey=0;
			updateFlag=1;
		}
		
  }else{
    data=USART_ReceiveData(UART4);
  }
  OSIntExit();
}


//初版
float Acceleration[3]={0.f};
float freeAcceleration[3]={0.f};
float deltaVelocity[3]={0.f};
float angularVelocity[3]={0.f};
float magneticField[3]={0.f};
float eulerAngles[3]={0.f};
float Quaternion[4]={0.f};
float deltaQuaternion[4]={0.f};

void AT_CMD_Judge(void)
{
	int num=4;
	union
	{
		int value;
		uint8_t data[4];
	}convert_t={0};
	
	/*Len=0x78*/
	if(buffer[2]==0x78)
	{
		if(buffer[3]==0x10&&buffer[4]==0x0C&&buffer[17]==0x11&&buffer[18]==0x0C)
		{
			num=3;
			for(int i=0;i<num;i++)
				for(int j=0;j<4;j++)
				{
					convert_t.data[j]=buffer[3+i*4+j+2];
					if(j==3)
						Acceleration[i]=convert_t.value*0.000001;
				}
		}
		if(buffer[17]==0x11&&buffer[18]==0x0C&&buffer[31]==0x12&&buffer[32]==0x0C)
		{
			num=3;
			for(int i=0;i<num;i++)
				for(int j=0;j<4;j++)
				{
					convert_t.data[j]=buffer[17+i*4+j+2];
					if(j==3)
						freeAcceleration[i]=convert_t.value*0.000001;
				}
		}
		if(buffer[31]==0x12&&buffer[32]==0x0C&&buffer[45]==0x20&&buffer[46]==0x0C)
		{
			num=3;
			for(int i=0;i<num;i++)
				for(int j=0;j<4;j++)
				{
					convert_t.data[j]=buffer[31+i*4+j+2];
					if(j==3)
						deltaVelocity[i]=convert_t.value*0.000001;
				}
		}
		if(buffer[45]==0x20&&buffer[46]==0x0C&&buffer[59]==0x30&&buffer[60]==0x0C)
		{
			num=3;
			for(int i=0;i<num;i++)
				for(int j=0;j<4;j++)
				{
					convert_t.data[j]=buffer[45+i*4+j+2];
					if(j==3)
						angularVelocity[i]=convert_t.value*0.000001;
				}
		}
		if(buffer[59]==0x30&&buffer[60]==0x0C&&buffer[73]==0x40&&buffer[74]==0x0C)
		{
			num=3;
			for(int i=0;i<num;i++)
				for(int j=0;j<4;j++)
				{
					convert_t.data[j]=buffer[59+i*4+j+2];
					if(j==3)
						magneticField[i]=convert_t.value*0.001;
				}
		}
		if(buffer[73]==0x40&&buffer[74]==0x0C&&buffer[87]==0x42&&buffer[88]==0x10)
		{
			num=3;
			for(int i=0;i<num;i++)
				for(int j=0;j<4;j++)
				{
					convert_t.data[j]=buffer[73+i*4+j+2];
					if(j==3)
						eulerAngles[i]=convert_t.value*0.000001;
				}
		}
		if(buffer[87]==0x42&&buffer[88]==0x10&&buffer[105]==0x42&&buffer[106]==0x10)
		{
			num=4;
			for(int i=0;i<num;i++)
				for(int j=0;j<4;j++)
				{
					convert_t.data[j]=buffer[87+i*4+j+2];
					if(j==3)
						Quaternion[i]=convert_t.value;
				}
		}
		if(buffer[105]==0x42&&buffer[106]==0x10&&buffer[123]==0x78)
		{
			num=4;
			for(int i=0;i<num;i++)
				for(int j=0;j<4;j++)
				{
					convert_t.data[j]=buffer[105+i*4+j+2];
					if(j==3)
						deltaQuaternion[i]=convert_t.value*0.000001;
				}
		}
//		for(int i=0;i<3;i++)
//			USART_OUTByDMAF(Acceleration[i]);
//		for(int i=0;i<3;i++)
//			USART_OUTByDMAF(freeAcceleration[i]);
//		for(int i=0;i<3;i++)
//			USART_OUTByDMAF(deltaVelocity[i]);
		for(int i=0;i<3;i++)
			USART_OUTByDMAF(angularVelocity[i]);
//		for(int i=0;i<3;i++)
//			USART_OUTByDMAF(magneticField[i]);
//		for(int i=0;i<3;i++)
//			USART_OUTByDMAF(eulerAngles[i]);
//		for(int i=0;i<3;i++)
//			USART_OUTByDMAF(deltaVelocity[i]);
//		for(int i=0;i<3;i++)
//			USART_OUTByDMAF(angularVelocity[i]);
		USART_OUTByDMA("\r\n");
	}
}






