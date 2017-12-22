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

extern Robot_t gRobot;

void AT_CMD_Judge(void);

void RemoteCtr(void)
{
	
}

static char buffer[20];
static int bufferI=0;
static int atCommand=0;

void bufferInit(void){
  bufferI=0;
  for(int i=0;i<20;i++)
    buffer[i]=0;
		atCommand=0;
}
void USART3_IRQHandler(void)
{
  uint8_t data;
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();
  if(USART_GetITStatus(USART3,USART_IT_RXNE)==SET)
  {
    USART_ClearITPendingBit( USART3,USART_IT_RXNE);
    data=USART_ReceiveData(USART3);
    buffer[bufferI]=data;
    bufferI++;
    if(bufferI>1&&buffer[bufferI-1]=='\n'&&buffer[bufferI-2]=='\r'){
      AT_CMD_Judge();
    }else{
      if(buffer[0]!='A'){
        bufferInit();
        USART_OUT(USART3,"NOT START WITH 'A'\r\n");
      }
    }
  }else{
    data=USART_ReceiveData(USART3);
  }
	OSIntExit();
}

void AT_CMD_Judge(void){
  if((bufferI == 4) && strncmp(buffer, "AT\r\n", 4)==0)//AT    
    atCommand=1;
  else if((bufferI == 9) && strncmp(buffer, "AT+open\r\n", 9)==0)//AT    
    atCommand=2;
  else if((bufferI == 9) && strncmp(buffer, "AT+shut\r\n", 9)==0)//AT    
    atCommand=3;
  else 
    atCommand=666;
  
}

#define CLAW 	1
#define SHOOT 2
#define PITCH 3
#define STEER 4
#define GAS  	5

int robsPositon = 0;
int clawStatus = 0;
int reset;
void AT_CMD_Handle(void){
	float value;
	switch(atCommand)
	{
		case 0:
			break;
		/*控制张爪*/
		case CLAW:
			if(*(buffer + 4) == '1') 
			{
				GasValveControl(GASVALVE_BOARD_ID , CLAW_ID, CLAW_SHUT);
				SetMotionFlag(~CLAW_STATUS_OPEN);
			}
			else if(*(buffer + 4) == '0') 
			{
				GasValveControl(GASVALVE_BOARD_ID , CLAW_ID , CLAW_OPEN);
				SetMotionFlag(CLAW_STATUS_OPEN);
			} 
			else{}
			break;
		
		/*控制是否射击*/
		case SHOOT:
			if(*(buffer + 4) == '1')
			{
				if(gRobot.motionFlag&(CLAW_STATUS_OPEN|STEER_READY))
				{
					GasValveControl(GASVALVE_BOARD_ID , SHOOT_SMALL_ID , 1);
					reset = 1;
					GasValveControl(GASVALVE_BOARD_ID , SHOOT_BIG_ID , 1);
					SetMotionFlag(SHOOT_BIG_ENABLE);
					SetMotionFlag(SHOOT_SMALL_ENABLE);
				}
			}
			else if(*(buffer + 4) == '0') 
			{
				if(gRobot.motionFlag&(SHOOT_BIG_ENABLE|SHOOT_SMALL_ENABLE))
				{
					GasValveControl(GASVALVE_BOARD_ID , SHOOT_SMALL_ID , 0);
					GasValveControl(GASVALVE_BOARD_ID , SHOOT_BIG_ID, 0);
					SetMotionFlag(~SHOOT_BIG_ENABLE);
					SetMotionFlag(~SHOOT_SMALL_ENABLE);
				}
				else
				{
					GasValveControl(GASVALVE_BOARD_ID , SHOOT_SMALL_ID , 1);
					SetMotionFlag(~SHOOT_SMALL_ENABLE);
					GasValveControl(GASVALVE_BOARD_ID , CLAW_ID , CLAW_SHUT);
					SetMotionFlag(~CLAW_STATUS_OPEN);
				}
			}
			break;

		case PITCH:
			value = atof(buffer + 4);
			value = value*28*19*8192/360;
			PosCrl(CAN1, 1,ABSOLUTE_MODE,value);

			break;
		
		case STEER:
			value = atof(buffer + 4);
			if(value <= -45.f)
			{
				ROBS_PosCrl(90, 95, 1000);
				SetMotionFlag(~STEER_READY);
			}
			else if(value <= 45.f)
			{
				ROBS_PosCrl(0, 0, 1000);
				SetMotionFlag(STEER_READY);
			}
			else
			{
				ROBS_PosCrl(-90, -90, 1000);
				SetMotionFlag(~STEER_READY);
			}

			break;
		
		case GAS:
			//平板的值
			value = atof(buffer + 4);
			CAN_TxMsg(CAN2,0X20,(uint8_t*)(&value),4);
			break;

		default:
			break;
	}
	
  bufferInit();
}



void SetMotionFlag(uint32_t status){
	
	switch(status){
		case CLAW_STATUS_OPEN:
			gRobot.motionFlag|=CLAW_STATUS_OPEN;
			break;
		case ~CLAW_STATUS_OPEN:
			gRobot.motionFlag&=~CLAW_STATUS_OPEN;
			break;
		case STEER_READY:
			gRobot.motionFlag|=STEER_READY;
			break;
		case ~STEER_READY:
			gRobot.motionFlag&=~STEER_READY;
			break;
		case SHOOT_BIG_ENABLE:
			gRobot.motionFlag|=SHOOT_BIG_ENABLE;
			break;
		case ~SHOOT_BIG_ENABLE:
			gRobot.motionFlag&=~SHOOT_BIG_ENABLE;
			break;
		case SHOOT_SMALL_ENABLE:
			gRobot.motionFlag|=SHOOT_SMALL_ENABLE;
			break;
		case ~SHOOT_SMALL_ENABLE:
			gRobot.motionFlag&=~SHOOT_SMALL_ENABLE;
			break;
	}
}


/****************舵机一串口接收中断****start****************/

void USART1_IRQHandler(void)
{
	//static uint8_t ch;

	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if(USART_GetITStatus(USART1, USART_IT_RXNE)==SET)   
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		//ch=USART_ReceiveData(USART1);

	}else{
    USART_ReceiveData(USART1);
  }
	OSIntExit();
}

/****************舵机二串口接收中断****start****************/

void USART2_IRQHandler(void)
{
	//static uint8_t ch;

	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if(USART_GetITStatus(USART2, USART_IT_RXNE)==SET)   
	{
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
		//ch=USART_ReceiveData(USART2);

	}else{
    USART_ReceiveData(USART2);
  }
	OSIntExit();
}

/*调试蓝牙中断*/
void UART5_IRQHandler(void)
{
	//static uint8_t ch;

	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if(USART_GetITStatus(UART5, USART_IT_RXNE)==SET)   
	{
		USART_ClearITPendingBit(UART5,USART_IT_RXNE);
		//ch=USART_ReceiveData(UART5);

	}else{
    USART_ReceiveData(UART5);
  }
	OSIntExit();
}

