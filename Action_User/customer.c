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

extern Robot_t gRobot;

void AT_CMD_Judge(void);

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
		if(bufferI==20)
			bufferI=0;
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
#define CLAW 						1
#define SHOOT 					2
#define PITCH 					3
#define STEER 					4
#define GAS  						5
#define COURSE  				6

void AT_CMD_Judge(void){
  if((bufferI >= 4) && strncmp(buffer, "AT+1", 4)==0)//AT    
    atCommand=CLAW;
  else if((bufferI >= 4) && strncmp(buffer, "AT+2", 4)==0)//AT    
    atCommand=SHOOT;
  else if((bufferI >= 4) && strncmp(buffer, "AT+3", 4)==0)//AT    
    atCommand=PITCH;
	else if((bufferI >= 4) && strncmp(buffer, "AT+4", 4)==0)//发射按钮   
    atCommand=STEER;
	else if((bufferI >= 4) && strncmp(buffer, "AT+5", 4)==0)//发射按钮   
    atCommand=GAS;
	else if((bufferI >= 4) && strncmp(buffer, "AT+6", 4)==0)//发射按钮   
    atCommand=COURSE;
  else 
    atCommand=666;
  
}


void AT_CMD_Handle(void){
	float value=0.0f;
	switch(atCommand)
	{
		case 0:
			break;
		/*控制张爪*/
		case CLAW:
			USART_OUT(DEBUG_USART,"OK\r\n");
			if(*(buffer + 4) == '1') 
			{
				GasValveControl(GASVALVE_BOARD_ID , CLAW_ID, CLAW_SHUT);
				SetMotionFlag(~AT_CLAW_STATUS_OPEN);
			}
			else if(*(buffer + 4) == '0') 
			{
				GasValveControl(GASVALVE_BOARD_ID , CLAW_ID , CLAW_OPEN);
				SetMotionFlag(AT_CLAW_STATUS_OPEN);
			} 
			else{}
			break;
		
		/*控制是否射击*/
		case SHOOT:
			USART_OUT(DEBUG_USART,"OK\r\n");
			if(*(buffer + 4) == '1')
			{
				if(gRobot.AT_motionFlag&(AT_CLAW_STATUS_OPEN|AT_STEER_READY))
				{
					GasValveControl(GASVALVE_BOARD_ID , SHOOT_SMALL_ID , 1);
					GasValveControl(GASVALVE_BOARD_ID , SHOOT_BIG_ID , 1);
					SetMotionFlag(AT_SHOOT_BIG_ENABLE);
					SetMotionFlag(AT_SHOOT_SMALL_ENABLE);
				}
			}
			else if(*(buffer + 4) == '0') 
			{
				if(gRobot.AT_motionFlag&(AT_SHOOT_BIG_ENABLE|AT_SHOOT_SMALL_ENABLE))
				{
					GasValveControl(GASVALVE_BOARD_ID , SHOOT_SMALL_ID , 0);
					GasValveControl(GASVALVE_BOARD_ID , SHOOT_BIG_ID, 0);
					SetMotionFlag(~AT_SHOOT_BIG_ENABLE);
					SetMotionFlag(~AT_SHOOT_SMALL_ENABLE);
				}
				else
				{
					GasValveControl(GASVALVE_BOARD_ID , SHOOT_SMALL_ID , 1);
					SetMotionFlag(~AT_SHOOT_SMALL_ENABLE);
					GasValveControl(GASVALVE_BOARD_ID , CLAW_ID , CLAW_SHUT);
					SetMotionFlag(~AT_CLAW_STATUS_OPEN);
				}
			}
			break;

		case PITCH:
			USART_OUT(DEBUG_USART,"OK\r\n");
			value = atof(buffer + 4);
			PitchAngleMotion(value);
			break;
		
		case STEER:
			USART_OUT(DEBUG_USART,"OK\r\n");
			value = atof(buffer + 4);
//				ROBS_PosCrl(value, value, 1000);
//				SetMotionFlag(~AT_STEER_READY);
			if(value <= -45.f)
			{
				ROBS_PosCrl(90, 90, 2000);
				SetMotionFlag(~AT_STEER_READY);
			}
			else if(value <= 45.f)
			{
				ROBS_PosCrl(0, 0, 2000);
				SetMotionFlag(AT_STEER_READY);
			}
			else
			{
				ROBS_PosCrl(-90, -90, 2000);
				SetMotionFlag(~AT_STEER_READY);
			}
			break;
		
		case GAS:
			USART_OUT(DEBUG_USART,"OK\r\n");
			//平板的值
			value = atof(buffer + 4);
			CAN_TxMsg(CAN2,SEND_TO_GASSENSOR,(uint8_t*)(&value),4);
			break;
		
		case COURSE:
			USART_OUT(DEBUG_USART,"OK\r\n");
			value = atof(buffer + 4);
			CourseAngleMotion(value);
			break;

		default:
			break;
	}
	
  bufferInit();
}



void SetMotionFlag(uint32_t status){
	
	switch(status){
		case AT_CLAW_STATUS_OPEN:
			gRobot.AT_motionFlag|=AT_CLAW_STATUS_OPEN;
			break;
		case ~AT_CLAW_STATUS_OPEN:
			gRobot.AT_motionFlag&=~AT_CLAW_STATUS_OPEN;
			break;
		case AT_STEER_READY:
			gRobot.AT_motionFlag|=AT_STEER_READY;
			break;
		case ~AT_STEER_READY:
			gRobot.AT_motionFlag&=~AT_STEER_READY;
			break;
		case AT_SHOOT_BIG_ENABLE:
			gRobot.AT_motionFlag|=AT_SHOOT_BIG_ENABLE;
			break;
		case ~AT_SHOOT_BIG_ENABLE:
			gRobot.AT_motionFlag&=~AT_SHOOT_BIG_ENABLE;
			break;
		case AT_SHOOT_SMALL_ENABLE:
			gRobot.AT_motionFlag|=AT_SHOOT_SMALL_ENABLE;
			break;
		case ~AT_SHOOT_SMALL_ENABLE:
			gRobot.AT_motionFlag&=~AT_SHOOT_SMALL_ENABLE;
			break;
		case AT_STEER1_SUCCESS:
			gRobot.AT_motionFlag|=AT_STEER1_SUCCESS;
			break;
		case ~AT_STEER1_SUCCESS:
			gRobot.AT_motionFlag&=~AT_STEER1_SUCCESS;
			break;
		case AT_STEER2_SUCCESS:
			gRobot.AT_motionFlag|=AT_STEER2_SUCCESS;
			break;
		case ~AT_STEER2_SUCCESS:
			gRobot.AT_motionFlag&=~AT_STEER2_SUCCESS;
			break;
		case AT_STEER1_READ_SUCCESS:
			gRobot.AT_motionFlag|=AT_STEER1_READ_SUCCESS;
			break;
		case ~AT_STEER1_READ_SUCCESS:
			gRobot.AT_motionFlag&=~AT_STEER1_READ_SUCCESS;
			break;
		case AT_STEER2_READ_SUCCESS:
			gRobot.AT_motionFlag|=AT_STEER2_READ_SUCCESS;
			break;
		case ~AT_STEER2_READ_SUCCESS:
			gRobot.AT_motionFlag&=~AT_STEER2_READ_SUCCESS;
			break;
	}
}


/*调试蓝牙中断*/
static char bufferUART5[20];
static int bufferUART5_I=0;

void UART5_bufferInit(void){
  bufferUART5_I=0;
  for(int i=0;i<20;i++)
    bufferUART5[i]=0;
}
void UART5_AT_CMD_Judge(void);
void UART5_IRQHandler(void)
{
  uint8_t data;
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();
  if(USART_GetITStatus(UART5,USART_IT_RXNE)==SET)
  {
    USART_ClearITPendingBit( UART5,USART_IT_RXNE);
    data=USART_ReceiveData(UART5);
    bufferUART5[bufferUART5_I]=data;
    bufferUART5_I++;
		if(bufferUART5_I==20)
			bufferUART5_I=0;
    if(bufferUART5_I>1&&bufferUART5[bufferUART5_I-1]=='\n'&&bufferUART5[bufferUART5_I-2]=='\r'){
      UART5_AT_CMD_Judge();
    }else{
      if(bufferUART5[0]!='A'){
        UART5_bufferInit();
        USART_OUT(UART5,"NOT START WITH 'A'\r\n");
      }
    }
  }else{
    data=USART_ReceiveData(UART5);
  }
	OSIntExit();
}

void UART5_AT_CMD_Judge(void){
  if((bufferUART5_I >= 9) && strncmp(bufferUART5, "AT+report", 9)==0)//AT    
  {
		StatusReport();
	}
  else if((bufferUART5_I >= 4) && strncmp(bufferUART5, "AT+PE", 5)==0)//AT    
	{
		USART_OUT(DEBUG_USART,"PE\t%d\r\n",PE_FOR_THE_BALL);
	}
  else if((bufferUART5_I >= 4) && strncmp(bufferUART5, "AT+init",7 )==0)//AT    
	{
		PhotoelectricityInit();
	}
  
  UART5_bufferInit();
}

