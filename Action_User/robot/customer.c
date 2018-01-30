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
        gRobot.posY = -posture.ActVal[3];
        gRobot.posX = -posture.ActVal[4];
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



#define CLAW 						1
#define SHOOT 					2
#define PITCH 					3
#define STEER 					4
#define GAS  						5
#define COURSE  				6
#define TEST_GAS  			7
#define CAMERA	  			8

/*调试蓝牙中断*/
static char buffer[20];
static int bufferI=0;
static int atCommand=0;

void BufferInit(void){
  bufferI=0;
	atCommand=0;
  for(int i=0;i<20;i++)
    buffer[i]=0;
}
void AT_CMD_Judge(void);

void USART6_IRQHandler(void)
{
  uint8_t data;
  OS_CPU_SR  cpu_sr;
  OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
  OSIntNesting++;
  OS_EXIT_CRITICAL();
  if(USART_GetITStatus(USART6,USART_IT_RXNE)==SET)
  {
    USART_ClearITPendingBit( USART6,USART_IT_RXNE);
    data=USART_ReceiveData(USART6);
    buffer[bufferI]=data;
    bufferI++;
    if(bufferI>=20)
      bufferI=0;
    if(bufferI>1&&buffer[bufferI-1]=='\n'&&buffer[bufferI-2]=='\r'){
      AT_CMD_Judge();
    }else{
      if(buffer[0]!='A'){
        BufferInit();
        //USART_OUT(USART6,"NOT START WITH 'A'\r\n");
      }
    }
  }else{
    data=USART_ReceiveData(USART6);
  }
  OSIntExit();
}


void UART4_IRQHandler(void)
{
  uint8_t data;
  OS_CPU_SR  cpu_sr;
  OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
  OSIntNesting++;
  OS_EXIT_CRITICAL();
  if(USART_GetITStatus(UART4,USART_IT_RXNE)==SET)
  {
    USART_ClearITPendingBit( UART4,USART_IT_RXNE);
    data=USART_ReceiveData(UART4);
    buffer[bufferI]=data;
    bufferI++;
    if(bufferI==20)
      bufferI=0;
    if(bufferI>1&&buffer[bufferI-1]=='\n'&&buffer[bufferI-2]=='\r'){
      AT_CMD_Judge();
    }else{
      if(buffer[0]!='A'){
        BufferInit();
        //USART_OUT(UART4,"NOT START WITH 'A'\r\n");
      }
    }
  }else{
    data=USART_ReceiveData(UART4);
  }
  OSIntExit();
}

void AT_CMD_Judge(void){
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
  else if((bufferI == 4) && strncmp(buffer, "AT\r\n",4 )==0)//AT    
  {
		SetMotionFlag(AT_CAMERA_TALK_SUCCESS);
    //摄像头连接成功
  }
	//打开气阀返回
  else if((bufferI == 9) && strncmp(buffer, "AT+OPST\r\n",9 )==0)
  {
		gRobot.isOpenGasReturn=1;
    USART_OUT(DEBUG_USART,"OK\r\n");
  }
	//关闭气阀返回
  else if((bufferI == 9) && strncmp(buffer, "AT+STST\r\n",9 )==0)
  {
		gRobot.isOpenGasReturn=0;
    USART_OUT(DEBUG_USART,"OK\r\n");
  }
  
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
  else if((bufferI >= 4) && strncmp(buffer, "AT+7", 4)==0)//发射按钮   
    atCommand=TEST_GAS;
  else if((bufferI >= 4) && strncmp(buffer, "AT+8", 4)==0)//发射按钮   
    atCommand=CAMERA;
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
      ClawShut();
      SetMotionFlag(~AT_CLAW_STATUS_OPEN);
    }
    else if(*(buffer + 4) == '0') 
    {
      ClawOpen();
      SetMotionFlag(AT_CLAW_STATUS_OPEN);
    } 
    else{
		
		}
    break;
    
    /*控制是否射击*/
  case SHOOT:
    USART_OUT(DEBUG_USART,"OK\r\n");
    if(*(buffer + 4) == '1')
    {
      if(gRobot.AT_motionFlag&(AT_CLAW_STATUS_OPEN|AT_STEER_READY))
      {
        ShootSmallOpen();
        ShootBigOpen();
        SetMotionFlag(AT_SHOOT_BIG_ENABLE);
        SetMotionFlag(AT_SHOOT_SMALL_ENABLE);
      }
    }
    else if(*(buffer + 4) == '0') 
    {
      if(gRobot.AT_motionFlag&(AT_SHOOT_BIG_ENABLE|AT_SHOOT_SMALL_ENABLE))
      {
        ShootSmallShut();
				ShootBigShut();
        SetMotionFlag(~AT_SHOOT_BIG_ENABLE);
        SetMotionFlag(~AT_SHOOT_SMALL_ENABLE);
      }
      else
      {
        ShootSmallOpen();
        SetMotionFlag(~AT_SHOOT_SMALL_ENABLE);
        ClawShut();
        SetMotionFlag(~AT_CLAW_STATUS_OPEN);
      }
    }
    break;
    
  case STEER:
    USART_OUT(DEBUG_USART,"OK\r\n");
    value = atof(buffer + 4);

    if(value <= -45.f)
    {
			gRobot.holdBallAimAngle=90.f;
      SetMotionFlag(~AT_STEER_READY);
    }
    else if(value <= 45.f)
    {
			gRobot.holdBallAimAngle=0.f;
      SetMotionFlag(AT_STEER_READY);
    }
    else
    {
			gRobot.holdBallAimAngle=-90.f;
      SetMotionFlag(~AT_STEER_READY);
    }
			gRobot.holdBallAimAngle=value;
      SetMotionFlag(~AT_STEER_READY);
    break;
    
  case GAS:
    USART_OUT(DEBUG_USART,"OK\r\n");
    //平板的值
    value = atof(buffer + 4);
    CAN_TxMsg(CAN2,SEND_TO_GASSENSOR,(uint8_t*)(&value),4);
    break;
    
  case PITCH:
    USART_OUT(DEBUG_USART,"OK\r\n");
    value = atof(buffer + 4);
		gRobot.pitchAimAngle=value;
    break;
    
  case COURSE:
    USART_OUT(DEBUG_USART,"OK\r\n");
    value = atof(buffer + 4);
		gRobot.courseAimAngle=value;
    break;
   
	case TEST_GAS:
    USART_OUT(DEBUG_USART,"OK\r\n");
		
		//GasValveControl(GASVALVE_BOARD_ID,*(buffer+4)-'0',*(buffer+5)-'0');
		break;
	
  case CAMERA:
    USART_OUT(DEBUG_USART,"OK\r\n");
    value = atof(buffer + 4);

    if(value <= -45.f)
    {
			gRobot.cameraAimAngle=-45.f;
      SetMotionFlag(~AT_STEER_READY);
    }
    else if(value >= 45.f)
    {
			gRobot.cameraAimAngle=45.f;
      SetMotionFlag(AT_STEER_READY);
    }
    else
    {
			gRobot.cameraAimAngle=value;
      SetMotionFlag(~AT_STEER_READY);
    }
    break;
		
  default:
    break;
  }
  
  BufferInit();
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
  case AT_HOLD_BALL1_SUCCESS:
    gRobot.AT_motionFlag|=AT_HOLD_BALL1_SUCCESS;
    break;
  case ~AT_HOLD_BALL1_SUCCESS:
    gRobot.AT_motionFlag&=~AT_HOLD_BALL1_SUCCESS;
    break;
  case AT_HOLD_BALL2_SUCCESS:
    gRobot.AT_motionFlag|=AT_HOLD_BALL2_SUCCESS;
    break;
  case ~AT_HOLD_BALL2_SUCCESS:
    gRobot.AT_motionFlag&=~AT_HOLD_BALL2_SUCCESS;
    break;
  case AT_HOLD_BALL1_READ_SUCCESS:
    gRobot.AT_motionFlag|=AT_HOLD_BALL1_READ_SUCCESS;
    break;
  case ~AT_HOLD_BALL1_READ_SUCCESS:
    gRobot.AT_motionFlag&=~AT_HOLD_BALL1_READ_SUCCESS;
    break;
  case AT_HOLD_BALL2_READ_SUCCESS:
    gRobot.AT_motionFlag|=AT_HOLD_BALL2_READ_SUCCESS;
    break;
  case ~AT_HOLD_BALL2_READ_SUCCESS:
    gRobot.AT_motionFlag&=~AT_HOLD_BALL2_READ_SUCCESS;
    break;
  case AT_COURSE_READ_SUCCESS:
    gRobot.AT_motionFlag|=AT_COURSE_READ_SUCCESS;
    break;
  case ~AT_COURSE_READ_SUCCESS:
    gRobot.AT_motionFlag&=~AT_COURSE_READ_SUCCESS;
    break;
  case AT_PITCH_READ_SUCCESS:
    gRobot.AT_motionFlag|=AT_PITCH_READ_SUCCESS;
    break;
  case ~AT_PITCH_READ_SUCCESS:
    gRobot.AT_motionFlag&=~AT_PITCH_READ_SUCCESS;
    break;
  case AT_COURSE_SUCCESS:
    gRobot.AT_motionFlag|=AT_COURSE_SUCCESS;
    break;
  case ~AT_COURSE_SUCCESS:
    gRobot.AT_motionFlag&=~AT_COURSE_SUCCESS;
    break;
  case AT_PITCH_SUCCESS:
    gRobot.AT_motionFlag|=AT_PITCH_SUCCESS;
    break;
  case ~AT_PITCH_SUCCESS:
    gRobot.AT_motionFlag&=~AT_PITCH_SUCCESS;
    break;
  case AT_GAS_SUCCESS:
    gRobot.AT_motionFlag|=AT_GAS_SUCCESS;
    break;
  case ~AT_GAS_SUCCESS:
    gRobot.AT_motionFlag&=~AT_GAS_SUCCESS;
    break;
  case AT_CAMERA_TALK_SUCCESS:
    gRobot.AT_motionFlag|=AT_CAMERA_TALK_SUCCESS;
    break;
  case ~AT_CAMERA_TALK_SUCCESS:
    gRobot.AT_motionFlag&=~AT_CAMERA_TALK_SUCCESS;
    break;
  case AT_CAMERA_ROTATE_SUCCESS:
    gRobot.AT_motionFlag|=AT_CAMERA_ROTATE_SUCCESS;
    break;
  case ~AT_CAMERA_ROTATE_SUCCESS:
    gRobot.AT_motionFlag&=~AT_CAMERA_ROTATE_SUCCESS;
    break;
  }
}


