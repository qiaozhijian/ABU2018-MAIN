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


#define CLAW 						1
#define SHOOT 					2
#define PITCH 					3
#define STEER 					4
#define GAS  						5
#define COURSE  				6
#define TEST_GAS  			7
#define CAMERA	  			8
#define STEER1 					9
#define STEER2 					10
#define BOOST 					11
#define LOWER_CLAW_STAIR 					12
#define STAIR2 					13

extern Robot_t gRobot;

/*调试蓝牙中断*/
static char buffer[20];
static int bufferI=0;
static int atCommand=0;

void AT_CMD_Judge(void);
	
void BufferInit(void){
  bufferI=0;
	atCommand=0;
  for(int i=0;i<20;i++)
    buffer[i]=0;
}

/*平板控制蓝牙串口中断*/
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
      }
    }
  }else{
    data=USART_ReceiveData(UART4);
  }
  OSIntExit();
}



void AT_CMD_Judge(void){
	
  if((bufferI == 7) && strncmp(buffer, "AT+1", 4)==0)//AT    
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
  else if((bufferI >= 4) && strncmp(buffer, "AT+9", 4)==0)//发射按钮   
    atCommand=STEER1;
  else if((bufferI >= 5) && strncmp(buffer, "AT+12", 5)==0)//发射按钮   
    atCommand=STEER2;
  else if((bufferI >= 5) && strncmp(buffer, "AT+13", 5)==0)//发射按钮   
    atCommand=BOOST;
  else if((bufferI >= 5) && strncmp(buffer, "AT+14", 5)==0)//发射按钮   
    atCommand=LOWER_CLAW_STAIR;
  else if((bufferI >= 5) && strncmp(buffer, "AT+15", 5)==0)//发射按钮   
    atCommand=STAIR2;
	
//  if((bufferI == 4) && strncmp(buffer, "AT\r\n",4 )==0)//AT    
//  {
//		
//		SetMotionFlag(AT_CAMERA_TALK_SUCCESS);
//    //摄像头连接成功
//  }
	
  /*如果是及时处理的命令，就初始化*/
	if(atCommand==0)
	{
		BufferInit();
	}
}



void AT_CMD_Handle(void){
  float value=0.0f;
  switch(atCommand)
  {
  case 0:
    break;
    /*控制张爪*/
  case CLAW:
    USART_OUTByDMA("OK\r\n");
    if(*(buffer + 4) == '1') 
    {
      ClawShut();
    }
    else if(*(buffer + 4) == '0') 
    {
      ClawOpen();
    } 
    else{
		
		}
    break;
    
    /*控制是否射击*/
  case SHOOT:
    USART_OUTByDMA("OK\r\n");
    if(*(buffer + 4) == '1')
    {
			ShootSmallOpen();
			Delay_ms(300);
			if(PE_FOR_THE_BALL){
				ClawOpen();
				Delay_ms(50);
				ShootBigOpen();
			}
    }
    else if(*(buffer + 4) == '0') 
    {
      if(gRobot.sDta.AT_motionFlag&(AT_SHOOT_BIG_ENABLE|AT_SHOOT_SMALL_ENABLE))
      {
        ShootSmallShut();
				ShootBigShut();
				ClawShut();
      }
    }
    break;
    
  case GAS:
    USART_OUTByDMA("OK\r\n");
    //平板的值
    value = atof(buffer + 4);
  
  	GasMotion(value);
  	GasEnable();

    break;
    
  case PITCH:
    USART_OUTByDMA("OK\r\n");
    value = atof(buffer + 4);
		gRobot.sDta.pitchAimAngle=value;
		PitchAngleMotion(gRobot.sDta.pitchAimAngle);
    break;
    
  case COURSE:
    USART_OUTByDMA("OK\r\n");
    value = atof(buffer + 4);
		gRobot.sDta.courseAimAngle=value;
		CourseAngleMotion(gRobot.sDta.courseAimAngle);
    break;
   
	case TEST_GAS:
    USART_OUTByDMA("OK\r\n");
		//GasValveControl(GASVALVE_BOARD_ID,*(buffer+4)-'0',*(buffer+5)-'0');
		break;
	
  case CAMERA:
    USART_OUTByDMA("OK\r\n");
    value = atof(buffer + 4);
		gRobot.sDta.cameraAimAngle=value;
		CameraSteerPosCrl(gRobot.sDta.cameraAimAngle);
    break;
    
  case STEER:
    USART_OUTByDMA("OK\r\n");
    value = atof(buffer + 4);
		gRobot.sDta.holdBallAimAngle[0]=value;
		gRobot.sDta.holdBallAimAngle[1]=value;
		HoldBallPosCrl(gRobot.sDta.holdBallAimAngle[0]);
    break;
		
	case STEER1:
    USART_OUTByDMA("OK\r\n");
    value = atof(buffer + 4);
		gRobot.sDta.holdBallAimAngle[0]=value;
		HoldSteer1PosCrl(gRobot.sDta.holdBallAimAngle[0]);
		break;
		
	case STEER2:
    USART_OUTByDMA("OK\r\n");
    value = atof(buffer + 5);
		gRobot.sDta.holdBallAimAngle[1]=value;
		HoldSteer2PosCrl(gRobot.sDta.holdBallAimAngle[1]);
		break;
  
  case LOWER_CLAW_STAIR:
    USART_OUTByDMA("OK\r\n");
    if(*(buffer + 5) == '1') 
    {
      LowerClawStairOn();
    }
    else if(*(buffer + 5) == '0') 
    {
      LowerClawStairOff();
    } 
    break;
  case STAIR2:
    USART_OUTByDMA("OK\r\n");
    if(*(buffer + 5) == '1') 
    {
      GoldBallGraspStairTwoOn();
    }
    else if(*(buffer + 5) == '0') 
    {
      GoldBallGraspStairTwoOff();
    } 
    break;
	
  default:
    break;
  }
  
  BufferInit();
}





