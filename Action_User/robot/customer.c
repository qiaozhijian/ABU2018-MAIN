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
#define STAIR1 					12
#define STAIR2 					13

/*���������ж�*/
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

void USART2_IRQHandler(void)
{
  uint8_t data;
	static uint8_t step=0;
	static uint16_t num;
  OS_CPU_SR  cpu_sr;
  OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
  OSIntNesting++;
  OS_EXIT_CRITICAL();
  if(USART_GetITStatus(USART2,USART_IT_RXNE)==SET)
  {
    USART_ClearITPendingBit( USART2,USART_IT_RXNE);
    data=USART_ReceiveData(USART2);
		//0XFF 0XFF 0X01(ID) 0X03(LENGTH) 0X00(STATUS) 0X20(DATA) 0XDD
		switch(step)
		{
			case 0:
				if(data==0xff)
					step++;
				else 
					step=0;
				break;
			case 1:
				if(data==0xff)
					step++;
				else 
					step=0;
				break;
			case 2:
				if((data==0x01)||(data==0x02)||(data==0x03))
				{
					step++;
					num=data;
				}
				else step=0;
				break;
			case 3:
				//���ݳ���
				step++;
				break;	
			case 4:
				//����״̬
				if(num==1)
				{
					gRobot.holdBall1Error=data;
				}else if(num==2)
				{
					gRobot.holdBall2Error=data;
				}else if(num==3)
				{
					gRobot.cameraSteerError=data;
				}
				step++;
				break;	
			case 5:
				gRobot.steerByte=data;
				step++;
				break;
			case 6:
				if(num==1)
				{
					SetMotionFlag(AT_HOLD_BALL_1_RESPONSE_SUCCESS);
				}else if(num==2)
				{
					SetMotionFlag(AT_HOLD_BALL_2_RESPONSE_SUCCESS);
				}else if(num==3)
				{
					SetMotionFlag(AT_CAMERA_RESPONSE_SUCCESS);
				}
				num=0;
				step=0;
				break;
		}
    
  }else{
    data=USART_ReceiveData(USART2);
  }
  OSIntExit();
}

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
	
  if((bufferI == 7) && strncmp(buffer, "AT+1", 4)==0)//AT    
    atCommand=CLAW;
  else if((bufferI >= 4) && strncmp(buffer, "AT+2", 4)==0)//AT    
    atCommand=SHOOT;
  else if((bufferI >= 4) && strncmp(buffer, "AT+3", 4)==0)//AT    
    atCommand=PITCH;
  else if((bufferI >= 4) && strncmp(buffer, "AT+4", 4)==0)//���䰴ť   
    atCommand=STEER;
  else if((bufferI >= 4) && strncmp(buffer, "AT+5", 4)==0)//���䰴ť   
    atCommand=GAS;
  else if((bufferI >= 4) && strncmp(buffer, "AT+6", 4)==0)//���䰴ť   
    atCommand=COURSE;
  else if((bufferI >= 4) && strncmp(buffer, "AT+7", 4)==0)//���䰴ť   
    atCommand=TEST_GAS;
  else if((bufferI >= 4) && strncmp(buffer, "AT+8", 4)==0)//���䰴ť   
    atCommand=CAMERA;
  else if((bufferI >= 4) && strncmp(buffer, "AT+9", 4)==0)//���䰴ť   
    atCommand=STEER1;
  else if((bufferI >= 5) && strncmp(buffer, "AT+12", 5)==0)//���䰴ť   
    atCommand=STEER2;
  else if((bufferI >= 5) && strncmp(buffer, "AT+13", 5)==0)//���䰴ť   
    atCommand=BOOST;
  else if((bufferI >= 5) && strncmp(buffer, "AT+14", 5)==0)//���䰴ť   
    atCommand=STAIR1;
  else if((bufferI >= 5) && strncmp(buffer, "AT+15", 5)==0)//���䰴ť   
    atCommand=STAIR2;
	
  if((bufferI == 4) && strncmp(buffer, "AT\r\n",4 )==0)//AT    
  {
		
		SetMotionFlag(AT_CAMERA_TALK_SUCCESS);
    //����ͷ���ӳɹ�
  }
	
  /*����Ǽ�ʱ���������ͳ�ʼ��*/
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
    /*������צ*/
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
    
    /*�����Ƿ����*/
  case SHOOT:
    USART_OUT(DEBUG_USART,"OK\r\n");
    if(*(buffer + 4) == '1')
    {
      if(gRobot.sDta.AT_motionFlag&(AT_CLAW_STATUS_OPEN|AT_STEER_READY))
      {
        ShootSmallOpen();
        ShootBigOpen();
        SetMotionFlag(AT_SHOOT_BIG_ENABLE);
        SetMotionFlag(AT_SHOOT_SMALL_ENABLE);
      }
    }
    else if(*(buffer + 4) == '0') 
    {
      if(gRobot.sDta.AT_motionFlag&(AT_SHOOT_BIG_ENABLE|AT_SHOOT_SMALL_ENABLE))
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
    
  case GAS:
    USART_OUT(DEBUG_USART,"OK\r\n");
    //ƽ���ֵ
    value = atof(buffer + 4);
    CAN_TxMsg(CAN2,SEND_TO_GASSENSOR,(uint8_t*)(&value),4);
    break;
    
  case PITCH:
    USART_OUT(DEBUG_USART,"OK\r\n");
    value = atof(buffer + 4);
		gRobot.sDta.pitchAimAngle=value;
		#ifdef TEST
		PitchAngleMotion(gRobot.sDta.pitchAimAngle);
		#endif
    break;
    
  case COURSE:
    USART_OUT(DEBUG_USART,"OK\r\n");
    value = atof(buffer + 4);
		gRobot.sDta.courseAimAngle=value;
		#ifdef TEST
		CourseAngleMotion(gRobot.sDta.courseAimAngle);
		#endif
    break;
   
	case TEST_GAS:
    USART_OUT(DEBUG_USART,"OK\r\n");
		//GasValveControl(GASVALVE_BOARD_ID,*(buffer+4)-'0',*(buffer+5)-'0');
		break;
	
  case CAMERA:
    USART_OUT(DEBUG_USART,"OK\r\n");
    value = atof(buffer + 4);
		gRobot.sDta.cameraAimAngle=value;
		CameraSteerPosCrl(gRobot.sDta.cameraAimAngle);
    break;
    
  case STEER:
    USART_OUT(DEBUG_USART,"OK\r\n");
    value = atof(buffer + 4);
		#ifdef TEST
		gRobot.sDta.holdBallAimAngle[0]=value;
		gRobot.sDta.holdBallAimAngle[1]=value;
		HoldBallPosCrl(gRobot.sDta.holdBallAimAngle[0],2000);
		#else
		gRobot.sDta.holdBallAimAngle=value;
		HoldBallPosCrl(gRobot.sDta.holdBallAimAngle,2000);
		#endif
    break;
		
	case STEER1:
    USART_OUT(DEBUG_USART,"OK\r\n");
    value = atof(buffer + 4);
		#ifdef TEST
		gRobot.sDta.holdBallAimAngle[0]=value;
		HoldSteer1PosCrl(gRobot.sDta.holdBallAimAngle[0],2000);
		#else
		gRobot.sDta.holdBallAimAngle=value;
		HoldSteer1PosCrl(gRobot.sDta.holdBallAimAngle,2000);
		#endif
		break;
		
	case STEER2:
    USART_OUT(DEBUG_USART,"OK\r\n");
    value = atof(buffer + 5);
		#ifdef TEST
		gRobot.sDta.holdBallAimAngle[1]=value;
		HoldSteer2PosCrl(gRobot.sDta.holdBallAimAngle[1],2000);
		#else
		gRobot.sDta.holdBallAimAngle=value;
		HoldSteer2PosCrl(gRobot.sDta.holdBallAimAngle,2000);
		#endif
		break;
  case BOOST:
    USART_OUT(DEBUG_USART,"OK\r\n");
    if(*(buffer + 5) == '1') 
    {
      BoostPolePush();
    }
    else if(*(buffer + 5) == '0') 
    {
      BoostPoleReturn();
    } 
    break;
  case STAIR1:
    USART_OUT(DEBUG_USART,"OK\r\n");
    if(*(buffer + 5) == '1') 
    {
      GoldBallGraspStairOneOn();
    }
    else if(*(buffer + 5) == '0') 
    {
      GoldBallGraspStairOneOff();
    } 
    break;
  case STAIR2:
    USART_OUT(DEBUG_USART,"OK\r\n");
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



void SetMotionFlag(uint32_t status){
  
  switch(status){
  case AT_CLAW_STATUS_OPEN:
    gRobot.sDta.AT_motionFlag|=AT_CLAW_STATUS_OPEN;
    break;
  case ~AT_CLAW_STATUS_OPEN:
    gRobot.sDta.AT_motionFlag&=~AT_CLAW_STATUS_OPEN;
    break;
  case AT_STEER_READY:
    gRobot.sDta.AT_motionFlag|=AT_STEER_READY;
    break;
  case ~AT_STEER_READY:
    gRobot.sDta.AT_motionFlag&=~AT_STEER_READY;
    break;
  case AT_SHOOT_BIG_ENABLE:
    gRobot.sDta.AT_motionFlag|=AT_SHOOT_BIG_ENABLE;
    break;
  case ~AT_SHOOT_BIG_ENABLE:
    gRobot.sDta.AT_motionFlag&=~AT_SHOOT_BIG_ENABLE;
    break;
  case AT_SHOOT_SMALL_ENABLE:
    gRobot.sDta.AT_motionFlag|=AT_SHOOT_SMALL_ENABLE;
    break;
  case ~AT_SHOOT_SMALL_ENABLE:
    gRobot.sDta.AT_motionFlag&=~AT_SHOOT_SMALL_ENABLE;
    break;
  case AT_HOLD_BALL_1_SUCCESS:
    gRobot.sDta.AT_motionFlag|=AT_HOLD_BALL_1_SUCCESS;
    break;
  case ~AT_HOLD_BALL_1_SUCCESS:
    gRobot.sDta.AT_motionFlag&=~AT_HOLD_BALL_1_SUCCESS;
    break;
  case AT_HOLD_BALL_2_SUCCESS:
    gRobot.sDta.AT_motionFlag|=AT_HOLD_BALL_2_SUCCESS;
    break;
  case ~AT_HOLD_BALL_2_SUCCESS:
    gRobot.sDta.AT_motionFlag&=~AT_HOLD_BALL_2_SUCCESS;
    break;
  case AT_HOLD_BALL_1_RESPONSE_SUCCESS:
    gRobot.sDta.AT_motionFlag|=AT_HOLD_BALL_1_RESPONSE_SUCCESS;
    break;
  case ~AT_HOLD_BALL_1_RESPONSE_SUCCESS:
    gRobot.sDta.AT_motionFlag&=~AT_HOLD_BALL_1_RESPONSE_SUCCESS;
    break;
  case AT_HOLD_BALL_2_RESPONSE_SUCCESS:
    gRobot.sDta.AT_motionFlag|=AT_HOLD_BALL_2_RESPONSE_SUCCESS;
    break;
  case ~AT_HOLD_BALL_2_RESPONSE_SUCCESS:
    gRobot.sDta.AT_motionFlag&=~AT_HOLD_BALL_2_RESPONSE_SUCCESS;
    break;
  case AT_COURSE_READ_SUCCESS:
    gRobot.sDta.AT_motionFlag|=AT_COURSE_READ_SUCCESS;
    break;
  case ~AT_COURSE_READ_SUCCESS:
    gRobot.sDta.AT_motionFlag&=~AT_COURSE_READ_SUCCESS;
    break;
  case AT_PITCH_READ_SUCCESS:
    gRobot.sDta.AT_motionFlag|=AT_PITCH_READ_SUCCESS;
    break;
  case ~AT_PITCH_READ_SUCCESS:
    gRobot.sDta.AT_motionFlag&=~AT_PITCH_READ_SUCCESS;
    break;
  case AT_COURSE_SUCCESS:
    gRobot.sDta.AT_motionFlag|=AT_COURSE_SUCCESS;
    break;
  case ~AT_COURSE_SUCCESS:
    gRobot.sDta.AT_motionFlag&=~AT_COURSE_SUCCESS;
    break;
  case AT_PITCH_SUCCESS:
    gRobot.sDta.AT_motionFlag|=AT_PITCH_SUCCESS;
    break;
  case ~AT_PITCH_SUCCESS:
    gRobot.sDta.AT_motionFlag&=~AT_PITCH_SUCCESS;
    break;
  case AT_GAS_SUCCESS:
    gRobot.sDta.AT_motionFlag|=AT_GAS_SUCCESS;
    break;
  case ~AT_GAS_SUCCESS:
    gRobot.sDta.AT_motionFlag&=~AT_GAS_SUCCESS;
    break;
  case AT_CAMERA_TALK_SUCCESS:
    gRobot.sDta.AT_motionFlag|=AT_CAMERA_TALK_SUCCESS;
    break;
  case ~AT_CAMERA_TALK_SUCCESS:
    gRobot.sDta.AT_motionFlag&=~AT_CAMERA_TALK_SUCCESS;
    break;
  case AT_CAMERA_RESPONSE_SUCCESS:
    gRobot.sDta.AT_motionFlag|=AT_CAMERA_RESPONSE_SUCCESS;
    break;
  case ~AT_CAMERA_RESPONSE_SUCCESS:
    gRobot.sDta.AT_motionFlag&=~AT_CAMERA_RESPONSE_SUCCESS;
    break;
  case AT_PREPARE_READY:
    gRobot.sDta.AT_motionFlag|=AT_PREPARE_READY;
    break;
  case ~AT_PREPARE_READY:
    gRobot.sDta.AT_motionFlag&=~AT_PREPARE_READY;
    break;
  case AT_IS_SEND_DEBUG_DATA:
    gRobot.sDta.AT_motionFlag|=AT_IS_SEND_DEBUG_DATA;
    break;
  case ~AT_IS_SEND_DEBUG_DATA:
    gRobot.sDta.AT_motionFlag&=~AT_IS_SEND_DEBUG_DATA;
    break;
  }
}


