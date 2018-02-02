#include "steer.h"
#include "usart.h"
#include "stm32f4xx_tim.h"
#include  <includes.h>
#include "customer.h"
#include "task.h"
#include "timer.h"
#include "process.h"

extern Robot_t gRobot;

//以vel的速度转到angle角度
void HoldBallPosCrl(float angle,int vel)
{
  HoldSteer1PosCrl(angle,vel);
  HoldSteer2PosCrl(angle,vel);
}


void HoldSteer1PosCrl(float angle,int vel)
{  
	int pos=0.f;
  /*检测爪子是否闭合，张开时转动会有干涉*/
  if(gRobot.AT_motionFlag&AT_CLAW_STATUS_OPEN)
  {
    ClawShut();
  }
  if(angle>100.f)
    angle=100.f;
  if(angle<-100.f)
    angle=-100.f;
  
  /*1/4096.f*360.f=11.378*/
  pos=(int)((180.f-(angle-3.4f))*11.378f);  
  
  SteerPosCrlBy485(0x01,pos);
}

void HoldSteer2PosCrl(float angle,int vel)
{
  int pos=0.f;
  /*检测爪子是否闭合，张开时转动会有干涉*/
  if(gRobot.AT_motionFlag&AT_CLAW_STATUS_OPEN)
  {
    ClawShut();
  }
  if(angle>100.f)
    angle=100.f;
  if(angle<-100.f)
    angle=-100.f;
  
  /*1/4096.f*360.f=11.378*/
  pos=(int)(((angle-14.5f)/7.f*6.f+180.f)*11.378f);	 
  
  SteerPosCrlBy485(0x02,pos);
}

void CameraSteerPosCrl(float angle)
{
  int pos=0.f;
  /*
  *字头：0XFF, 0XFF 
  *ID ： num
  *长度：0x05 3+2 （1个数据长度 位置）
  *指令：0x03 写指令
  *地址：0x2A 储存ID的地址
  *校验和：checkSum
  */	
  if(angle>45.f)
    angle=45.f;
  else if(angle<-45.f)
    angle=-45.f;
  /*减速比为2*/
  angle=angle*2.f+259.7f;
  
  pos=angle*11.378f;
  
  SteerPosCrlBy485(0x03,pos);
  
  /*应答包 FF FF 01（ID） 02（数据长度） 00（错误状态） FA（校验和） */
}


void CameraAlign(void)
{
  float x=0.f;
  float y=0.f;
  float direction=0.f;
  
  x=gRobot.posX+CAMERA_TO_GYRO_X;
  y=gRobot.posY-CAMERA_TO_GYRO_Y;
  
  if(gRobot.process>=TO_GET_BALL_1&&gRobot.process<TO_GET_BALL_3)
  {
    direction=atan2f(QUICK_MARK_X_1-x,y-QUICK_MARK_Y);
  }else if(gRobot.process>=TO_GET_BALL_3)
  {
    direction=atan2f(QUICK_MARK_X_2-x,y-QUICK_MARK_Y);
  }
  
  direction=RAD_TO_ANGLE(direction);
  
  if(direction>30.f)
    direction=30.f;
  else if(direction<-30.f)
    direction=-30.f;
  
  CameraSteerPosCrl(direction);
}



void SteerPosCrlBy485(int num,int pos)
{
  u8 command[9]={0XFF, 0XFF, num, 0X05, 0X03, 0X2A, 0x01,0x01,0x00};
  command[6]=pos&0xFF;
  command[7]=(pos>>8)&0xFF;
  u8 checkSum = (command[2]+command[3]+command[4]+command[5]+command[6]+command[7])&0xFF;
  checkSum=~checkSum;
  command[8]=checkSum;
  
  RS485_Send_Data(command,9);
}


void SetSteerByte(uint8_t num,uint8_t address,uint8_t value)
{
	unsigned char command[8]={0XFF, 0XFF, num, 0X04, 0X03, address, value, 0x00};
	unsigned char checkSum = (command[2]+command[3]+command[4]+command[5]+command[6])&0xFF;
	checkSum=~checkSum;
	command[7]=checkSum;
	
	while(ReadOneByte(num,address)!=value)
		RS485_Send_Data(command,8);
}


/*设置回答等级*/
uint8_t ReadOneByte(int num,int address)
{
  unsigned char command[8]={0XFF, 0XFF, num, 0X04, 0X02, address, 0x01, 0x00};
  unsigned char checkSum = (command[2]+command[3]+command[4]+command[5]+command[6])&0xFF;
  checkSum=~checkSum;
  command[7]=checkSum;
	
	if(num==0x01)
	{
		while(!(gRobot.AT_motionFlag&AT_HOLD_BALL1_RESPONSE_SUCCESS))
		{
			RS485_Send_Data(command,8);
			Delay_us(500);
		};
		SetMotionFlag(~AT_HOLD_BALL1_RESPONSE_SUCCESS);
	}else if(num==0x02){

		while(!(gRobot.AT_motionFlag&AT_HOLD_BALL2_RESPONSE_SUCCESS))
		{
			RS485_Send_Data(command,8);
			Delay_us(500);
		};
		SetMotionFlag(~AT_HOLD_BALL2_RESPONSE_SUCCESS);
	}else if(num==0x03){
		while(!(gRobot.AT_motionFlag&AT_CAMERA_RESPONSE_SUCCESS))
		{
			RS485_Send_Data(command,8);
			Delay_us(500);
		};
		SetMotionFlag(~AT_CAMERA_RESPONSE_SUCCESS);
	}
	
	return gRobot.steerByte;
}

void OpenSteerAll(void)
{
	SetSteerByte(0x01,TORQUE_SWITCH,0x01);
	SetSteerByte(0x02,TORQUE_SWITCH,0x01);
	SetSteerByte(0x03,TORQUE_SWITCH,0x01);
}

void ShutAllSteerResponse(void)
{
	SetSteerByte(0x01,RESPONSE_STAIR,0x00);
	SetSteerByte(0x02,RESPONSE_STAIR,0x00);
	SetSteerByte(0x03,RESPONSE_STAIR,0x00);
}
void SetSteerNum(uint8_t num)
{
	//解锁flash
	SetSteerByte(0xfe,LOCK_SWITCH,0x00);
	
	unsigned char command[8]={0XFF, 0XFF, 0xfe, 0X04, 0X03, ID_AREA, num, 0x00};
	unsigned char checkSum = (command[2]+command[3]+command[4]+command[5]+command[6])&0xFF;
	checkSum=~checkSum;
	command[7]=checkSum;
	
	RS485_Send_Data(command,8);
}


/*******DEFINE*******/
#define INSTRUCTION_PING												0x01 
#define INSTRUCTION_READ												0x02 
#define INSTRUCTION_WRITE												0x03
#define INSTRUCTION_REG_WRITE										0x04
#define INSTRUCTION_ACTION											0x05 
#define INSTRUCTION_SYNC_WRITE									0x83 
#define INSTRUCTION_BULKWRITE_DATA							0x09 
#define INSTRUCTION_RESET												0x06 

void SteerResponseError(unsigned char errorWord)
{
  if(errorWord&0x20)
  {
    
  }else if(errorWord&0x08)
  {
    
  }else if(errorWord&0x04)
  {
    
  }else if(errorWord&0x02)
  {
    
  }else if(errorWord&0x01)
  {
    
  }
  else
  {
    //成功
  }
}
/****************舵机一串口接收中断****start****************/

void USART1_IRQHandler(void)
{
  static uint8_t ch;
  static uint8_t step;
  static unsigned char buffer[4]={0};
  static int i=0;
  static float pos=0;
  OS_CPU_SR  cpu_sr;
  OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
  OSIntNesting++;
  OS_EXIT_CRITICAL();
  
  if(USART_GetITStatus(USART1, USART_IT_RXNE)==SET)   
  {
    USART_ClearITPendingBit(USART1,USART_IT_RXNE);
    ch=USART_ReceiveData(USART1);
    if(ch=='@') step=0;
    
    switch(step)
    {
    case 0:
      if(ch=='@')
        step++;
      else
        step=0;
      break;
    case 1:
      if(ch=='1')
        step++;
      else
        step=0;
      break;
    case 2:
      if(ch==' ')
        step++;
      else
        step=0;
      break;
    case 3:
      if(ch=='A')
        step++;
      else
        step=0;
      break;
    case 4:
      if(ch=='C')
        step++;
      else
        step=0;
      break;
    case 5:
      if(ch=='K')
        step++;
      else
        step=0;
      break;
    case 6:
      if(ch=='\r')
        step++;
      else if(ch==' ')
        step=8;
      else
        step=0;
      break;
    case 7:
      if(ch=='\n')
      {
        SetMotionFlag(AT_HOLD_BALL1_SUCCESS);
        step=0;
      }else
        step=0;
      break;
    case 8:
      if(ch=='5')
        step++;
      else
        step=0;
      break;
    case 9:
      if(ch=='6')
        step++;
      else
        step=0;
      break;
      //56,2,4095\r\n
    case 10:
      if(ch==',')
        step++;
      else
        step=0;
      break;
    case 11:
      if(ch=='2')
        step++;
      else
        step=0;
      break;
    case 12:
      if(ch==',')
        step++;
      else
        step=0;
      break;
    case 13:
      if(ch=='\r')
      {
        step++;
        for(int j=0;j<i;j++)
        {
          pos+=(buffer[j]-'0')*pow(10,i-j-1);
        }
        if(pos>4095||pos<0)
        {
          USART_OUT(DEBUG_USART,"steer1readbackvalueout\r\n");
          step=0;
        }else
        {
          gRobot.holdBallAngle[0]=180.f-pos/11.378f+3.3f;
        }
        i=100;
      }
      pos=0;
      if(i<4){
        buffer[i]=ch;
        i++;
      }else
        i=0;
      break;
    case 14:
      if(ch=='\n')
      {
        SetMotionFlag(AT_HOLD_BALL1_RESPONSE_SUCCESS);
        step=0;
      }else
        step=0;
      break;
    }
    
  }else{
    USART_ReceiveData(USART1);
  }
  OSIntExit();
}

/****************舵机二串口接收中断****start****************/

void UART5_IRQHandler(void)
{
  static uint8_t ch;
  static uint8_t step;
  static unsigned char buffer[4]={0};
  static int i=0;
  static int pos=0;
  
  OS_CPU_SR  cpu_sr;
  OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
  OSIntNesting++;
  OS_EXIT_CRITICAL();
  
  if(USART_GetITStatus(UART5, USART_IT_RXNE)==SET)   
  {
    USART_ClearITPendingBit(UART5,USART_IT_RXNE);
    ch=USART_ReceiveData(UART5);
    if(ch=='@') step=0;
    switch(step)
    {
    case 0:
      if(ch=='@')
        step++;
      else
        step=0;
      break;
    case 1:
      if(ch=='1')
        step++;
      else
        step=0;
      break;
    case 2:
      if(ch==' ')
        step++;
      else
        step=0;
      break;
    case 3:
      if(ch=='A')
        step++;
      else
        step=0;
      break;
    case 4:
      if(ch=='C')
        step++;
      else
        step=0;
      break;
    case 5:
      if(ch=='K')
        step++;
      else
        step=0;
      break;
    case 6:
      if(ch=='\r')
        step++;
      else if(ch==' ')
        step=8;
      else
        step=0;
      break;
    case 7:
      if(ch=='\n')
      {
        SetMotionFlag(AT_HOLD_BALL2_SUCCESS);
        step=0;
      }else
        step=0;
      break;
    case 8:
      if(ch=='5')
        step++;
      else
        step=0;
      break;
    case 9:
      if(ch=='6')
        step++;
      else
        step=0;
      break;
      //56,2,4095\r\n
    case 10:
      if(ch==',')
        step++;
      else
        step=0;
      break;
    case 11:
      if(ch=='2')
        step++;
      else
        step=0;
      break;
    case 12:
      if(ch==',')
        step++;
      else
        step=0;
      break;
    case 13:
      if(ch=='\r')
      {
        step++;
        for(int j=0;j<i;j++)
        {
          pos+=(buffer[j]-'0')*pow(10,i-j-1);
        }
        if(pos>4095||pos<0)
        {
          USART_OUT(DEBUG_USART,"steer2readbackvalueout\r\n");
          step=0;
        }else
        {
          gRobot.holdBallAngle[1]=pos/11.378f-180.f;
        }
        i=100;
      }
      pos=0;
      
      if(i<4){
        buffer[i]=ch;
        i++;
      }else
        i=0;
      break;
    case 14:
      if(ch=='\n')
      {
        SetMotionFlag(AT_HOLD_BALL2_RESPONSE_SUCCESS);
        step=0;
      }else
        step=0;
      break;
    }
    
  }else{
    USART_ReceiveData(UART5);
  }
  OSIntExit();
}




