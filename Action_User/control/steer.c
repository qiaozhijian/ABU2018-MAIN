#include "steer.h"
#include "usart.h"
#include "stm32f4xx_tim.h"
#include  <includes.h>
#include "customer.h"
#include "task.h"
#include "timer.h"
#include "process.h"
#include "robot.h"

extern Robot_t gRobot;

//以vel的速度转到angle角度,但由于现在给定一直都是最大加速度所以就没用到vel
void HoldBallPosCrl(float angle,int vel)
{
  HoldSteer1PosCrl(angle,vel);
  HoldSteer2PosCrl(angle,vel);
}
/*vel1 为舵机1的速度上舵机  vel2为下舵机的速度*///两个舵机分开控制
void HoldBallPosCrlSeparate(float angle0,float angle1,int vel)
{
  HoldSteer1PosCrl(angle0,vel);
  HoldSteer2PosCrl(angle1,vel);
}
void Enable_ROBS(void)
{
  static int times;
  while(!(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_2_SUCCESS))
  {
    USART_OUT(UART5,"#254 W 40,1,1\r\n");
    Delay_ms(1);
    times ++;
    if(times >10)
    {
      USART_OUT(DEBUG_USART,"HOLD_BALL_2_ENABLE_FAIL\r\n");
      break;
    }
  }
  SetMotionFlag(~AT_HOLD_BALL_2_SUCCESS);

  times =0;
  //#1 W 40,1,1\r\n
}

void HoldSteer2PosCrl(float angle,int vel)
{
  int pos=0.f;

  if(angle>100.f)
    angle=100.f;
  if(angle<-100.f)
    angle=-100.f;
 
	/*1/4096.f*360.f=11.378*/
	pos=(int)(((angle+39.f)+180.f)*11.378f);  
  
  SteerPosCrlBy485(HOLD_BALL_2,pos);
}
void HoldSteer1PosCrl(float angle,int vel)
{
  int pos=0.f;

	if(gRobot.sDta.AT_motionFlag&AT_CLAW_STATUS_OPEN)
  {
    ClawShut();
  }
	if(angle>100.f)
		angle=100.f;
	if(angle<-100.f)
		angle=-100.f;

	/*1/4096.f*360.f=11.378*//*减速比25/16*/
  pos=(int)((180.f-(angle+9.f)*25.f/16.f)*11.378f);  
	
  USART_OUT(UART5,"#254 W 42,2,%d:46,2,%d\r\n",pos,vel);
}

//void HoldSteer2PosCrl(float angle,int vel)
//{
//  int pos=0.f;

//	if(angle>100.f)
//		angle=100.f;
//	if(angle<-100.f)
//		angle=-100.f;

//	/*1/4096.f*360.f=11.378*/
//	pos=(int)(((angle-1)*6.f/7.f+180.f)*11.378f);  
//	
//  USART_OUT(UART5,"#1 W 42,2,%d:46,2,%d\r\n",pos,vel);
//}
//void HoldSteer1PosCrl(float angle,int vel)
//{  
//	int pos=0.f;
//  /*检测爪子是否闭合，张开时转动会有干涉*/
//  if(gRobot.sDta.AT_motionFlag&AT_CLAW_STATUS_OPEN)
//  {
//    ClawShut();
//  }
//  if(angle>100.f)
//    angle=100.f;
//  if(angle<-100.f)
//    angle=-100.f;
//  
//  /*1/4096.f*360.f=11.378*//*减速比25/16*/
//  pos=(int)((180.f-(angle)*25/16)*11.378f);  
//  
//  SteerPosCrlBy485(HOLD_BALL_1,pos);
//}


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
  
  SteerPosCrlBy485(CAMERA_STEER,pos);
  
  /*应答包 FF FF 01（ID） 02（数据长度） 00（错误状态） FA（校验和） */
}


void CameraAlign(void)
{
  float x=0.f;
  float y=0.f;
  float direction=0.f;
  
  x=gRobot.posX+CAMERA_TO_GYRO_X;
  y=gRobot.posY-CAMERA_TO_GYRO_Y;
  
  if(gRobot.sDta.process>=TO_GET_BALL_1&&gRobot.sDta.process<TO_GET_BALL_3)
  {
    direction=atan2f(QUICK_MARK_X_1-x,y-QUICK_MARK_Y);
  }else if(gRobot.sDta.process>=TO_GET_BALL_3)
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
	/*此时0x01,0x01都是value 此时这个value是16位的，先输入低字节，再输入高字节*/
  u8 command[9]={0XFF, 0XFF, num, 0X05, 0X03, AIM_POS, 0x01,0x01,0x00};
  command[6]=pos&0xFF;
  command[7]=(pos>>8)&0xFF;
  u8 checkSum = (command[2]+command[3]+command[4]+command[5]+command[6]+command[7])&0xFF;
  checkSum=~checkSum;
  command[8]=checkSum;
  
  RS485_Send_Data(command,9);
	/*设置舵机转动为最大转动速度*/
	SetSteerWord(num,AIM_TIME,0x0000);
}

void SetSteerWord(uint8_t num,uint8_t address,uint8_t value)
{

  u8 command[9]={0XFF, 0XFF, num, 0X05, 0X03, address, 0x01,0x01,0x00};
  command[6]=value&0xFF;
  command[7]=(value>>8)&0xFF;
  u8 checkSum = (command[2]+command[3]+command[4]+command[5]+command[6]+command[7])&0xFF;
  checkSum=~checkSum;
  command[8]=checkSum;
  
  RS485_Send_Data(command,9);
}

//舵机发送OCS指令包
void SetSteerByte(uint8_t num,uint8_t address,uint8_t value)
{
	/*八位分别代表 第0,1位字头 第2位舵机ID号 第3位数据长度 第4位指令 第5，6位参数    第7位校验和*/
	unsigned char command[8]={0XFF, 0XFF, num, 0X04, 0X03, address, value, 0x00};
	unsigned char checkSum = (command[2]+command[3]+command[4]+command[5]+command[6])&0xFF;
	/*校验和 Check Sum = ~ (ID + Length + Instruction + Parameter1 + ... Parameter N)*/
	checkSum=~checkSum;
	command[7]=checkSum;
	/*ReadOneByte函数里面也存在对扭矩开关打开的发数，现在一个周期发两次，一次是ReadOneByte里面的扭矩开关打开，和自定义的while的发数*/
	while(ReadOneByte(num,address)!=value)
		RS485_Send_Data(command,8);
}

void ReadSteerError(int num)
{
	/*写入对舵机的 RAM的ID进行写入  为了获取错误状态*/
  unsigned char command[8]={0XFF, 0XFF, num, 0X04, 0X02, ID_AREA, 0x01, 0x00};
  unsigned char checkSum = (command[2]+command[3]+command[4]+command[5]+command[6])&0xFF;
  checkSum=~checkSum;
  command[7]=checkSum;
	RS485_Send_Data(command,8);
}

void ReadSteerErrorAll(void)
{
	static int num=HOLD_BALL_1;
	switch(num)
	{
		case HOLD_BALL_1:
			ReadSteerError(HOLD_BALL_1);
			num=HOLD_BALL_2;
			break;
		case HOLD_BALL_2:
			ReadSteerError(HOLD_BALL_2);
			num=CAMERA_STEER;
			break;
		case CAMERA_STEER:
			ReadSteerError(CAMERA_STEER);
			num=HOLD_BALL_1;
			break;
	}
}
	

/*设置回答等级*//*在串口2中断中读取舵机OCS模式应答包，如果舵机不能回复那就在while中反复问询,直到舵机回复为止*//*注意485是半双工*/
uint8_t ReadOneByte(int num,int address)
{
  unsigned char command[8]={0XFF, 0XFF, num, 0X04, 0X02, address, 0x01, 0x00};
  unsigned char checkSum = (command[2]+command[3]+command[4]+command[5]+command[6])&0xFF;
  checkSum=~checkSum;
  command[7]=checkSum;
	
	if(num==HOLD_BALL_1)
	{
		while(!(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_1_RESPONSE_SUCCESS))
		{
			RS485_Send_Data(command,8);
			Delay_us(500);
		};
		SetMotionFlag(~AT_HOLD_BALL_1_RESPONSE_SUCCESS);
	}else if(num==HOLD_BALL_2){

		while(!(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_2_RESPONSE_SUCCESS))
		{
			RS485_Send_Data(command,8);
			Delay_us(500);
		};
		SetMotionFlag(~AT_HOLD_BALL_2_RESPONSE_SUCCESS);
	}else if(num==CAMERA_STEER){
		while(!(gRobot.sDta.AT_motionFlag&AT_CAMERA_RESPONSE_SUCCESS))
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
	/*485 版本的舵机
	通过写入舵机的内存控制表,TORQUE_SWITCH-0x28是扭矩开关所位于的地址写入1将其打开*/
//	SetSteerByte(HOLD_BALL_1,TORQUE_SWITCH,0x01);
	/*485 版本的舵机*/
	SetSteerByte(HOLD_BALL_2,TORQUE_SWITCH,0x01);
	/*ttl版本的舵机*/
	Enable_ROBS();
//	SetSteerByte(HOLD_BALL_2,TORQUE_SWITCH,0x01);


}

void LetSteerRound(int num,float angle){
	static int cnt=5;
	unsigned char command[8]={0XFF, 0XFF, 0xfe, 0X04, 0X03, TORQUE_SWITCH, 0x01, 0x00};
	unsigned char checkSum = (command[2]+command[3]+command[4]+command[5]+command[6])&0xFF;
	while(cnt--)
	{
		checkSum=~checkSum;
	  command[7]=checkSum;
		RS485_Send_Data(command,8);
		Delay_ms(100);
	}
	Delay_ms(500);
	cnt=5;
	while(cnt--)
	{
		int pos=(int)((180.f+angle)*11.378f);
		SteerPosCrlBy485(num,pos);
		Delay_ms(10);
	}
}

void ShutAllSteerResponse(void)
{
//	SetSteerByte(HOLD_BALL_1,RESPONSE_STAIR,0x00);
	SetSteerByte(HOLD_BALL_2,RESPONSE_STAIR,0x00);
	//SetSteerByte(CAMERA_STEER,RESPONSE_STAIR,0x00);
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
/*485舵机的各种错误状态*/
#define VOLTAGE_ERROR														0x01 
#define ANGLE_ERROR															0x02 
#define TEMPERATURE_ERROR												0x04
#define CURRENT_ERROR														0x08
#define OVERLOAD_ERROR													0x20 

void SteerResponseError(uint8_t num, uint8_t errorWord)
{
	if(errorWord==0x00) return;
	switch(num)
	{
		case HOLD_BALL_1:
			USART_OUT(DEBUG_USART,"HOLD_BALL_1\t");
			break;
		case HOLD_BALL_2:
			USART_OUT(DEBUG_USART,"HOLD_BALL_2\t");
			break;
		case CAMERA_STEER:
			USART_OUT(DEBUG_USART,"CAMERA_STEER\t");
			break;
	}
  if(errorWord&VOLTAGE_ERROR)
  {
		USART_OUT(DEBUG_USART,"VOLTAGE_ERROR\t");
  }else if(errorWord&ANGLE_ERROR)
  {
		USART_OUT(DEBUG_USART,"ANGLE_ERROR\t");
  }else if(errorWord&TEMPERATURE_ERROR)
  {
		USART_OUT(DEBUG_USART,"TEMPERATURE_ERROR\t");
  }else if(errorWord&CURRENT_ERROR)
  {
		USART_OUT(DEBUG_USART,"CURRENT_ERROR\t");
  }else if(errorWord&OVERLOAD_ERROR)
  {
		USART_OUT(DEBUG_USART,"OVERLOAD_ERROR\t");
  }
  else
  {
    //成功
  }
	USART_Enter();
}

/****************舵机一 和摄像头舵机串口接收中断********************/

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
				//数据长度
				step++;
				break;	
			case 4:
				//错误状态
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
        SetMotionFlag(AT_HOLD_BALL_2_SUCCESS);
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
        SetMotionFlag(AT_HOLD_BALL_2_RESPONSE_SUCCESS);
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


