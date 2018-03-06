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

//��vel���ٶ�ת��angle�Ƕ�,���������ڸ���һֱ���������ٶ����Ծ�û�õ�vel
void HoldBallPosCrl(float angle,int vel)
{
  HoldSteer1PosCrl(angle,vel);
  HoldSteer2PosCrl(angle,vel);
}
/*vel1 Ϊ���1���ٶ��϶��  vel2Ϊ�¶�����ٶ�*///��������ֿ�����
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

	/*1/4096.f*360.f=11.378*//*���ٱ�25/16*/
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
//  /*���צ���Ƿ�պϣ��ſ�ʱת�����и���*/
//  if(gRobot.sDta.AT_motionFlag&AT_CLAW_STATUS_OPEN)
//  {
//    ClawShut();
//  }
//  if(angle>100.f)
//    angle=100.f;
//  if(angle<-100.f)
//    angle=-100.f;
//  
//  /*1/4096.f*360.f=11.378*//*���ٱ�25/16*/
//  pos=(int)((180.f-(angle)*25/16)*11.378f);  
//  
//  SteerPosCrlBy485(HOLD_BALL_1,pos);
//}


void CameraSteerPosCrl(float angle)
{
  int pos=0.f;
  /*
  *��ͷ��0XFF, 0XFF 
  *ID �� num
  *���ȣ�0x05 3+2 ��1�����ݳ��� λ�ã�
  *ָ�0x03 дָ��
  *��ַ��0x2A ����ID�ĵ�ַ
  *У��ͣ�checkSum
  */	
  if(angle>45.f)
    angle=45.f;
  else if(angle<-45.f)
    angle=-45.f;
  /*���ٱ�Ϊ2*/
  angle=angle*2.f+259.7f;
  
  pos=angle*11.378f;
  
  SteerPosCrlBy485(CAMERA_STEER,pos);
  
  /*Ӧ��� FF FF 01��ID�� 02�����ݳ��ȣ� 00������״̬�� FA��У��ͣ� */
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
	/*��ʱ0x01,0x01����value ��ʱ���value��16λ�ģ���������ֽڣ���������ֽ�*/
  u8 command[9]={0XFF, 0XFF, num, 0X05, 0X03, AIM_POS, 0x01,0x01,0x00};
  command[6]=pos&0xFF;
  command[7]=(pos>>8)&0xFF;
  u8 checkSum = (command[2]+command[3]+command[4]+command[5]+command[6]+command[7])&0xFF;
  checkSum=~checkSum;
  command[8]=checkSum;
  
  RS485_Send_Data(command,9);
	/*���ö��ת��Ϊ���ת���ٶ�*/
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

//�������OCSָ���
void SetSteerByte(uint8_t num,uint8_t address,uint8_t value)
{
	/*��λ�ֱ���� ��0,1λ��ͷ ��2λ���ID�� ��3λ���ݳ��� ��4λָ�� ��5��6λ����    ��7λУ���*/
	unsigned char command[8]={0XFF, 0XFF, num, 0X04, 0X03, address, value, 0x00};
	unsigned char checkSum = (command[2]+command[3]+command[4]+command[5]+command[6])&0xFF;
	/*У��� Check Sum = ~ (ID + Length + Instruction + Parameter1 + ... Parameter N)*/
	checkSum=~checkSum;
	command[7]=checkSum;
	/*ReadOneByte��������Ҳ���ڶ�Ť�ؿ��ش򿪵ķ���������һ�����ڷ����Σ�һ����ReadOneByte�����Ť�ؿ��ش򿪣����Զ����while�ķ���*/
	while(ReadOneByte(num,address)!=value)
		RS485_Send_Data(command,8);
}

void ReadSteerError(int num)
{
	/*д��Զ���� RAM��ID����д��  Ϊ�˻�ȡ����״̬*/
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
	

/*���ûش�ȼ�*//*�ڴ���2�ж��ж�ȡ���OCSģʽӦ��������������ܻظ��Ǿ���while�з�����ѯ,ֱ������ظ�Ϊֹ*//*ע��485�ǰ�˫��*/
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
	/*485 �汾�Ķ��
	ͨ��д�������ڴ���Ʊ�,TORQUE_SWITCH-0x28��Ť�ؿ�����λ�ڵĵ�ַд��1�����*/
//	SetSteerByte(HOLD_BALL_1,TORQUE_SWITCH,0x01);
	/*485 �汾�Ķ��*/
	SetSteerByte(HOLD_BALL_2,TORQUE_SWITCH,0x01);
	/*ttl�汾�Ķ��*/
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
	//����flash
	SetSteerByte(0xfe,LOCK_SWITCH,0x00);
	
	unsigned char command[8]={0XFF, 0XFF, 0xfe, 0X04, 0X03, ID_AREA, num, 0x00};
	unsigned char checkSum = (command[2]+command[3]+command[4]+command[5]+command[6])&0xFF;
	checkSum=~checkSum;
	command[7]=checkSum;
	
	RS485_Send_Data(command,8);
}


/*******DEFINE*******/
/*485����ĸ��ִ���״̬*/
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
    //�ɹ�
  }
	USART_Enter();
}

/****************���һ ������ͷ������ڽ����ж�********************/

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


/****************��������ڽ����ж�****start****************/

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


