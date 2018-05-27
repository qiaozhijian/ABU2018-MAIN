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
#define WHEEL 					12
#define STAIR2 					13
#define EXTEND_THE_CAR 	14
#define GOLDEN_COURCE_SPEED 77
#define COLOR_COURCE_SPEED  78


//ƽ�������Ķ���ballMode
#define PICK_BALL       1
#define SHOOT_BALL      2
extern Robot_t gRobot;
extern motionPara_t PrepareGetBall1;
extern motionPara_t PrepareGetBall2;
extern motionPara_t PrepareGetBall3Wait;
extern motionPara_t PrepareGetBall3;
extern motionPara_t PrepareGetBall4;
extern motionPara_t PrepareShootBall1;
extern motionPara_t PrepareShootBall2;
extern motionPara_t PrepareShootBall3;
extern motionPara_t PrepareShootBall4;

/*���������ж�*/
static char buffer[20];
static int bufferI=0;
static int atCommand=0;

static int getGoldBallStep=0;
/*��һ����*/
static int WhichBall=3;
/*��ǰ��Ҫ�޸�д��Ĳ�������Ķ���*/
static int ballMode=0;
/*�����־λ*/
static int shootBallFlag=0;

void AT_CMD_Judge(void);
void ChangeParamTemp(float value);	
void BufferInit(void){
  bufferI=0;
	atCommand=0;
  for(int i=0;i<20;i++)
    buffer[i]=0;
}

/*ƽ��������������ж�*/
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
  else if((bufferI >= 4) && strncmp(buffer, "AT+4", 4)==0)//���䰴ť   
    atCommand=STEER;
  else if((bufferI >= 4) && strncmp(buffer, "AT+5", 4)==0)//���䰴ť   
    atCommand=GAS;
  else if((bufferI >= 4) && strncmp(buffer, "AT+6", 4)==0)//   
    atCommand=COURSE;
  else if((bufferI >= 4) && strncmp(buffer, "AT+9", 4)==0)//  
    atCommand=STEER1;
  else if((bufferI >= 5) && strncmp(buffer, "AT+12", 5)==0)//  
    atCommand=STEER2;
  else if((bufferI >= 5) && strncmp(buffer, "AT+13", 5)==0)//  
    atCommand=BOOST;
  else if((bufferI >= 5) && strncmp(buffer, "AT+14", 5)==0)//  
    atCommand=WHEEL;
  else if((bufferI >= 5) && strncmp(buffer, "AT+15", 5)==0)//   
    atCommand=STAIR2;
	else if((bufferI >= 5) && strncmp(buffer, "AT+16", 5)==0)//   
    atCommand=EXTEND_THE_CAR;
	else if((bufferI >= 5) && strncmp(buffer, "AT+77", 5)==0)//   
    atCommand=GOLDEN_COURCE_SPEED;
	else if((bufferI >= 5) && strncmp(buffer, "AT+78", 5)==0)//   
    atCommand=COLOR_COURCE_SPEED;
//  else if((bufferI >= 4) && strncmp(buffer, "AT+7", 4)==0)//   
//    atCommand=TEST_GAS;
//  else if((bufferI >= 4) && strncmp(buffer, "AT+8", 4)==0)//   
//    atCommand=CAMERA;
//  if((bufferI == 4) && strncmp(buffer, "AT\r\n",4 )==0)//AT    
//  {
//		
//		SetMotionFlag(AT_CAMERA_TALK_SUCCESS);
//    //����ͷ���ӳɹ�
//  }
	
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
    
    /*�����Ƿ����*/
  case SHOOT:
		
    USART_OUTByDMA("OK\r\n");
    if(*(buffer + 4) == '1')
    {
			shootBallFlag=1;
			ballMode=SHOOT_BALL;
			ShootSmallOpen();
			Delay_ms(300);
			if(*(buffer + 5) =='0'){
				WhichBall=BALL_1;
				PrepareShootBall(BALL_1);
//				LedBallInto();
				gRobot.sDta.AT_motionFlag=0;
			}else if(*(buffer + 5) =='1'){
				WhichBall=BALL_2;
				PrepareShootBall(BALL_2);
				LedBallInto();
				gRobot.sDta.AT_motionFlag=0;
			}else if(*(buffer + 5) =='2'){
				WhichBall=BALL_3;
				gRobot.sDta.courseAimAngle = 179.9f;
				CourseAngleMotion(gRobot.sDta.courseAimAngle);
				while(1){
					Delay_ms(5);
					ReadActualPos(CAN2,COURCE_MOTOR_ID);
					if(fabs(gRobot.courseAngle - gRobot.sDta.courseAimAngle)<45.f){
						break;
					}
				}
				PrepareShootBall(BALL_3);
				gRobot.sDta.AT_motionFlag=0;
			}else if(*(buffer + 5) =='3'){
				WhichBall=BALL_4;
				gRobot.sDta.courseAimAngle = 179.9f;
				CourseAngleMotion(gRobot.sDta.courseAimAngle);
				while(1){
					Delay_ms(5);
					ReadActualPos(CAN2,COURCE_MOTOR_ID);
					if(fabs(gRobot.courseAngle - gRobot.sDta.courseAimAngle)<45.f){
						break;
					}
				}
				PrepareShootBall(BALL_4);
				gRobot.sDta.AT_motionFlag=0;
			}else if(*(buffer + 5) =='4'){
				WhichBall=0;
				gRobot.sDta.AT_motionFlag=0;
				if(PE_FOR_THE_BALL){
					ClawOpen();
					Delay_ms(50);
					ShootBigOpen();
				}
			}
			
    }
    else if(*(buffer + 4) == '0') 
    {
			ballMode=PICK_BALL;
      if(gRobot.sDta.AT_motionFlag&(AT_SHOOT_BIG_ENABLE|AT_SHOOT_SMALL_ENABLE))
      {
        ShootSmallShut();
				ShootBigShut();
				ClawShut();
				Delay_ms(500);
      }
			
			if(*(buffer + 5) =='0'){
				WhichBall=BALL_1;
				PrepareGetBall(BALL_1);
				gRobot.sDta.AT_motionFlag=0;
			}else if(*(buffer + 5) =='1'){
				WhichBall=BALL_2;
				PrepareGetBall(BALL_2);
				gRobot.sDta.AT_motionFlag=0;
			}else if(*(buffer + 5) =='2'){
				WhichBall=BALL_3;
				if(getGoldBallStep==0){
					WhichBall=BALL_3_WAIT;
					PrepareGetBall(BALL_3_WAIT);
					getGoldBallStep=1;
				}else if(getGoldBallStep==1){
					WhichBall=BALL_3;
					PrepareGetBall(BALL_3);
					getGoldBallStep=0;
				}
				gRobot.sDta.AT_motionFlag=0;
			}else if(*(buffer + 5) =='3'){
				WhichBall=BALL_4;
				PrepareGetBall(BALL_4);
				gRobot.sDta.AT_motionFlag=0;
			}else if(*(buffer + 5) =='4'){
				WhichBall=0;
				gRobot.sDta.AT_motionFlag=0;
				ShootSmallShut();
				ShootBigShut();
				ClawShut();
				Delay_ms(500);
			}
    }
		
		SendParamToUpPositionMachine();
    break;
    
  case GAS:
    USART_OUTByDMA("OK\r\n");
    //ƽ���ֵ
    value = atof(buffer + 4);
  	GasMotion(value);
		ChangeParamTemp(value);
  	GasEnable();

    break;
    
  case PITCH:
    USART_OUTByDMA("OK\r\n");
    value = atof(buffer + 4);
		gRobot.sDta.pitchAimAngle=value;
		ChangeParamTemp(value);
		PitchAngleMotion(gRobot.sDta.pitchAimAngle);
    break;
    
  case COURSE:
    USART_OUTByDMA("OK\r\n");
    value = atof(buffer + 4);
		gRobot.sDta.courseAimAngle=value;
		ChangeParamTemp(value);
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
		ChangeParamTemp(value);
		HoldSteer1PosCrl(gRobot.sDta.holdBallAimAngle[0]);
		break;
		
	case STEER2:
    USART_OUTByDMA("OK\r\n");
    value = atof(buffer + 5);
		gRobot.sDta.holdBallAimAngle[1]=value;
		ChangeParamTemp(value);
		HoldSteer2PosCrl(gRobot.sDta.holdBallAimAngle[1]);
		break;
  
  case WHEEL:
    USART_OUTByDMA("OK\r\n");
    if(*(buffer + 5) == '1') 
    {
      MotionCardCMDSend(NOTIFY_MOTIONCARD_ENABLE_WHEEL);
    }
    else if(*(buffer + 5) == '0') 
    {
      MotionCardCMDSend(NOTIFY_MOTIONCARD_DISABLE_WHEEL);
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
		
	case EXTEND_THE_CAR:
    USART_OUTByDMA("OK\r\n");
    if(*(buffer + 5) == '1') 
    {
      ExtendCarOn();
    }
    else if(*(buffer + 5) == '0') 
    {
      ExtendCarOff();
    } 
   break;
		
	case COLOR_COURCE_SPEED:
		  PosLoopCfg(CAN2, COURCE_MOTOR_ID, 8000000, 8000000,12500000);
	break;
	
	case GOLDEN_COURCE_SPEED:
		  PosLoopCfg(CAN2, COURCE_MOTOR_ID, 8000000, 8000000,6250000);
	break;
	
  default:
    break;
  }
  
  BufferInit();
}

void TestFightForBall(void){

	/*ģ������*/
	if(WhichBall!=0&&ballMode==SHOOT_BALL&&shootBallFlag){
		if(PE_FOR_THE_BALL
				/*��������λ*/
				&&(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_1_SUCCESS)
					/*��������λ*/
	  			&&(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_2_SUCCESS)
						/*������λ��*/
						&&(gRobot.sDta.AT_motionFlag&AT_PITCH_SUCCESS)
							/*����λ*/
							&&(gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS)
									/*��ѹ��λ*/
									&&(gRobot.sDta.AT_motionFlag&AT_GAS_SUCCESS))
    {
			if(WhichBall==BALL_1||WhichBall==BALL_2){
				 ShootBall();
				//�Ƹ�ִ��ʱ��
				 Delay_ms(125);
			}else if(WhichBall==BALL_3||WhichBall==BALL_4){
				 //����ǰ����ʱ
			   if(WhichBall==BALL_3){
//						ClawOpen();
//				    Delay_ms(50);
//						ClawShut();
//						Delay_ms(250);
					  Delay_ms(300);
			   }else if(WhichBall==BALL_4){
            Delay_ms(300);
				 }
				 ShootBigOpen();
	       Delay_ms(50);
		     ClawOpen();
				 //�Ƹ�ִ��ʱ��
				 if(WhichBall==BALL_3){
					  Delay_ms(125);
			   }else if(WhichBall==BALL_4){
            Delay_ms(300);
				 }
			}
			
      /*���������λ*/
      ShootReset();
			/*��־λ��λ0��ֹ�ظ�����*/
			shootBallFlag=0;
		}
	}
	
	
	
}
//������������λ��
void SendParamToUpPositionMachine(void){
	motionPara_t  * temp;
	char paramWhich[]="PG";
	
	//��ʱ��ż�����ɵ���ģʽ
	if(WhichBall==0){
		return;
	}
	switch(ballMode){
		case PICK_BALL:
			strcpy(paramWhich,"PG");
			switch(WhichBall){
				case BALL_1:
				  temp=&PrepareGetBall1;
				break;
				
				case BALL_2:
					temp=&PrepareGetBall2;
				break;
				
				case BALL_3:
					temp=&PrepareGetBall3;
				break;
				
				case BALL_3_WAIT:
					temp=&PrepareGetBall3Wait;
				break;
				
				case BALL_4:
					temp=&PrepareGetBall4;
				break;
			}
		break;
		
		case SHOOT_BALL:
			strcpy(paramWhich,"PS");
			switch(WhichBall){
				case BALL_1:
					temp=&PrepareShootBall1;
				break;
				
				case BALL_2:
					temp=&PrepareShootBall2;
				break;
				
				case BALL_3:
					temp=&PrepareShootBall3;
				break;
				
				case BALL_4:
					temp=&PrepareShootBall4;
				break;
			}
		break;
	}
	
	USART_OUT(CONTROL_USART,"%s",paramWhich);
	USART_OUT_F(CONTROL_USART,(*temp).courseAngle);
	USART_OUT_F(CONTROL_USART,(*temp).pitchAngle);
	USART_OUT_F(CONTROL_USART,(*temp).upSteerAngle);
	USART_OUT_F(CONTROL_USART,(*temp).downSteerAngle);
	USART_OUT_F(CONTROL_USART,(*temp).gasAim);
	USART_OUT(CONTROL_USART,"\r\n");
	
	
	
	
}
void ChangeParamTemp(float value){
	motionPara_t  * temp;
	
	switch(ballMode){
		case PICK_BALL:
			switch(WhichBall){
				case BALL_1:
				  temp=&PrepareGetBall1;
				break;
				
				case BALL_2:
					temp=&PrepareGetBall2;
				break;
				
				case BALL_3:
					temp=&PrepareGetBall3;
				break;
				
				case BALL_3_WAIT:
					temp=&PrepareGetBall3Wait;
				break;
				
				case BALL_4:
					temp=&PrepareGetBall4;
				break;
			}
		break;
		
		case SHOOT_BALL:
			switch(WhichBall){
				case BALL_1:
					temp=&PrepareShootBall1;
				break;
				
				case BALL_2:
					temp=&PrepareShootBall2;
				break;
				
				case BALL_3:
					temp=&PrepareShootBall3;
				break;
				
				case BALL_4:
					temp=&PrepareShootBall4;
				break;
			}
		break;
	}
	
	if(WhichBall!=0){
		switch(atCommand){
			case STEER1:
				(*temp).upSteerAngle=value;
			break;
			
			case STEER2:
				(*temp).downSteerAngle=value;
			break;
			
			case COURSE:
				(*temp).courseAngle=value;
			break;
			
			case PITCH:
				(*temp).pitchAngle=value;
			break;
			
			case GAS:
				(*temp).gasAim=value;
			break;
		}
		
  }
	
	
}

