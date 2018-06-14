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
#define BALL_PARAM_BACKUP	  8
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

extern motionPara_t PrepareShootColorBall[2];
extern motionPara_t PrepareShootGoldBall[2];


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
	else if((bufferI >= 4) && strncmp(buffer, "AT+8", 4)==0)//   
    atCommand=BALL_PARAM_BACKUP;
//  else if((bufferI >= 4) && strncmp(buffer, "AT+7", 4)==0)//   
//    atCommand=TEST_GAS;
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
	//ĳһ����������MotionStatusUpdate����ѹ����߽������
		gRobot.sDta.WhichGoldBall=0;
    if(*(buffer + 4) == '1')
    {
			shootBallFlag=1;
			ballMode=SHOOT_BALL;
			ShootSmallOpen();
			Delay_ms(300);
			gRobot.sDta.AT_motionFlag=0;
			if(*(buffer + 5) =='0'){
				WhichBall=BALL_1;
				PrepareShootBall(BALL_1);
			}else if(*(buffer + 5) =='1'){
				WhichBall=BALL_2;
				PrepareShootBall(BALL_2);
			}else if(*(buffer + 5) =='2'){
				WhichBall=BALL_1_BACKUP;
				PrepareShootBall(BALL_1_BACKUP);
			}else if(*(buffer + 5) =='3'){
				WhichBall=BALL_2_BACKUP;
				PrepareShootBall(BALL_2_BACKUP);
			}else if(*(buffer + 5) =='4'){
				WhichBall=BALL_3;
				gRobot.sDta.WhichGoldBall=WhichBall;
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
			}else if(*(buffer + 5) =='5'){
				WhichBall=BALL_4;
				gRobot.sDta.WhichGoldBall=WhichBall;
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
			}else if(*(buffer + 5) =='6'){
				WhichBall=BALL_3_BACKUP;
				gRobot.sDta.WhichGoldBall=WhichBall;
				gRobot.sDta.courseAimAngle = 179.9f;
				CourseAngleMotion(gRobot.sDta.courseAimAngle);
				while(1){
					Delay_ms(5);
					ReadActualPos(CAN2,COURCE_MOTOR_ID);
					if(fabs(gRobot.courseAngle - gRobot.sDta.courseAimAngle)<45.f){
						break;
					}
				}
				PrepareShootBall(BALL_3_BACKUP);
			}else if(*(buffer + 5) =='7'){
				WhichBall=BALL_4_BACKUP;
				gRobot.sDta.WhichGoldBall=WhichBall;
				gRobot.sDta.courseAimAngle = 179.9f;
				CourseAngleMotion(gRobot.sDta.courseAimAngle);
				while(1){
					Delay_ms(5);
					ReadActualPos(CAN2,COURCE_MOTOR_ID);
					if(fabs(gRobot.courseAngle - gRobot.sDta.courseAimAngle)<45.f){
						break;
					}
				}
				PrepareShootBall(BALL_4_BACKUP);
			}else if(*(buffer + 5) =='8'){
				WhichBall=0;
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
			
			if(*(buffer + 5) =='0'||*(buffer + 5) =='2'){
				WhichBall=BALL_1;
				if(*(buffer + 5) =='0'){
					PrepareGetBall1.gasAim=PrepareShootBall1.gasAim;
				}else {
					PrepareGetBall1.gasAim=PrepareShootColorBall[0].gasAim;
				}
				PrepareGetBall(BALL_1);
				gRobot.sDta.AT_motionFlag=0;
			}else if(*(buffer + 5) =='1'||*(buffer + 5) =='3'){
				WhichBall=BALL_2;
				if(*(buffer + 5) =='1'){
					PrepareGetBall2.gasAim=PrepareShootBall2.gasAim;
				}else {
					PrepareGetBall2.gasAim=PrepareShootColorBall[1].gasAim;
				}
				PrepareGetBall(BALL_2);
				gRobot.sDta.AT_motionFlag=0;
			}else if(*(buffer + 5) =='4'||*(buffer + 5) =='6'){
				WhichBall=BALL_3;
			  gRobot.sDta.WhichGoldBall=WhichBall;
				if(*(buffer + 5) =='4'){
					PrepareGetBall3Wait.gasAim=PrepareShootBall3.gasAim;
					PrepareGetBall3.gasAim=PrepareShootBall3.gasAim;
				}else {
					PrepareGetBall3Wait.gasAim=PrepareShootGoldBall[0].gasAim;
					PrepareGetBall3.gasAim=PrepareShootGoldBall[0].gasAim;
				}
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
			}else if(*(buffer + 5) =='5'||*(buffer + 5) =='7'){
				WhichBall=BALL_4;
				gRobot.sDta.WhichGoldBall=WhichBall;
				if(*(buffer + 5) =='5'){
					PrepareGetBall4.gasAim=PrepareShootBall4.gasAim;
				}else {
					PrepareGetBall4.gasAim=PrepareShootGoldBall[1].gasAim;
				}
				PrepareGetBall(BALL_4);
				gRobot.sDta.AT_motionFlag=0;
			}else if(*(buffer + 5) =='8'){
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
	
	//�������ѡ�������͵�ƽ��
  case BALL_PARAM_BACKUP:
		
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
			}else if(WhichBall==BALL_3||WhichBall==BALL_4||WhichBall==BALL_3_BACKUP||WhichBall==BALL_4_BACKUP){
				 //����ǰ����ʱ
			   if(WhichBall==BALL_3||WhichBall==BALL_3_BACKUP){
					  Delay_ms(250);
			   }else if(WhichBall==BALL_4||WhichBall==BALL_4_BACKUP){
            Delay_ms(300);
				 }
				 ShootBigOpen();
	       Delay_ms(50);
		     ClawOpen();
				 //�Ƹ�ִ��ʱ��
				 if(WhichBall==BALL_3||WhichBall==BALL_3_BACKUP){
					  Delay_ms(125);
			   }else if(WhichBall==BALL_4||WhichBall==BALL_4_BACKUP){
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
				
				case BALL_1_BACKUP:
					temp=&PrepareShootColorBall[0];
				break;
				
				case BALL_2_BACKUP:
					temp=&PrepareShootColorBall[1];
				break;
				
				case BALL_3_BACKUP:
					temp=&PrepareShootGoldBall[0];
				break;
				
				case BALL_4_BACKUP:
					temp=&PrepareShootGoldBall[1];
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
	//���ڵȴ�ȡ��3��ȡ��3������ѹ����Ƕȶ�һ�����ı�һ������һ��ҲҪ�ı�
	motionPara_t  * ball3ChangeOther=NULL;
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
				  ball3ChangeOther=&PrepareGetBall3Wait;
				break;
				
				case BALL_3_WAIT:
					temp=&PrepareGetBall3Wait;
				  ball3ChangeOther=&PrepareGetBall3;
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
				
			  case BALL_1_BACKUP:
					temp=&PrepareShootColorBall[0];
				break;
				
				case BALL_2_BACKUP:
					temp=&PrepareShootColorBall[1];
				break;
				
				case BALL_3_BACKUP:
					temp=&PrepareShootGoldBall[0];
				break;
				
				case BALL_4_BACKUP:
					temp=&PrepareShootGoldBall[1];
				break;
			}
		break;
	}
	
	if(WhichBall!=0){
		switch(atCommand){
			case STEER1:
				(*temp).upSteerAngle=value;
		     if(ball3ChangeOther!=NULL){
					 (*ball3ChangeOther).upSteerAngle=value;
				 }
			break;
			
			case STEER2:
				(*temp).downSteerAngle=value;
			  if(ball3ChangeOther!=NULL){
					 (*ball3ChangeOther).downSteerAngle=value;
				}
			break;
			
			case COURSE:
				(*temp).courseAngle=value;
			break;
			
			case PITCH:
				(*temp).pitchAngle=value;
			  if(ball3ChangeOther!=NULL){
					 (*ball3ChangeOther).pitchAngle=value;
				}
			break;
			
			case GAS:
				//��֤��ѹһ��
   			if(WhichBall==BALL_1){
					PrepareGetBall1.gasAim=value;
					PrepareShootBall1.gasAim=value;
				}else if(WhichBall==BALL_2){
					PrepareGetBall2.gasAim=value;
					PrepareShootBall2.gasAim=value;
				}else if(WhichBall==BALL_3){
					PrepareGetBall3Wait.gasAim=value;
					PrepareGetBall3.gasAim=value;
					PrepareShootBall3.gasAim=value;
				}else if(WhichBall==BALL_4){
					PrepareGetBall4.gasAim=value;
					PrepareShootBall4.gasAim=value;
				}else if(WhichBall==BALL_1_BACKUP){
					PrepareGetBall1.gasAim=value;
					PrepareShootColorBall[0].gasAim=value;
				}else if(WhichBall==BALL_2_BACKUP){
					PrepareGetBall2.gasAim=value;
					PrepareShootColorBall[1].gasAim=value;
				}else if(WhichBall==BALL_3_BACKUP){
					PrepareGetBall3Wait.gasAim=value;
					PrepareGetBall3.gasAim=value;
					PrepareShootGoldBall[0].gasAim=value;
				}else if(WhichBall==BALL_4_BACKUP){
					PrepareGetBall4.gasAim=value;
					PrepareShootGoldBall[1].gasAim=value;
				}
			break;
		}
		
  }
	//���������
	ball3ChangeOther=NULL;

	
	
}

