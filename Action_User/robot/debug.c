#include "task.h"
#include "debug.h"
#include "includes.h"
#include "process.h"
#include "gpio.h"
#include "dma.h"
extern Robot_t gRobot;
void debugFunction(void)
{
	
	USART_OUT_FByDEBUG(gRobot.sDta.cameraAimAngle++);
	USART_OUT_FByDEBUG(gRobot.sDta.AT_motionFlag--);
	USART_OUT_FByDEBUG(gRobot.sDta.courseAimAngle++);
	USART_OUT_FByDEBUG(gRobot.sDta.gasAimValue++);
	//USART_OUT_FByDEBUG(gRobot.sDta.holdBallAimAngle++);
	USART_OUT_FByDEBUG(gRobot.sDta.isReset);
	USART_OUT_FByDEBUG(gRobot.sDta.pitchAimAngle++);
	USART_OUT_FByDEBUG(gRobot.sDta.process++);
	USART_OUT_FByDEBUG(gRobot.sDta.robocon2018++);
	USART_Enter();
}



/*调试蓝牙中断*/
static char buffer[20];
static int bufferI=0;

void AT_DEBUG_Judge(void);
void BufferDebugInit(void);

/****************串口一接收中断****start****************/

void USART1_IRQHandler(void)
{
  static uint8_t data;
  OS_CPU_SR  cpu_sr;
  OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
  OSIntNesting++;
  OS_EXIT_CRITICAL();
  
  if(USART_GetITStatus(USART1,USART_IT_RXNE)==SET)
  {
    USART_ClearITPendingBit( USART1,USART_IT_RXNE);
    data=USART_ReceiveData(USART1);
    buffer[bufferI]=data;
    bufferI++;
    if(bufferI==20)
      bufferI=0;
    if(bufferI>1&&buffer[bufferI-1]=='\n'&&buffer[bufferI-2]=='\r'){
      AT_DEBUG_Judge();
    }else{
      if(buffer[0]!='A'){
        BufferDebugInit();
        //USART_OUT(USART1,"NOT START WITH 'A'\r\n");
      }
    }
  }else{
    data=USART_ReceiveData(USART1);
  }
  OSIntExit();
}



void AT_DEBUG_Judge(void){
	
  if((bufferI >= 9) && strncmp(buffer, "AT+report", 9)==0)//AT    
  {
    StatusReport();
  }
  else if((bufferI >= 4) && strncmp(buffer, "AT+PE", 5)==0)//AT    
  {
    USART_OUTByDMA("PE\t%d\r\n",PE_FOR_THE_BALL);
  }
  else if((bufferI >= 4) && strncmp(buffer, "AT+init",7 )==0)//AT    
  {
    PhotoelectricityInit();
    USART_OUTByDMA("OK\r\n");
  }
  else if((bufferI >= 12) && strncmp(buffer, "AT+HARDFAULT\r\n",12 )==0)
  {
		int a[2];
    USART_OUTByDMA("hardfault\r\n");
		for(int i=10000;i<80000;i++)
			a[i]=100;
		a[1]=a[0];
  }

	BufferDebugInit();
}

void BufferDebugInit(void){
  bufferI=0;
  for(int i=0;i<20;i++)
    buffer[i]=0;
}

void DebugDataUSART_OUT(void){
	if(gRobot.sDta.AT_motionFlag&AT_IS_SEND_DEBUG_DATA)
	{
		processReponse();
		USART_OUTByDMA("P ");
		USART_OUTByDMAF(gRobot.posX);
	  USART_OUTByDMAF(gRobot.posY);
		USART_OUTByDMAF(gRobot.angle);
		USART_OUTByDMA("V ");
//		USART_OUTByDMAF(gRobot.robotVel.countXVel);
//		USART_OUTByDMAF(gRobot.robotVel.countYVel);
		USART_OUTByDMAF(gRobot.robotVel.countVel);
		USART_OUTByDMA("MV ");
		USART_OUTByDMAF(gRobot.courseAngle);
		USART_OUTByDMAF(gRobot.holdBallAngle[0]);
		USART_OUTByDMAF(gRobot.holdBallAngle[1]);
		USART_OUTByDMAF(gRobot.pitchAngle);
		USART_OUTByDMAF(gRobot.gasValue);
		USART_OUTByDMAF(gRobot.gasControl);
		USART_OUTByDMA("MAV ");
		USART_OUTByDMAF(gRobot.sDta.courseAimAngle);
		USART_OUTByDMAF(gRobot.sDta.holdBallAimAngle[0]);
		USART_OUTByDMAF(gRobot.sDta.pitchAimAngle);
		USART_OUTByDMAF(gRobot.sDta.gasAimValue);
		USART_OUTByDMA("PE%d ",PE_FOR_THE_BALL);
		USART_OUTByDMA("T ");
		USART_OUTByDMAF(gRobot.raceTime.roboconTime);
//		if(gRobot.sDta.robocon2018==COLORFUL_BALL_2){
//			USART_OUTByDMAF(gRobot.raceTime.colorBall1WaitTime);
//			USART_OUTByDMAF(gRobot.raceTime.colorBall1ThrowTime);
//			USART_OUTByDMAF(gRobot.raceTime.colorBall1Time);
//			USART_OUTByDMAF(gRobot.raceTime.colorBall2WaitTime);
//		}else if(gRobot.sDta.robocon2018==GOLD_BALL){
//			USART_OUTByDMAF(gRobot.raceTime.colorBall1WaitTime);
//			USART_OUTByDMAF(gRobot.raceTime.colorBall1ThrowTime);
//			USART_OUTByDMAF(gRobot.raceTime.colorBall1Time);
//			USART_OUTByDMAF(gRobot.raceTime.colorBall2WaitTime);
//			USART_OUTByDMAF(gRobot.raceTime.colorBall2ThrowTime);
//			USART_OUTByDMAF(gRobot.raceTime.colorBall2Time);
//			USART_OUTByDMAF(gRobot.raceTime.goldBallWaitTime);
//			USART_OUTByDMAF(gRobot.raceTime.goldBallThrowTime);
//			USART_OUTByDMAF(gRobot.raceTime.goldBallTime);
//		}
	}
	
}
