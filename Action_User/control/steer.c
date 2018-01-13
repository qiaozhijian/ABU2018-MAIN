#include "steer.h"
#include "usart.h"
#include "stm32f4xx_tim.h"
#include  <includes.h>
#include "customer.h"
#include "task.h"
#include "timer.h"

extern Robot_t gRobot;
//打开舵机的扭力输出
void Enable_ROBS()
{
	USART_OUT(USART1,"#1 W 40,1,1\r\n");
	USART_OUT(USART2,"#1 W 40,1,1\r\n");
	 //#1 W 40,1,1\r\n
}

//使能伺服模式
void Enable_ServoMode()
{
	USART_OUT(USART1,"#1 W 20,1,0\r\n");
	USART_OUT(USART2,"#1 W 20,1,0\r\n");
	 //#1 W 20,1,0\r\n
}

//使能电机模式
void Enable_MotorMode()
{
	USART_OUT(USART1,"#1 W 20,1,1\r\n");
	USART_OUT(USART2,"#1 W 20,1,1\r\n");
   //#1 W 20,1,1\r\n
}



//伺服模式下函数

 //以vel的速度转到angle角度
void ROBS_PosCrl(float angleUP, float angleDOWN, int vel)
{
	static int times[2]={0,0};
	float pos,pos1;

	//Enable_ROBS();
//  pos= (2048-angle/360.0f*4096.0f/30.0f*52.0f);
  pos= ((angleUP-3.6)/360.0f*4096.0f)+2048;
  pos1= (-(angleDOWN-10.8)/360.0f*4096.0f)+2048;
	
	while(!(gRobot.AT_motionFlag&AT_STEER1_SUCCESS))
	{
		USART_OUT(USART1,"#1 W 42,2,%d:46,2,%d\r\n",(int)pos,vel);
		Delay_ms(1);
		times[0]++;
		if(times[0]>10)
		{
			USART_OUT(DEBUG_USART,"usart1 fail\r\n");
		}
	}
	SetMotionFlag(~AT_STEER1_SUCCESS);
	while(!(gRobot.AT_motionFlag&AT_STEER2_SUCCESS))
	{
		USART_OUT(USART2,"#1 W 42,2,%d:46,2,%d\r\n",(int)pos1,vel);
		Delay_ms(1);
		times[1]++;
		if(times[1]>10)
		{
			USART_OUT(DEBUG_USART,"usart2 fail\r\n");
		}
	}
	SetMotionFlag(~AT_STEER2_SUCCESS);
	times[0]=0;
	times[1]=0;
	//#1 W 42,2,pos:46,2,vel\r\nca
}
                           


//电机模式下函数


//以vel的速度转time ms,time=0代表一直转
void ROBS_VelCrl(int vel,int time)
{
	USART_OUT(USART1,"#1 W 46,2,%d:44,2,%d\r\n",vel,time);
	USART_OUT(USART2,"#1 W 46,2,%d:44,2,%d\r\n",vel,time);
	//#1 W 46,2,vel:44,2,time\r\n
}

void TurnLeft(int vel)
{
	  USART_OUT(USART1,"#1 W 46,2,%d:44,2,0\r\n",-vel);
		USART_OUT(USART2,"#1 W 46,2,%d:44,2,0\r\n",-vel);
}

void TurnRight(int vel)
{
	USART_OUT(USART1,"#1 W 46,2,%d:44,2,0\r\n",vel);
	USART_OUT(USART2,"#1 W 46,2,%d:44,2,0\r\n",vel);

}

void Stop(void)
{
	USART_OUT(USART1,"#1 W 46,2,0:44,2,0\r\n");
	USART_OUT(USART2,"#1 W 46,2,0:44,2,0\r\n");
} 

void ReadROBSAngle(void)
{
	USART_OUT(USART1,"#1 R 56,2,1\r\n");
	USART_OUT(USART2,"#1 R 56,2,1\r\n");
   //#1 W 20,1,1\r\n
}

/****************舵机一串口接收中断****start****************/

void USART1_IRQHandler(void)
{
	static uint8_t ch;
	static uint8_t step;
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
				{
					SetMotionFlag(AT_STEER1_SUCCESS);
				}
				
				step=0;
				break;
		}
		
	}else{
    USART_ReceiveData(USART1);
  }
	OSIntExit();
}

/****************舵机二串口接收中断****start****************/

void USART2_IRQHandler(void)
{
	static uint8_t ch;
	static uint8_t step;

	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if(USART_GetITStatus(USART2, USART_IT_RXNE)==SET)   
	{
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
		ch=USART_ReceiveData(USART2);
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
				{
					SetMotionFlag(AT_STEER2_SUCCESS);
				}
				
				step=0;
				break;
		}

	}else{
    USART_ReceiveData(USART2);
  }
	OSIntExit();
}




