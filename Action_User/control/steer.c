#include "steer.h"
#include "usart.h"
#include "stm32f4xx_tim.h"
#include  <includes.h>
#include "customer.h"
#include "task.h"
#include "timer.h"
#include "process.h"

extern Robot_t gRobot;
//打开舵机的扭力输出
void Enable_ROBS()
{
	static int times[2]={0,0};
	while(!(gRobot.AT_motionFlag&AT_STEER1_SUCCESS))
	{
		USART_OUT(USART1,"#1 W 40,1,1\r\n");
		Delay_ms(1);
		times[0]++;
		if(times[0]>10)
		{
			USART_OUT(DEBUG_USART,"STEER1_ENABLE_FAIL\r\n");
			SteerErrorRecord(STEER1_ENABLE_FAIL);
		}
	}
	SetMotionFlag(~AT_STEER1_SUCCESS);
	while(!(gRobot.AT_motionFlag&AT_STEER2_SUCCESS))
	{
		USART_OUT(USART2,"#1 W 40,1,1\r\n");
		Delay_ms(1);
		times[1]++;
		if(times[1]>10)
		{
			USART_OUT(DEBUG_USART,"STEER2_ENABLE_FAIL\r\n");
			SteerErrorRecord(STEER2_ENABLE_FAIL);
		}
	}
	SetMotionFlag(~AT_STEER2_SUCCESS);
	times[0]=0;
	times[1]=0;
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
	Steer1ROBS_PosCrl(angleUP, vel);
	Steer2ROBS_PosCrl(angleDOWN, vel);
}

void Steer1ROBS_PosCrl(float angleUP, int vel)
{
	static int times=0;
	float pos;
	
  pos= ((angleUP-3.6)/360.0f*4096.0f)+2048;
	
	gRobot.steer_t.steerAimPos[0][0]=angleUP;
	gRobot.steer_t.steerAimPos[0][1]=pos;
	
	while(!(gRobot.AT_motionFlag&AT_STEER1_SUCCESS))
	{
		USART_OUT(USART1,"#1 W 42,2,%d:46,2,%d\r\n",(int)pos,vel);
		Delay_ms(1);
		times++;
		if(times>10)
		{
			USART_OUT(DEBUG_USART,"STEER1_ROTATE_SEND_FAIL\r\n");
			SteerErrorRecord(STEER1_ROTATE_SEND_FAIL);
		}
	}
	SetMotionFlag(~AT_STEER1_SUCCESS);
	DelayMsRun(DELAY_STEER1_CHECK_POS,1000);
	
	times=0;
}

void Steer2ROBS_PosCrl(float angleDOWN, int vel)
{
	static int times=0;
	float pos1;

	//Enable_ROBS();
//  pos= (2048-angle/360.0f*4096.0f/30.0f*52.0f);
  pos1= (-(angleDOWN-10.8)/360.0f*4096.0f)+2048;
	
	gRobot.steer_t.steerAimPos[1][0]=angleDOWN;
	gRobot.steer_t.steerAimPos[1][1]=pos1;
	
	while(!(gRobot.AT_motionFlag&AT_STEER2_SUCCESS))
	{
		USART_OUT(USART2,"#1 W 42,2,%d:46,2,%d\r\n",(int)pos1,vel);
		Delay_ms(1);
		times++;
		if(times>10)
		{
			USART_OUT(DEBUG_USART,"STEER2_ROTATE_SEND_FAIL\r\n");
			SteerErrorRecord(STEER2_ROTATE_SEND_FAIL);
		}
	}
	ReadROBSAngle();
	SetMotionFlag(~AT_STEER2_SUCCESS);
	DelayMsRun(DELAY_STEER2_CHECK_POS,1000);
	times=0;
}

//电机模式下函数
void Steer1ROBS_PosTimeCrl(float angleUP, int time);
void Steer2ROBS_PosTimeCrl(float angleUP, int time);

//以vel的速度转time ms,time=0代表一直转
void ROBS_PosTimeCrl(float angleUP, float angleDOWN, int time)
{
	Steer1ROBS_PosTimeCrl(angleUP, time);
	Steer2ROBS_PosTimeCrl(angleUP, time);
	//#1 W 46,2,vel:44,2,time\r\n
}

void Steer1ROBS_PosTimeCrl(float angleUP, int time)
{
	static int times=0;
	float pos;
	
  pos= ((angleUP-3.6)/360.0f*4096.0f)+2048;
	
	gRobot.steer_t.steerAimPos[0][0]=angleUP;
	gRobot.steer_t.steerAimPos[0][1]=pos;
	
//	while(!(gRobot.AT_motionFlag&AT_STEER1_SUCCESS))
//	{
		USART_OUT(USART1,"#1 W 42,2,%d:44,2,%d\r\n",pos,time);
//		Delay_ms(1);
//		times++;
//		if(times>10)
//		{
//			USART_OUT(DEBUG_USART,"STEER1_ROTATE_SEND_FAIL\r\n");
//			SteerErrorRecord(STEER1_ROTATE_SEND_FAIL);
//		}
//	}
//	SetMotionFlag(~AT_STEER1_SUCCESS);
//	DelayMsRun(DELAY_STEER1_CHECK_POS,1000);
//	
//	times=0;
}

void Steer2ROBS_PosTimeCrl(float angleDOWN, int time)
{
	static int times=0;
	float pos1;

	//Enable_ROBS();
//  pos= (2048-angle/360.0f*4096.0f/30.0f*52.0f);
  pos1= (-(angleDOWN-10.8)/360.0f*4096.0f)+2048;
	
	gRobot.steer_t.steerAimPos[1][0]=angleDOWN;
	gRobot.steer_t.steerAimPos[1][1]=pos1;
	
//	while(!(gRobot.AT_motionFlag&AT_STEER2_SUCCESS))
//	{
		USART_OUT(USART2,"#1 W 42,2,%d:44,2,%d\r\n",pos1,time);
//		Delay_ms(1);
//		times++;
//		if(times>10)
//		{
//			USART_OUT(DEBUG_USART,"STEER2_ROTATE_SEND_FAIL\r\n");
//			SteerErrorRecord(STEER2_ROTATE_SEND_FAIL);
//		}
//	}
//	ReadROBSAngle();
//	SetMotionFlag(~AT_STEER2_SUCCESS);
//	DelayMsRun(DELAY_STEER2_CHECK_POS,1000);
//	times=0;
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

//读位置
void ReadROBSAngle(void)
{
	static int times1=0;
	static int times2=0;
	
	//如果没有读成功
	while(!(gRobot.AT_motionFlag&AT_STEER1_READ_SUCCESS))
	{
		USART_OUT(USART1,"#1 R 56,2,1\r\n");
		Delay_ms(1);
		times1++;
		if(times1>10)
		{
			USART_OUT(DEBUG_USART,"AT_STEER1_READ_FAIL\r\n");
		}
	}
	//复位
	SetMotionFlag(~AT_STEER1_READ_SUCCESS);
	times1=0;
	
	//如果没有读成功
	while(!(gRobot.AT_motionFlag&AT_STEER2_READ_SUCCESS))
	{
		USART_OUT(USART2,"#1 R 56,2,1\r\n");
		Delay_ms(1);
		times2++;
		if(times2>10)
		{
			USART_OUT(DEBUG_USART,"AT_STEER2_READ_FAIL\r\n");
		}
	}
	//复位
	SetMotionFlag(~AT_STEER2_READ_SUCCESS);
	times2=0;
}

void SteerErrorReport(void)
{
	for(int i=0;i<gRobot.steer_t.errorTime;i++)
	{
		switch(gRobot.steer_t.error[i][0])
		{
			case STEER1_ENABLE_FAIL:
				USART_OUT(DEBUG_USART,"STEER1_ENABLE_FAIL\t");
				break;
			case STEER1_ROTATE_FAIL:
				USART_OUT(DEBUG_USART,"STEER1_ROTATE_FAIL\t");
				break;
			case STEER2_ENABLE_FAIL:
				USART_OUT(DEBUG_USART,"STEER2_ENABLE_FAIL\t");
				break;
			case STEER2_ROTATE_FAIL:
				USART_OUT(DEBUG_USART,"STEER2_ROTATE_FAIL\t");
				break;
			case STEER1_ROTATE_SEND_FAIL:
				USART_OUT(DEBUG_USART,"STEER1_ROTATE_SEND_FAIL\t");
				break;
			case STEER2_ROTATE_SEND_FAIL:
				USART_OUT(DEBUG_USART,"STEER2_ROTATE_SEND_FAIL\t");
				break;
		}
		switch(gRobot.steer_t.error[i][1])
		{
			case TO_START:
				USART_OUT(DEBUG_USART,"TO_START");
				USART_Enter();
				break;
			case TO_GET_BALL_1:
				USART_OUT(DEBUG_USART,"TO_GET_BALL_1");
				USART_Enter();
				break;
			case TO_THE_AREA_1:
				USART_OUT(DEBUG_USART,"TO_THE_AREA_1");
				USART_Enter();
				break;
			case TO_THROW_BALL_1:
				USART_OUT(DEBUG_USART,"TO_THROW_BALL_1");
				USART_Enter();
				break;
			case TO_GET_BALL_2:
				USART_OUT(DEBUG_USART,"TO_GET_BALL_2");
				USART_Enter();
				break;
			case TO_THE_AREA_2:
				USART_OUT(DEBUG_USART,"TO_THE_AREA_2");
				USART_Enter();
				break;
			case TO_THROW_BALL_2:
				USART_OUT(DEBUG_USART,"TO_THROW_BALL_2");
				USART_Enter();
				break;
			case TO_GET_BALL_3:
				USART_OUT(DEBUG_USART,"TO_GET_BALL_3");
				USART_Enter();
				break;
			case TO_THE_AREA_3:
				USART_OUT(DEBUG_USART,"TO_THE_AREA_3");
				USART_Enter();
				break;
			case TO_THROW_BALL_3:
				USART_OUT(DEBUG_USART,"TO_THROW_BALL_3");
				USART_Enter();
				break;
			case END_COMPETE:
				USART_OUT(DEBUG_USART,"END_COMPETE");
				USART_Enter();
				break;
		}
	}
}

void SteerErrorRecord(char type)
{
	gRobot.steer_t.error[gRobot.steer_t.errorTime][0]=type;
	gRobot.steer_t.error[gRobot.steer_t.errorTime][1]=gRobot.process;
	gRobot.steer_t.errorTime++;
}

/****************舵机一串口接收中断****start****************/

void USART1_IRQHandler(void)
{
	static uint8_t ch;
	static uint8_t step;
	static char buffer[4]={0};
	static int i=0;
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if(USART_GetITStatus(USART1, USART_IT_RXNE)==SET)   
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		ch=USART_ReceiveData(USART1);
		USART_SendData(DEBUG_USART,ch);
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
					SetMotionFlag(AT_STEER1_SUCCESS);
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
					gRobot.steer_t.steerPos[0]=0;
					for(int j=0;j<i;j++)
					{
						gRobot.steer_t.steerPos[0]+=(buffer[j]-'0')*pow(10,i-j-1);
					}
					if(gRobot.steer_t.steerPos[0]>4095||gRobot.steer_t.steerPos[0]<0)
					{
						USART_OUT(DEBUG_USART,"steer1readbackvalueout\r\n");
						step=0;
					}
					i=100;
				}
				if(i<4){
					buffer[i]=ch;
					i++;
				}else
					i=0;
				break;
			case 14:
				if(ch=='\n')
				{
					SetMotionFlag(AT_STEER1_READ_SUCCESS);
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

void USART2_IRQHandler(void)
{
	static uint8_t ch;
	static uint8_t step;
	static char buffer[4]={0};
	static int i=0;

	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if(USART_GetITStatus(USART2, USART_IT_RXNE)==SET)   
	{
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
		ch=USART_ReceiveData(USART2);
		USART_SendData(DEBUG_USART,ch);
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
					SetMotionFlag(AT_STEER2_SUCCESS);
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
					gRobot.steer_t.steerPos[1]=0;
					for(int j=0;j<i;j++)
					{
						gRobot.steer_t.steerPos[1]+=(buffer[j]-'0')*pow(10,i-j-1);
					}
					if(gRobot.steer_t.steerPos[1]>4095||gRobot.steer_t.steerPos[1]<0)
					{
						USART_OUT(DEBUG_USART,"steer2readbackvalueout\r\n");
						step=0;
					}
					i=100;
				}
				if(i<4){
					buffer[i]=ch;
					i++;
				}else
					i=0;
				break;
			case 14:
				if(ch=='\n')
				{
					SetMotionFlag(AT_STEER2_READ_SUCCESS);
					step=0;
				}else
					step=0;
				break;
		}

	}else{
    USART_ReceiveData(USART2);
  }
	OSIntExit();
}




