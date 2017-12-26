#include "steer.h"
#include "usart.h"
#include "stm32f4xx_tim.h"


//�򿪶����Ť�����
void Enable_ROBS()
{
	USART_OUT(USART1,"#1 W 40,1,1\r\n");
	USART_OUT(USART2,"#1 W 40,1,1\r\n");
	 //#1 W 40,1,1\r\n
}

//ʹ���ŷ�ģʽ
void Enable_ServoMode()
{
	USART_OUT(USART1,"#1 W 20,1,0\r\n");
	USART_OUT(USART2,"#1 W 20,1,0\r\n");
	 //#1 W 20,1,0\r\n
}

//ʹ�ܵ��ģʽ
void Enable_MotorMode()
{
	USART_OUT(USART1,"#1 W 20,1,1\r\n");
	USART_OUT(USART2,"#1 W 20,1,1\r\n");
   //#1 W 20,1,1\r\n
}



//�ŷ�ģʽ�º���

 //��vel���ٶ�ת��angle�Ƕ�
void ROBS_PosCrl(float angleUP, float angleDOWN, int vel)
{
	float pos,pos1;

	//Enable_ROBS();
//  pos= (2048-angle/360.0f*4096.0f/30.0f*52.0f);
  pos= ((angleUP-3.6)/360.0f*4096.0f)+2048;
  pos1= (-(angleDOWN-10.8)/360.0f*4096.0f)+2048;
	
	USART_OUT(USART1,"#1 W 42,2,%d:46,2,%d\r\n",(int)pos,vel);
	USART_OUT(USART2,"#1 W 42,2,%d:46,2,%d\r\n",(int)pos1,vel);
	//#1 W 42,2,pos:46,2,vel\r\nca
}
                           


//���ģʽ�º���


//��vel���ٶ�תtime ms,time=0����һֱת
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




