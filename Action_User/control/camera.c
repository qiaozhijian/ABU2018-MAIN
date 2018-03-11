#include "camera.h"
#include "os_cpu.h"
#include "stdint.h"
#include "ucos_ii.h"
#include "stm32f4xx_usart.h"
#include "customer.h"
#include <stdio.h>
#include <string.h>
#include "robot.h"
#include "task.h"
#include "iwdg.h"
#include "timer.h"
static char buffer[20];
static int bufferI=0;

void AT_CAMEARA_CMD_Judge(void);
void CameraBufferInit(void);

extern Robot_t gRobot;
/*����ͷ���������ж�*/
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
      AT_CAMEARA_CMD_Judge();
    }else{
      if(buffer[0]!='A'){
        CameraBufferInit();
        //USART_OUT(USART6,"NOT START WITH 'A'\r\n");
      }
    }
  }else{
    data=USART_ReceiveData(USART6);
  }
  OSIntExit();
}

//��buffer���ݽ������
void CameraBufferInit(void){
  bufferI=0;
  for(int i=0;i<20;i++)
    buffer[i]=0;
}

/* ԭ��ΪAT_CAMEARA_CMD_Judge() */
void AT_CAMEARA_CMD_Judge(void){
	
  if((bufferI == 4) && strncmp(buffer, "AT\r\n",4 )==0)//AT    
  {
		
		SetMotionFlag(AT_CAMERA_TALK_SUCCESS);
    //����ͷ���ӳɹ�
  }
}


void TalkToCamera(uint32_t command)
{
	int times=0;
	switch(command)
	{
		case CAMERA_START:
			/*���������ͷͨ�ű�־λû����һ��500us��һ������*/
			while(!(gRobot.sDta.AT_motionFlag&AT_CAMERA_TALK_SUCCESS))
			{
				USART_OUT(CAMERA_USART,"AT\r\n");
				Delay_ms(3);
				times++;
				IWDG_Feed();
//				if(times>100)
//				{
////					USART_OUT(DEBUG_USART,"Camera dead\r\n");
////					break;
//				}
			}
			times=0;
			/*��ձ�־λ*/
			SetMotionFlag(~AT_CAMERA_TALK_SUCCESS);
			break;
		case CAMERA_SHUT_ALL:
			/*���������ͷͨ�ű�־λû����һ��500us��һ������*/
			while(!(gRobot.sDta.AT_motionFlag&AT_CAMERA_TALK_SUCCESS))
			{
				USART_OUT(CAMERA_USART,"AT+%d\r\n",CAMERA_SHUT_ALL);
				Delay_ms(3);
				times++;
				IWDG_Feed();
				if(times>100)
				{
//					USART_OUT(DEBUG_USART,"Camera dead\r\n");
//					break;
				}
			}
			times=0;
			/*��ձ�־λ*/
			SetMotionFlag(~AT_CAMERA_TALK_SUCCESS);
			break;
		case CAMERA_OPEN_NEAR:
			/*���������ͷͨ�ű�־λû����һ��500us��һ������*/
			while(!(gRobot.sDta.AT_motionFlag&AT_CAMERA_TALK_SUCCESS))
			{
				USART_OUT(CAMERA_USART,"AT+%d\r\n",CAMERA_OPEN_NEAR);
				Delay_ms(3);
				times++;
				IWDG_Feed();
				if(times>100)
				{
//					USART_OUT(DEBUG_USART,"Camera dead\r\n");
//					break;
				}
			}
			times=0;
			/*��ձ�־λ*/
			SetMotionFlag(~AT_CAMERA_TALK_SUCCESS);
			break;
		case CAMERA_OPEN_FAR:
			/*���������ͷͨ�ű�־λû����һ��500us��һ������*/
			while(!(gRobot.sDta.AT_motionFlag&AT_CAMERA_TALK_SUCCESS))
			{
				USART_OUT(CAMERA_USART,"AT+%d\r\n",CAMERA_OPEN_FAR);
				Delay_ms(3);
				times++;
				IWDG_Feed();
				if(times>100)
				{
//					USART_OUT(DEBUG_USART,"Camera dead\r\n");
//					break;
				}
			}
			times=0;
			/*��ձ�־λ*/
			SetMotionFlag(~AT_CAMERA_TALK_SUCCESS);
			break;
	}
}



