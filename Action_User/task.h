#ifndef __TASK_H
#define __TASK_H
#include "stdint.h"
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
#include "arm_math.h"
/**************#define area**********/

#define PE_FOR_THE_BALL								(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0))							


//��������
#define CAR_L													0.5389f
/*����ֱ��*/
#define WHEEL_D												0.1524f
#define SHOOT_ANGLE  (6)
#define COUNTS_PER_ROUND_REDUC  4096.f

/*���ӵĽǶȣ��Ƕ��ƣ���Ӧ�ű�������������*/
#define ANGLE_TO_CODE(angle) 																					((angle)*11.377778f)												//((angle)/360.f*COUNTS_PER_ROUND_REDUC)
/*�ű���������������Ӧ���ӵĽǶȣ��Ƕ��ƣ�*/
#define CODE_TO_ANGLE(code) 																					((code)*0.087890625f)											//((code) *360.f/COUNTS_PER_ROUND_REDUC)

/*�Ժ���ÿ����ٶ�ת��Ϊ����*/																					
#define MILLIMETER_TO_CODE(millimeter)   															((millimeter)*8.5551f)								//(ANGLE_TO_CODE(RAD_TO_ANGLE((centimeter)/1000.0/WHEEL_D*2.f)))
/*����ת��Ϊ�Ժ���ÿ����ٶ�*/
#define CODE_TO_MILLIMETER(code) 			  															((code)*0.11689f)												//(ANGLE_TO_RAD(CODE_TO_ANGLE((code)))*WHEEL_D/2.f*1000.0)

/*��ת���ٶȣ��Ƕ��ƣ���Ҫ��������*///float��ȷ�Ȼ᲻��С��ʹ�ó��ĵ�����С��ʧ�棿
#define ROTATE_W_TO_CODE(rotateW)																			((rotateW)*89.58883f)										//MILLIMETER_TO_CODE((ANGLE_TO_RAD((rotateW))*CAR_L*1000))
/*������ת��Ϊ��Ӧ����ת���ٶȣ��Ƕ��ƣ�*/
#define CODE_TO_ROTATE_W(code)																				((code)*0.0111621f)										


/*�Ƕ���ת��Ϊ������*/
#define ANGLE_TO_RAD(angle) 																					((angle)*0.0174533f)
/*������ת��Ϊ�Ƕ���*/
#define RAD_TO_ANGLE(rad) 																						((rad)*57.29578f)

//״̬������
#define AT_CLAW_STATUS_OPEN 										0x01u

#define AT_STEER_READY 													0x02u

#define AT_SHOOT_BIG_ENABLE 										0x04u

#define AT_SHOOT_SMALL_ENABLE 			  					0x08u

//״̬������
#define CAN_CLAW_STATUS_OPEN 										0x01u

#define CAN_STEER_READY 												0x02u

#define CAN_SHOOT_BIG_ENABLE 										0x04u

#define CAN_SHOOT_SMALL_ENABLE 			  					0x08u


#define CLAW_OPEN 									  					(1)
#define CLAW_SHUT 									 			 			(0)


/*״̬������*/
#define TO_START													1
#define TO_GET_BALL_1											2
#define TO_THE_AREA_1											3
#define TO_THROW_BALL_1										4
#define TO_GET_BALL_2											5
#define TO_THE_AREA_2											6
#define TO_THROW_BALL_2										7
#define TO_GET_BALL_3											8
#define TO_THE_AREA_3											9
#define TO_THROW_BALL_3										10




/**************typedef area**********/
typedef struct{
	
	float laserInit;
	float laser;
	
	uint16_t AT_motionFlag; 
	
	uint8_t process;
}Robot_t ;

#endif

