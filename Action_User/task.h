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

#define AT_STEER1_SUCCESS												0x10u

#define AT_STEER2_SUCCESS												0x20u

#define AT_STEER1_READ_SUCCESS									0x40u

#define AT_STEER2_READ_SUCCESS									0x80u

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
#define TO_THROW_BALL_3									  10
#define END_COMPETE												100

/*�������̺궨�����*/
#define ROBOT_START							0
#define COLORFUL_BALL_1					1
#define COLORFUL_BALL_2					2
#define GOLD_BALL								3

/*���ƿ�ͨ�Ž���*/
//��ʼ����
#define NOTIFY_MOTIONCARD_START						1
//֪ͨ�����Ѿ��õ���һ
#define NOTIFY_MOTIONCARD_GOT_BALL1				2
//֪ͨ���ƿ��Ѿ���������һ
#define NOTIFY_MOTIONCARD_SHOT_BALL1			3
//֪ͨ�����Ѿ��õ����
#define NOTIFY_MOTIONCARD_GOT_BALL2				4
//֪ͨ���ƿ��Ѿ����������
#define NOTIFY_MOTIONCARD_SHOT_BALL2			5
//֪ͨ�����Ѿ��õ�����
#define NOTIFY_MOTIONCARD_GOT_BALL3				6
//֪ͨ���ƿ���һ����
#define NOTIFY_MOTIONCARD_LOSE_BALL1			7
//֪ͨ���ƿ��������
#define NOTIFY_MOTIONCARD_LOSE_BALL2			8
//֪ͨ���ƿ���������
#define NOTIFY_MOTIONCARD_LOSE_BALL3			9

/*����*/
#define BALL_1														1
#define BALL_2														2
#define BALL_3														3

#define DELAY_TASK_NUM										2
/*��ʱ���е�����*/
#define DELAY_STEER1_CHECK_POS										0x01
#define DELAY_STEER2_CHECK_POS										0x02



typedef struct{
	/*�����*/
	float courseAngle;
	/*������*/
	float pitchAngle;
	/*�����*/
	float steerAngle;
	/*�����ת���ٶ�*/
	int steerSpeed;
	/*��ѹֵ*/
	float gasAim;
	
}shootPara_t;


/**************typedef area**********/
typedef struct{
	
	/*����ĳ�ʼֵ*/
	float laserInit;
	
	/*�����õ�ֵ*/
	float laser;
	
	/*���ڿ�������ִ�еĶ������*/
	uint16_t AT_motionFlag; 
	
	/*��¼��ʱ�����ĸ�����*/
	uint8_t process;
	
	/*��������*/
	uint8_t robocon2018;
	
	/*�����Ӧ�еĲ����ṹ��*/
	struct{
		
		shootPara_t PrepareGetBall1;
		
		shootPara_t PrepareGetBall2;
		
		shootPara_t PrepareGetBall3;
		
		shootPara_t PrepareShootBall1;
		
		shootPara_t PrepareShootBall2;
		
		shootPara_t PrepareShootBall3;
		
	}prepareMotion;
	
	
	struct{
		
		/*�����Ŀ��λ��*/
		int steerAimPos[2][2];
		/*�����λ��*/
		int steerPos[2];
		/*�����¼����һ���Ǵ������ͣ��ڶ����Ƿ����Ĺ���*/
		char error[10][2];
		/*�������Ĵ���*/
		char errorTime;
	}steer_t;
	
	struct{
		
		int courseAimPos;
		int coursePos;
		char courseReadSuccess;
		
		int pitchAimPos;
		int pitchPos;
		char pitchReadSuccess;
		
		float gasValue;
		float gasAimValue;
	}motorPara_t;
	
	uint16_t delayTask;
	uint16_t delayTaskMs[DELAY_TASK_NUM];
	
}Robot_t ;




#endif

