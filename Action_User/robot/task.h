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
/**************#define area**********/

//#define TEST
#define DEBUG

//��������
//��λϵͳλ���Զ���ˮƽ������м䣬��ֱ�����ǽmm505.f
//����ͷλ���Զ���ƫ��21.85mm����ǽ172.44mm����ĵط�
//��ά����Ӿ�ǽ505mm
#define CAMERA_TO_GYRO_X							21.85f
#define CAMERA_TO_GYRO_Y							332.56f
#define QUICK_MARK_X_1								5505.f
#define QUICK_MARK_X_2								6505.f
#define QUICK_MARK_Y									800.f


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

/*3508���ٱ�3591/187=19.203208  8192λ������*/
#define PITCH_CODE_TO_ANGLE(code)													((code)/8738.1333f)
#define PITCH_ANGLE_TO_CODE(angle)												((int)((angle)*8738.1333f))//((angle)*20.0f*19.2f*8192.f/360.f)

#define COURSE_CODE_TO_ANGLE(code)												((code)/6116.693f)
#define COURSE_ANGLE_TO_CODE(angle)												((int)(((angle))*6116.693f))//((angle-5.6f)*14.0f*19.2f*8192.f/360.f)


//״̬������
#define AT_CLAW_STATUS_OPEN 										0x01u

#define AT_STEER_READY 													0x02u

#define AT_SHOOT_BIG_ENABLE 										0x04u

#define AT_SHOOT_SMALL_ENABLE 			  					0x08u

#define AT_HOLD_BALL_1_SUCCESS										0x10u

#define AT_HOLD_BALL_2_SUCCESS										0x20u

#define AT_HOLD_BALL_1_RESPONSE_SUCCESS							0x40u

#define AT_HOLD_BALL_2_RESPONSE_SUCCESS							0x80u

#define AT_COURSE_READ_SUCCESS									0x200u

#define AT_PITCH_READ_SUCCESS										0x400u

#define AT_COURSE_SUCCESS												0x800u

#define AT_PITCH_SUCCESS												0x1000u

#define AT_CAMERA_RESPONSE_SUCCESS								0x2000u

#define AT_CAMERA_TALK_SUCCESS									0x100u

#define AT_GAS_SUCCESS													0x4000u

#define AT_PREPARE_READY													0x8000u

#define AT_IS_SEND_DEBUG_DATA													0x10000u


//״̬������
#define CAN_CLAW_STATUS_OPEN 										0x01u

#define CAN_STEER_READY 												0x02u

#define CAN_SHOOT_BIG_ENABLE 										0x04u

#define CAN_SHOOT_SMALL_ENABLE 			  					0x08u


/**���������**/
//��ʼ��ʱ���ʹ��������
#define HOLD_BALL_1_ENABLE_FAIL														1
#define HOLD_BALL_2_ENABLE_FAIL														2
//�������ץ��ʱ���û��ת����λ
#define HOLD_BALL_1_ROTATE_FAIL														3
#define HOLD_BALL_2_ROTATE_FAIL														4
//�������ץ��ʱת̨û��ת����λ
#define COURSE_ROTATE_FAIL																5
//�������ץ��ʱ����û��ת����λ
#define PITCH_ROTATE_FAIL																	6
#define HOLD_BALL_1_ROTATE_FAIL														3
#define HOLD_BALL_2_ROTATE_FAIL														4
#define CAN1_FAIL																					5
#define CAN2_FAIL																					6

/*���������*/
#define HOLD_BALL_1														1
#define HOLD_BALL_2														2
#define CAMERA_STEER													3

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
#define ROBOT_PREPARE						0
#define ROBOT_START							1
#define COLORFUL_BALL_1					2
#define COLORFUL_BALL_2					3
#define GOLD_BALL								4

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
//֪ͨ���ƿ���ʼ�Լ�										
#define NOTIFY_MOTIONCARD_START_SELFTEST	10

/*�ѵ�������һ������Ͷ��*/
#define GET_MOTIONCARD_REACH_AREA1				1
/*�ѵ��������������Ͷ��*/
#define GET_MOTIONCARD_REACH_AREA2				2
/*�ѵ���������������Ͷ��*/
#define GET_MOTIONCARD_REACH_AREA3				3
/*�Լ����*/
#define GET_MOTIONCARD_SELFTEST_FINISH		4
/*���ƿ���ʼ�����*/
#define GET_MOTIONCARD_PREPARE_READY			5

/*����*/
#define READY															0
#define BALL_1														1
#define BALL_2														2
#define BALL_3														3

#define DELAY_TASK_NUM										2
/*��ʱ���е�����*/
#define DELAY_HOLD_BALL_1_CHECK_POS										0x01
#define DELAY_HOLD_BALL_2_CHECK_POS										0x02

#define ERROR_TIME										10

typedef struct{
  
	/*�жϴ˴��Ƿ�Ϊ���Ź���λ*/
  uint32_t isReset;
  
  /*���ڿ�������ִ�еĶ������*/
  uint32_t AT_motionFlag; 
  
  /*��¼��ʱ�����ĸ�����*/
  uint32_t process;
  
  /*��������*/
  uint32_t robocon2018;
  
  /*��������Ŀ��λ��*/
	#ifndef TEST
  float holdBallAimAngle;
	#else
  float holdBallAimAngle[2];
	#endif
  
  /*��������Ŀ��λ��*/
  float cameraAimAngle;
  
  /*�����*/
  float courseAimAngle;
  
  /*�����*/
  float pitchAimAngle;
  
  /*��ѹ*/
  float gasAimValue;
}DataSave_t;

/**************typedef area**********/
typedef struct{
  
  /*����ĳ�ʼֵ*/
  float laserInit;
  
  /*�����õ�ֵ*/
  float laser;
  
  /*��������λ��*/
  float holdBallAngle[2];
	
	/*������ص�һ�ֽ�����*/
	uint8_t steerByte;
  
  /*��������λ��*/
  float cameraAngle;
  
  /*�����*/
  float courseAngle;
  
  /*�����*/
  float pitchAngle;
  
  /*��ѹ*/
  float gasValue;
  float gasAimValue;
  
  /*��ʱ����*/
  uint32_t delayTask;
  /*��ʱ��Ҫ��ʱ��*/
  uint32_t delayTaskMs[DELAY_TASK_NUM];
	
  float angle;
  float posX;
  float posY;
	uint32_t posSystemReady;
	
	/*�������ָʾ*/
	uint8_t holdBall1Error;
	uint8_t holdBall2Error;
	uint8_t cameraSteerError;
	
	/*ϵͳ��λ�йر���*/
	uint32_t isOpenGasReturn;
	
	DataSave_t sDta;
	
	/*��������*/
	uint32_t resetTime;
	
	/*���������ѡ����ִ�г���*/
	uint32_t resetFlag;
  
}Robot_t ;

#include "gasvalveControl.h"
#include "elmo.h"
#include "shoot.h"
#include "arm_math.h"
#include "errorReport.h"
#endif

