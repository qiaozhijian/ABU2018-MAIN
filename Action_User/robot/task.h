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
#define TEST_GOLD
//#define TEST
#define GAS_CONTOL_BY_PWM  1
//	 DEBUG

//��������
//��λϵͳλ���Զ���ˮƽ������м䣬��ֱ�����ǽmm505.f
//����ͷλ���Զ���ƫ��21.85mm����ǽ172.44mm����ĵط�
//��ά����Ӿ�ǽ505mm
#define CAMERA_TO_GYRO_X																							21.85f
#define CAMERA_TO_GYRO_Y																							332.56f
#define QUICK_MARK_X_1																								5505.f
#define QUICK_MARK_X_2																								6505.f
#define QUICK_MARK_Y																									800.f


#define CAR_L																													0.5389f
/*����ֱ��*/
#define WHEEL_D																												0.1524f
#define SHOOT_ANGLE																									  (6)
#define COUNTS_PER_ROUND_REDUC 																		  	4096.f

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
/*��λϵͳ�복���ĵĲ�*/
#define DISX_GYRO2CENTER (164.0f)
#define DISY_GYRO2CENTER (445.0f)
/*��������ת�����ĵ����*/
#define CAR_CENTER_TO_COURCE_CENTER   (15.f)

/*3508���ٱ�3591/187=19.203208  8192λ������*/
#define PITCH_CODE_TO_ANGLE(code)																			((code)/8738.1333f)
#define PITCH_ANGLE_TO_CODE(angle)																		((int)((angle)*8738.1333f))//((angle)*20.0f*19.2f*8192.f/360.f)

#define COURSE_CODE_TO_ANGLE(code)																		((code)/6116.693f)
#define COURSE_ANGLE_TO_CODE(angle)																		((int)(((angle))*6116.693f))//((angle)*14.0f*19.2f*8192.f/360.f)

#define UPSTEER_CODE_TO_ANGLE(code)                       						((code)/1280.f)       
#define DOWN_STEER_CODE_TO_ANGLE(code)                    						((code)/1638.4f)

#define UP_STEER_COMPENSATE                             						  (116.f)   
#define DOWN_STEER_COMPENSATE																					(107.f)
#define PITCH_COMPENSATE																							(-10.f)
#define COURCE_COMPENSATE																						  (193.f)
//״̬������
#define AT_CLAW_STATUS_OPEN 																					0x01u

#define AT_STEER_READY 																								0x02u

#define AT_SHOOT_BIG_ENABLE 																					0x04u

#define AT_SHOOT_SMALL_ENABLE 			  																0x08u

#define AT_HOLD_BALL_1_SUCCESS																				0x10u

#define AT_HOLD_BALL_2_SUCCESS																				0x20u

#define AT_HOLD_BALL_1_RESPONSE																				0x40u

#define AT_HOLD_BALL_2_RESPONSE																				0x80u

#define AT_COURSE_READ																								0x200u

#define AT_PITCH_READ																									0x400u

#define AT_COURSE_SUCCESS																							0x800u

#define AT_PITCH_SUCCESS																							0x1000u

#define AT_CAMERA_RESPONSE_SUCCESS																		0x2000u

#define AT_CAMERA_TALK_SUCCESS																				0x100u

#define AT_GAS_SUCCESS																								0x4000u

#define AT_PREPARE_READY																							0x8000u

#define AT_IS_SEND_DEBUG_DATA																					0x10000u

#define AT_REACH_FIRST_PLACE																					0x20000u

#define AT_REACH_SECOND_PLACE																					0x40000u

#define AT_REACH_THIRD_PLACE																					0x80000u

#define AT_THE_WHEEL_SELFTEST_OVER																		0x100000u

#define AT_THE_DUCT_SELFTEST_OVER																			0x200000u

#define AT_RESET_THE_ROBOT																						0x400000u
//�ж��Ƿ�Ͷ������
#define AT_RESET_SHOOT_GOLD																						0x800000u

#define AT_RESET_USE_GOLD_STANDYBY																		0x1000000u

#define AT_GET_MOTIONCARD_RESET_FINISH																0x2000000u

#define AT_GET_MOTIONCARD_GET_GOLDBALL_AREA         									0x4000000u

#define AT_GET_PPS_PROBLEM         									                  0x8000000u




//״̬������
#define CAN_CLAW_STATUS_OPEN 																					0x01u

#define CAN_STEER_READY 																							0x02u

#define CAN_SHOOT_BIG_ENABLE 																					0x04u

#define CAN_SHOOT_SMALL_ENABLE 			  																0x08u


/**���������**/
//��ʼ��ʱ���ʹ��������
#define HOLD_BALL_1_ENABLE_FAIL																				1
#define HOLD_BALL_2_ENABLE_FAIL																				2
//�������ץ��ʱ���û��ת����λ
#define HOLD_BALL_1_ROTATE_FAIL																				3
#define HOLD_BALL_2_ROTATE_FAIL																				4
//�������ץ��ʱת̨û��ת����λ
#define COURSE_ROTATE_FAIL																						5
//�������ץ��ʱ����û��ת����λ
#define PITCH_ROTATE_FAIL																							6
#define HOLD_BALL_1_ROTATE_FAIL																				3
#define HOLD_BALL_2_ROTATE_FAIL																				4
#define CAN1_FAIL																											5
#define CAN2_FAIL																											6

/*���������*/
#define HOLD_BALL_1																										1
#define HOLD_BALL_2																										2
#define CAMERA_STEER																									3

/*״̬������*/
#define TO_START																											1
#define TO_GET_BALL_1																									2
#define TO_THE_AREA_1																									3
#define TO_THROW_BALL_1																								4
#define TO_GET_BALL_2																									5
#define TO_THE_AREA_2																									6
#define TO_THROW_BALL_2																								7
#define TO_GET_BALL_3																									8
#define TO_THE_AREA_3																									9
#define TO_THROW_BALL_3									  														10
#define END_COMPETE																										100

/*�������̺궨�����*/
#define ROBOT_PREPARE																									0
#define ROBOT_START																										1
#define COLORFUL_BALL_1																								2
#define COLORFUL_BALL_2																								3
#define GOLD_BALL																											4
#define ROBOT_SELF_TEST      																					9
#define INTO_HARDFAULT																								10
/*����׼�����*/
#define INTO_RESET_PREPARE    																				16
/*���������Զ������*/
#define ROBOT_CONTROL_BY_BT   																				11

#define RACK3_BALL                                                    18
/*���ƿ�ͨ�Ž���*/
//��ʼ����
#define NOTIFY_MOTIONCARD_START																				1
//֪ͨ�����Ѿ��õ���һ
#define NOTIFY_MOTIONCARD_GOT_BALL1																		2
//֪ͨ���ƿ��Ѿ���������һ
#define NOTIFY_MOTIONCARD_SHOT_BALL1																	3
//֪ͨ�����Ѿ��õ����
#define NOTIFY_MOTIONCARD_GOT_BALL2																		4
//֪ͨ���ƿ��Ѿ����������
#define NOTIFY_MOTIONCARD_SHOT_BALL2																	5
//֪ͨ�����Ѿ��õ�����
#define NOTIFY_MOTIONCARD_GOT_BALL3																		6
//֪ͨ���ƿ���һ����
#define NOTIFY_MOTIONCARD_LOSE_BALL1																	7
//֪ͨ���ƿ��������
#define NOTIFY_MOTIONCARD_LOSE_BALL2																	8
//֪ͨ���ƿ���������
#define NOTIFY_MOTIONCARD_LOSE_BALL3																	9
//֪ͨ���ƿ���ʼ�Լ�										
#define NOTIFY_MOTIONCARD_START_SELFTEST															10
//֪ͨ���ƿ��Զ�������׼�����									
#define NOTIFY_MOTIONCARD_PREPARE_FINISH															11

//֪ͨ���ƿ������Լ�
#define NOTIFY_MOTIONCARD_SELFTEST																		12
//֪ͨ���ƿ����������Լ�
#define NOTIFY_MOTIONCARD_SELFTEST_THE_WHEEL													13
//֪ͨ���ƿ����м����Լ�
#define NOTIFY_MOTIONCARD_SELFTEST_THE_LASER													14
//֪ͨ���ƿ��Լ캭��
#define NOTIFY_MOTIONCARD_SELFTEST_THE_DUCT														33
//֪ͨ���ƿ����ؽ����Լ�ģʽ
#define NOTIFY_MOTIONCARD_INTO_BT_CTRL																84
//֪ͨ���ƿ�ʹ������
#define NOTIFY_MOTIONCARD_ENABLE_WHEEL																78
//֪ͨ���ƿ�ʧ������
#define NOTIFY_MOTIONCARD_DISABLE_WHEEL																70
//֪ͨ���ƿ���������	
#define NOTIFY_MOTIONCARD_INTO_RESET																	97
//֪ͨ���ƿ���������
#define NOTIFY_MOTIONCARD_RESET_GOLD																	98 
//֪ͨ���ƿ���������
#define NOTIFY_MOTIONCARD_RESET_ALL																		99
//֪ͨ���ƿ�������ֳ���
#define NOTIFY_MOTIONCARD_WIPE_WHEEL																	38               
//֪ͨ���ƿ�����������β��Բ���
#define NOTIFY_MOTIONCARD_INTO_TEST_GOLD                              68
//֪ͨ���ƿ�����ȥ�����ģʽ
#define NOTIFY_MOTIONCARD_INTO_GET_RACK3BALL                          51                          
//֪ͨ���ƿ�������ѡ��
#define NOTIFY_MOTIONCARD_CHOOSE_BLUE                                 63

#define NOTIFY_MOTIONCARD_CHOOSE_RED                                  64
/*�ѵ�������һ������Ͷ��*/
#define GET_MOTIONCARD_REACH_AREA1																		1
/*�ѵ��������������Ͷ��*/
#define GET_MOTIONCARD_REACH_AREA2																		2
/*�ѵ���������������Ͷ��*/
#define GET_MOTIONCARD_REACH_AREA3																		3
/*�Լ����*/
#define GET_MOTIONCARD_SELFTEST_FINISH																4
/*���ƿ���ʼ�����*/
#define GET_MOTIONCARD_PREPARE_READY																	5
/*���ƿ�����ӽ���λ��*/
#define GET_MOTIONCARD_GET_GOLDBALL_AREA															8
/*���ƿ������Һ����Լ����*/
#define GET_MOTIONCARD_DUCT_SELFTEST_OK																21
/*���ƿ������Һ����Լ����*/
#define GET_MOTIONCARD_RESET_FINISH																		33


/*���ƿ�֪ͨ����Ӳ���ж�*/
#define GET_MOTIONCARD_INTO_HARDFAULT																	44

#define GET_MOTIONCARD_SELFTEST_WHEEL_OVER 														66
	
#define GET_QIAO_ZHIJIAN_TEST          														    88

#define GET_PPS_PROBLEM                														    74


/*����*/
#define READY																													0
#define BALL_1																												1
#define BALL_2																												2
#define BALL_3																												3
#define BALL_4																												4
/*����12�ı���*/
#define BALL_1_BACKUP																									5
#define BALL_2_BACKUP																									8
/*����12�ı���*/
#define BALL_3_BACKUP																									9
#define BALL_4_BACKUP																									10

#define BALL_3_WAIT																										6
/*���3����*/
#define BALL_RACK3																										16

#define DELAY_TASK_NUM																								2
/*��ʱ���е�����*/
#define DELAY_HOLD_BALL_1_CHECK_POS																		0x01
#define DELAY_HOLD_BALL_2_CHECK_POS																		0x02

#define ERROR_TIME																										10

#define COLOR_BALL_FRAME_POSX																				  (515.f)
#define COLOR_BALL_FRAME_POSY																					(3275.f)

#define GOLD_BALL_FRAME_POSX																					(-3220.f)
#define GOLD_BALL_FRAME_POSY																					(6535.f)

#define ROBOT_COURCE_CENTER_TO_ARM																		(445.f)
#define ROBOT_ARM_TO_MOTOR																						(509.26f)
#define ROBOT_ARM_TO_THROW_CENTER																			(596.88f)
#define ROBOT_CENTER_TO_COURCE																				(0.f)

#define TZ_1_X																												(4565.0f)
#define TZ_1_Y																												(2180.0f)

#define TZ_2_X																												(6565.0f)
#define TZ_2_Y																												(2230.0f)

#define TZ_3_X																												(6894.0f)
#define TZ_3_Y																												(6030.0f)

#define HANDOVER_3_X																									(6894.f)
#define HANDOVER_3_Y																									(935.f)

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
//	#ifndef TEST
//  float holdBallAimAngle;
//	#else
  float holdBallAimAngle[2];
//	#endif
  
  /*��������Ŀ��λ��*/
  float cameraAimAngle;
  
  /*�����*/
  float courseAimAngle;
  
  /*�����*/
  float pitchAimAngle;
  
  /*��ѹ*/
  float gasAimValue;
	
	/*ĳһ������*/
	int WhichGoldBall;
}DataSave_t;
/*��������˵��ٶȽṹ��*/
typedef struct{
	float lastPosX[3];
  float lastPosY[3];
	uint32_t countTime;
	//���㳵xy�����ϵĺͺ��ٶ�
	
	float countVel;
	float countXVel;
	float countYVel;
	
	float courseVel;
	float steerVel[2];
	float readCourseVel;
	float readSteerVel[2];
}RobotVel_t;
/*ʱ�����ṹ��*/
typedef struct{
	/*��ʱ����������������*/
	uint32_t roboconCnt;
	/*��ʱ������������ʱ��*/
	float roboconTime;
		
	float colorBall1Time;
	float colorBall2Time;
	float goldBallTime;
	
	float colorBall1WaitTime;
	float colorBall2WaitTime;
	float goldBallWaitTime;
	
	float colorBall1ThrowTime;
	float colorBall2ThrowTime;
	float goldBallThrowTime;
}RobotconTime_t;
typedef struct{
	int readMotorTime[4];
}ErrorTime_t;
typedef struct{
	int colorBall1;
	int colorBall2;
	int goldBall;
	uint8_t gasSatisfyFlag;
  uint8_t gasSatisfyCnt;
}GetBallStep_t;
/**************typedef area**********/
typedef struct{
  
  /*����ĳ�ʼֵ*/
  float laserInit;
  
  /*�����õ�ֵ*/
  uint16_t laser[2];
  
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
	uint16_t gasAdc;
  
  /*��ʱ����*/
  uint32_t delayTask;
  /*��ʱ��Ҫ��ʱ��*/
  uint32_t delayTaskMs[DELAY_TASK_NUM];
	
  float angle;
  float posX;
  float posY;
	float speedX;
	float speedY;
	float AngularVelocity;
	float CarSpeed;
	int posSystemCode[2];
	float angleBais;
	float KalmanZ;
	
	float gasControl;
	uint32_t posSystemReady;
	
//	/*�������ָʾ*/
//	uint8_t holdBall1Error;
//	uint8_t holdBall2Error;
//	uint8_t cameraSteerError;
	
	/*ϵͳ��λ�йر���*/
	uint32_t isOpenGasReturn;
	
	DataSave_t sDta;
	
	/*��������*/
	uint32_t resetTime;
	
	/*���������ѡ����ִ�г���*/
	uint32_t resetFlag;
	
	
	/*��������˵��ٶȽṹ��*/
	RobotVel_t robotVel;
  
	/*ȡ����*/
	GetBallStep_t getBallStep;
	/*���������׶ν��е�ʱ��*/
	RobotconTime_t raceTime;
	
	ErrorTime_t  errorTime;
}Robot_t ;

#include "gasvalveControl.h"
#include "elmo.h"
#include "shoot.h"
#include "arm_math.h"
#include "errorReport.h"
#endif

