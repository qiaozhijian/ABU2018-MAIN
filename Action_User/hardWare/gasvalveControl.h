#ifndef __GASVALVECONTROL_H
#define __GASVALVECONTROL_H
#include  "can.h"

#define GASVALVE_BOARD_ID 				0
#define GASVALVE_BOARD_ID_DOWN    0
#define LED_BOARD_ID 							0

#define SHOOT_LED_ID							1
#define SHOOT_SMALL_ID										7
#define SHOOT_BIG_ID 									  	5
#define CLAW_ID 													4
#define GOLD_GET_STAIR2_ID 								3
#define EXTEND_THE_CAR_ID                 2


/*����������ĺ���*/
void GasValveControl(uint8_t boardNum , uint8_t valveNum , uint8_t valveState);

/*������צ�ſ��պϵ�����*/
void ClawOpen(void);

void ClawShut(void);

/*�������ʱ�Ĺ�λС����*/
void ShootSmallOpen(void);

void ShootSmallShut(void);

/*����ʱ������������*/
void ShootBigOpen(void);

void ShootBigShut(void);

/*Ͷ��LEDָʾ����*/
void ShootLedOn(void);

void ShootLedOff(void);



/*�����ץȡ��������*/
void GoldBallGraspStairTwoOn(void);

void GoldBallGraspStairTwoOff(void);

void ExtendCarOn(void);

void ExtendCarOff(void);
void LedBallInto(void);
#endif
