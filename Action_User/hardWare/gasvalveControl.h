#ifndef __GASVALVECONTROL_H
#define __GASVALVECONTROL_H
#include  "can.h"

#define GASVALVE_BOARD_ID 				0

#define SHOOT_LED_ID											1

#define LED_BOARD_ID 							0

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

/*ȥͶ��������ʱ����������*/
void BoostPolePush(void);

void BoostPoleReturn(void);

/*�����ץȡһ������*/
void GoldBallGraspStairOneOn(void);

void GoldBallGraspStairOneOff(void);

/*�����ץȡ��������*/
void GoldBallGraspStairTwoOn(void);

void GoldBallGraspStairTwoOff(void);
#endif
