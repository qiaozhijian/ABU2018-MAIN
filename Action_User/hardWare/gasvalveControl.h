#ifndef __GASVALVECONTROL_H
#define __GASVALVECONTROL_H
#include  "can.h"

#define GASVALVE_BOARD_ID 				0

#define SHOOT_LED_ID											1

#define LED_BOARD_ID 							0

/*控制气阀板的函数*/
void GasValveControl(uint8_t boardNum , uint8_t valveNum , uint8_t valveState);

/*控制下爪张开闭合的气阀*/
void ClawOpen(void);

void ClawShut(void);

/*射球完毕时的归位小气阀*/
void ShootSmallOpen(void);

void ShootSmallShut(void);

/*射球时的助力大气阀*/
void ShootBigOpen(void);

void ShootBigShut(void);

/*投球LED指示气阀*/
void ShootLedOn(void);

void ShootLedOff(void);

/*去投第三个球时的助推气阀*/
void BoostPolePush(void);

void BoostPoleReturn(void);

/*金球架抓取一级气阀*/
void GoldBallGraspStairOneOn(void);

void GoldBallGraspStairOneOff(void);

/*金球架抓取二级气阀*/
void GoldBallGraspStairTwoOn(void);

void GoldBallGraspStairTwoOff(void);
#endif
