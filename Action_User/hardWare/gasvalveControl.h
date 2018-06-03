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



/*金球架抓取二级气阀*/
void GoldBallGraspStairTwoOn(void);

void GoldBallGraspStairTwoOff(void);

void ExtendCarOn(void);

void ExtendCarOff(void);
void LedBallInto(void);
#endif
