#ifndef __GASVALVECONTROL_H
#define __GASVALVECONTROL_H
#include  "can.h"

#define SHOOT_BIG_ID 							3
#define SHOOT_SMALL_ID 						4
#define CLAW_ID 									7

#define GASVALVE_BOARD_ID 				0

void GasValveControl(uint8_t boardNum , uint8_t valveNum , uint8_t valveState);
#endif
