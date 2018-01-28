#include  "can.h"
#include "gasvalveControl.h"
#include "gpio.h"
#include "usart.h"
#include "task.h"



#define SHOOT_SMALL_ID										2
#define CLAW_ID 													3
#define GOLD_GET_STAIR2_ID 								4
#define SHOOT_BIG_ID 									  	5
#define GOLD_GET_STAIR1_ID								6
#define BOOST_POLE_ID											8


/*
车底两个助力气阀，一个气阀
持球下爪阀
抓金球架 两个阀 一级二级
发射架 大阀小阀
三位阀
*/

extern Robot_t gRobot;

/**
* @brief  气阀控制
* @param  boardNum：气阀板号
* @param  valveNum：气阀号
* @param  valveState： 气阀状态，0为关，1为开
* @author ACTION
*/
void GasValveControl(uint8_t boardNum , uint8_t valveNum , uint8_t valveState)
{
	uint8_t data = 0x00;
//	uint8_t mbox;
	CanTxMsg TxMessage;
	TxMessage.StdId = 0x0001 ;					 // standard identifier=1
	TxMessage.ExtId = 0x0001 ;				 	 // extended identifier=StdId
	TxMessage.IDE = CAN_Id_Standard;			 // type of identifier for the message is Standard
	TxMessage.RTR = CAN_RTR_Data;			 	 // the type of frame for the message that will be transmitted
	TxMessage.DLC = 1;
	
	data = boardNum<<5|valveNum<<1|valveState;
	
	TxMessage.Data[0] = data;

	OSCANSendCmd(CAN2, &TxMessage);

}

/*控制下爪张开闭合的气阀*/
void ClawOpen(void)
{
	GasValveControl(GASVALVE_BOARD_ID , CLAW_ID , 0);
}

void ClawShut(void)
{
	GasValveControl(GASVALVE_BOARD_ID , CLAW_ID , 1);
}

/*射球完毕时的归位小气阀*/
void ShootSmallOpen(void)
{
	GasValveControl(GASVALVE_BOARD_ID , SHOOT_SMALL_ID , 1);
}

void ShootSmallShut(void)
{
	GasValveControl(GASVALVE_BOARD_ID , SHOOT_SMALL_ID , 0);
}

/*射球时的助力大气阀*/
void ShootBigOpen(void)
{
	GasValveControl(GASVALVE_BOARD_ID , SHOOT_BIG_ID , 1);
}

void ShootBigShut(void)
{
	GasValveControl(GASVALVE_BOARD_ID , SHOOT_BIG_ID , 0);
}

/*投球LED指示气阀*/
void ShootLedOn(void)
{
	GasValveControl(LED_BOARD_ID , SHOOT_LED_ID , 1);
}

void ShootLedOff(void)
{
	GasValveControl(LED_BOARD_ID , SHOOT_LED_ID , 0);
}

/*去投第三个球时的助推气阀*/
void BoostPolePush(void)
{
	GasValveControl(GASVALVE_BOARD_ID , BOOST_POLE_ID , 1);
}

void BoostPoleReturn(void)
{
	GasValveControl(GASVALVE_BOARD_ID , BOOST_POLE_ID , 0);
}

/*金球架抓取一级气阀*/
void GoldBallGraspStairOneOff(void)
{
	GasValveControl(GASVALVE_BOARD_ID , GOLD_GET_STAIR1_ID , 1);
}

void GoldBallGraspStairOneOn(void)
{
	GasValveControl(GASVALVE_BOARD_ID , GOLD_GET_STAIR1_ID , 0);
}

/*金球架抓取二级气阀*/
void GoldBallGraspStairTwoOn(void)
{
	GasValveControl(GASVALVE_BOARD_ID , GOLD_GET_STAIR2_ID , 0);
}

void GoldBallGraspStairTwoOff(void)
{
	GasValveControl(GASVALVE_BOARD_ID , GOLD_GET_STAIR2_ID , 1);
}
