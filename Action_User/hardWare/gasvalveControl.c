#include  "can.h"
#include "gasvalveControl.h"
#include "gpio.h"
#include "usart.h"
#include "task.h"
#include "robot.h"





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
	SetMotionFlag(AT_CLAW_STATUS_OPEN);
	GasValveControl(GASVALVE_BOARD_ID , CLAW_ID , 1);
}

void ClawShut(void)
{
	SetMotionFlag(~AT_CLAW_STATUS_OPEN);
	GasValveControl(GASVALVE_BOARD_ID , CLAW_ID , 0);
}

/*射球完毕时的归位小气阀*/
void ShootSmallOpen(void)
{
	GasValveControl(GASVALVE_BOARD_ID , SHOOT_SMALL_ID , 0);
	SetMotionFlag(AT_SHOOT_SMALL_ENABLE);
}

void ShootSmallShut(void)
{
	GasValveControl(GASVALVE_BOARD_ID , SHOOT_SMALL_ID , 1);
	SetMotionFlag(~AT_SHOOT_SMALL_ENABLE);
}

/*射球时的助力大气阀*/
void ShootBigOpen(void)
{
	GasValveControl(GASVALVE_BOARD_ID , SHOOT_BIG_ID , 1);
	SetMotionFlag(AT_SHOOT_BIG_ENABLE);
}

void ShootBigShut(void)
{
	GasValveControl(GASVALVE_BOARD_ID , SHOOT_BIG_ID , 0);
	SetMotionFlag(~AT_SHOOT_BIG_ENABLE);
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


/*金球架抓取二级气阀*/
void GoldBallGraspStairTwoOn(void)
{
	GasValveControl(GASVALVE_BOARD_ID , GOLD_GET_STAIR2_ID , 0);
}

void GoldBallGraspStairTwoOff(void)
{
	GasValveControl(GASVALVE_BOARD_ID , GOLD_GET_STAIR2_ID , 1);
}
/*下爪手臂向下撑*/
void LowerClawStairOn(void)
{
	GasValveControl(GASVALVE_BOARD_ID_DOWN, LOW_CLAW_ID , 1);
}
/*下爪手臂向上抬*/
void LowerClawStairOff(void)
{
	GasValveControl(GASVALVE_BOARD_ID_DOWN, LOW_CLAW_ID , 0);
}
