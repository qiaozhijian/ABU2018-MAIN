#include "stm32f4xx_can.h"

#define CAN_ENABLE 1
#define INVALID_CANSEND_MAILBOX  10
#define CAN_SEND_OK 1
#define CAN_SEND_ERR -1

#define GET_FROM_GASSENSOR									 0X10
#define SEND_TO_GASSENSOR										 0X20
#define GET_FROM_GET_FROM_GASSENSOR_CONTROL  0x30				

#define SEND_TO_MOTIONCARD				0X30
#define GET_FROM_MOTIONCARD				0X40

void CAN_Config(CAN_TypeDef* CANx, 
				uint32_t CAN_BaudRate,
				GPIO_TypeDef * GPIOx,
				uint16_t CAN_RxPin,
				uint16_t CAN_TxPin);

uint8_t CAN_RxMsg(CAN_TypeDef* CANx,
				  uint32_t * StdId,
				  uint8_t * buf,
				  uint8_t len);

uint8_t CAN_TxMsg(CAN_TypeDef* CANx,
				  uint32_t StdId,
				  uint8_t * buf,
				  uint8_t len);

int OSCANSendCmd(CAN_TypeDef* CANx, CanTxMsg* TxMessage);

void MotionCardCMDSend(uint32_t cmd);
