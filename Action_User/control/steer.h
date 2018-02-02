#ifndef __STEER_H
#define __STEER_H
#include "task.h"

#define TORQUE_SWITCH					0X28
#define RESPONSE_STAIR				0X08
#define LOCK_SWITCH						0X30
#define ID_AREA								0X05

void CameraSteerPosCrl(float angle);

void CameraAlign(void);

void SteerPosCrlBy485(int num,int pos);

//��vel���ٶ�ת��angle�Ƕ�
void HoldBallPosCrl(float angle,int vel);

void HoldSteer1PosCrl(float angle,int vel);

void HoldSteer2PosCrl(float angle,int vel);
	
void OpenSteerAll(void);

uint8_t ReadOneByte(int num,int address);

void SetSteerByte(uint8_t num,uint8_t address,uint8_t value);

void SetSteerNum(uint8_t num);

void ShutAllSteerResponse(void);
#endif
