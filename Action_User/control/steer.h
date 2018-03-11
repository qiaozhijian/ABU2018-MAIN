#ifndef __STEER_H
#define __STEER_H
#include "task.h"

#define TORQUE_SWITCH					0X28
#define RESPONSE_STAIR				0X08
#define LOCK_SWITCH						0X30
#define ID_AREA								0X05
#define AIM_POS								0X2A
#define P_STEER_ADDRESS				0X15
#define AIM_VEL								0X2E
#define AIM_TIME							0X2C

void CameraSteerPosCrl(float angle);

void CameraAlign(void);

void SteerPosCrlBy485(int num,int pos);

//以vel的速度转到angle角度/*485 ttl舵机*/
void HoldBallPosCrl(float angle);

void HoldSteer1PosCrl(float angle);

void HoldSteer2PosCrl(float angle);

//舵机1，2分开控制
void HoldBallPosCrlSeparate(float angle0,float angle1);

void OpenSteerAll(void);

uint8_t ReadOneByte(int num,int address);

void SetSteerByte(uint8_t num,uint8_t address,uint8_t value);

void SetSteerNum(uint8_t num);

void ShutAllSteerResponse(void);

void SetSteerWord(uint8_t num,uint8_t address,uint8_t value);

void ReadSteerErrorAll(void);

void SteerResponseError(uint8_t num, uint8_t errorWord);

void LetSteerRound(int num,float angle);
#endif
