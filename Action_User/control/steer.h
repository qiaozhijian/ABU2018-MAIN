#ifndef __STEER_H
#define __STEER_H
#include "task.h"
void Enable_ROBS(void);

void ReadHoldBallSteerPos(void);

void CameraSteerPosCrl(float angle);

void CameraAlign(void);

void SetNumber(unsigned char num);

void SteerPosCrlBy485(int num,int pos);

void SetNumber(unsigned char num);

void UnLockSteer(int num);	

//以vel的速度转到angle角度
void HoldBallPosCrl(float angle,int vel);

void HoldSteer1PosCrl(float angle,int vel);

void HoldSteer2PosCrl(float angle,int vel);

#endif
