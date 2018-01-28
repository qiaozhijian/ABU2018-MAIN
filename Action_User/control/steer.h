#ifndef __STEER_H
#define __STEER_H

void Enable_ROBS(void);

void HoldBallPosCrl(float angle,int vel);

void ReadHoldBallSteerPos(void);

void SteerPosCrl(float angle);

void CameraAlign(void);

#endif
