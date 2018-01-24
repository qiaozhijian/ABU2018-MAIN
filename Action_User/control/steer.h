#ifndef __STEER_H
#define __STEER_H

//�򿪶����Ť�����
void Enable_ROBS(void);

//ʹ���ŷ�ģʽ
void Enable_ServoMode(void) ;

//ʹ�ܵ��ģʽ
void Enable_MotorMode(void);

void ROBS_PosTimeCrl(float angleUP, float angleDOWN, int time);

void ROBS_PosCrl(float angleUP, float angleDOWN, int vel);

void TurnLeft(int vel);

void TurnRight(int vel);

void Stop(void);

void ReadROBSAngle(void);

void Steer1ROBS_PosCrl(float angleUP, int vel);

void Steer2ROBS_PosCrl(float angleDOWN, int vel);

void ReadSteer1Pos(void);

void ReadSteer2Pos(void);

#endif
