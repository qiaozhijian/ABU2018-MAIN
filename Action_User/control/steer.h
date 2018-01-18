#ifndef __STEER_H
#define __STEER_H

#define STEER1_ENABLE_FAIL														1
#define STEER1_ROTATE_FAIL														2
#define STEER2_ENABLE_FAIL														3
#define STEER2_ROTATE_FAIL														4
#define STEER1_ROTATE_SEND_FAIL												5
#define STEER2_ROTATE_SEND_FAIL												6







//�򿪶����Ť�����
void Enable_ROBS(void);

//ʹ���ŷ�ģʽ
void Enable_ServoMode(void) ;

//ʹ�ܵ��ģʽ
void Enable_MotorMode(void);

void ROBS_VelCrl(int vel,int time);

void ROBS_PosCrl(float angleUP, float angleDOWN, int vel);

void TurnLeft(int vel);

void TurnRight(int vel);

void Stop(void);

void ReadROBSAngle(void);

void SteerErrorRecord(char type);

void SteerErrorReport(void);

void Steer1ROBS_PosCrl(float angleUP, int vel);

void Steer2ROBS_PosCrl(float angleDOWN, int vel);
#endif
