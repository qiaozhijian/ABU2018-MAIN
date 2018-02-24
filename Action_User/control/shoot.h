#ifndef __SHOOT_H
#define __SHOOT_H


typedef struct{
  /*�����*/
  float courseAngle;
  /*������*/
  float pitchAngle;
  /*�����*/
  float steerAngle;
  /*�����ת���ٶ�*/
  int steerSpeed;
  /*��ѹֵ*/
  float gasAim;
  
}motionPara_t;


void ShootBall(void);

void ShootReset(void);

void PrepareGetBall(int index);

void PrepareShootBall(int index);

void prepareMotionParaInit(void);

void PrepareWork(void);
#endif

