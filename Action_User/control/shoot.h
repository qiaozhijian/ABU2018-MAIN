#ifndef __SHOOT_H
#define __SHOOT_H


typedef struct{
  /*航向角*/
  float courseAngle;
  /*俯仰角*/
  float pitchAngle;
  /*上舵机角*/
  float upSteerAngle;
	/*下舵机角*/
  float downSteerAngle;
  /*舵机旋转的速度*/
  int steerSpeed;
  /*气压值*/
  float gasAim;
  
}motionPara_t;


void ShootBall(void);

void ShootReset(void);

void PrepareGetBall(int index);

void PrepareShootBall(int index);

void prepareMotionParaInit(void);

void PrepareWork(void);

void SmallChange(void);
#endif

