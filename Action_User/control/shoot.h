#ifndef __SHOOT_H
#define __SHOOT_H


typedef struct{
  /*航向角*/
  float courseAngle;
  /*俯仰角*/
  float pitchAngle;
  /*舵机角*/
  float steerAngle;
  /*舵机旋转的速度*/
  int steerSpeed;
  /*气压值*/
  float gasAim;
  
}motionPara_t;


void PitchAngleMotion(float angle);

void CourseAngleMotion(float angle);

void ShootBall(void);

void ShootReset(void);

void PrepareGetBall(int index);

void PrepareShootBall(int index);

void prepareMotionParaInit(void);
#endif

