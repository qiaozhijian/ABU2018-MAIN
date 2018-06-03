#ifndef __SHOOT_H
#define __SHOOT_H

/*ϣ��΢�����ϵ�ƫ����*/
#define  COLOR_BALL1_OFFSET  (0.f)
#define  COLOR_BALL2_OFFSET  (0.f)
#define  GOLD_BALL1_OFFSET   (-0.15f)
#define  GOLD_BALL2_OFFSET   (0.f)

typedef struct{
  /*�����*/
  float courseAngle;
  /*������*/
  float pitchAngle;
  /*�϶����*/
  float upSteerAngle;
	/*�¶����*/
  float downSteerAngle;
  /*��ѹֵ*/
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

