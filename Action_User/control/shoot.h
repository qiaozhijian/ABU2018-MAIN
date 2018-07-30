#ifndef __SHOOT_H
#define __SHOOT_H

////ѡ����һ���ֱ۵Ĳ���
//#define  SHOOT_ARM             2

/*ϣ��΢�����ϵ�ƫ����*/
#define  COLOR_BALL1_OFFSET  (0.f)
#define  COLOR_BALL2_OFFSET  (0.f)
#define  GOLD_BALL1_OFFSET   (0.f)
#define  GOLD_BALL2_OFFSET   (0.f)

//下爪张开和投射的时间间隔
#define CLAW_OPEN_GAP (0)

//金球投射前的延时
#define GOLD_BALL_DELAY (300)

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

float GetPrepareShootGoldBallGasAim(int ballNum);

void ShootBall(void);

void ShootReset(void);

void PrepareGetBall(int index);

void PrepareShootBall(int index);

void prepareMotionParaInit(void);

void PrepareWork(void);

void SmallChange(void);

void PrepareParamByRaBSwitch(void);

void SetResetGoldGetBallGasaim(void);

void SetNormalGoldGetBallGasaim(void);

float GetGetBallUpSteerAngle(int process);
#endif

