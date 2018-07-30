#ifndef __SHOOT_H
#define __SHOOT_H

////选择哪一个手臂的参数
//#define  SHOOT_ARM             2

/*希望微调加上的偏移量*/
#define  COLOR_BALL1_OFFSET  (0.f)
#define  COLOR_BALL2_OFFSET  (0.f)
#define  GOLD_BALL1_OFFSET   (0.f)
#define  GOLD_BALL2_OFFSET   (0.f)

//涓嬬埅寮犲紑鍜屾姇灏勭殑鏃堕棿闂撮殧
#define CLAW_OPEN_GAP (0)

//閲戠悆鎶曞皠鍓嶇殑寤舵椂
#define GOLD_BALL_DELAY (300)

typedef struct{
  /*航向角*/
  float courseAngle;
  /*俯仰角*/
  float pitchAngle;
  /*上舵机角*/
  float upSteerAngle;
	/*下舵机角*/
  float downSteerAngle;
  /*气压值*/
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

