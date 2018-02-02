#include "task.h"
#include "timer.h"
#include "gpio.h"
#include "process.h"
#include "stm32f4xx_it.h"
#include "steer.h"
#include "customer.h"
#include "motion.h"
#include "includes.h"

extern OS_EVENT *PeriodSem;

extern Robot_t gRobot;
extern int flagggg;
void SelfTest(void)
{
	
  CPU_INT08U  os_err;
  os_err = os_err;
	
	AT_CMD_Handle();
	USART_BLE_SEND(gRobot.gasValue);
	static int step=10;
	switch(step)
	{
		case 10:
//			SetResponseStair(0x01);
//	HoldSteer1PosCrl(0.f,2000);		
//	OpenSteerTorque(0xfe);
//  SteerPosCrlBy485(0xfe,(int)(180.f*11.378f));
//		HoldSteer2PosCrl(30.f,2000);
		
//		if(flagggg==1)
//		{
//			BoostPolePush();
//			step++;
//		}
//		

			break;
		case 0:
			ShootSmallOpen();
			//���λ�û�
			PosLoopCfg(CAN2, 5, 100000, 100000,100000);
			//���λ�û�
			PosLoopCfg(CAN2, 6, 100000, 100000,100000);
			step++;
			break;
		case 1:
			CourseAngleMotion(0.f);
			PitchAngleMotion(0.f);
			step++;
			break;
		case 2:
			if(PE_FOR_THE_BALL)
			{
				step++;
				Delay_ms(1000);
				/*����ź���*/
				OSSemSet(PeriodSem, 0, &os_err);
				CourseAngleMotion(180.f);
				Delay_ms(1000);
				/*����ź���*/
				OSSemSet(PeriodSem, 0, &os_err);
			}
			break;
		case 3:
			//��С����
		
			ReadActualPos(CAN2,6);
			if(gRobot.courseAngle>160.f)
			{
				//��С����
				step++;
			}
			break;
		case 4:
			if(gRobot.courseAngle>179.f)
			{
				ShootBall();
				Delay_ms(1000);
				/*����ź���*/
				OSSemSet(PeriodSem, 0, &os_err);
				ShootReset();
				step=0;
			}
			break;
	}
}

/*���Ͷ�����һ������*/
void FightForBall1(void)
{
  CPU_INT08U  os_err;
  os_err = os_err;
	
  switch(gRobot.process)
  {
    /*ȥȡ��һ����*/
  case TO_GET_BALL_1:
    
    if(PE_FOR_THE_BALL)
    {	
      Delay_ms(500);
			/*����ź���*/
			OSSemSet(PeriodSem, 0, &os_err);
      
      MotionCardCMDSend(NOTIFY_MOTIONCARD_GOT_BALL1);
      
			//TalkToCamera(CAMERA_OPEN_NEAR);
			
      PrepareShootBall(BALL_1);
      
      gRobot.process=TO_THE_AREA_1;
      
    }
    break;
    /*��һ����ȡ����ϣ�ȥͶ����һ*/
  case TO_THE_AREA_1:
    if(!PE_FOR_THE_BALL)
    {
      MotionCardCMDSend(NOTIFY_MOTIONCARD_LOSE_BALL1);
    }
    break;
    /*����Ͷ����һ������*/
  case TO_THROW_BALL_1:
		/*��絽λ*/
    if(PE_FOR_THE_BALL
				/*��������λ*/
				&&(gRobot.AT_motionFlag&AT_HOLD_BALL1_SUCCESS)
					/*��������λ*/
					&&(gRobot.AT_motionFlag&AT_HOLD_BALL2_SUCCESS)
						/*������λ��*/
						&&(gRobot.AT_motionFlag&AT_PITCH_SUCCESS)
							/*����λ*/
							&&(gRobot.AT_motionFlag&AT_COURSE_SUCCESS)&&(gRobot.posY>1800.f))
								/*��ѹ��λ*/
							//	&&(gRobot.AT_motionFlag&AT_GAS_SUCCESS))||PE_FOR_THE_BALL)
    {
      /*����*/
      ShootBall();
      /*����ʱʹ�������ִ�е�λ*/
      Delay_ms(500);
			/*����ź���*/
			OSSemSet(PeriodSem, 0, &os_err);
      /*֪ͨ���ƿ�*/
      MotionCardCMDSend(NOTIFY_MOTIONCARD_SHOT_BALL1);
      /*���������λ*/
      ShootReset();
      /*׼�������*/
      PrepareGetBall(BALL_2);
      /*������һ״̬*/
      gRobot.process=TO_GET_BALL_2;
      gRobot.robocon2018=COLORFUL_BALL_2;
    }
		else
		{
			if(!PE_FOR_THE_BALL)
				USART_OUT_ONCE("!PE1\t");
			if(!(gRobot.AT_motionFlag&AT_HOLD_BALL1_SUCCESS))
				USART_OUT_ONCE("!HB11\t");
			if(!(gRobot.AT_motionFlag&AT_HOLD_BALL2_SUCCESS))
				USART_OUT_ONCE("!HB21\t");
			if(!(gRobot.AT_motionFlag&AT_PITCH_SUCCESS))
				USART_OUT_ONCE("!PITCH1\t");
			if(!(gRobot.AT_motionFlag&AT_COURSE_SUCCESS))
				USART_OUT_ONCE("!COURSE1\t");
			if(!(gRobot.AT_motionFlag&AT_GAS_SUCCESS))
				USART_OUT_ONCE("!GAS1\t");
			USART_Enter();
		}
    break;
  }
}

/*���Ͷ������������*/
void FightForBall2(void)
{
  CPU_INT08U  os_err;
  os_err = os_err;
	
  switch(gRobot.process)
  {
    /*ȥȡ�ڶ�����*/
  case TO_GET_BALL_2:
    if(PE_FOR_THE_BALL&&gRobot.posY<1600.f)
    {	
      /*ɨ������Ϊ�˸��ȵؽӵ����������ʱ*/
      Delay_ms(500);
			/*����ź���*/
			OSSemSet(PeriodSem, 0, &os_err);
      gRobot.process=TO_THE_AREA_2;
      
      MotionCardCMDSend(NOTIFY_MOTIONCARD_GOT_BALL2);
      
      PrepareShootBall(BALL_2);
      
    }
    break;
    /*�ڶ�����ȡ����ϣ�ȥͶ������*/
  case TO_THE_AREA_2:
    if(!PE_FOR_THE_BALL)
    {
      MotionCardCMDSend(NOTIFY_MOTIONCARD_LOSE_BALL2);
    }
    break;
    /*����Ͷ������������*/
  case TO_THROW_BALL_2:
		/*��絽λ*/
    if(PE_FOR_THE_BALL
				/*��������λ*/
				&&(gRobot.AT_motionFlag&AT_HOLD_BALL1_SUCCESS)
					/*��������λ*/
					&&(gRobot.AT_motionFlag&AT_HOLD_BALL2_SUCCESS)
						/*������λ��*/
						&&(gRobot.AT_motionFlag&AT_PITCH_SUCCESS)
							/*����λ*/
							&&(gRobot.AT_motionFlag&AT_COURSE_SUCCESS)&&(gRobot.posY>1800.f))
								/*��ѹ��λ*/
						//		&&(gRobot.AT_motionFlag&AT_GAS_SUCCESS))||PE_FOR_THE_BALL)
    {
      /*����*/
      ShootBall();
      
      /*����ʱʹ�������ִ�е�λ*/
      Delay_ms(500);
			/*����ź���*/
			OSSemSet(PeriodSem, 0, &os_err);
      MotionCardCMDSend(NOTIFY_MOTIONCARD_SHOT_BALL2);
      
      /*���������λ*/
      ShootReset();
      
      /*׼��������*/
      PrepareGetBall(BALL_3);
      
      gRobot.process=TO_GET_BALL_3;
      gRobot.robocon2018=GOLD_BALL;
    }
		else
		{
			if(!PE_FOR_THE_BALL)
				USART_OUT_ONCE("!PE2\t");
			if(!(gRobot.AT_motionFlag&AT_HOLD_BALL1_SUCCESS))
				USART_OUT_ONCE("!HB12\t");
			if(!(gRobot.AT_motionFlag&AT_HOLD_BALL2_SUCCESS))
				USART_OUT_ONCE("!HB22\t");
			if(!(gRobot.AT_motionFlag&AT_PITCH_SUCCESS))
				USART_OUT_ONCE("!PITCH2\t");
			if(!(gRobot.AT_motionFlag&AT_COURSE_SUCCESS))
				USART_OUT_ONCE("!COURSE2\t");
			if(!(gRobot.AT_motionFlag&AT_GAS_SUCCESS))
				USART_OUT_ONCE("!GAS2\t");
			USART_Enter();
		}
    break;
  }
  
}

/*���Ͷ����������*/
void FightForGoldBall(void)
{
  CPU_INT08U  os_err;
  os_err = os_err;
	
  switch(gRobot.process)
  {
    /*ȥȡ��������*/
  case TO_GET_BALL_3:
    if(PE_FOR_THE_BALL&&gRobot.posY<1600.f)
    {	
      /*ɨ������Ϊ�˸��ȵؽӵ����������ʱ*/
      Delay_ms(500);
			/*����ź���*/
			OSSemSet(PeriodSem, 0, &os_err);
			//TalkToCamera(CAMERA_OPEN_FAR);
      
      gRobot.process=TO_THE_AREA_3;
      
      MotionCardCMDSend(NOTIFY_MOTIONCARD_GOT_BALL3);
      
      PrepareShootBall(BALL_3);
      
    }
    break;
    /*��������ȡ����ϣ�ȥͶ������*/
  case TO_THE_AREA_3:
    if(!PE_FOR_THE_BALL)
    {
      MotionCardCMDSend(NOTIFY_MOTIONCARD_LOSE_BALL3);
    }
    break;
    /*����Ͷ������������*/
  case TO_THROW_BALL_3:
		/*��絽λ*/
    if(PE_FOR_THE_BALL
				/*��������λ*/
				&&(gRobot.AT_motionFlag&AT_HOLD_BALL1_SUCCESS)
					/*��������λ*/
					&&(gRobot.AT_motionFlag&AT_HOLD_BALL2_SUCCESS)
						/*������λ��*/
						&&(gRobot.AT_motionFlag&AT_PITCH_SUCCESS)
							/*����λ*/
							&&(gRobot.AT_motionFlag&AT_COURSE_SUCCESS)&&(gRobot.posY>1800.f))
								/*��ѹ��λ*/
							//	&&(gRobot.AT_motionFlag&AT_GAS_SUCCESS))||PE_FOR_THE_BALL)
    {
      /*����*/
      ShootBall();
      
      /*����ʱʹ�������ִ�е�λ*/
      Delay_ms(500);
			/*����ź���*/
			OSSemSet(PeriodSem, 0, &os_err);
      /*���������λ*/
      ShootReset();
      
      gRobot.process=END_COMPETE;
    }
		else
		{
			if(!PE_FOR_THE_BALL)
				USART_OUT_ONCE("!PE3\t");
			if(!(gRobot.AT_motionFlag&AT_HOLD_BALL1_SUCCESS))
				USART_OUT_ONCE("!HB13\t");
			if(!(gRobot.AT_motionFlag&AT_HOLD_BALL2_SUCCESS))
				USART_OUT_ONCE("!HB23\t");
			if(!(gRobot.AT_motionFlag&AT_PITCH_SUCCESS))
				USART_OUT_ONCE("!PITCH3\t");
			if(!(gRobot.AT_motionFlag&AT_COURSE_SUCCESS))
				USART_OUT_ONCE("!COURSE3\t");
			if(!(gRobot.AT_motionFlag&AT_GAS_SUCCESS))
				USART_OUT_ONCE("!GAS3\t");
			USART_Enter();
		}
    break;
  }
}


void MotionStatus(void)
{
	
	#ifdef TEST
	/*���ض��һ��״̬*/
  USART_OUT(DEBUG_USART,"steer 1 aimAngle ");
	USART_OUT_F(gRobot.holdBallAimAngle[0]);
  USART_OUT(DEBUG_USART,"realpos ");
	USART_OUT_F(gRobot.holdBallAngle[0]);
	USART_Enter();
	
	/*���ض������״̬*/
  USART_OUT(DEBUG_USART,"steer 2 aimAngle ");
	USART_OUT_F(gRobot.holdBallAimAngle[1]);
  USART_OUT(DEBUG_USART,"realpos ");
	USART_OUT_F(gRobot.holdBallAngle[1]);
	USART_Enter();
	#else
	/*���ض��һ��״̬*/
  USART_OUT(DEBUG_USART,"steer 1 aimAngle ");
	USART_OUT_F(gRobot.holdBallAimAngle);
  USART_OUT(DEBUG_USART,"realpos ");
	USART_OUT_F(gRobot.holdBallAngle[0]);
	USART_Enter();
	
	/*���ض������״̬*/
  USART_OUT(DEBUG_USART,"steer 2 aimAngle ");
	USART_OUT_F(gRobot.holdBallAimAngle);
  USART_OUT(DEBUG_USART,"realpos ");
	USART_OUT_F(gRobot.holdBallAngle[1]);
	USART_Enter();
	#endif
	
	/*���ض������״̬*/
  USART_OUT(DEBUG_USART,"steer 3 aimAngle ");
	USART_OUT_F(gRobot.cameraAimAngle);
  USART_OUT(DEBUG_USART,"realpos ");
	USART_OUT_F(gRobot.cameraAngle);
	USART_Enter();
	
	/*���غ���ǵ�״̬*/
  USART_OUT(DEBUG_USART,"course aimAngle ");
	USART_OUT_F(gRobot.courseAimAngle);
  USART_OUT(DEBUG_USART,"realpos ");
	USART_OUT_F(gRobot.courseAngle);
	USART_Enter();
	
	/*���ظ����ǵ�״̬*/
  USART_OUT(DEBUG_USART,"course aimAngle ");
	USART_OUT_F(gRobot.pitchAimAngle);
  USART_OUT(DEBUG_USART,"realpos ");
	USART_OUT_F(gRobot.pitchAngle);
	USART_Enter();
  
  USART_OUT(DEBUG_USART,"gasValue aim ");
  USART_OUT_F(gRobot.gasAimValue);
  USART_OUT(DEBUG_USART,"\treal ");
  USART_OUT_F(gRobot.gasValue);
  USART_OUT(DEBUG_USART,"\r\n");
}

void processReponse(void)
{
  switch(gRobot.process)
  {
  case TO_START:
    USART_OUT(DEBUG_USART,"TO_START");
    USART_Enter();
    break;
  case TO_GET_BALL_1:
    USART_OUT(DEBUG_USART,"TO_GET_BALL_1");
    USART_Enter();
    break;
  case TO_THE_AREA_1:
    USART_OUT(DEBUG_USART,"TO_THE_AREA_1");
    USART_Enter();
    break;
  case TO_THROW_BALL_1:
    USART_OUT(DEBUG_USART,"TO_THROW_BALL_1");
    USART_Enter();
    break;
  case TO_GET_BALL_2:
    USART_OUT(DEBUG_USART,"TO_GET_BALL_2");
    USART_Enter();
    break;
  case TO_THE_AREA_2:
    USART_OUT(DEBUG_USART,"TO_THE_AREA_2");
    USART_Enter();
    break;
  case TO_THROW_BALL_2:
    USART_OUT(DEBUG_USART,"TO_THROW_BALL_2");
    USART_Enter();
    break;
  case TO_GET_BALL_3:
    USART_OUT(DEBUG_USART,"TO_GET_BALL_3");
    USART_Enter();
    break;
  case TO_THE_AREA_3:
    USART_OUT(DEBUG_USART,"TO_THE_AREA_3");
    USART_Enter();
    break;
  case TO_THROW_BALL_3:
    USART_OUT(DEBUG_USART,"TO_THROW_BALL_3");
    USART_Enter();
    break;
  case END_COMPETE:
    USART_OUT(DEBUG_USART,"END_COMPETE");
    USART_Enter();
    break;
  }
}



void StatusReport(void)
{
  processReponse();
  ErrorReport();
  MotionStatus();
}

/*
*�������Լ���Ƶ���ʱ���к��������ڼ�����Ƿ�����ִ�е�λ
*֮�󲻲��ô˷�ʽ
*����ÿ�����ڶ��ȷ���ת����ٷ���ȡ����ķ�ʽ
*��Ȼ���˼��Σ���һ��Ҳ��ռ̫��ʱ�䣬���ǿ���ʡ�鷳
*����ô��ο϶�����ȥ�ˣ�����������⣬�ǿ϶��Ƕ������������߶���
*/

void DelayMsRun(char task,short ms)
{
  int index=0;
  gRobot.delayTask|=task;
  while(task!=1)
  {
    task>>=1;
    index++;
  }
  gRobot.delayTaskMs[index]=ms;
}
void DelayStop(char task)
{
  int index=0;
  gRobot.delayTask&=~task;
  
  while(task!=1)
  {
    task>>=1;
    index++;
  }
  gRobot.delayTaskMs[index]=0;

}


/*״̬������*/
#define TO_START													1
#define TO_GET_BALL_1											2
#define TO_THE_AREA_1											3
#define TO_THROW_BALL_1										4
#define TO_GET_BALL_2											5
#define TO_THE_AREA_2											6
#define TO_THROW_BALL_2										7
#define TO_GET_BALL_3											8
#define TO_THE_AREA_3											9
#define TO_THROW_BALL_3									  10
#define END_COMPETE												100
extern Robot_t gRobot;
void processReport(void)
{
  static uint8_t processLast=99;
  
  if(gRobot.process==processLast)
    return;
  
  switch(gRobot.process)
  {
  case TO_START:
    USART_OUT(DEBUG_USART,"TO_START");
    USART_Enter();
    break;
  case TO_GET_BALL_1:
    USART_OUT(DEBUG_USART,"TO_GET_BALL_1");
    USART_Enter();
    break;
  case TO_THE_AREA_1:
    USART_OUT(DEBUG_USART,"TO_THE_AREA_1");
    USART_Enter();
    break;
  case TO_THROW_BALL_1:
    USART_OUT(DEBUG_USART,"TO_THROW_BALL_1");
    USART_Enter();
    break;
  case TO_GET_BALL_2:
    USART_OUT(DEBUG_USART,"TO_GET_BALL_2");
    USART_Enter();
    break;
  case TO_THE_AREA_2:
    USART_OUT(DEBUG_USART,"TO_THE_AREA_2");
    USART_Enter();
    break;
  case TO_THROW_BALL_2:
    USART_OUT(DEBUG_USART,"TO_THROW_BALL_2");
    USART_Enter();
    break;
  case TO_GET_BALL_3:
    USART_OUT(DEBUG_USART,"TO_GET_BALL_3");
    USART_Enter();
    break;
  case TO_THE_AREA_3:
    USART_OUT(DEBUG_USART,"TO_THE_AREA_3");
    USART_Enter();
    break;
  case TO_THROW_BALL_3:
    USART_OUT(DEBUG_USART,"TO_THROW_BALL_3");
    USART_Enter();
    break;
  case END_COMPETE:
    USART_OUT(DEBUG_USART,"END_COMPETE");
    USART_Enter();
    break;
  }
  processLast=gRobot.process;
}




//void DelayTaskRun(void)
//{
//  static int delayMs[DELAY_TASK_NUM]={0};
//  
//  /*�����1��λ����û�е���*/
//  if(gRobot.delayTask&DELAY_HOLD_BALL1_CHECK_POS)
//  {
//    ReadSteer1Pos();
//    delayMs[0]++;
//    if(delayMs[0]*PERIOD_COUNTER>gRobot.delayTaskMs[0])
//    {
//      DelayStop(DELAY_HOLD_BALL1_CHECK_POS);
//      //4096/360=11.377
//      if(abs(gRobot.steerAimPos[0][1]-gRobot.steerPos[0])>2*11.377f)
//      {
//        Steer1HoldBallPosCrl(gRobot.steerAimPos[0][0],2000);
//        //USART_OUT(DEBUG_USART,"DELAY_HOLD_BALL1_CHECK_POS FAIL\r\n");
//        ErrorRecord(HOLD_BALL1_ROTATE_FAIL);
//      }
//      else
//      {
//        USART_OUT(DEBUG_USART,"DELAY_HOLD_BALL1_CHECK_POS SUCCESS\r\n");
//      }
//      delayMs[0]=0;
//    }
//    else{
//      //			USART_OUT_F(abs(gRobot.steerAimPos[0][1]-gRobot.steerPos[0]));
//    }
//  }
//  
//  /*�����2��λ����û�е���*/
//  if(gRobot.delayTask&DELAY_HOLD_BALL2_CHECK_POS)
//  {
//    ReadSteer2Pos();
//    delayMs[1]++;
//    if(delayMs[1]*PERIOD_COUNTER>gRobot.delayTaskMs[1])
//    {
//      DelayStop(DELAY_HOLD_BALL2_CHECK_POS);
//      //4096/360=11.377
//      if(abs(gRobot.steerAimPos[1][1]-gRobot.steerPos[1])>2*11.377f)
//      {
//        Steer2HoldBallPosCrl(gRobot.steerAimPos[1][0],2000);
//        //USART_OUT(DEBUG_USART,"DELAY_HOLD_BALL2_CHECK_POS FAIL\r\n");
//        ErrorRecord(HOLD_BALL2_ROTATE_FAIL);
//      }
//      else
//      {
//        USART_OUT(DEBUG_USART,"DELAY_HOLD_BALL2_CHECK_POS SUCCESS\r\n");
//      }
//      delayMs[1]=0;
//    }
//    else{
//      //			USART_OUT_F(abs(gRobot.steerAimPos[1][1]-gRobot.steerPos[1]));
//      //			USART_Enter();
//    }
//  }
//}

