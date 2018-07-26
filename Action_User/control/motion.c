#include "task.h"
#include "steer.h"
#include "can.h"
#include "shoot.h"
#include "stm32f4xx_it.h"
#include "timer.h"
#include "iwdg.h"
#include "robot.h"
#include "motion.h"
#include "adc.h"
extern Robot_t gRobot;

void PitchAngleMotion(float angle)
{
  /*���Ƶ�Ϊ-20��-10��ʵ�ʸ��������30��-0��*/
  if(angle>30.f)//????????????????????????????????
    angle=0.f;//?????????????????????????
  else if(angle<-10.f)
    angle=-10.f;
	
	angle=PITCH_COMPENSATE - angle;
	
  PosCrl(CAN2, PITCH_MOTOR_ID,ABSOLUTE_MODE,PITCH_ANGLE_TO_CODE(angle));
}

void CourseAngleMotion(float angle)
{
  if(angle<0.f)
    angle=0.f;
  else if(angle>COURCE_COMPENSATE&&angle<210.f)
    angle=190.f;
	else if(angle>=210.f&&angle<=260.f)
    angle=260.f;
	else if(angle>360.f)
    angle=350.f;
	
	if(angle-COURCE_COMPENSATE<0.f){
		angle=angle-COURCE_COMPENSATE;
	}else {
	  angle=(angle-COURCE_COMPENSATE)-360.f ;
	}		
	
	
  PosCrl(CAN2, COURCE_MOTOR_ID,ABSOLUTE_MODE,COURSE_ANGLE_TO_CODE(angle));
}

void GasMotion(float value)
{
	#ifndef  GAS_CONTOL_BY_PWM
	uint32_t data[2]={0x00005045,0x00000000};
	union
	{
		float dataFloat;
		uint32_t data32;
	}sendData;
	
	sendData.dataFloat = value;
	
	data[1] = sendData.data32;
  CAN_TxMsg(CAN2,SEND_TO_GASSENSOR,(uint8_t*)(&data),8);
	#else
  GasControlByPWM(value);
	#endif
}

void GasEnable(void)
{
	#ifndef  GAS_CONTOL_BY_PWM
	uint32_t data[2]={0x00004145,1};
  CAN_TxMsg(CAN2,SEND_TO_GASSENSOR,(uint8_t*)(&data),8);	
	#endif
}

void GasDisable(void){
	#ifndef  GAS_CONTOL_BY_PWM
	uint32_t data[2]={0x00008368,1};
	CAN_TxMsg(CAN2,SEND_TO_GASSENSOR,(uint8_t*)(&data),8);
	#endif
}


//�������
void GasIF(void){
	
	uint32_t send[2]={0x00004649,1};
	
	CAN_TxMsg(CAN2,SEND_TO_GASSENSOR,(uint8_t*)(&send),8);	

}




/* ����ִ�к���
* 
* ����Ķ��һ������
*	��������ͷת��Ķ����
*
*/

void MotionExecute(void)
{
	/*���������û�е�λ*/
	if(!(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_1_SUCCESS)
		||!(gRobot.sDta.AT_motionFlag&AT_HOLD_BALL_2_SUCCESS))
	{
		HoldBallPosCrlSeparate(gRobot.sDta.holdBallAimAngle[0],gRobot.sDta.holdBallAimAngle[1]);
	}
	
	if(!(gRobot.sDta.AT_motionFlag&AT_COURSE_SUCCESS))
	{
		CourseAngleMotion(gRobot.sDta.courseAimAngle);
	}
	
	if(!(gRobot.sDta.AT_motionFlag&AT_PITCH_SUCCESS))
	{
		PitchAngleMotion(gRobot.sDta.pitchAimAngle);
	}
	
	if(!(gRobot.sDta.AT_motionFlag&AT_GAS_SUCCESS))
	{
		GasMotion(gRobot.sDta.gasAimValue);
	}
	
}


/* ����״̬��ѯ����
* 
* ����Ķ��һ������
*	��������ͷת��Ķ����
*	���Ʒ���ܺ���ǵĵ��
* ���Ʒ���ܸ����ǵĵ��
* ������ѹ��������
*
*/
void MotionRead(void)
{
	/*��ȡ��ѹֵ��adc��*/
	#ifdef  GAS_CONTOL_BY_PWM
	GasRead();
	#endif
	/*��ȡ������*/
	ReadActualPos(CAN2,PITCH_MOTOR_ID);
  /*������������̬�ı�־λ��0*/
	SetMotionFlag(AT_PITCH_READ);
	/*��ȡ����ǽ�*//*�������ٶ�ʱ����մ����ʱ��ʼ��ʱ*/
	ReadActualPos(CAN2,COURCE_MOTOR_ID);
	/*�����������̬�ı�־λ��0*/
	SetMotionFlag(AT_COURSE_READ);
	/*��ȡ�ϵ������ǽ�*/
	ReadActualPos(CAN2,UP_STEER_MOTOR_ID);	
	/*��ȡ�ϵ����־λ��0*/
	SetMotionFlag(AT_HOLD_BALL_1_RESPONSE);
	/*��ȡ�µ������ǽ�*/
	ReadActualPos(CAN2,DOWN_STEER_MOTOR_ID);
	/*��ȡ�ϵ����־λ��0*/
	SetMotionFlag(AT_HOLD_BALL_2_RESPONSE);
}


/* ����״̬���º���
* 
* ����Ķ��һ������
*	��������ͷת��Ķ����
*	���Ʒ���ܺ���ǵĵ��
* ���Ʒ���ܸ����ǵĵ��
* ������ѹ��������
*
*/
void MotionStatusUpdate(void)
{
	/*�жϺ�����Ƿ�λ*/
	if(fabs(gRobot.sDta.courseAimAngle-gRobot.courseAngle)<0.1f)
	{
		SetMotionFlag(AT_COURSE_SUCCESS);
	}
	else
	{
		SetMotionFlag(~AT_COURSE_SUCCESS);
	}
	
	/*�жϸ������Ƿ�λ*/
	if(fabs(gRobot.sDta.pitchAimAngle-gRobot.pitchAngle)<0.1f)
	{
		SetMotionFlag(AT_PITCH_SUCCESS);
	}
	else
	{
		SetMotionFlag(~AT_PITCH_SUCCESS);
	}
	
	
	//��ѹ�жϱ߽�
	float gasBoundary=0.005f;//������ѹ�߽�
	if(gRobot.sDta.WhichGoldBall!=0){
		gasBoundary=0.003f;//������ѹ�߽�
	}
	if(fabs(gRobot.sDta.gasAimValue-gRobot.gasValue)< gasBoundary)
	{
		SetMotionFlag(AT_GAS_SUCCESS);
	}
	else
	{
		SetMotionFlag(~AT_GAS_SUCCESS);
	}

	
	
	/*�жϳ�����һ�Ƿ�λ*/
	if(fabs(gRobot.sDta.holdBallAimAngle[0]-gRobot.holdBallAngle[0])<1.5f)
	{
		SetMotionFlag(AT_HOLD_BALL_1_SUCCESS);
	}	
	else
	{
		SetMotionFlag(~AT_HOLD_BALL_1_SUCCESS);
	}
	
	/*�жϳ��������Ƿ�λ*/
	if(fabs(gRobot.sDta.holdBallAimAngle[1]-gRobot.holdBallAngle[1])<1.5f)
	{
		SetMotionFlag(AT_HOLD_BALL_2_SUCCESS);	
	}
	else
	{
		SetMotionFlag(~AT_HOLD_BALL_2_SUCCESS);
	}
	
	/*�ȵ�����������̵�ʱ�����һ�μ���΢������*/
  SmallChange();
	
}


void GasRead(void){
  int adc = Get_Adc_Average(10,15);
  gRobot.gasValue=((float)(adc-370))*2.f/4095.f*1.145f;
}

