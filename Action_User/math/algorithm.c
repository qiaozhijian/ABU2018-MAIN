#include "algorithm.h"
#include "stdint.h"
#include "usart.h"

/*��ʱ�󣬲����ˣ�*/
#define THRESHOLD_TEMP 		 0.01f
#define THRESHOLD_COUNT  	 100
#define STD_VALUE					 0.2f
float LowPassFilter(float newValue)
{
	static uint8_t Dr_flagLst = { 0 };
	static uint8_t Dr_flag = { 0 };
	static uint8_t F_count = { 0 };
	static float coeff = { 0.f };
	static float valueLast = { 0.f };
	float diffValue = { 0.f };
	if(valueLast>newValue)
	{
		diffValue = valueLast - newValue;
    Dr_flag = 0;
	}
	else
	{
    diffValue = newValue - valueLast;
    Dr_flag = 1;
	}		
  if (!(Dr_flag^Dr_flagLst))    //ǰ�����ݱ仯����һ��
	{
    F_count=F_count+2;
    if (diffValue >= THRESHOLD_TEMP)
		{
			F_count=F_count+4;
		}
    if (F_count >= THRESHOLD_COUNT)
      F_count = THRESHOLD_COUNT;
    coeff = STD_VALUE * F_count;
	}
  else{
    coeff = STD_VALUE;
    F_count = 0;
  }
  //һ���˲��㷨
  if (Dr_flag == 0)     //��ǰֵС��ǰһ��ֵ
    newValue = valueLast - coeff*(valueLast - newValue) / 256;
  else
		newValue = valueLast + coeff*(newValue - valueLast) / 256;
	
  valueLast=newValue;
  Dr_flagLst = Dr_flag;
	
	return newValue;
}


/*�㷨�� H,��,����Ϊһ*/
float KalmanFilter(float measureData)
{
  static float act_value=0;  //ʵ��ֵ
  float predict;             //Ԥ��ֵ
  
  static float P_last=0.0001;   //��һ�ε�Ԥ�����
  static float P_mid;        //��Ԥ������Ԥ��
  static float Kk;           //�˲�����ϵ��
  
  static float Q=0.003;       //ϵͳ����        
  static float R=20.f;      //�������� 
 
  static char ignore=0;
  static float sum=0.0;
  ignore++;
  if(ignore<11){
    sum+=measureData;
    return measureData;
  }
  else if(ignore==11){
    act_value=sum/9.f;
  }else
    ignore=12;
	
  //��Ԥ��ֵΪ��һ�ε���ʵֵ
  predict=act_value;
  
  /* Ԥ�Ȿ�ε�Ԥ����� */
  P_mid=P_last+Q;
  
  /* ����ϵ����������ֵ */
  Kk=P_mid/(P_mid+R);
  
  act_value=predict+Kk*(measureData-predict);
  
  /* ����Ԥ����� */
  P_last=(1-Kk)*P_mid;
	
  return act_value;
}
