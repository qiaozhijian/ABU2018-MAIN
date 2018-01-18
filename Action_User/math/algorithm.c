#include "algorithm.h"
#include "stdint.h"
#include "usart.h"

/*延时大，不用了！*/
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
  if (!(Dr_flag^Dr_flagLst))    //前后数据变化方向一致
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
  //一阶滤波算法
  if (Dr_flag == 0)     //当前值小于前一个值
    newValue = valueLast - coeff*(valueLast - newValue) / 256;
  else
		newValue = valueLast + coeff*(newValue - valueLast) / 256;
	
  valueLast=newValue;
  Dr_flagLst = Dr_flag;
	
	return newValue;
}


/*算法中 H,φ,Γ均为一*/
float KalmanFilter(float measureData)
{
  static float act_value=0;  //实际值
  float predict;             //预测值
  
  static float P_last=0.0001;   //上一次的预测误差
  static float P_mid;        //对预测误差的预测
  static float Kk;           //滤波增益系数
  
  static float Q=0.003;       //系统噪声        
  static float R=20.f;      //测量噪声 
 
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
	
  //令预测值为上一次的真实值
  predict=act_value;
  
  /* 预测本次的预测误差 */
  P_mid=P_last+Q;
  
  /* 计算系数，求得输出值 */
  Kk=P_mid/(P_mid+R);
  
  act_value=predict+Kk*(measureData-predict);
  
  /* 更新预测误差 */
  P_last=(1-Kk)*P_mid;
	
  return act_value;
}
