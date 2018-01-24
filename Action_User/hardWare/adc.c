#include "adc.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
#include "timer.h"
#include "usart.h"
									   
void  Laser_Init(void)
{    
  GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef   ADC_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//ʹ��GPIOʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //ʹ��ADCʱ��

  /*����GPIO�˿ںż��������Ϣ*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//ģ������ģʽ�������Ǹ���
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
 
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1 ��λ
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//��λ����
 
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//����ģʽ
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//���������׶�֮����ӳ�������
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;// DMA ģʽ��ֹ
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//84MHz4��Ƶ�����ó���36MHz
  ADC_CommonInit(&ADC_CommonInitStructure);
	
	
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;/*���� ADC �ķֱ��ʣ�12λ�ķֱ��ʶ�Ӧ15��ADCCLK����*/
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;/*�ر�ɨ��ģʽ����ɨ��ģʽ�£��� ADC_SQRx �� ADC_JSQRx �Ĵ���ѡ�е�ͨ����ת������������� EOCIE �� JEOCIE��ֻ�����һ��ͨ��ת����Ϻ�Ż���� EOC �� JEOC �ж�*/
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;/*����������ת��������ʹ�õ���ת��*/
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//��ֹ������⣬ʹ���������
	ADC_InitStructure.ADC_ExternalTrigConv= ADC_ExternalTrigConv_T1_CC1;//�����ⲿ����Դ �þ������Ϊ�Ͼ��ʧȥ������
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;/*����Ϊ�Ҷ���ģʽ*/
  ADC_InitStructure.ADC_NbrOfConversion = 1;//����ת������Ϊ1
  ADC_Init(ADC1, &ADC_InitStructure);
	
 
	ADC_Cmd(ADC1, ENABLE);//���� AD ת����

}				  


u16 Get_Adc(u8 channel)   
{
	int times=0;
	/*ʹ�ù��������еĵ� 1 ��ת��������ʱ��Ϊ480�����ڣ���ʱ23.42us����480+12��/21��*/
	ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_480Cycles );
  //ʹ��ָ���� ADC1 �����ת����������
	ADC_SoftwareStartConv(ADC1);		
	//�ȴ�ת������
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ))
	{
		times++;
		if(times>300)
		{
			USART_OUT(DEBUG_USART,"ADC dead while\r\n");
			
			break;
		}
	}
	//��ȡת�� ADC ת���������
	return ADC_GetConversionValue(ADC1);	
}



u16 Get_Adc_Average(u8 channel,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(channel);
	}
	return temp_val/times;
} 
	 

