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
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIO时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //使能ADC时钟

  /*配置GPIO端口号及其相关信息*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入模式，而不是复用
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
 
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1 复位
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//复位结束
 
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//两个采样阶段之间的延迟周期数
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;// DMA 模式禁止
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//84MHz4分频，不得超过36MHz
  ADC_CommonInit(&ADC_CommonInitStructure);
	
	
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;/*设置 ADC 的分辨率，12位的分辨率对应15个ADCCLK周期*/
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;/*关闭扫描模式。在扫描模式下，由 ADC_SQRx 或 ADC_JSQRx 寄存器选中的通道被转换。如果设置了 EOCIE 或 JEOCIE，只在最后一个通道转换完毕后才会产生 EOC 或 JEOC 中断*/
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;/*不进行连续转换，我们使用单次转换*/
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
	ADC_InitStructure.ADC_ExternalTrigConv= ADC_ExternalTrigConv_T1_CC1;//设置外部出发源 该句可能因为上句而失去了意义
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;/*设置为右对齐模式*/
  ADC_InitStructure.ADC_NbrOfConversion = 1;//单词转换配置为1
  ADC_Init(ADC1, &ADC_InitStructure);
	
 
	ADC_Cmd(ADC1, ENABLE);//开启 AD 转换器

}				  


u16 Get_Adc(u8 channel)   
{
	int times=0;
	/*使用规则序列中的第 1 个转换，采样时间为480个周期，耗时23.42us（（480+12）/21）*/
	ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_480Cycles );
  //使能指定的 ADC1 的软件转换启动功能
	ADC_SoftwareStartConv(ADC1);		
	//等待转换结束
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ))
	{
		times++;
		if(times>300)
		{
			USART_OUT(DEBUG_USART,"ADC dead while\r\n");
			
			break;
		}
	}
	//获取转换 ADC 转换结果数据
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
	 

