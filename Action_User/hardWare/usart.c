#include "usart.h"
#include "math.h"
#include "misc.h"
#include "math.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include  <includes.h>
#include "stm32f4xx_usart.h"
#include "timer.h"
#include "dma.h"
#include "elmo.h"
#include "task.h"
#include "gasvalveControl.h"
#include "stdio.h"
#include "string.h"
#include "stdint.h"
#include <stdint.h>
#include "stdarg.h"
#include "shoot.h"
/**
* @brief  Retargets the C library printf function to the USART.
* @param  None
* @retval None
*/
int fputc(int ch, FILE *f)
{
  USART_SendData(UART5, (uint8_t) ch);
  
  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(UART5, USART_FLAG_TC) == RESET)
  {
    
  }
  
  return ch;
}
void Steer2Init(uint32_t BaudRate)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
  
  /* Connect PXx to USARTx_Tx*/
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
  /* Connect PXx to USARTx_Rx*/
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource2,  GPIO_AF_UART5);
  
  /* Configure USART Tx as alternate function  */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  USART_InitStructure.USART_BaudRate = BaudRate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  USART_DeInit(UART5);  //复位串口5
  /* USART configuration */
  USART_Init(UART5, & USART_InitStructure);
  
  //////////   设置UART5中断       ///////////////
  NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  NVIC_Init(&NVIC_InitStructure);
  
  USART_ClearFlag(UART5, USART_FLAG_TC);
  
  USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
  /* Enable USART */
  USART_Cmd(UART5, ENABLE);
}
void DebugBLE_Init(uint32_t BaudRate)
{
  GPIO_InitTypeDef 	GPIO_InitStructure;
  USART_InitTypeDef   USART_InitStructure;
  NVIC_InitTypeDef 	NVIC_InitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOB时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
  
  //串口3对应引脚复用映射
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOC10复用为USART1
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOC11复用为USART1
  
  //USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_9; //GPIOC10与GPIOC11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
  GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化
  
  //USART1 初始化设置
  USART_InitStructure.USART_BaudRate = BaudRate;//波特率设置
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
  USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口1
  
  USART_ClearFlag(USART1, USART_FLAG_TC);
  
  
  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//子优先级3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
  NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断
  USART_Cmd(USART1, ENABLE);  //使能串口1 	
  
}
void SteerInit(uint32_t BaudRate)
{
  GPIO_InitTypeDef 	GPIO_InitStructure;
  USART_InitTypeDef   USART_InitStructure;
  NVIC_InitTypeDef 	NVIC_InitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //使能GPIOB时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART1时钟
  
  //串口3对应引脚复用映射
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2); //GPIOC10复用为USART1
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2); //GPIOC11复用为USART1
  
  //USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6; //GPIOC10与GPIOC11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
  GPIO_Init(GPIOD,&GPIO_InitStructure); //初始化
  
  //PG8推挽输出，485模式控制  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13; //GPIOG8
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽输出
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
  GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化PG8
  
  //USART1 初始化设置
  USART_InitStructure.USART_BaudRate = BaudRate;//波特率设置
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
  USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART2, &USART_InitStructure); //初始化串口1
  
  USART_ClearFlag(USART2, USART_FLAG_TC);
  
  
  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//串口1中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//子优先级3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
  NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启相关中断
  USART_Cmd(USART2, ENABLE);  //使能串口1 	
  
  RS485_RX_EN;
}

void RS485_Send_Data(unsigned char *buf,unsigned char len)
{
  unsigned char t;
  RS485_TX_EN;																										//设置为发送模式
  t=GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13);
  t=GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_13);
  for(t=0;t<len;t++)																						//循环发送数据
  {
    while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET){}; 		//等待发送结束		
    USART_SendData(USART2,buf[t]); 																//发送数据
  }	 
  while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET); 				//等待发送结束		
  RS485_RX_EN;																										//设置为接收模式	
}

/*********************************WIFI*************************/
/**************************************************************/
//PC12:  UART5 Tx
//PD2 :  UART5 Rx
void GYRO_Init(uint32_t BaudRate)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //使能GPIOD时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//使能USART3时钟
  
  //串口3对应引脚复用映射
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3); //GPIOD8复用为USART3
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource9, GPIO_AF_USART3); //GPIOD9复用为USART3
  
  //USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9; //GPIOD8与GPIOD9
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
  GPIO_Init(GPIOD,&GPIO_InitStructure); //初始化PD8，PD9
  
  //USART1 初始化设置
  USART_InitStructure.USART_BaudRate = BaudRate;//波特率设置
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
  USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART3, &USART_InitStructure); //初始化串口3
  
  USART_Cmd(USART3, ENABLE);  //使能串口3
  
  USART_ClearFlag(USART3, USART_FLAG_TC);
  
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启相关中断
  
  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//串口3中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//子优先级1
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
  NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}


void CameraTalkInit(uint32_t BaudRate)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOD时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//使能USART6时钟
  
  //串口3对应引脚复用映射
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6); //GPIOD8复用为USART6
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource7, GPIO_AF_USART6); //GPIOD9复用为USART6
  
  //USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //GPIOD8与GPIOD9
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
  GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化PD8，PD9
  
  //USART1 初始化设置
  USART_InitStructure.USART_BaudRate = BaudRate;//波特率设置
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
  USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART6, &USART_InitStructure); //初始化串口3
  
  USART_Cmd(USART6, ENABLE);  //使能串口3
  
  USART_ClearFlag(USART6, USART_FLAG_TC);
  
  USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//开启相关中断
  
  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//串口3中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//子优先级1
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
  NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
      
}

void ControlBLE_Init(uint32_t BaudRate)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOD时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//使能UART4时钟
  
  //串口3对应引脚复用映射
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource10, GPIO_AF_UART4); //GPIOD8复用为UART4
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource11, GPIO_AF_UART4); //GPIOD9复用为UART4
  
  //USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOD8与GPIOD9
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
  GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化PD8，PD9
  
  //USART1 初始化设置
  USART_InitStructure.USART_BaudRate = BaudRate;//波特率设置
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
  USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(UART4, &USART_InitStructure); //初始化串口3
  
  USART_Cmd(UART4, ENABLE);  //使能串口3
  
  USART_ClearFlag(UART4, USART_FLAG_TC);
  
  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启相关中断
  
  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;//串口3中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//子优先级1
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
  NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}

/*使用操作系统没有设置任务堆栈8字节对齐，直接使用sprintf会一直输出0*/
void USART_OUT_F(float value)
{
  char s[9]={0};
  int integer=( int )value;
  sprintf( (char*)s, "%d.%04d\t", ( int )value, (unsigned int)((fabs(value) - abs(integer))  * 10000));
  
  USART_OUT(DEBUG_USART,s);
}
void USART_Enter(void){
  USART_OUT(DEBUG_USART,"\r\n");
}
void USART_BLE_SEND(float value)
{
  char s[9]={0};
  int integer=( int )value;
  sprintf( (char*)s, "AT+5%d.%04d\t", ( int )value, (unsigned int)((fabs(value) - abs(integer))  * 10000));
  USART_OUT(DEBUG_USART,s);
	USART_Enter();
}

/*字符串长度不能超出20个字符，调试信息内容不能超出20个*/
//这个函数可以保证每个调试语句只发一次
#define DEBUG_STRING_LENTH 		40
#define DEBUG_STRING_NUM  		40

void USART_OUT_ONCE(const char * s)
{
  static char stringArray[DEBUG_STRING_NUM][DEBUG_STRING_LENTH];
  
  static int count;
  
  static int i;
  
  /*如果调试字符串长度超出限制*/
  if(strlen(s)>DEBUG_STRING_LENTH)
  {
    USART_OUT(DEBUG_USART,"USART_OUT_ONCE LENTH error\r\n");
    return;
  }	
  
  /*如果调试条数超出限制*/
  if(count==DEBUG_STRING_NUM)
  {
    USART_OUT(DEBUG_USART,"USART_OUT_ONCE NUM error\r\n");
    return;
  }
  
  /*对已储存的字符串进行遍历*/
  for(i=0;i<count+1;i++)
  {
    /*判断是否和s相同*/
    if(strcmp(*(stringArray+i),s)==0)
    {
      //如果相同就跳出，不执行发数
      break;
    }
    /*如果已经到最后一个了，还没相同的那就添加一个新的字符串，并发送*/
    else if(i==count)
    {
      strcpy(*(stringArray+count),s);
      
      USART_OUT(DEBUG_USART,s);
      
      count++;
      break;
    }
  }
}



/****************************************************************************
* 名    称：void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...)
* 功    能：格式化串口输出函数
* 入口参数：USARTx:  指定串口
Data：   发送数组
...:     不定参数
* 出口参数：无
* 说    明：格式化串口输出函数
"\r"	回车符	   USART_OUT(USART1, "abcdefg\r")   
"\n"	换行符	   USART_OUT(USART1, "abcdefg\r\n")
"%s"	字符串	   USART_OUT(USART1, "字符串是：%s","abcdefg")
"%d"	十进制	   USART_OUT(USART1, "a=%d",10)
* 调用方法：无 
****************************************************************************/
void USART_OUT(USART_TypeDef* USARTx, const char *Data, ...)
{ 
  const char *s;
	static int aa=0;
  int d;
  char buf[16];
  va_list ap;
  va_start(ap, Data);
  
  while(*Data != 0)				                          //判断是否到达字符串结束符
  {
    if(*Data == 0x5c)									  //'\'
    {	
      switch (*++Data)
      {
      case 'r':							          //回车符
        USART_SendData(USARTx, 0x0d);	   
        
        Data++;
        break;
      case 'n':							          //换行符
        USART_SendData(USARTx, 0x0a);	
        Data++;
        break;
        
      default:
        Data++;
        break;
      }	 
    }
    else if(*Data == '%')									  //
    {
      switch (*++Data)				
      {
      case 's':										  //字符串
        s = va_arg(ap, const char *);
        for (; *s; s++) 
        {
          USART_SendData(USARTx, *s);
          
          while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET){
						aa++;
						//100这个值跟串口波特率有关，波特率小的发送时间就长一点就容易进if(aa>100)
						if(aa>100)
						{
							//USART_OUT(DEBUG_USART,"TCDEAD\r\n");
							break;
						}
					};
					aa=0;
        }
        Data++;
        break;
      case 'd':										  //十进制
        d = va_arg(ap, int);
        itoa(d, buf, 10);
        for (s = buf; *s; s++) 
        {
          USART_SendData(USARTx, *s);
					//100
          while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET){
						aa++;
						if(aa>100)
						{
							//USART_OUT(DEBUG_USART,"TCDEAD\r\n");
							break;
						}
					};
					aa=0;
        }
        Data++;
        break;
      default:
        Data++;
        break;
      }		 
    }
    else USART_SendData(USARTx, *Data++);
          while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET){
						aa++;
						if(aa>100)
						{
							//USART_OUT(DEBUG_USART,"TCDEAD\r\n");
							break;
						}
					};
			aa=0;
  }
}
/******************************************************
整形数据转字符串函数
char *itoa(int value, char *string, int radix)
radix=10 标示是10进制	非十进制，转换结果为0;  

例：d=-379;
执行	itoa(d, buf, 10); 后

buf="-379"							   			  
**********************************************************/
char *itoa(int value, char *string, int radix)
{
  uint32_t i;
  int d;
  int     flag = 0;
  char    *ptr = string;
  
  /* This implementation only works for decimal numbers. */
  if (radix != 10)
  {
    *ptr = 0;
    return string;
  }
  
  if (!value)
  {
    *ptr++ = 0x30;
    *ptr = 0;
    return string;
  }
  
  /* if this is a negative value insert the minus sign. */
  if (value < 0)
  {
    *ptr++ = '-';
    
    /* Make the value positive. */
    value *= -1;
  }
  
  for (i = 100000000; i > 0; i /= 10)
  {
    d = value / i;
    
    if (d || flag)
    {
      *ptr++ = (char)(d + 0x30);
      value -= (d * i);
      flag = 1;
    }
  }
  
  /* Null terminate the string. */
  *ptr = 0;
  
  return string;
  
} 

