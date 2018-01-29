/**
******************************************************************************
* @file     
* @author  qiaozhijian
* @version 
* @date    
* @brief   
******************************************************************************
* @attention
*
*
*`
* 
******************************************************************************
*/ 
/* Includes -------------------------------------------------------------------*/
#include "DataRecover.h"
#include "stm32f4xx_flash.h"
#include "string.h"
#include "usart.h"
#include "config.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/

/*该结构体永远是最新更新的结构体*/
DataSave_t dataSave;
extern AllPara_t allPara;

uint16_t STMFLASH_GetFlashSector(u32 addr)
{
  if(addr<ADDR_FLASH_SECTOR_1)return FLASH_Sector_0;
  else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_Sector_1;
  else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_Sector_2;
  else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_Sector_3;
  else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_Sector_4;
  else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_Sector_5;
  else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_Sector_6;
  else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_Sector_7;
  else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_Sector_8;
  else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_Sector_9;
  else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_Sector_10;
  return FLASH_Sector_11;	
}


void STMFLASH_ERASE(void)	
{ 
  FLASH_Unlock();									//解锁 
  FLASH_DataCacheCmd(DISABLE);//FLASH擦除期间,必须禁止数据缓存
  
  if(FLASH_EraseSector(STMFLASH_GetFlashSector(FLASH_SAVE_ADDR),VoltageRange_3)!=FLASH_COMPLETE) 
  {
    
  }
  FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
  FLASH_Lock();//上锁
} 


void STMFLASH_Write(DataSave_t *pBuffer,u32 resetTime)	
{ 
  
  u32* address=(u32*)pBuffer;
  u32 WriteAddr=FLASH_SAVE_ADDR+resetTime*sizeof(DataSave_t);
  u32 endaddr=WriteAddr+sizeof(DataSave_t);
  
  FLASH_Unlock();									//解锁 
  FLASH_DataCacheCmd(DISABLE);//FLASH擦除期间,必须禁止数据缓存
  
  while(WriteAddr<endaddr)//写数据
  {
    uint32_t aaa=*address;
    if(FLASH_ProgramWord(WriteAddr,*address)!=FLASH_COMPLETE)//写入数据
    { 
      address=address;	//写入异常
    }
    WriteAddr+=MAX_SIZE;
    address++;
  }
  
  FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
  FLASH_Lock();//上锁
} 
void WriteFlashData(AllPara_t *pBuffer,u32 resetTime)
{
  for(int i=0;i<3;i++)
    dataSave.GYRO_TemperatureAim[i]=pBuffer->GYRO_TemperatureAim[i];
  
  for(int i=0;i<3;i++)
    dataSave.GYRO_Bais[i]=pBuffer->GYRO_Bais[i];
  
  for(int i=0;i<3;i++)
    dataSave.Result_Angle[i]=pBuffer->Result_Angle[i];
  
  for(int i=0;i<2;i++)
    dataSave.data_last[i]=pBuffer->data_last[i];
  
  dataSave.posx=pBuffer->posx;
  dataSave.posy=pBuffer->posy;
  
  dataSave.isReset=pBuffer->isReset;
  
  STMFLASH_Write(&dataSave,allPara.resetTime);
}


//读取指定地址的半字(16位数据) 
//faddr:读地址 
//返回值:对应数据.
u32 STMFLASH_ReadWord(u32 faddr)
{
  return *(vu32*)faddr; 
}  

float STMFLASH_ReadFloat(u32 faddr)
{
  return *(volatile float*)faddr; 
}  

/*读出第resetTime个结构体的值，resetTime取0 1 2 3*/
void STMFLASH_Read(DataSave_t* temp,uint32_t resetTime)   	
{
  uint32_t baseAdd=FLASH_SAVE_ADDR+resetTime*sizeof(DataSave_t);
  
  temp->isReset=STMFLASH_ReadWord(baseAdd);
  
  temp->posx=STMFLASH_ReadFloat(baseAdd+MAX_SIZE);
  
  temp->posy=STMFLASH_ReadFloat(baseAdd+MAX_SIZE*2);
  
  (temp->GYRO_Bais)[0]=STMFLASH_ReadFloat(baseAdd+MAX_SIZE*3);
  
  (temp->GYRO_Bais)[1]=STMFLASH_ReadFloat(baseAdd+MAX_SIZE*4);
  
  (temp->GYRO_Bais)[2]=STMFLASH_ReadFloat(baseAdd+MAX_SIZE*5);
  
  (temp->Result_Angle)[0]=STMFLASH_ReadFloat(baseAdd+MAX_SIZE*6);
  
  (temp->Result_Angle)[1]=STMFLASH_ReadFloat(baseAdd+MAX_SIZE*7);
  
  (temp->Result_Angle)[2]=STMFLASH_ReadFloat(baseAdd+MAX_SIZE*8);
  
  (temp->GYRO_TemperatureAim)[0]=STMFLASH_ReadFloat(baseAdd+MAX_SIZE*9);
  
  (temp->GYRO_TemperatureAim)[1]=STMFLASH_ReadFloat(baseAdd+MAX_SIZE*10);
  
  (temp->GYRO_TemperatureAim)[2]=STMFLASH_ReadFloat(baseAdd+MAX_SIZE*11);
  
  (temp->data_last)[0]=STMFLASH_ReadWord(baseAdd+MAX_SIZE*12);
  
  (temp->data_last)[1]=STMFLASH_ReadWord(baseAdd+MAX_SIZE*13);
}

/*  
**获取未开发空间的前一个结构体的序列号
**resetTime 取 0 1 2...  
*/
void FindResetTime(void)
{
  allPara.resetTime=0;
  
  DataSave_t flashdata;
  STMFLASH_Read(&flashdata,allPara.resetTime);
  
  while(flashdata.isReset!=0xffffffff)
  {
    allPara.resetTime++;
    STMFLASH_Read(&flashdata,allPara.resetTime);
    if(allPara.resetTime>=500)
    {
      //USART_OUT(USART1,"extend!\r\n");
      return;
    }
    
    //printf("%f\t%f\r\n",flashdata.posx,flashdata.posy);
  }
}
/*************

softReset

第一次上电→flash为空→刷新flash（allPara.resetTime为0）→出故障→在hardfault写flash第一个结构体（allPara.resetTime为0，flashdata.isReset=1）
→重启→allPara.resetTime为1→得到第0个结构体的值保存到flashdata（永远最新）→把flashdata的内容拷贝给allPara→flashdata的isReset为0，把flash写给第allPara.resetTime个结构体，为下次开电做准备（下次可以识别到flashdata的isReset为位0）
→如果再发生中断，在allPara.resetTime写保存数据的结构体（此时allPara.resetTime已加一）


************/


void SoftWareReset(void)
{
  FindResetTime();
  
  //如果flash被写过
  if(allPara.resetTime>0)
  {
    //得到最后一个结构体
    STMFLASH_Read(&dataSave,allPara.resetTime-1);
  }
  
  /*上电状态  分别对应第一次下程序和以后运行得出的结果*/
  if(allPara.resetTime==0||dataSave.isReset==0||allPara.resetTime>=500)
  {
    STMFLASH_ERASE();
  }
  /*进了硬件中断后重启*/
  else
  {
    //将进硬件中断前的数据恢复
    {
      for(int i=0;i<3;i++)
        allPara.GYRO_TemperatureAim[i]=dataSave.GYRO_TemperatureAim[i];
      
      for(int i=0;i<3;i++)
        allPara.GYRO_Bais[i]=(double)(dataSave.GYRO_Bais[i]);
      
      for(int i=0;i<3;i++)
        allPara.Result_Angle[i]=(double)(dataSave.Result_Angle[i]);
      
      for(int i=0;i<2;i++)
        allPara.data_last[i]=dataSave.data_last[i];
      
      allPara.posx=(double)(dataSave.posx);
      allPara.posy=(double)(dataSave.posy);
    }
    /*写一个reset=set的*/
    dataSave.isReset=0;
    //再往下一个空位写一个结构体
    STMFLASH_Write(&dataSave,allPara.resetTime);
		
		allPara.resetFlag=1;
  }
}



