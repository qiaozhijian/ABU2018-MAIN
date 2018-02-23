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
#include "task.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/

/*该结构体永远是最新更新的结构体*/
DataSave_t dataSave;
extern Robot_t gRobot;

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
    if(FLASH_ProgramWord(WriteAddr,*address)!=FLASH_COMPLETE)//写入数据
    { 
      address=address;	//写入异常
    }
		/*WriteAddr是整数，要加4，address是指针只用加一*/
    WriteAddr+=MAX_SIZE;
    address++;
  }
  
  FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
  FLASH_Lock();//上锁
} 

void copyDataSave_tFromOther(DataSave_t* copy,DataSave_t* const reality)
{
	u32* address=(u32*)reality;
  u32* WriteAddr=(u32*)copy;
  u32* endaddr=WriteAddr+sizeof(DataSave_t)/MAX_SIZE;
	
  while(WriteAddr<endaddr)//写数据
  {
    *WriteAddr=*address;
    WriteAddr++;
    address++;
  }
}

void copyDataSave_tAlongAddress(DataSave_t* copy,uint32_t address)
{
  u32* WriteAddr=(u32*)copy;
  u32* endaddr=WriteAddr+sizeof(DataSave_t)/MAX_SIZE;
	
  while(WriteAddr<endaddr)//写数据
  {
    *WriteAddr=STMFLASH_ReadWord(address);
    WriteAddr++;
    address+=MAX_SIZE;
  }
}

/*把传入的机器人结构体的参数保存到“flash写入结构体”里*/
void WriteFlashData(Robot_t robot,u32 resetTime)
{
	copyDataSave_tFromOther(&dataSave,&robot.sDta);
  
  STMFLASH_Write(&dataSave,gRobot.resetTime);
}


/*读出第resetTime个结构体的值，resetTime取0 1 2 3*/
void STMFLASH_Read(DataSave_t* temp,uint32_t resetTime)   	
{
  uint32_t baseAdd=FLASH_SAVE_ADDR+resetTime*sizeof(DataSave_t);
  
	copyDataSave_tAlongAddress(temp,baseAdd);
}

/*  
**获取未开发空间的前一个结构体的序列号
**resetTime 取 0 1 2...  
*/
void FindResetTime(void)
{
  gRobot.resetTime=0;
  
  DataSave_t flashdata;
	
  STMFLASH_Read(&flashdata,gRobot.resetTime);
  
  while(flashdata.isReset!=0xffffffff)
  {
    gRobot.resetTime++;
    STMFLASH_Read(&flashdata,gRobot.resetTime);
    if(gRobot.resetTime>=500)
    {
      //USART_OUT(USART1,"extend!\r\n");
      return;
    }
    
    //printf("%f\t%f\r\n",flashdata.posx,flashdata.posy);
  }
}
/*************
softReset

第一次上电→flash为空→刷新flash（gRobot.resetTime为0）→出故障→在hardfault写flash第一个结构体（gRobot.resetTime为0，flashdata.sDta.isReset=1）
→重启→gRobot.resetTime为1→得到第0个结构体的值保存到flashdata（永远最新）→把flashdata的内容拷贝给gRobot→flashdata的isReset为0，把flash写给第gRobot.resetTime个结构体，为下次开电做准备（下次可以识别到flashdata的isReset为位0）
→如果再发生中断，在gRobot.resetTime写保存数据的结构体（此时gRobot.resetTime已加一）
************/


#ifndef TEST
void SoftWareReset(void)
{
	/*计算重启次数*/
  FindResetTime();
  
  //如果flash被写过（曾经重启过）
  if(gRobot.resetTime>0)
  {
    //得到最后一个结构体（更新dataSave）
    STMFLASH_Read(&dataSave,gRobot.resetTime-1);
  }
  
  /*上电状态  分别对应第一次下程序和以后运行得出的结果*/
  if(gRobot.resetTime==0||dataSave.isReset==0||gRobot.resetTime>=500)
  {
    STMFLASH_ERASE();
  }
  /*进了硬件中断后重启*/
  else
  {
    //将进硬件中断前的数据恢复
		copyDataSave_tFromOther(&gRobot.sDta,&dataSave);
		
    /*写一个reset=set的*/
    dataSave.isReset=0;
    //再往下一个空位写一个结构体，便于下次开电识别
    STMFLASH_Write(&dataSave,gRobot.resetTime);
		
		/*表示这次是重启程序*/
		gRobot.resetFlag=1;
  }
}
#endif


