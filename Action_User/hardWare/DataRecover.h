/**
******************************************************************************
* @file    *.h
* @author  Qzj Action
* @version 
* @date   
* @brief   This file contains the headers of 
******************************************************************************
* @attention
*
*
* 
* 
*
******************************************************************************
*/ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DATA_RECOVERY_H
#define __DATA_RECOVERY_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
//#include "config.h"

//FLASH起始地址
#define STM32_FLASH_BASE 					0x08000000 	//STM32 FLASH的起始地址
#define FLASH_SAVE_ADDR 					0x08080000 	//设置FLASH 保存地址(必须为偶数，且所在扇区,要大于本代码所占用到的扇区.//否则,写操作的时候,可能会导致擦除整个扇区,从而引起部分程序丢失.引起死机.
#define MAX_SIZE									4


//FLASH 扇区的起始地址
#define ADDR_FLASH_SECTOR_0     ((u32)0x08000000) 	//扇区0起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_1     ((u32)0x08004000) 	//扇区1起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_2     ((u32)0x08008000) 	//扇区2起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_3     ((u32)0x0800C000) 	//扇区3起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_4     ((u32)0x08010000) 	//扇区4起始地址, 64 Kbytes  
#define ADDR_FLASH_SECTOR_5     ((u32)0x08020000) 	//扇区5起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_6     ((u32)0x08040000) 	//扇区6起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_7     ((u32)0x08060000) 	//扇区7起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_8     ((u32)0x08080000) 	//扇区8起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_9     ((u32)0x080A0000) 	//扇区9起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_10    ((u32)0x080C0000) 	//扇区10起始地址,128 Kbytes  
#define ADDR_FLASH_SECTOR_11    ((u32)0x080E0000) 	//扇区11起始地址,128 Kbytes  

typedef struct{
  
  uint32_t isReset;
  
  float posx;
  
  float posy;
  
  /*陀螺仪处理后的数据*/
  float GYRO_Bais[3];
  
  /*最终确定的三轴角度*/
  float Result_Angle[3];
  
  float GYRO_TemperatureAim[3];
  
  uint32_t data_last[2];
  
}DataSave_t;


void STMFLASH_ERASE(void);
void WriteFlashData(AllPara_t *pBuffer,unsigned int resetTime);
void SoftWareReset(void);
void STMFLASH_Read(DataSave_t* flashdata,uint32_t resetTime) ;
void FindResetTime(void);
#endif

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/
/**
******************************************************************************
* @file    *.h
* @author  Qzj Action
* @version 
* @date   
* @brief   This file contains the headers of 
******************************************************************************
* @attention
*
*
* 
* 
*
******************************************************************************
*/ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DATA_RECOVERY_H
#define __DATA_RECOVERY_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "config.h"

//FLASH起始地址
#define STM32_FLASH_BASE 					0x08000000 	//STM32 FLASH的起始地址
#define FLASH_SAVE_ADDR 					0x08080000 	//设置FLASH 保存地址(必须为偶数，且所在扇区,要大于本代码所占用到的扇区.//否则,写操作的时候,可能会导致擦除整个扇区,从而引起部分程序丢失.引起死机.
#define MAX_SIZE									4


//FLASH 扇区的起始地址
#define ADDR_FLASH_SECTOR_0     ((u32)0x08000000) 	//扇区0起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_1     ((u32)0x08004000) 	//扇区1起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_2     ((u32)0x08008000) 	//扇区2起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_3     ((u32)0x0800C000) 	//扇区3起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_4     ((u32)0x08010000) 	//扇区4起始地址, 64 Kbytes  
#define ADDR_FLASH_SECTOR_5     ((u32)0x08020000) 	//扇区5起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_6     ((u32)0x08040000) 	//扇区6起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_7     ((u32)0x08060000) 	//扇区7起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_8     ((u32)0x08080000) 	//扇区8起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_9     ((u32)0x080A0000) 	//扇区9起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_10    ((u32)0x080C0000) 	//扇区10起始地址,128 Kbytes  
#define ADDR_FLASH_SECTOR_11    ((u32)0x080E0000) 	//扇区11起始地址,128 Kbytes  

typedef struct{
  
  uint32_t isReset;
  
  float posx;
  
  float posy;
  
  /*陀螺仪处理后的数据*/
  float GYRO_Bais[3];
  
  /*最终确定的三轴角度*/
  float Result_Angle[3];
  
  float GYRO_TemperatureAim[3];
  
  uint32_t data_last[2];
  
}DataSave_t;


void STMFLASH_ERASE(void);
void WriteFlashData(AllPara_t *pBuffer,unsigned int resetTime);
void SoftWareReset(void);
void STMFLASH_Read(DataSave_t* flashdata,uint32_t resetTime) ;
void FindResetTime(void);
#endif

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/
