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

//FLASH��ʼ��ַ
#define STM32_FLASH_BASE 					0x08000000 	//STM32 FLASH����ʼ��ַ
#define FLASH_SAVE_ADDR 					0x08080000 	//����FLASH �����ַ(����Ϊż��������������,Ҫ���ڱ�������ռ�õ�������.//����,д������ʱ��,���ܻᵼ�²�����������,�Ӷ����𲿷ֳ���ʧ.��������.
#define MAX_SIZE									4


//FLASH ��������ʼ��ַ
#define ADDR_FLASH_SECTOR_0     ((u32)0x08000000) 	//����0��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_1     ((u32)0x08004000) 	//����1��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_2     ((u32)0x08008000) 	//����2��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_3     ((u32)0x0800C000) 	//����3��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_4     ((u32)0x08010000) 	//����4��ʼ��ַ, 64 Kbytes  
#define ADDR_FLASH_SECTOR_5     ((u32)0x08020000) 	//����5��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_6     ((u32)0x08040000) 	//����6��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_7     ((u32)0x08060000) 	//����7��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_8     ((u32)0x08080000) 	//����8��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_9     ((u32)0x080A0000) 	//����9��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_10    ((u32)0x080C0000) 	//����10��ʼ��ַ,128 Kbytes  
#define ADDR_FLASH_SECTOR_11    ((u32)0x080E0000) 	//����11��ʼ��ַ,128 Kbytes  

typedef struct{
  
  uint32_t isReset;
  
  float posx;
  
  float posy;
  
  /*�����Ǵ���������*/
  float GYRO_Bais[3];
  
  /*����ȷ��������Ƕ�*/
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

//FLASH��ʼ��ַ
#define STM32_FLASH_BASE 					0x08000000 	//STM32 FLASH����ʼ��ַ
#define FLASH_SAVE_ADDR 					0x08080000 	//����FLASH �����ַ(����Ϊż��������������,Ҫ���ڱ�������ռ�õ�������.//����,д������ʱ��,���ܻᵼ�²�����������,�Ӷ����𲿷ֳ���ʧ.��������.
#define MAX_SIZE									4


//FLASH ��������ʼ��ַ
#define ADDR_FLASH_SECTOR_0     ((u32)0x08000000) 	//����0��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_1     ((u32)0x08004000) 	//����1��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_2     ((u32)0x08008000) 	//����2��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_3     ((u32)0x0800C000) 	//����3��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_4     ((u32)0x08010000) 	//����4��ʼ��ַ, 64 Kbytes  
#define ADDR_FLASH_SECTOR_5     ((u32)0x08020000) 	//����5��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_6     ((u32)0x08040000) 	//����6��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_7     ((u32)0x08060000) 	//����7��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_8     ((u32)0x08080000) 	//����8��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_9     ((u32)0x080A0000) 	//����9��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_10    ((u32)0x080C0000) 	//����10��ʼ��ַ,128 Kbytes  
#define ADDR_FLASH_SECTOR_11    ((u32)0x080E0000) 	//����11��ʼ��ַ,128 Kbytes  

typedef struct{
  
  uint32_t isReset;
  
  float posx;
  
  float posy;
  
  /*�����Ǵ���������*/
  float GYRO_Bais[3];
  
  /*����ȷ��������Ƕ�*/
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
