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

/*�ýṹ����Զ�����¸��µĽṹ��*/
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


void STMFLASH_ERASE(void)	
{ 
  FLASH_Unlock();									//���� 
  FLASH_DataCacheCmd(DISABLE);//FLASH�����ڼ�,�����ֹ���ݻ���
  
  if(FLASH_EraseSector(STMFLASH_GetFlashSector(FLASH_SAVE_ADDR),VoltageRange_3)!=FLASH_COMPLETE) 
  {
    
  }
  FLASH_DataCacheCmd(ENABLE);	//FLASH��������,�������ݻ���
  FLASH_Lock();//����
} 


void STMFLASH_Write(DataSave_t *pBuffer,u32 resetTime)	
{ 
  
  u32* address=(u32*)pBuffer;
  u32 WriteAddr=FLASH_SAVE_ADDR+resetTime*sizeof(DataSave_t);
  u32 endaddr=WriteAddr+sizeof(DataSave_t);
  
  FLASH_Unlock();									//���� 
  FLASH_DataCacheCmd(DISABLE);//FLASH�����ڼ�,�����ֹ���ݻ���
  
  while(WriteAddr<endaddr)//д����
  {
    if(FLASH_ProgramWord(WriteAddr,*address)!=FLASH_COMPLETE)//д������
    { 
      address=address;	//д���쳣
    }
    WriteAddr+=MAX_SIZE;
    address++;
  }
  
  FLASH_DataCacheCmd(ENABLE);	//FLASH��������,�������ݻ���
  FLASH_Lock();//����
} 

/*�Ѵ���Ļ����˽ṹ��Ĳ������浽��flashд��ṹ�塱��*/
void WriteFlashData(Robot_t *pBuffer,u32 resetTime)
{
  STMFLASH_Write(&dataSave,gRobot.resetTime);
}


//��ȡָ����ַ�İ���(16λ����) 
//faddr:����ַ 
//����ֵ:��Ӧ����.
u32 STMFLASH_ReadWord(u32 faddr)
{
  return *(vu32*)faddr; 
}  

float STMFLASH_ReadFloat(u32 faddr)
{
  return *(volatile float*)faddr; 
}  


/*������resetTime���ṹ���ֵ��resetTimeȡ0 1 2 3*/
void STMFLASH_Read(DataSave_t* temp,uint32_t resetTime)   	
{
  uint32_t baseAdd=FLASH_SAVE_ADDR+resetTime*sizeof(DataSave_t);
  
//  uint32_t isReset;
  temp->isReset=STMFLASH_ReadWord(baseAdd);
  
//  uint32_t AT_motionFlag; 
  temp->AT_motionFlag=STMFLASH_ReadWord(baseAdd+MAX_SIZE);
  
//  uint32_t process;
  temp->process=STMFLASH_ReadWord(baseAdd+MAX_SIZE*2);
  
//  uint32_t robocon2018;
  temp->robocon2018=STMFLASH_ReadWord(baseAdd+MAX_SIZE*3);
  
//  float holdBallAimAngle;
  temp->holdBallAimAngle=STMFLASH_ReadFloat(baseAdd+MAX_SIZE*4);
  
//  float cameraAimAngle;
  temp->cameraAimAngle=STMFLASH_ReadFloat(baseAdd+MAX_SIZE*5);
  
//  float courseAimAngle;
  temp->courseAimAngle=STMFLASH_ReadFloat(baseAdd+MAX_SIZE*6);
  
//  float pitchAimAngle;
  temp->pitchAimAngle=STMFLASH_ReadFloat(baseAdd+MAX_SIZE*7);
  
//  float gasAimValue;
  temp->gasAimValue=STMFLASH_ReadFloat(baseAdd+MAX_SIZE*8);
  
//	uint32_t isOpenGasReturn;
  temp->isOpenGasReturn=STMFLASH_ReadWord(baseAdd+MAX_SIZE*9);
	
//  uint32_t errorTime;
	temp->errorTime=STMFLASH_ReadWord(baseAdd+MAX_SIZE*10);
  
//  uint32_t error[ERROR_TIME][2];
	for(int i=0;i<ERROR_TIME;i++)
	{
		(temp->error)[i][0]=STMFLASH_ReadWord(baseAdd+MAX_SIZE*11+i*2);
		(temp->error)[i][1]=STMFLASH_ReadWord(baseAdd+MAX_SIZE*11+i*2+1);
	}
	
}

/*  
**��ȡδ�����ռ��ǰһ���ṹ������к�
**resetTime ȡ 0 1 2...  
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

��һ���ϵ��flashΪ�ա�ˢ��flash��gRobot.resetTimeΪ0���������ϡ���hardfaultдflash��һ���ṹ�壨gRobot.resetTimeΪ0��flashdata.isReset=1��
��������gRobot.resetTimeΪ1���õ���0���ṹ���ֵ���浽flashdata����Զ���£�����flashdata�����ݿ�����gRobot��flashdata��isResetΪ0����flashд����gRobot.resetTime���ṹ�壬Ϊ�´ο�����׼�����´ο���ʶ��flashdata��isResetΪλ0��
������ٷ����жϣ���gRobot.resetTimeд�������ݵĽṹ�壨��ʱgRobot.resetTime�Ѽ�һ��
************/


#ifndef TEST
void SoftWareReset(void)
{
  FindResetTime();
  
  //���flash��д��
  if(gRobot.resetTime>0)
  {
    //�õ����һ���ṹ��
    STMFLASH_Read(&dataSave,gRobot.resetTime-1);
  }
  
  /*�ϵ�״̬  �ֱ��Ӧ��һ���³�����Ժ����еó��Ľ��*/
  if(gRobot.resetTime==0||dataSave.isReset==0||gRobot.resetTime>=500)
  {
    STMFLASH_ERASE();
  }
  /*����Ӳ���жϺ�����*/
  else
  {
    //����Ӳ���ж�ǰ�����ݻָ�
    {
			gRobot.AT_motionFlag=dataSave.AT_motionFlag;
			
			gRobot.process=dataSave.process;
			
			gRobot.robocon2018=dataSave.robocon2018;
			
			gRobot.holdBallAimAngle=dataSave.holdBallAimAngle;
			
			gRobot.cameraAimAngle=dataSave.cameraAimAngle;
			
			gRobot.courseAimAngle=dataSave.courseAimAngle;
			
			gRobot.pitchAimAngle=dataSave.pitchAimAngle;
			
			gRobot.gasAimValue=dataSave.gasAimValue;
			
			gRobot.isOpenGasReturn=dataSave.isOpenGasReturn;
			
			gRobot.errorTime=dataSave.errorTime;
			
			for(int i=0;i<ERROR_TIME;i++)
			{
				gRobot.error[i][0]=dataSave.error[i][0];
				gRobot.error[i][1]=dataSave.error[i][1];
			}
    }
    /*дһ��reset=set��*/
    dataSave.isReset=0;
    //������һ����λдһ���ṹ��
    STMFLASH_Write(&dataSave,gRobot.resetTime);
		
		gRobot.resetFlag=1;
  }
}
#endif


