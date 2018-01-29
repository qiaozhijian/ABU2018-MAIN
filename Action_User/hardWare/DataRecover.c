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

/*�ýṹ����Զ�����¸��µĽṹ��*/
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
    uint32_t aaa=*address;
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
**��ȡδ�����ռ��ǰһ���ṹ������к�
**resetTime ȡ 0 1 2...  
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

��һ���ϵ��flashΪ�ա�ˢ��flash��allPara.resetTimeΪ0���������ϡ���hardfaultдflash��һ���ṹ�壨allPara.resetTimeΪ0��flashdata.isReset=1��
��������allPara.resetTimeΪ1���õ���0���ṹ���ֵ���浽flashdata����Զ���£�����flashdata�����ݿ�����allPara��flashdata��isResetΪ0����flashд����allPara.resetTime���ṹ�壬Ϊ�´ο�����׼�����´ο���ʶ��flashdata��isResetΪλ0��
������ٷ����жϣ���allPara.resetTimeд�������ݵĽṹ�壨��ʱallPara.resetTime�Ѽ�һ��


************/


void SoftWareReset(void)
{
  FindResetTime();
  
  //���flash��д��
  if(allPara.resetTime>0)
  {
    //�õ����һ���ṹ��
    STMFLASH_Read(&dataSave,allPara.resetTime-1);
  }
  
  /*�ϵ�״̬  �ֱ��Ӧ��һ���³�����Ժ����еó��Ľ��*/
  if(allPara.resetTime==0||dataSave.isReset==0||allPara.resetTime>=500)
  {
    STMFLASH_ERASE();
  }
  /*����Ӳ���жϺ�����*/
  else
  {
    //����Ӳ���ж�ǰ�����ݻָ�
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
    /*дһ��reset=set��*/
    dataSave.isReset=0;
    //������һ����λдһ���ṹ��
    STMFLASH_Write(&dataSave,allPara.resetTime);
		
		allPara.resetFlag=1;
  }
}



