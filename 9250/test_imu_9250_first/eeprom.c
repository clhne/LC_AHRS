/* eeprom.c file

���ܣ�
��Flash����EEPROM ���ڱ���ƫ�úͱ궨����
------------------------------------
 */
#include "eeprom.h"

#define  PAGE_Gyro   (0x08000000 + 60 * 1024) //�������ǵ�ƫ�ô���ڵ�60ҳFlash
//#define  PAGE_Magic  (0x08000000 + 61 * 1024) //�������Ƶı궨ֵ����ڵ�61ҳFlash

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void Read_Gyro_Offset(int16_t *offset_gx,int16_t *offset_gy,int16_t *offset_gz)
*��������:	   ��ȡ�����Flash�е�������ƫ�����ݡ�
���������  �������ƫ�ã����ָ��
���������  ��
*******************************************************************************/
void Read_Gyro_Offset(int16_t *offset_gx,int16_t *offset_gy,int16_t *offset_gz)                                
{
	
    int16_t *temp_addr = (int16_t *)PAGE_Gyro;
	//FLASH_Unlock();
	*offset_gx=*temp_addr;
	temp_addr++;
    *offset_gy=*temp_addr;
	temp_addr++;
    *offset_gz=*temp_addr; 
	//FLASH_Lock();                                                                                            
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void Write_Gyro_Offset(int16_t offset_gx,int16_t offset_gy,int16_t offset_gz)
*��������:	   ��������ƫ�ô�ŵ�Flash
���������  �������ƫ��
���������  ��
*******************************************************************************/
void Write_Gyro_Offset(int16_t offset_gx,int16_t offset_gy,int16_t offset_gz)
{
 	uint32_t temp_addr = PAGE_Gyro;
 	//FLASH_Unlock();
 	//FLASH_ErasePage(PAGE_Gyro); 

 	//FLASH_ProgramHalfWord(temp_addr,offset_gx);
 	temp_addr+=2;

 	//FLASH_ProgramHalfWord(temp_addr,offset_gy);
 	temp_addr+=2;

 	//FLASH_ProgramHalfWord(temp_addr,offset_gz);
 	temp_addr+=2;

 	//FLASH_Lock();    
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void Read_Magic_Offset(int16_t *min_mx,int16_t *min_my,int16_t *min_mz,
										int16_t *max_mx,int16_t *max_my,int16_t *max_mz)
*��������:	   ��Flash��ȡ�����Ʊ궨
���������  ������������Ķ�Ӧ�����ֵ����Сֵ
���������  ��
*******************************************************************************/
/*
void Read_Magic_Offset(int16_t *min_mx,int16_t *min_my,int16_t *min_mz,
int16_t *max_mx,int16_t *max_my,int16_t *max_mz)                                
{
	
    int16_t *temp_addr = (int16_t *)PAGE_Magic;
	FLASH_Unlock();
	*min_mx=*temp_addr;
	temp_addr++;
    *min_my=*temp_addr;
	temp_addr++;
    *min_mz=*temp_addr;   
	temp_addr++;

	*max_mx=*temp_addr;   
	temp_addr++;
	*max_my=*temp_addr;   
	temp_addr++;
	*max_mz=*temp_addr;  
	FLASH_Lock();                                                                
}
*/
/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void Write_Magic_Offset(int16_t min_mx,int16_t min_my,int16_t min_mz,
							int16_t max_mx,int16_t max_my,int16_t max_mz)
*��������:	   �������Ʊ궨д��Flash
���������  ������������Ķ�Ӧ�����ֵ����Сֵ
���������  ��
*******************************************************************************/
/*
void Write_Magic_Offset(int16_t min_mx,int16_t min_my,int16_t min_mz,
int16_t max_mx,int16_t max_my,int16_t max_mz)
{
 	uint32_t temp_addr = PAGE_Magic;
 	FLASH_Unlock();
 	FLASH_ErasePage(PAGE_Magic); 

 	FLASH_ProgramHalfWord(temp_addr,min_mx);
 	temp_addr+=2;

 	FLASH_ProgramHalfWord(temp_addr,min_my);
 	temp_addr+=2;

 	FLASH_ProgramHalfWord(temp_addr,min_mz);
 	temp_addr+=2;

 	FLASH_ProgramHalfWord(temp_addr,max_mx);
 	temp_addr+=2;

 	FLASH_ProgramHalfWord(temp_addr,max_my);
 	temp_addr+=2;

 	FLASH_ProgramHalfWord(temp_addr,max_mz);

 	FLASH_Lock();    
}

//------------------End of File----------------------------
*/