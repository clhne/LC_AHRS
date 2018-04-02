#ifndef __EEPROM_H
#define __EEPROM_H

#include "stm32f10x.h"
/*
使用 STM32F 内部Flash做EEPROM
引出的API 子程序
*/

//读取陀螺仪偏置
void Read_Gyro_Offset(int16_t *offset_gx,int16_t *offset_gy,int16_t *offset_gz);

//写入陀螺仪偏置
void Write_Gyro_Offset(int16_t offset_gx,int16_t offset_gy,int16_t offset_gz);

//读取磁力计标定值
void Read_Magic_Offset(int16_t *min_mx,int16_t *min_my,int16_t *min_mz,
int16_t *max_mx,int16_t *max_my,int16_t *max_mz);

//写入磁力计标定值
void Write_Magic_Offset(int16_t min_mx,int16_t min_my,int16_t min_mz,
int16_t max_mx,int16_t max_my,int16_t max_mz);

#endif /* __EEPROM_H */

//------------------End of File----------------------------
