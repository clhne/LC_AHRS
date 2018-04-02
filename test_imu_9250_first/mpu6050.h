#ifndef __MPU9250_H
#define __MPU9250_H
#include "mpuiic.h" 
void mpu_9250_init(void);
u8 mpu_6050_write_byte(u8 addr, u8 reg, u8 data);
u8 mpu_6050_read_byte(u8 addr, u8 reg);
u8 mpu_6050_write_len(u8 addr, u8 reg, u8 len, u8 *buf);
u8 mpu_6050_read_len(u8 addr ,u8 reg, u8 len, u8 *buf);
#endif















