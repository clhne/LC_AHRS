#ifndef __MPUIIC_H
#define __MPUIIC_H
#include "sys.h"
void mpu_i2c_init(void); // 初始化I2C的IO口
void mpu_i2c_delay(void); // I2C延时函数
void mpu_i2c_start(void); // 发送I2C开始信号
void mpu_i2c_stop(void); // 发送I2C停止信号
void mpu_i2c_send_byte(u8 txd); // I2C发送一个字节
u8 mpu_i2c_read_byte(unsigned char ack); // I2C读取一个字节
u8 mpu_i2c_wait_ack(void); // I2C等待ACK信号
void mpu_i2c_ack(void); // I2C发送ACK信号
void mpu_i2c_nack(void); // I2C不发送ACK信号
void mpu_i2c_write_one_byte(u8 daddr, u8 addr, u8 data);
u8 mpu_i2c_read_one_byte(u8 daddr,u8 addr);
#endif
