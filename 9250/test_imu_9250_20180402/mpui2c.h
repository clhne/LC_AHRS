#ifndef __MPUIIC_H
#define __MPUIIC_H
#include "sys.h"
void mpu_i2c_init(void); // ��ʼ��I2C��IO��
void mpu_i2c_delay(void); // I2C��ʱ����
void mpu_i2c_start(void); // ����I2C��ʼ�ź�
void mpu_i2c_stop(void); // ����I2Cֹͣ�ź�
void mpu_i2c_send_byte(u8 txd); // I2C����һ���ֽ�
u8 mpu_i2c_read_byte(unsigned char ack); // I2C��ȡһ���ֽ�
u8 mpu_i2c_wait_ack(void); // I2C�ȴ�ACK�ź�
void mpu_i2c_ack(void); // I2C����ACK�ź�
void mpu_i2c_nack(void); // I2C������ACK�ź�
void mpu_i2c_write_one_byte(u8 daddr, u8 addr, u8 data);
u8 mpu_i2c_read_one_byte(u8 daddr,u8 addr);
#endif
