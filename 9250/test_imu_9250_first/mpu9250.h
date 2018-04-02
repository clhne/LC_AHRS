#ifndef __MPU9250_H
#define __MPU9250_H
#include "mpuiic.h" 
#include "eeprom.h"

// ����MPU9250�ڲ���ַ
//****************************************
#define	SMPLRT_DIV		0x19	//�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define	CONFIG			0x1A	//��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define	ACCEL_CONFIG	0x1C	//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)

#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40

#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

		
#define MAG_XOUT_L		0x03
#define MAG_XOUT_H		0x04
#define MAG_YOUT_L		0x05
#define MAG_YOUT_H		0x06
#define MAG_ZOUT_L		0x07
#define MAG_ZOUT_H		0x08


#define	PWR_MGMT_1		0x6B	//��Դ��������ֵ��0x00(��������)
#define	WHO_AM_I		  0x75	//IIC��ַ�Ĵ���(Ĭ����ֵ0x71��ֻ��)

#define	GYRO_ADDRESS   0xD0	  //���ݵ�ַ
#define MAG_ADDRESS    0x18   //�ų���ַ
#define ACCEL_ADDRESS  0xD0 
extern float Acc1G_Values;
void mpu_9250_init(void);
u8 mpu_9250_write(u8 addr,u8 reg,u8 len,u8 *buf);
u8 mpu_9250_read(u8 addr,u8 reg,u8 len,u8 *buf);
u8 MPU9250_getDeviceID(void);
u8 MPU9250_testConnection(void);
float MPU9250_1GValue(void);
void MPU9250_newValues(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);
void MPU9250_getlastMotion6(int16_t* ax,int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
void MPU9250_getMotion6(int16_t* ax,int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
void MPU9250_InitGyro_Offset(void);
void EXTI1_IRQHandler(void);

#endif