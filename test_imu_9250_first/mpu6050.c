#include "mpuiic.h"
#include "mpu6050.h"
#include "stm32f10x_gpio.h" 
#include "stm32f10x_exti.h"
#include "stm32f10x_rcc.h"
#include <stdio.h>


//****************************************
#define	SMPLRT_DIV		0x19	
#define	CONFIG			0x1A	
#define	GYRO_CONFIG		0x1B	
#define	ACCEL_CONFIG	0x1C	

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

#define	PWR_MGMT_1		0x6B	
#define	WHO_AM_I		  0x75	

#define	GYRO_ADDRESS   0xD0	  //define SlaveAddress IIC写入时的地址字节数据，+1为读取
#define ACCEL_ADDRESS  0xD0   //define SlaveAddress IIC写入时的地址字节数据，+1为读取
//向IIC设备写入一个字节数据
u8 mpu_6050_write_byte(u8 addr, u8 reg, u8 data)
{
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr<<1)|0);
	if(MPU_IIC_Wait_Ack())
	{
		MPU_IIC_Stop();
		return 1;
	}
	MPU_IIC_Send_Byte(reg);
	MPU_IIC_Wait_Ack();
	MPU_IIC_Send_Byte(data);
	if(MPU_IIC_Wait_Ack())
	{
		MPU_IIC_Stop();
		return 1;
	}
	MPU_IIC_Stop();
	return 0;
}

//从IIC设备读取一个字节数据
u8 mpu_6050_read_byte(u8 addr, u8 reg)
{
	u8 res;
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr <<1)|0);
	MPU_IIC_Wait_Ack();
	MPU_IIC_Send_Byte(reg);
	MPU_IIC_Wait_Ack();
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr <<1)|1);
	if(MPU_IIC_Wait_Ack()){
		MPU_IIC_Stop();
		return 1;
	}
	res = MPU_IIC_Read_Byte(0);
	MPU_IIC_Stop();
	return res;
}

u8 mpu_6050_write_len(u8 addr, u8 reg, u8 len, u8 *buf)
{
	u8 i;
	MPU_IIC_Start();
  MPU_IIC_Send_Byte((addr << 1)|0);
	if(MPU_IIC_Wait_Ack())
	{
		MPU_IIC_Stop();
		return 1;
	}
	MPU_IIC_Send_Byte(reg);
	MPU_IIC_Wait_Ack();
	for(i = 0; i < len; i++)
	{
		MPU_IIC_Send_Byte(buf[i]);
		if(MPU_IIC_Wait_Ack())
		{
			MPU_IIC_Stop();
			return 1;
		}
	}
	MPU_IIC_Stop();
	return 0;
}

u8 mpu_6050_read_len(u8 addr ,u8 reg, u8 len, u8 *buf)
{
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr << 1)|0);
	if(MPU_IIC_Wait_Ack())
	{
		MPU_IIC_Stop();
		return 1;
	}
	MPU_IIC_Send_Byte(reg);
	MPU_IIC_Wait_Ack();
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr << 1)|1);
	MPU_IIC_Wait_Ack();
	while(len)
	{
		if(len == 1)
			*buf = MPU_IIC_Read_Byte(0);
		else
			*buf = MPU_IIC_Read_Byte(1);
		len--;
		buf++;
	}
	MPU_IIC_Stop();
	return 0;
}
u8 mpu_6050_init(void)
{
	u8 res;
 
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	//Config the GPIOA_1

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	//Config the Exti
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource1);
	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_Init(&NVIC_InitStructure);
    
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP,ENABLE);
	PWR_BackupAccessCmd(ENABLE);
	RCC_BackupResetCmd(ENABLE);
	RCC_BackupResetCmd(DISABLE);
  
  mpu_iic_init();
  
    delay_ms(1000); 
 
    res = mpu_6050_read_byte(GYRO_ADDRESS,WHO_AM_I);
    printf("res = %d\n",res);
    res = mpu_6050_write_byte(GYRO_ADDRESS,PWR_MGMT_1,0x00);
    printf("val = %d\n",res);
    
	
}

