#include "mpuiic.h"
#include "mpu9250.h"
#include "stm32f10x_gpio.h" 
#include "stm32f10x_exti.h"
#include "stm32f10x_rcc.h"
#include <stdio.h>

// definition
u8 buffer[14];
int16_t MPU9250_FIFO[6][11], Gx_offset = 0, Gy_offset = 0, Gz_offset = 0;
float Acc1G_Values;

uint8_t MPU9250_getDeviceID(void) {

    u8 tmp = mpu_9250_read(GYRO_ADDRESS, WHO_AM_I, 1, buffer);
	  printf("tmp = %d\n",tmp);
    return buffer[0];
}

u8 MPU9250_testConnection(void){
  u8 id = MPU9250_getDeviceID();
	printf("DeviceID = 0x%x\n",id);
	if(id == 0x71){
	printf("Find MU9250 !\n");
	return 1;
	}
	else
		return 0;
}
float MPU9250_1GValue(void)
{
	return Acc1G_Values;
}
void MPU9250_newValues(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz)
{
	unsigned char i;
	int32_t sum = 0;
	for(i = 0; i < 10; i++)
	{
		MPU9250_FIFO[0][i-1] = MPU9250_FIFO[0][i];
		MPU9250_FIFO[1][i-1] = MPU9250_FIFO[1][i];
		MPU9250_FIFO[2][i-1] = MPU9250_FIFO[2][i];
		MPU9250_FIFO[3][i-1] = MPU9250_FIFO[3][i];
		MPU9250_FIFO[4][i-1] = MPU9250_FIFO[4][i];
		MPU9250_FIFO[5][i-1] = MPU9250_FIFO[5][i];
	}
	MPU9250_FIFO[0][9] = ax;
	MPU9250_FIFO[1][9] = ay;
	MPU9250_FIFO[2][9] = az;
	MPU9250_FIFO[3][9] = gx;
	MPU9250_FIFO[4][9] = gy;
	MPU9250_FIFO[5][9] = gz;
	sum = 0;
	for(i = 0; i < 10; i++)
	{
		sum+=MPU9250_FIFO[0][i];
	}
	MPU9250_FIFO[0][10] = sum/10;
	
	sum = 0;
	for(i = 0;i < 10; i++)
	{
		sum+=MPU9250_FIFO[1][i];
	}
	MPU9250_FIFO[1][10] = sum/10;
	
	sum = 0;
	for(i = 0;i < 10; i++)
	{
		sum+=MPU9250_FIFO[2][i];
	}
	MPU9250_FIFO[2][10] = sum/10;
	
	sum = 0;
	for(i = 0;i < 10; i++)
	{
		sum+=MPU9250_FIFO[3][i];
	}
	MPU9250_FIFO[3][10] = sum/10;
	
	sum = 0;
	for(i = 0;i < 10; i++)
	{
		sum+=MPU9250_FIFO[4][i];
	}
	MPU9250_FIFO[4][10] = sum/10;
	
	sum = 0;
	for(i = 0;i < 10; i++)
	{
		sum+=MPU9250_FIFO[5][i];
	}
	MPU9250_FIFO[5][10] = sum/10;
}
u8 mpu_9250_write(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
    MPU_IIC_Start(); 
	MPU_IIC_Send_Byte(addr|0);
	//MPU_IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(MPU_IIC_Wait_Ack())	//等待应答
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//写寄存器地址
    MPU_IIC_Wait_Ack();		//等待应答
	for(i=0;i<len;i++)
	{
		MPU_IIC_Send_Byte(buf[i]);	//发送数据
		if(MPU_IIC_Wait_Ack())		//等待ACK
		{
			MPU_IIC_Stop();	 
			return 1;		 
		}		
	}
    MPU_IIC_Stop();	 
	return 0;	
} 

//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 mpu_9250_read(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
	u8 count = 0;
 	MPU_IIC_Start(); 
	MPU_IIC_Send_Byte(addr);
	//MPU_IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(MPU_IIC_Wait_Ack())	//等待应答
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//写寄存器地址
    MPU_IIC_Wait_Ack();		//等待应答
    MPU_IIC_Start();
	  MPU_IIC_Send_Byte(addr+1);
	  //MPU_IIC_Send_Byte((addr<<1)|1);//发送器件地址+读命令	
    MPU_IIC_Wait_Ack();		//等待应答 
	
	while(len)
	{
		if(len==1)*buf=MPU_IIC_Read_Byte(0);//读数据,发送nACK 
		else *buf=MPU_IIC_Read_Byte(1);		//读数据,发送ACK  
		len--;
		buf++; 
	} 
		
    MPU_IIC_Stop();	//产生一个停止条件 
	return 0;
	
}

void EXTI1_IRQHandler(void){
  /* Handle new gyro */
  gyro_data_ready_cb();
  EXTI_ClearITPendingBit(EXTI_Line1);
}

void mpu_9250_init(void) {
  u8 val;
  u8 ret;
	int16_t temp[6];
  unsigned char i;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	//Config the GPIOA_1
	//开启外部时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
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
  for(i = 0; i < 10; i++)
	{
		delay_us(50);
		MPU9250_getMotion6(&temp[0],&temp[1],&temp[2],&temp[3],&temp[4],&temp[5]);
	}
	Read_Gyro_Offset(&Gx_offset,&Gy_offset,&Gz_offset);
  //delay_ms(1000); 
  
  val = 0x00;
  ret = mpu_9250_read(GYRO_ADDRESS, WHO_AM_I, 1, &val);

  printf("ret=%d val=%d\n", ret, val);
  val = 0x00;
  ret = mpu_9250_write(GYRO_ADDRESS, PWR_MGMT_1, 1, &val);	//解除休眠状态
  printf("val=%d\n", ret);
  val = 0x07;
	ret = mpu_9250_write(GYRO_ADDRESS,SMPLRT_DIV, 1, &val);
  val = 0x06;
	ret = mpu_9250_write(GYRO_ADDRESS,CONFIG, 1, &val);
  val = 0x18;
	ret = mpu_9250_write(GYRO_ADDRESS,GYRO_CONFIG, 1, &val);
  val = 0x01;
	ret = mpu_9250_write(GYRO_ADDRESS,ACCEL_CONFIG, 1, &val);
}

unsigned char MPU9250_is_DRY(void)
{
	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8) == Bit_SET){
	return 1;
	}
	else 
		return 0;
}

u16 MPU9250_Lastax,MPU9250_Lastay,MPU9250_Lastaz,MPU9250_Lastgx,MPU9250_Lastgy,MPU9250_Lastgz;
void MPU9250_getMotion6(int16_t* ax,int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
{
	if(MPU9250_is_DRY())
	{
		mpu_9250_read(ACCEL_ADDRESS,ACCEL_XOUT_H,14,buffer);
		MPU9250_Lastax = (((int16_t)buffer[0]) << 8) | buffer[1];
		MPU9250_Lastay = (((int16_t)buffer[2]) << 8) | buffer[3];
		MPU9250_Lastaz = (((int16_t)buffer[4]) << 8) | buffer[5];
		
		MPU9250_Lastgx = (((int16_t)buffer[8]) << 8) | buffer[9];
		MPU9250_Lastgy = (((int16_t)buffer[10]) << 8) | buffer[11];
		MPU9250_Lastgz = (((int16_t)buffer[12]) << 8) | buffer[13];
		MPU9250_newValues(MPU9250_Lastax,MPU9250_Lastay,MPU9250_Lastaz,MPU9250_Lastgx,MPU9250_Lastgy,MPU9250_Lastgz);
		*ax = MPU9250_FIFO[0][10];
		*ay = MPU9250_FIFO[1][10];
		*az = MPU9250_FIFO[2][10];
		*gx = MPU9250_FIFO[3][10] - Gx_offset;
		*gy = MPU9250_FIFO[4][10] - Gy_offset;
		*gz = MPU9250_FIFO[5][10] - Gz_offset;
	}
	else
	{
		*ax = MPU9250_FIFO[0][10];
		*ay = MPU9250_FIFO[1][10];
		*az = MPU9250_FIFO[2][10];
		*gx = MPU9250_FIFO[3][10] - Gx_offset;
		*gy = MPU9250_FIFO[4][10] - Gy_offset;
		*gz = MPU9250_FIFO[5][10] - Gz_offset;
	}
}

void MPU9250_getlastMotion6(int16_t* ax,int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
{
		*ax = MPU9250_FIFO[0][10];
		*ay = MPU9250_FIFO[1][10];
		*az = MPU9250_FIFO[2][10];
		*gx = MPU9250_FIFO[3][10] - Gx_offset;
		*gy = MPU9250_FIFO[4][10] - Gy_offset;
		*gz = MPU9250_FIFO[5][10] - Gz_offset;	
}

void MPU9250_InitGyro_Offset(void)
{
	unsigned char i;
	int16_t temp[6];
	int32_t	tempgx=0,tempgy=0,tempgz=0;
	int32_t	tempax=0,tempay=0,tempaz=0;
	Gx_offset=0;
	Gy_offset=0;												  
	Gz_offset=0;
	for(i=0;i<50;i++)
	{
  		delay_us(100);
  		MPU9250_getMotion6(&temp[0],&temp[1],&temp[2],&temp[3],&temp[4],&temp[5]);
//  		LED_Change();
	}
 	for(i=0;i<100;i++)
	{
		delay_us(200);
		MPU9250_getMotion6(&temp[0],&temp[1],&temp[2],&temp[3],&temp[4],&temp[5]);
		tempax+= temp[0];
		tempay+= temp[1];
		tempaz+= temp[2];
		tempgx+= temp[3];
		tempgy+= temp[4];
		tempgz+= temp[5];
//		LED_Change();
	}

	Gx_offset=tempgx/100;//MPU6050_FIFO[3][10];
	Gy_offset=tempgy/100;//MPU6050_FIFO[4][10];
	Gz_offset=tempgz/100;//MPU6050_FIFO[5][10];
	tempax/=100;
	tempay/=100;
	tempaz/=100;
	Acc1G_Values= (float)(tempax+tempay+tempaz);
	Write_Gyro_Offset(Gx_offset,Gy_offset,Gz_offset);
	Gx_offset=0;
	Gy_offset=0;
	Gz_offset=0;
	Read_Gyro_Offset(&Gx_offset,&Gy_offset,&Gz_offset);
}
