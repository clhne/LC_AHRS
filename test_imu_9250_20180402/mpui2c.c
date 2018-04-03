#include "mpui2c.h"
#include "delay.h"
#include "usart.h"

// IO��������
#define MPU_SDA_IN()  { GPIOB->CRL&=0X0FFFFFFF; GPIOB->CRL|=0x80000000; }
#define MPU_SDA_OUT() { GPIOB->CRL&=0X0FFFFFFF; GPIOB->CRL|=0x30000000; }

// IO��������
#define MPU_I2C_SCL    PBout(6)  // SCL
#define MPU_I2C_SDA    PBout(7)  // SDA
#define MPU_READ_SDA   PBin(7)   // ����SDA

// I2C ��ʱ����
void mpu_i2c_delay(void) {
  delay_us(2);
}


// ����I2C��ʼ�ź�, ��ÿ���Ȩ
void mpu_i2c_start(void) {
  MPU_SDA_OUT(); // sda�����
  MPU_I2C_SDA = 1;
  MPU_I2C_SCL = 1;
  mpu_i2c_delay();
  MPU_I2C_SDA = 0; // START: when CLK is high, DATA change form high to low 
  mpu_i2c_delay();
  MPU_I2C_SCL = 0; // ǯסI2C���ߣ�׼�����ͻ�������� 
}

// ����I2Cֹͣ�ź�
void mpu_i2c_stop(void) {
  MPU_SDA_OUT(); // sda�����
  MPU_I2C_SCL = 0;
  MPU_I2C_SDA = 0; // STOP: when CLK is high, DATA change form low to high
  mpu_i2c_delay();
  MPU_I2C_SCL = 1; 
  MPU_I2C_SDA = 1; // ����I2C���߽����ź�
  mpu_i2c_delay();
}

// �ȴ�Ӧ���źŵ���
// ����ֵ��1, ����Ӧ��ʧ�� 0, ����Ӧ��ɹ�
u8 mpu_i2c_wait_ack(void) {
  u8 ucErrTime=0;
  MPU_SDA_IN(); // SDA����Ϊ����
  MPU_I2C_SDA = 1;
  mpu_i2c_delay();
  MPU_I2C_SCL = 1;
  mpu_i2c_delay();
  while (MPU_READ_SDA) {
    ucErrTime++;
    if (ucErrTime > 250) {
      mpu_i2c_stop();
      return 1;
    }
  }
  MPU_I2C_SCL = 0; // ʱ�����0
  return 0;  
} 

// ����ACKӦ��
void mpu_i2c_ack(void) {
  MPU_I2C_SCL = 0;
  MPU_SDA_OUT();
  MPU_I2C_SDA = 0;
  mpu_i2c_delay();
  MPU_I2C_SCL = 1;
  mpu_i2c_delay();
  MPU_I2C_SCL = 0;
}

// ������ACKӦ��
void mpu_i2c_nack(void) {
  MPU_I2C_SCL = 0;
  MPU_SDA_OUT();
  MPU_I2C_SDA = 1;
  mpu_i2c_delay();
  MPU_I2C_SCL = 1;
  mpu_i2c_delay();
  MPU_I2C_SCL = 0;
}

// I2C����һ���ֽ�
// ���شӻ�����Ӧ�� 1,��Ӧ�� 0, ��Ӧ��
void mpu_i2c_send_byte(u8 txd) {
  u8 t;
  MPU_SDA_OUT();
  MPU_I2C_SCL = 0; // ����ʱ�ӿ�ʼ���ݴ���
  for (t = 0; t<8; t++) {
    MPU_I2C_SDA = (txd & 0x80) >> 7;
    txd <<= 1;
    MPU_I2C_SCL = 1;
    mpu_i2c_delay(); 
    MPU_I2C_SCL = 0;
    mpu_i2c_delay();
  }
}

// ��1���ֽ�, ack=1ʱ,����ACK; ack=0, ����nACK
u8 mpu_i2c_read_byte(unsigned char ack) {
  unsigned char i, receive = 0;
  MPU_SDA_IN(); // SDA����Ϊ����
  for (i = 0; i < 8; i++ ) {
    MPU_I2C_SCL = 0; 
    mpu_i2c_delay();
    MPU_I2C_SCL = 1;
    receive <<= 1;
    if (MPU_READ_SDA) receive++;
    mpu_i2c_delay(); 
  }
  if (!ack)
    mpu_i2c_nack(); // ����nACK
  else
    mpu_i2c_ack(); // ����ACK
  return receive;
}

// ��ʼ��I2C
void mpu_i2c_init(void) {
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // ��ʹ������IO PORTBʱ�� 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // �˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  // �������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  // IO���ٶ�Ϊ50MHz
  GPIO_Init(GPIOB, &GPIO_InitStructure);  // �����趨������ʼ��GPIO 
  GPIO_SetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7); // PB6,PB7 �����
}