#include <stdio.h>
#include "stm32f10x_gpio.h" 
#include "stm32f10x_exti.h"
#include "stm32f10x_rcc.h"
#include "delay.h"
#include "mpui2c.h"
#include "mpu9250.h"

/* 定义MPU9250内部地址及相关寄存器 */
#define MPU6500_PWR_MGMT_1    0x6B  // 电源管理，典型值：0x00(正常启用)
#define MPU6500_PWR_MGMT_2    0x6C  // 电源管理，典型值：0x00(正常启用)
#define MPU6500_SMPLRT_DIV    0x19  // 陀螺仪采样率，典型值：0x07(125Hz)
#define MPU6500_CONFIG        0x1A  // 低通滤波频率，典型值：0x06(5Hz)
#define MPU6500_GYRO_CONFIG   0x1B  // 陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define MPU6500_ACCEL_CONFIG  0x1C  // 加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)

#define MPU6500_WHO_AM_I      0x75  // 器件ID查询
#define MPU6500_DEVICE_ID     0x71  // 在MPU9250中, 器件ID=0x71
#define MPU6500_USER_CTRL     0x6A  // 用户配置当为0x10时使用SPI模式

#define MPU6500_INT_PIN_CFG   0x37

#define MPU6500_I2C_ADDR      0x68
#define AK8963_I2C_ADDR       0x0C

// [0] accel_x_hi [1] accel_x_lo
// [2] accel_y_hi [3] accel_y_lo
// [4] accel_z_hi [5] accel_z_lo
// [6] temp_hi    [7] temp_lo
// [8] gyro_x_hi  [9] gyro_x_lo
// [10] gyro_y_hi [11] gyro_y_lo
// [12] gyro_z_hi [13] gyro_z_lo
#define MPU6500_DATA_START    0x3B
 
// [0] mag_x_lo [1] mag_x_hi
// [2] mag_y_lo [3] mag_y_hi
// [4] mag_z_lo [5] mag_z_hi
#define AK8963_DATA_START     0x03

/* 定义MPU9250 陀螺仪,加速度计和磁力计量程相关 */
#define G2MSS (9.86) // g转m*s^-2
#define UT2MGS (10.0) // uT转mGs, 10000GS(高斯)等于1T(特斯拉), 地磁场约为0.6Gs
#define DEG2RAD (3.14159265358979323846 / 180.0) //deg/s转rad/s 

// 陀螺仪量程
#define MPU9250_GYRO_FS_250DPS    0x00
#define MPU9250_GYRO_FS_500DPS    0x08
#define MPU9250_GYRO_FS_1000DPS   0x10
#define MPU9250_GYRO_FS_2000DPS   0x18
// 陀螺仪采样频率
// | LPF | BandW | Delay  | Sample |
// +-----+-------+--------+--------+
// |  0  | 256Hz | 0.98ms |  8kHz  |
// |  1  | 188Hz |  1.9ms |  1kHz  |
// |  2  |  98Hz |  2.8ms |  1kHz  |
// |  3  |  42Hz |  4.8ms |  1kHz  |
// |  4  |  20Hz |  8.3ms |  1kHz  |
// |  5  |  10Hz | 13.4ms |  1kHz  |
// |  6  |   5Hz | 18.6ms |  1kHz  |
// |  7  | -- Reserved -- |  8kHz  |
#define MPU9250_GYRO_LPS_250HZ    0x00
#define MPU9250_GYRO_LPS_184HZ    0x01
#define MPU9250_GYRO_LPS_92HZ     0x02
#define MPU9250_GYRO_LPS_41HZ     0x03
#define MPU9250_GYRO_LPS_20HZ     0x04
#define MPU9250_GYRO_LPS_10HZ     0x05
#define MPU9250_GYRO_LPS_5HZ      0x06
#define MPU9250_GYRO_LPS_DISABLE  0x07
// 陀螺仪灵敏度系数
#define MPU9250_GYRO_STY_250DPS   ((float)0.007633587786f) // 0.007633587786 dps/LSB
#define MPU9250_GYRO_STY_500DPS   ((float)0.015267175572f) // 0.015267175572 dps/LSB
#define MPU9250_GYRO_STY_1000DPS  ((float)0.030487804878f) // 0.030487804878 dps/LSB
#define MPU9250_GYRO_STY_2000DPS  ((float)0.060975609756f) // 0.060975609756 dps/LSB

// 加速度计量程
#define MPU9250_ACCEL_FS_2G     0x00
#define MPU9250_ACCEL_FS_4G     0x08
#define MPU9250_ACCEL_FS_8G     0x10
#define MPU9250_ACCEL_FS_16G    0x18
// 加速度计采样频率
//  | LPF | BandW | Delay  | Sample |
//  +-----+-------+--------+--------+
//  |  0  | 260Hz |    0ms |  1kHz  |
//  |  1  | 184Hz |  2.0ms |  1kHz  |
//  |  2  |  94Hz |  3.0ms |  1kHz  |
//  |  3  |  44Hz |  4.9ms |  1kHz  |
//  |  4  |  21Hz |  8.5ms |  1kHz  |
//  |  5  |  10Hz | 13.8ms |  1kHz  |
//  |  6  |   5Hz | 19.0ms |  1kHz  |
//  |  7  | -- Reserved -- |  1kHz  |
#define MPU9250_ACCEL_LPS_460HZ   0x00
#define MPU9250_ACCEL_LPS_184HZ   0x01
#define MPU9250_ACCEL_LPS_92HZ    0x02
#define MPU9250_ACCEL_LPS_41HZ    0x03
#define MPU9250_ACCEL_LPS_20HZ    0x04
#define MPU9250_ACCEL_LPS_10HZ    0x05
#define MPU9250_ACCEL_LPS_5HZ     0x06
#define MPU9250_ACCEL_LPS_DISABLE 0x08
// 加速度计灵敏度系数
#define MPU9250_ACCEL_STY_2G       ((float)0.000061035156f) // 0.000061035156 g/LSB
#define MPU9250_ACCEL_STY_4G       ((float)0.000122070312f) // 0.000122070312 g/LSB
#define MPU9250_ACCEL_STY_8G       ((float)0.000244140625f) // 0.000244140625 g/LSB
#define MPU9250_ACCEL_STY_16G      ((float)0.000488281250f) // 0.000488281250 g/LSB

// 温度计敏度系数
#define MPU9250_TEMP_STY_85DEGC    ((float)0.002995177763f) // 0.002995177763 degC/LSB

// 磁力计灵敏度系数
#define MPU9250_MAG_STY_4800UT    ((float)0.6f)            // 0.6 uT/LSB


/* 陀螺仪,加速度计和磁力计量程及灵敏度 */
u8 CUR_GYRO_FS = MPU9250_GYRO_FS_1000DPS;
u8 CUR_GYRO_LPS = MPU9250_GYRO_LPS_184HZ;
u8 CUR_ACCEL_FS = MPU9250_ACCEL_FS_4G;
u8 CUR_ACCEL_LPS = MPU9250_ACCEL_LPS_184HZ;
float CUR_GYRO_STY = MPU9250_GYRO_STY_1000DPS;
float CUR_ACCEL_STY = MPU9250_ACCEL_STY_4G;
float CUR_TEMP_STY = MPU9250_TEMP_STY_85DEGC;
float CUR_MAG_STY = MPU9250_MAG_STY_4800UT;
int gyro_bias_x = 0;
int gyro_bias_y = 0;
int gyro_bias_z = 0;
float mag_adjust_x = 1.0f;
float mag_adjust_y = 1.0f;
float mag_adjust_z = 1.0f;
u8 mag_bias_x = 0;
u8 mag_bias_y = 0;
u8 mag_bias_z = 0;

u8 mpu_9250_write(u8 addr, u8 reg, u8 len, u8 *buf) {
  u8 i; 
  mpu_i2c_start(); 
  mpu_i2c_send_byte((addr<<1)|0); // 发送器件地址+写标志0
  if (mpu_i2c_wait_ack()) { // 等待应答
    mpu_i2c_stop(); 
    return 1;
  }
  mpu_i2c_send_byte(reg); // 写寄存器地址
  mpu_i2c_wait_ack(); // 等待应答
  for (i = 0; i < len; i++) {
    mpu_i2c_send_byte(buf[i]); // 发送数据
    if (mpu_i2c_wait_ack()) { // 等待ACK
      mpu_i2c_stop();
      return 1;
    }
  }
  mpu_i2c_stop();
  return 0;
} 

u8 mpu_9250_read(u8 addr,u8 reg,u8 len,u8 *buf) {
  mpu_i2c_start(); 
  mpu_i2c_send_byte((addr<<1)|0);// 发送器件地址+写标志0
  if (mpu_i2c_wait_ack()) { // 等待应答
    mpu_i2c_stop();
    return 1;
  }
  mpu_i2c_send_byte(reg); // 写寄存器地址
  mpu_i2c_wait_ack(); // 等待应答
  mpu_i2c_start();
  mpu_i2c_send_byte((addr << 1) | 1); // 发送器件地址+读标志1
  mpu_i2c_wait_ack(); // 等待应答 
  while (len) {
    if (len == 1) *buf = mpu_i2c_read_byte(0); // 读数据,发送nACK 
    else *buf = mpu_i2c_read_byte(1); // 读数据,发送ACK  
    len--;
    buf++; 
  }    
  mpu_i2c_stop(); // 产生一个停止条件 
  return 0;
}

void EXTI1_IRQHandler(void){
  EXTI_ClearITPendingBit(EXTI_Line1);
}

void mpu_9250_exti_config(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

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
}

u8 mpu_9250_work_config(void) {
  u8 reg_value = 0x00;
  u8 ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_PWR_MGMT_1, 1, &reg_value); // 解除休眠状态
  if (ret_code) {
    return 1;
  }
  delay_ms(100);
  ret_code = mpu_9250_read(MPU6500_I2C_ADDR, MPU6500_WHO_AM_I, 1, &reg_value);
  if (ret_code) {
    return 1;
  }
  if (reg_value != MPU6500_DEVICE_ID) {
    return 2;
  }
  delay_ms(1000);

  reg_value = 0x00;
  ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_PWR_MGMT_2, 1, &reg_value); // 使能寄存器X, Y, Z加速度
  if (ret_code) {
    return 1;
  }

  reg_value = 0x07;
  ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_SMPLRT_DIV, 1, &reg_value); // 设置采样频率
  if (ret_code) {
    return 1;
  }

  reg_value = 0x06;
  ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_CONFIG, 1, &reg_value); // 设置低通滤波频率
  if (ret_code) {
    return 1;
  }

  reg_value = CUR_GYRO_FS | CUR_GYRO_LPS;
  ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_GYRO_CONFIG, 1, &reg_value); // 设置陀螺仪量程
  if (ret_code) {
    return 1;
  }

  reg_value = 0x00;
  ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_USER_CTRL, 1, &reg_value); // 初始化IIC
  if (ret_code) {
    return 1;
  }

  reg_value = CUR_ACCEL_FS | CUR_ACCEL_LPS;
  ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_ACCEL_CONFIG, 1, &reg_value); // 设置加速度计量程
  if (ret_code) {
    return 1;
  }

  reg_value = 0x02;
  ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_INT_PIN_CFG, 1, &reg_value); // 进入bypass模式, 控制磁力计
  if (ret_code) {
    return 1;
  }

  return 0;
}

u8 mpu_9250_read_accel_gyro_raw(short *raw_ax, short *raw_ay, short *raw_az, short *raw_gx, short *raw_gy, short *raw_gz) {
  u8 tmp_buf[14];

  // read accel and gyro data
  if (mpu_9250_read(MPU6500_I2C_ADDR, MPU6500_DATA_START, 14, tmp_buf)) {
    return 1;
  }

  *raw_ax = (((short)tmp_buf[0]) << 8) | (short)tmp_buf[1];
  *raw_ay = (((short)tmp_buf[2]) << 8) | (short)tmp_buf[3];
  *raw_az = (((short)tmp_buf[4]) << 8) | (short)tmp_buf[5];
  *raw_gx = (((short)tmp_buf[8]) << 8) | (short)tmp_buf[9];
  *raw_gy = (((short)tmp_buf[10]) << 8) | (short)tmp_buf[11];
  *raw_gz = (((short)tmp_buf[12]) << 8) | (short)tmp_buf[13];

  return 0;
}

u8 mpu_9250_read_mag_raw(short *raw_mx, short *raw_my, short *raw_mz) {
  u8 tmp_buf[6];
  
  // turn on bypass mode and read mag data
  u8 reg_value = 0x01;
  if (mpu_9250_write(AK8963_I2C_ADDR, 0x0A, 1, &reg_value)) {
    return 1;
  }

  delay_ms(10);

  if (mpu_9250_read(AK8963_I2C_ADDR, AK8963_DATA_START, 6, tmp_buf)) {
    return 1;
  }
  
  *raw_mx = (((short)tmp_buf[1]) << 8) | (short)tmp_buf[0];
  *raw_my = (((short)tmp_buf[3]) << 8) | (short)tmp_buf[2];
  *raw_mz = (((short)tmp_buf[5]) << 8) | (short)tmp_buf[4];

  return 0;
}

u8 mpu_9250_calibrate() {
  int i;
  u8 reg_value;
  u8 tmp_x, tmp_y, tmp_z;
  for (i = 0; i < 15; i++) {
    short raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
    if (mpu_9250_read_accel_gyro_raw(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz)) {
      goto ERROR;
    }
    if (i >= 5) {
      gyro_bias_x += raw_gx;
      gyro_bias_y += raw_gy;
      gyro_bias_z += raw_gz;
    }
  }
  gyro_bias_x /= 10;
  gyro_bias_y /= 10;
  gyro_bias_z /= 10;
  
  reg_value = 0x01;
  if (mpu_9250_write(AK8963_I2C_ADDR, 0x0A, 1, &reg_value)) {
    return 1;
  }

  delay_ms(10);
  
  if (mpu_9250_read(AK8963_I2C_ADDR, 0x10, 1, &tmp_x) ) {
    goto ERROR;
  }
  
  if (mpu_9250_read(AK8963_I2C_ADDR, 0x11, 1, &tmp_y) ) {
    goto ERROR;
  }
  
  if (mpu_9250_read(AK8963_I2C_ADDR, 0x12, 1, &tmp_z) ) {
    goto ERROR;
  }
  
  mag_adjust_x = (float)(tmp_x - 128)/256.0f + 1.0f;   // return x-axis sensitivity adjustment values, etc.
  mag_adjust_y = (float)(tmp_y - 128)/256.0f + 1.0f;  
  mag_adjust_z = (float)(tmp_z - 128)/256.0f + 1.0f;
  
  printf("gyro bias: %d, %d, %d\n", gyro_bias_x, gyro_bias_y, gyro_bias_z);
  printf("mag adjust: %d, %d, %d->%f, %f, %f\n", tmp_x, tmp_y, tmp_z, mag_adjust_x, mag_adjust_y, mag_adjust_z);

  return 0;
  
ERROR:
  gyro_bias_x = 0;
  gyro_bias_y = 0;
  gyro_bias_z = 0;
  mag_bias_x = 0;
  mag_bias_y = 0;
  mag_bias_z = 0;
  return 1; 
}

u8 mpu_9250_is_dry(void) {
  return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8) == Bit_SET ? 1 : 0;
}

u8 mpu_9250_read_accel_gyro(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) {
  short raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
  if (mpu_9250_read_accel_gyro_raw(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz)) {
    return 1;
  }
  *ax = (float)raw_ax * (CUR_ACCEL_STY * G2MSS);
  *ay = (float)raw_ay * (CUR_ACCEL_STY * G2MSS);
  *az = (float)raw_az * (CUR_ACCEL_STY * G2MSS);
//  *ax = (float)raw_ax * (CUR_ACCEL_STY);
//  *ay = (float)raw_ay * (CUR_ACCEL_STY);
//  *az = (float)raw_az * (CUR_ACCEL_STY);
  *gx = (float)(raw_gx - gyro_bias_x) * (CUR_GYRO_STY * DEG2RAD);
  *gy = (float)(raw_gy - gyro_bias_y) * (CUR_GYRO_STY * DEG2RAD);
  *gz = (float)(raw_gz - gyro_bias_z) * (CUR_GYRO_STY * DEG2RAD);
  return 0;
}

u8 mpu_9250_read_mag(float *mx, float *my, float *mz) {
  short raw_mx, raw_my, raw_mz;
  if (mpu_9250_read_mag_raw(&raw_mx, &raw_my, &raw_mz)) {
    return 1;
  }
  *mx = (float)raw_mx * mag_adjust_x * (CUR_MAG_STY * UT2MGS);
  *my = (float)raw_my * mag_adjust_y * (CUR_MAG_STY * UT2MGS);
  *mz = (float)raw_mz * mag_adjust_z * (CUR_MAG_STY * UT2MGS);
  return 0;
}

u8 mpu_9250_init(void) {
  u8 ret_code;
  mpu_i2c_init();
  mpu_9250_exti_config();
  ret_code = mpu_9250_work_config();
  if (ret_code) {
    return ret_code;
  }
  return mpu_9250_calibrate();
}

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
   u8 data_write[2];
   data_write[0] = subAddress;
   data_write[1] = data;
   mpu_9250_write(address, data_write[0], 1, &data_write[1]);
}
// https://github.com/kriswiner/MPU6050/wiki/Simple-and-Effective-Magnetometer-Calibration
void magcalMPU9250(float * dest1, float * dest2,float *mag_bias_x, float *mag_bias_y, float *mag_bias_z) 
 {
 u8 MPU9250Mmode = 0x06; 
 float ii = 0, sample_count = 0;
 //float mag_bias[3] = {0, 0, 0}, 
 float mag_scale[3] = {0, 0, 0};
 float mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

 printf("Mag Calibration: Wave device in a figure eight until done!\n");
 delay_ms(4000);

// shoot for ~fifteen seconds of mag data
if(MPU9250Mmode == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
if(MPU9250Mmode == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
for(ii = 0; ii < sample_count; ii++) {
mpu_9250_read_mag(&mag_temp[0],&mag_temp[1],&mag_temp[2]);  // Read the mag data   
for (int jj = 0; jj < 3; jj++) {
  if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
  if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
}
if(MPU9250Mmode == 0x02) delay_ms(135);  // at 8 Hz ODR, new mag data is available every 125 ms
if(MPU9250Mmode == 0x06) delay_ms(12);  // at 100 Hz ODR, new mag data is available every 10 ms
}


// Get hard iron correction
 *mag_bias_x  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
 *mag_bias_y  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
 *mag_bias_z  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
 printf("mag_bias[0] = %f, mag_bias[1] = %f, mag_bias[2] = %f\n",*mag_bias_x,*mag_bias_y,*mag_bias_z);
 

 printf("Mag Calibration done!\n");

 }
 
void initAK8963(uint8_t Mscale, uint8_t Mmode, float * destination)
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  writeByte(AK8963_I2C_ADDR, 0x0A, 0x00); // Power down magnetometer  
  delay_ms(100);
  writeByte(AK8963_I2C_ADDR, 0x0A, 0x0F); // Enter Fuse ROM access mode
  delay_ms(100);
  mpu_9250_read(AK8963_I2C_ADDR, 0x10, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;  
  destination[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f; 
  writeByte(AK8963_I2C_ADDR, 0x0A, 0x00); // Power down magnetometer  
  delay_ms(100);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(AK8963_I2C_ADDR, 0x0A, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
  delay_ms(100);
}