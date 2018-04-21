#include <stdio.h>
#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_rcc.h"
#include "delay.h"
#include "mpui2c.h"
#include "mpu9250.h"

/* 定义MPU6500内部地址及相关寄存器 */
#define MPU6500_I2C_ADDR      0x68
#define MPU6500_PWR_MGMT_1    0x6B  // 电源管理，典型值：0x00(正常启用)
#define MPU6500_PWR_MGMT_2    0x6C  // 电源管理，典型值：0x00(正常启用)
#define MPU6500_SMPLRT_DIV    0x19  // 陀螺仪采样率，典型值：0x07(125Hz)
#define MPU6500_CONFIG        0x1A  // 低通滤波频率，典型值：0x06(5Hz)
#define MPU6500_GYRO_CONFIG   0x1B  // 陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define MPU6500_ACCEL_CONFIG  0x1C  // 加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define MPU6500_ACCEL_CONFIG2 0x1D
#define MPU6500_INT_PIN_CFG   0x37
#define MPU6500_INT_ENABLE    0x38
#define MPU6500_FIFO_EN       0x23
#define MPU6500_FIFO_COUNT    0x72
#define MPU6500_FIFO_R_W      0x74
#define MPU6500_I2C_MST_CTRL  0x24
#define MPU6500_WHO_AM_I      0x75  // 器件ID查询
#define MPU6500_DEVICE_ID     0x71  // 在MPU9250中, 器件ID=0x71
#define MPU6500_USER_CTRL     0x6A  // 用户配置当为0x10时使用SPI模式
#define MPU6500_INT_STATUS    0x3A
// [0] accel_x_hi [1] accel_x_lo
// [2] accel_y_hi [3] accel_y_lo
// [4] accel_z_hi [5] accel_z_lo
// [6] temp_hi    [7] temp_lo
// [8] gyro_x_hi  [9] gyro_x_lo
// [10] gyro_y_hi [11] gyro_y_lo
// [12] gyro_z_hi [13] gyro_z_lo
#define MPU6500_DATA_START    0x3B

/* 定义AK8963内部地址及相关寄存器 */
#define AK8963_I2C_ADDR       0x0C
#define AK8963_CNTL           0x0A
#define AK8963_ASAX           0x10
#define AK8963_ASAY           0x11
#define AK8963_ASAZ           0x12
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
#define MPU9250_GYRO_RES_250DPS   ((float)250.0/32768.0)
#define MPU9250_GYRO_RES_500DPS   ((float)500.0/32768.0)
#define MPU9250_GYRO_RES_1000DPS  ((float)1000.0/32768.0)
#define MPU9250_GYRO_RES_2000DPS  ((float)2000.0/32768.0)

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
#define MPU9250_ACCEL_RES_2G       ((float)2.0/32768.0)
#define MPU9250_ACCEL_RES_4G       ((float)4.0/32768.0)
#define MPU9250_ACCEL_RES_8G       ((float)8.0/32768.0)
#define MPU9250_ACCEL_RES_16G      ((float)16.0/32768.0)

// 磁力计灵敏度系数
#define MPU9250_MAG_FS_14BITS      0x00
#define MPU9250_MAG_FS_16BITS      0x10
// 磁力计采样频率
#define MPU9250_MAG_LPS_8HZ        0x02
#define MPU9250_MAG_LPS_100HZ      0x06
// 磁力计灵敏度系数
#define MPU9250_MAG_RES_14BITS     ((float)4912.0/8190.0)
#define MPU9250_MAG_RES_16BITS     ((float)4912.0/32760.0)

/* 设置当前陀螺仪,加速度计和磁力计量程及灵敏度 */
u8 CUR_GYRO_LPS = MPU9250_GYRO_LPS_41HZ;
u8 CUR_ACCEL_LPS = MPU9250_ACCEL_LPS_41HZ;
u8 CUR_MAG_LPS = MPU9250_MAG_LPS_100HZ;
u8 CUR_GYRO_FS = MPU9250_GYRO_FS_250DPS;
u8 CUR_ACCEL_FS = MPU9250_ACCEL_FS_2G;
u8 CUR_MAG_FS = MPU9250_MAG_FS_16BITS;
float CUR_GYRO_RES = MPU9250_GYRO_RES_250DPS;
float CUR_ACCEL_RES = MPU9250_ACCEL_RES_2G;
float CUR_MAG_RES = MPU9250_MAG_RES_16BITS;
float gyro_bias_x = 0.0f;
float gyro_bias_y = 0.0f;
float gyro_bias_z = 0.0f;
float accel_bias_x = 0.0f;
float accel_bias_y = 0.0f;
float accel_bias_z = 0.0f;
float mag_adjust_x = 1.0f;
float mag_adjust_y = 1.0f;
float mag_adjust_z = 1.0f;
float mag_bias_x = 0.0f;
float mag_bias_y = 0.0f;
float mag_bias_z = 0.0f;

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

void EXTI1_IRQHandler(void) {
    EXTI_ClearITPendingBit(EXTI_Line1);
}

void mpu_9250_exti_config(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);
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

u8 mpu_9250_is_dry(void) {
    u8 reg_value;
    u8 ret_code;
    ret_code = mpu_9250_read(MPU6500_I2C_ADDR, MPU6500_INT_STATUS, 1, &reg_value); // get current GYRO_CONFIG register value
    if (ret_code == 0 && (reg_value & 0x01)) {
        return 1;
    }
    return 0;
}

u8 mpu_9250_read_accel_gyro(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) {
    short raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
    if (mpu_9250_read_accel_gyro_raw(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz)) {
        return 1;
    }
    *ax = (raw_ax * CUR_ACCEL_RES - accel_bias_x) * G2MSS;
    *ay = (raw_ay * CUR_ACCEL_RES - accel_bias_y) * G2MSS;
    *az = (raw_az * CUR_ACCEL_RES - accel_bias_z) * G2MSS;
    *gx = (raw_gx * CUR_GYRO_RES - gyro_bias_x) * DEG2RAD;
    *gy = (raw_gy * CUR_GYRO_RES - gyro_bias_y) * DEG2RAD;
    *gz = (raw_gz * CUR_GYRO_RES - gyro_bias_z) * DEG2RAD;
    return 0;
}

u8 mpu_9250_read_mag(float *mx, float *my, float *mz) {
    short raw_mx, raw_my, raw_mz;
    if (mpu_9250_read_mag_raw(&raw_mx, &raw_my, &raw_mz)) {
        return 1;
    }
    *mx = (raw_mx * mag_adjust_x * CUR_MAG_RES - mag_bias_x) * UT2MGS;
    *my = (raw_my * mag_adjust_y * CUR_MAG_RES - mag_bias_y) * UT2MGS;
    *mz = (raw_mz * mag_adjust_z * CUR_MAG_RES - mag_bias_z) * UT2MGS;
    return 0;
}

u8 mpu_9250_get_whoami(u8 *whoami) {
    return mpu_9250_read(MPU6500_I2C_ADDR, MPU6500_WHO_AM_I, 1, whoami);
}

u8 mpu_9250_reset(void) {
    u8 reg_value = 0x80;
    u8 ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_PWR_MGMT_1, 1, &reg_value); // Write a one to bit 7 reset bit; toggle reset device
    delay_ms(100);
    return ret_code;
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
u8 mpu_6500_calibrate(void) {
    u8 reg_value;
    u8 ret_code;

    u8 data[12]; // data array to hold accelerometer and gyro x, y, z, data
    u16 i, packet_count, fifo_count;
    int gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
    u16 gyrosensitivity = 131; // = 131 LSB/degrees/sec
    u16 accelsensitivity = 16384; // = 16384 LSB/g
    
    printf("Gyro&Accel Calibration: keep still.\n");

    // reset device, reset all registers, clear gyro and accelerometer bias registers
    reg_value = 0x80;
    ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_PWR_MGMT_1, 1, &reg_value); // Write a one to bit 7 reset bit; toggle reset device
    if (ret_code) {
        goto MPU_9250_CALIBRATE_ERROR;
    }
    delay_ms(100);

    // get stable time source
    // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
    reg_value = 0x01;
    ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_PWR_MGMT_1, 1, &reg_value);
    if (ret_code) {
        goto MPU_9250_CALIBRATE_ERROR;
    }
    reg_value = 0x00;
    ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_PWR_MGMT_2, 1, &reg_value);
    if (ret_code) {
        goto MPU_9250_CALIBRATE_ERROR;
    }
    delay_ms(200);

    // Configure device for bias calculation
    reg_value = 0x00;
    ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_INT_ENABLE, 1, &reg_value);   // Disable all interrupts
    if (ret_code) {
        goto MPU_9250_CALIBRATE_ERROR;
    }
    reg_value = 0x00;
    ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_FIFO_EN, 1, &reg_value);   // Disable FIFO
    if (ret_code) {
        goto MPU_9250_CALIBRATE_ERROR;
    }
    reg_value = 0x00;
    ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_PWR_MGMT_1, 1, &reg_value);   // Turn on internal clock source
    if (ret_code) {
        goto MPU_9250_CALIBRATE_ERROR;
    }
    reg_value = 0x00;
    ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_I2C_MST_CTRL, 1, &reg_value);   // Disable I2C master
    if (ret_code) {
        goto MPU_9250_CALIBRATE_ERROR;
    }
    reg_value = 0x00;
    ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_USER_CTRL, 1, &reg_value);   // Disable FIFO and I2C master modes
    if (ret_code) {
        goto MPU_9250_CALIBRATE_ERROR;
    }
    reg_value = 0x0C;
    ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_USER_CTRL, 1, &reg_value);   // Reset FIFO and DMP
    if (ret_code) {
        goto MPU_9250_CALIBRATE_ERROR;
    }
    delay_ms(15);

    // Configure MPU9250 gyro and accelerometer for bias calculation
    reg_value = 0x01;
    ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_CONFIG, 1, &reg_value);   // Set low-pass filter to 188 Hz
    if (ret_code) {
        goto MPU_9250_CALIBRATE_ERROR;
    }
    reg_value = 0x00;
    ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_SMPLRT_DIV, 1, &reg_value);   // Set sample rate to 1 kHz
    if (ret_code) {
        goto MPU_9250_CALIBRATE_ERROR;
    }
    reg_value = 0x00;
    ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_GYRO_CONFIG, 1, &reg_value);   // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    if (ret_code) {
        goto MPU_9250_CALIBRATE_ERROR;
    }
    reg_value = 0x00;
    ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_ACCEL_CONFIG, 1, &reg_value);   // Set accelerometer full-scale to 2 g, maximum sensitivity
    if (ret_code) {
        goto MPU_9250_CALIBRATE_ERROR;
    }

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    reg_value = 0x40;
    ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_USER_CTRL, 1, &reg_value);   // Enable FIFO
    if (ret_code) {
        goto MPU_9250_CALIBRATE_ERROR;
    }

    reg_value = 0x78;
    ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_FIFO_EN, 1, &reg_value);   // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250)
    if (ret_code) {
        goto MPU_9250_CALIBRATE_ERROR;
    }
    delay_ms(40); // accumulate 40 samples in 80 milliseconds = 480 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    reg_value = 0x00;
    ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_FIFO_EN, 1, &reg_value);   // Disable gyro and accelerometer sensors for FIFO
    if (ret_code) {
        goto MPU_9250_CALIBRATE_ERROR;
    }
    ret_code = mpu_9250_read(MPU6500_I2C_ADDR, MPU6500_FIFO_COUNT, 2, &data[0]);   // read FIFO sample count
    if (ret_code) {
        goto MPU_9250_CALIBRATE_ERROR;
    }
    fifo_count = ((u16)data[0] << 8) | data[1];
    packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

    for (i = 0; i < packet_count; i++) {
        short fifo_accel[3] = {0, 0, 0}, fifi_gyro[3] = {0, 0, 0};
        ret_code = mpu_9250_read(MPU6500_I2C_ADDR, MPU6500_FIFO_R_W, 12, &data[0]);   // read data for averaging
        if (ret_code) {
            goto MPU_9250_CALIBRATE_ERROR;
        }
        fifo_accel[0] = (short)(((short)data[0] << 8) | data[1]);  // Form signed 16-bit integer for each sample in FIFO
        fifo_accel[1] = (short)(((short)data[2] << 8) | data[3]);
        fifo_accel[2] = (short)(((short)data[4] << 8) | data[5]);
        fifi_gyro[0] = (short)(((short)data[6] << 8) | data[7]);
        fifi_gyro[1] = (short)(((short)data[8] << 8) | data[9]);
        fifi_gyro[2] = (short)(((short)data[10] << 8) | data[11]);
        accel_bias[0] += (int)fifo_accel[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int)fifo_accel[1];
        accel_bias[2] += (int)fifo_accel[2];
        gyro_bias[0] += (int)fifi_gyro[0];
        gyro_bias[1] += (int)fifi_gyro[1];
        gyro_bias[2] += (int)fifi_gyro[2];
    }
    accel_bias[0] /= (int) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int) packet_count;
    accel_bias[2] /= (int) packet_count;
    gyro_bias[0] /= (int) packet_count;
    gyro_bias[1] /= (int) packet_count;
    gyro_bias[2] /= (int) packet_count;

    if (accel_bias[2] > 0L) {
        accel_bias[2] -= (int)accelsensitivity;   // Remove gravity from the z-axis accelerometer bias calculation
    }
    else {
        accel_bias[2] += (int)accelsensitivity;
    }
    gyro_bias_x = (float)gyro_bias[0] / (float)gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
    gyro_bias_y = (float)gyro_bias[1] / (float)gyrosensitivity;
    gyro_bias_z = (float)gyro_bias[2] / (float)gyrosensitivity;
    accel_bias_x = (float)accel_bias[0] / (float)accelsensitivity; // construct accel bias in deg/s for later manual subtraction
    accel_bias_y = (float)accel_bias[1] / (float)accelsensitivity;
    accel_bias_z = (float)accel_bias[2] / (float)accelsensitivity;
    return 0;
MPU_9250_CALIBRATE_ERROR:
    gyro_bias_x = 0.0f;
    gyro_bias_y = 0.0f;
    gyro_bias_z = 0.0f;
    accel_bias_x = 0.0f;
    accel_bias_y = 0.0f;
    accel_bias_z = 0.0f;
    printf("Gyro&Accel Calibration: error occur, can't read gyro&accel data!\n");
    return 1;
}

u8 mpu_6500_init() {
    u8 reg_value;
    u8 ret_code;

    // Initialize MPU 6500 device
    // wake up device
    reg_value = 0x00;
    ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_PWR_MGMT_1, 1, &reg_value); // Clear sleep mode bit (6), enable all sensors
    if (ret_code) {
        return 1;
    }
    delay_ms(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

    // get stable time source
    reg_value = 0x01;
    ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_PWR_MGMT_1, 1, &reg_value);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
    if (ret_code) {
        return 1;
    }

    // Configure Gyro and Accelerometer
    // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
    // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
    // Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
    reg_value = 0x03;
    ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_CONFIG, 1, &reg_value);
    if (ret_code) {
        return 1;
    }

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    reg_value = 0x04;
    ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_SMPLRT_DIV, 1, &reg_value);  // Use a 200 Hz rate; the same rate set in CONFIG above
    if (ret_code) {
        return 1;
    }

    // Set gyroscope full scale range
    // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    ret_code = mpu_9250_read(MPU6500_I2C_ADDR, MPU6500_GYRO_CONFIG, 1, &reg_value); // get current GYRO_CONFIG register value
    if (ret_code) {
        return 1;
    }
    // reg_value = reg_value & ~0xE0; // Clear self-test bits [7:5]
    reg_value = reg_value & ~0x02; // Clear Fchoice bits [1:0]
    reg_value = reg_value & ~0x18; // Clear AFS bits [4:3]
    reg_value = reg_value | CUR_GYRO_FS; // Set full scale range for the gyro
    // reg_value =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
    ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_GYRO_CONFIG, 1, &reg_value); // Write new GYRO_CONFIG value to register
    if (ret_code) {
        return 1;
    }

    // Set accelerometer full-scale range configuration
    ret_code = mpu_9250_read(MPU6500_I2C_ADDR, MPU6500_ACCEL_CONFIG, 1, &reg_value); // get current ACCEL_CONFIG register value
    if (ret_code) {
        return 1;
    }
    // reg_value = reg_value & ~0xE0; // Clear self-test bits [7:5]
    reg_value = reg_value & ~0x18;  // Clear AFS bits [4:3]
    reg_value = reg_value | CUR_ACCEL_FS; // Set full scale range for the accelerometer
    ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_ACCEL_CONFIG, 1, &reg_value); // Write new ACCEL_CONFIG register value
    if (ret_code) {
        return 1;
    }

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    ret_code = mpu_9250_read(MPU6500_I2C_ADDR, MPU6500_ACCEL_CONFIG2, 1, &reg_value); // get current ACCEL_CONFIG2 register value
    if (ret_code) {
        return 1;
    }
    reg_value = reg_value & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    reg_value = reg_value | CUR_ACCEL_LPS;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_ACCEL_CONFIG2, 1, &reg_value); // Write new ACCEL_CONFIG2 register value
    if (ret_code) {
        return 1;
    }

    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
    // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled by the Arduino as master
    reg_value = 0x22;
    ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_INT_PIN_CFG, 1, &reg_value);
    if (ret_code) {
        return 1;
    }
    reg_value = 0x01;
    ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_INT_ENABLE, 1, &reg_value); // Enable data ready (bit 0) interrupt
    if (ret_code) {
        return 1;
    }
    return 0;
}

u8 ak8963_init(void) {
    u8 reg_value;
    u8 ret_code;
    // First extract the factory calibration for each magnetometer axis
    u8 tmp_x, tmp_y, tmp_z;  // x/y/z gyro calibration data stored here

    reg_value = 0x00;
    ret_code = mpu_9250_write(AK8963_I2C_ADDR, AK8963_CNTL, 1, &reg_value); // Power down magnetometer
    if (ret_code) {
        goto AK8963_INIT_ERROR;
    }
    delay_ms(10);
    reg_value = 0x0F;
    ret_code = mpu_9250_write(AK8963_I2C_ADDR, AK8963_CNTL, 1, &reg_value); // Enter Fuse ROM access mode
    if (ret_code) {
        goto AK8963_INIT_ERROR;
    }
    delay_ms(10);

    // Read the x-, y-, and z-axis calibration values
    if (mpu_9250_read(AK8963_I2C_ADDR, AK8963_ASAX, 1, &tmp_x) ) {
        goto AK8963_INIT_ERROR;
    }
    if (mpu_9250_read(AK8963_I2C_ADDR, AK8963_ASAY, 1, &tmp_y) ) {
        goto AK8963_INIT_ERROR;
    }
    if (mpu_9250_read(AK8963_I2C_ADDR, AK8963_ASAZ, 1, &tmp_z) ) {
        goto AK8963_INIT_ERROR;
    }

    mag_adjust_x = (float)(tmp_x - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
    mag_adjust_y = (float)(tmp_y - 128)/256.0f + 1.0f;
    mag_adjust_z = (float)(tmp_z - 128)/256.0f + 1.0f;
    reg_value = 0x00;
    ret_code = mpu_9250_write(AK8963_I2C_ADDR, AK8963_CNTL, 1, &reg_value); // Power down magnetometer
    if (ret_code) {
        goto AK8963_INIT_ERROR;
    }
    delay_ms(10);
    // Configure the magnetometer for continuous read and highest resolution
    // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
    // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
    reg_value = CUR_MAG_FS | CUR_MAG_LPS;
    ret_code = mpu_9250_write(AK8963_I2C_ADDR, AK8963_CNTL, 1, &reg_value); // Set magnetometer data resolution and sample ODR
    if (ret_code) {
        goto AK8963_INIT_ERROR;
    }
    delay_ms(10);
    return 0;
AK8963_INIT_ERROR:
    mag_adjust_x = 0;
    mag_adjust_y = 0;
    mag_adjust_z = 0;
    return 1;
}

u8 ak8963_calibrate(void) {
    u16 i = 0, sample_count = 0;
    int mag_bias[3] = {0, 0, 0};
    short mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_raw[3] = {0, 0, 0};
    printf("Mag Calibration: Wave device in a figure eight until done!\n");
    delay_ms(4000);
    // shoot for ~fifteen seconds of mag data
    if (CUR_MAG_LPS == MPU9250_MAG_LPS_8HZ) {
        sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
    } else if (CUR_MAG_LPS == MPU9250_MAG_LPS_100HZ) {
        sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
    }
    for(i = 0; i < sample_count; i++) {
        // Read the mag data
        if (mpu_9250_read_mag_raw(&mag_raw[0], &mag_raw[1], &mag_raw[2])) {
            goto AK8963_CALIBRATE_ERROR;
        }
        for (int jj = 0; jj < 3; jj++) {
            if(mag_raw[jj] > mag_max[jj]) mag_max[jj] = mag_raw[jj];
            if(mag_raw[jj] < mag_min[jj]) mag_min[jj] = mag_raw[jj];
        }
        if (CUR_MAG_LPS == MPU9250_MAG_LPS_8HZ) {
            delay_ms(135); // at 8 Hz ODR, new mag data is available every 125 ms
        } else if (CUR_MAG_LPS == MPU9250_MAG_LPS_100HZ) {
            delay_ms(12);  // at 100 Hz ODR, new mag data is available every 10 ms
        }
    }
    // Get hard iron correction
    mag_bias[0] = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1] = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2] = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
    mag_bias_x = (float) mag_bias[0] * mag_adjust_x * CUR_MAG_RES;  // save mag biases in G for main program
    mag_bias_y = (float) mag_bias[1] * mag_adjust_y * CUR_MAG_RES;
    mag_bias_z = (float) mag_bias[2] * mag_adjust_z * CUR_MAG_RES;
    printf("Mag Calibration done!\n");
    return 0;
AK8963_CALIBRATE_ERROR:
    printf("Mag Calibration: error occur, can't read mag data!\n");
    return 1;
}

u8 mpu_9250_init(void) {
    u8 whoami = 0x0;
    mpu_i2c_init();
    if (mpu_9250_get_whoami(&whoami)) {
        return 1;
    }
    if (whoami != MPU6500_DEVICE_ID) {
        return 2;
    }
    delay_ms(1000);
    if (mpu_9250_reset()) {
        return 1;
    }
    if (mpu_6500_calibrate()) {
        return 1;
    }
    printf("Gyro bias: %f, %f, %f Accel bias: %f, %f, %f\n", gyro_bias_x, gyro_bias_y, gyro_bias_z, accel_bias_x, accel_bias_y, accel_bias_z);
    delay_ms(2000);
    if (mpu_6500_init()) {
        return 1;
    }
    if (ak8963_init()) {
        return 1;
    }
#if 0
    if (ak8963_calibrate()) {
        return 1;
    }
#else
    mag_bias_x = 0.357276f;
    mag_bias_y = 8.217358f;
    mag_bias_z = -12.914664f;
#endif
    printf("Mag adjust: %f, %f, %f Mag bias: %f, %f, %f\n", mag_adjust_x, mag_adjust_y, mag_adjust_z, mag_bias_x, mag_bias_y, mag_bias_z);
    delay_ms(2000);
    return 0;
}
