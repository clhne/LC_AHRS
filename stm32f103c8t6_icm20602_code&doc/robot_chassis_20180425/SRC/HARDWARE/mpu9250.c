#include <stdio.h>
#include <math.h>
#include "delay.h"
#include "mpui2c.h"
#include "mpu9250.h"

/* 定义MPU6500内部地址及相关寄存器 */
#define MPU6500_I2C_ADDR          0x68
#define MPU6500_PWR_MGMT_1        0x6B  // 电源管理，典型值：0x00(正常启用)
#define MPU6500_PWR_MGMT_2        0x6C  // 电源管理，典型值：0x00(正常启用)
#define MPU6500_SMPLRT_DIV        0x19  // 陀螺仪采样率，典型值：0x07(125Hz)
#define MPU6500_CONFIG            0x1A  // 低通滤波频率，典型值：0x06(5Hz)
#define MPU6500_GYRO_CONFIG       0x1B  // 陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define MPU6500_ACCEL_CONFIG      0x1C  // 加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define MPU6500_ACCEL_CONFIG2     0x1D
#define MPU6500_INT_PIN_CFG       0x37
#define MPU6500_INT_ENABLE        0x38
#define MPU6500_FIFO_EN           0x23
#define MPU6500_FIFO_COUNT        0x72
#define MPU6500_FIFO_R_W          0x74
#define MPU6500_I2C_MST_CTRL      0x24
#define MPU6500_WHO_AM_I          0x75  // 器件ID查询
#define MPU6500_DEVICE_ID         0x71  // 在MPU9250中, 器件ID=0x71
#define MPU6500_USER_CTRL         0x6A  // 用户配置当为0x10时使用SPI模式
#define MPU6500_INT_STATUS        0x3A
#define MPU6500_SELF_TEST_X_GYRO  0x00
#define MPU6500_SELF_TEST_Y_GYRO  0x01
#define MPU6500_SELF_TEST_Z_GYRO  0x02
#define MPU6500_SELF_TEST_X_ACCEL 0x0D
#define MPU6500_SELF_TEST_Y_ACCEL 0x0E
#define MPU6500_SELF_TEST_Z_ACCEL 0x0F
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

u8 mpu_9250_read_accel_gyro(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) {
    short raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
    if (mpu_9250_read_accel_gyro_raw(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz)) {
        return 1;
    }
    *ax = (raw_ax * CUR_ACCEL_RES) * G2MSS - accel_bias_x;
    *ay = (raw_ay * CUR_ACCEL_RES) * G2MSS - accel_bias_y;
    *az = (raw_az * CUR_ACCEL_RES) * G2MSS - accel_bias_z;
    *gx = (raw_gx * CUR_GYRO_RES) * DEG2RAD  - gyro_bias_x;
    *gy = (raw_gy * CUR_GYRO_RES) * DEG2RAD - gyro_bias_y;
    *gz = (raw_gz * CUR_GYRO_RES) * DEG2RAD - gyro_bias_z;
    return 0;
}


u8 mpu_9250_read_accel_gyro_and_statistic(
    float *ax, float *ay, float *az,
    float *gx, float *gy, float *gz,
    float *mean_ax, float *mean_ay, float *mean_az,
    float *var_ax, float *var_ay, float *var_az,
    float *mean_gx, float *mean_gy, float *mean_gz,
    float *var_gx, float *var_gy, float *var_gz
) {
    const u8 STATISTIC_NUM = 20;
    static u8 statistic_cnt = 0, statistic_full = 0;
    static float axs[STATISTIC_NUM], ays[STATISTIC_NUM], azs[STATISTIC_NUM];
    static float sum_ax = 0, sum_ay = 0, sum_az = 0;
    float d_ax, d_ay, d_az;
    static float gxs[STATISTIC_NUM], gys[STATISTIC_NUM], gzs[STATISTIC_NUM];
    static float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    float d_gx, d_gy, d_gz;
    u8 i;
    *ax = 0;
    *ay = 0;
    *az = 0;
    *mean_ax = 0;
    *mean_ay = 0;
    *mean_az = 0;
    *var_ax = 0;
    *var_ay = 0;
    *var_az = 0;
    *gx = 0;
    *gy = 0;
    *gz = 0;
    *mean_gx = 0;
    *mean_gy = 0;
    *mean_gz = 0;
    *var_gx = 0;
    *var_gy = 0;
    *var_gz = 0;
    if (mpu_9250_read_accel_gyro(&d_ax, &d_ay, &d_az, &d_gx, &d_gy, &d_gz)) {
        return 1; // invalid
    }
    *ax = d_ax;
    *ay = d_ay;
    *az = d_az;
    *gx = d_gx;
    *gy = d_gy;
    *gz = d_gz;
    axs[statistic_cnt] = d_ax;
    ays[statistic_cnt] = d_ay;
    azs[statistic_cnt] = d_az;
    gxs[statistic_cnt] = d_gx;
    gys[statistic_cnt] = d_gy;
    gzs[statistic_cnt] = d_gz;
    sum_ax += d_ax;
    sum_ay += d_ay;
    sum_az += d_az;
    sum_gx += d_gx;
    sum_gy += d_gy;
    sum_gz += d_gz;
    if (++statistic_cnt >= STATISTIC_NUM) {
        statistic_cnt = 0;
        statistic_full = 1;
    }
    if (statistic_full == 1) {
        float m_ax = sum_ax / (float)STATISTIC_NUM;
        float m_ay = sum_ay / (float)STATISTIC_NUM;
        float m_az = sum_az / (float)STATISTIC_NUM;
        float m_gx = sum_gx / (float)STATISTIC_NUM;
        float m_gy = sum_gy / (float)STATISTIC_NUM;
        float m_gz = sum_gz / (float)STATISTIC_NUM;
        float v_ax = 0, v_ay = 0, v_az = 0, v_gx = 0, v_gy = 0, v_gz = 0;
        for (i = 0; i < STATISTIC_NUM; i++) {
            float diff = axs[i] - m_ax;
            v_ax += diff * diff;
            diff = ays[i] - m_ay;
            v_ay += diff * diff;
            diff = azs[i] - m_az;
            v_az += diff * diff;
            //diff = gxs[i] - m_gx;
            //v_gx += diff * diff;
            //diff = gys[i] - m_gy;
            //v_gy += diff * diff;
            //diff = gzs[i] - m_gz;
            //v_gz += diff * diff;
        }
        *mean_ax = m_ax;
        *mean_ay = m_ay;
        *mean_az = m_az;
        *mean_gx = m_gx;
        *mean_gy = m_gy;
        *mean_gz = m_gz;
        *var_ax = v_ax / ((float)STATISTIC_NUM - 1);
        *var_ay = v_ay / ((float)STATISTIC_NUM - 1);
        *var_az = v_az / ((float)STATISTIC_NUM - 1);
        //*var_gx = v_gx / ((float)STATISTIC_NUM - 1);
        //*var_gy = v_gy / ((float)STATISTIC_NUM - 1);
        //*var_gz = v_gz / ((float)STATISTIC_NUM - 1);
        sum_ax -= axs[statistic_cnt];
        sum_ay -= ays[statistic_cnt];
        sum_az -= azs[statistic_cnt];
        sum_gx -= gxs[statistic_cnt];
        sum_gy -= gys[statistic_cnt];
        sum_gz -= gzs[statistic_cnt];
        return 0; // mean and var is valid
    }
    return 1; // invalid
}

void mpu_9250_set_accel_gyro_bias(float ax, float ay, float az, float gx, float gy, float gz) {
    accel_bias_x = ax;
    accel_bias_y = ay;
    accel_bias_z = az;
    gyro_bias_x = gx;
    gyro_bias_y = gy;
    gyro_bias_z = gz;
}

u8 mpu_9250_prepare_mag_data(void) {
    // turn on bypass mode and read mag data
    u8 reg_value = 0x01;
    if (mpu_9250_write(AK8963_I2C_ADDR, 0x0A, 1, &reg_value)) {
        return 1;
    }
    return 0;
}

u8 mpu_9250_read_mag_raw(short *raw_mx, short *raw_my, short *raw_mz) {
    u8 tmp_buf[6];
    if (mpu_9250_read(AK8963_I2C_ADDR, AK8963_DATA_START, 6, tmp_buf)) {
        return 1;
    }
    *raw_mx = (((short)tmp_buf[1]) << 8) | (short)tmp_buf[0];
    *raw_my = (((short)tmp_buf[3]) << 8) | (short)tmp_buf[2];
    *raw_mz = (((short)tmp_buf[5]) << 8) | (short)tmp_buf[4];
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

u8 mpu_9250_read_mag_and_statistic(
    float *mx, float *my, float *mz,
    float *mean_mx, float *mean_my, float *mean_mz,
    float *var_mx, float *var_my, float *var_mz
) {
    const u8 STATISTIC_NUM = 20;
    static u8 statistic_cnt = 0, statistic_full = 0;
    static float mxs[STATISTIC_NUM], mys[STATISTIC_NUM], mzs[STATISTIC_NUM];
    static float sum_mx = 0, sum_my = 0, sum_mz = 0;
    float d_mx, d_my, d_mz;
    u8 i;
    *mx = 0;
    *my = 0;
    *mz = 0;
    *mean_mx = 0;
    *mean_my = 0;
    *mean_mz = 0;
    *var_mx = 0;
    *var_my = 0;
    *var_mz = 0;
    if (mpu_9250_read_mag(&d_mx, &d_my, &d_mz)) {
        return 1;
    }
    *mx = d_mx;
    *my = d_my;
    *mz = d_mz;
    mxs[statistic_cnt] = d_mx;
    mys[statistic_cnt] = d_my;
    mzs[statistic_cnt] = d_mz;
    sum_mx += d_mx;
    sum_my += d_my;
    sum_mz += d_mz;
    if (++statistic_cnt >= STATISTIC_NUM) {
        statistic_cnt = 0;
        statistic_full = 1;
    }
    if (statistic_full == 1) {
        float m_mx, m_my, m_mz;
        float v_mx = 0, v_my = 0, v_mz = 0;
        m_mx = sum_mx / (float)STATISTIC_NUM;
        m_my = sum_my / (float)STATISTIC_NUM;
        m_mz = sum_mz / (float)STATISTIC_NUM;
        for (i = 0; i < STATISTIC_NUM; i++) {
            float diff = mxs[i] - m_mx;
            v_mx += diff * diff;
            diff = mys[i] - m_my;
            v_my += diff * diff;
            diff = mzs[i] - m_mz;
            v_mz += diff * diff;
        }
        *mean_mx = m_mx;
        *mean_my = m_my;
        *mean_mz = m_mz;
        *var_mx = v_mx / ((float)STATISTIC_NUM - 1);
        *var_my = v_my / ((float)STATISTIC_NUM - 1);
        *var_mz = v_mz / ((float)STATISTIC_NUM - 1);
        sum_mx -= mxs[statistic_cnt];
        sum_my -= mys[statistic_cnt];
        sum_mz -= mzs[statistic_cnt];
        return 0; // mean and var is valid
    }
    return 1; // invalid
}

void mpu_9250_set_mag_bias(float mx, float my, float mz) {
    mag_bias_x = mx;
    mag_bias_y = my;
    mag_bias_z = mz;
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

// Accelerometer and gyroscope self test; check calibration wrt factory settings
u8 mpu_6500_selftest(float *self_test_ax, float *self_test_ay, float *self_test_az, float *self_test_gx, float *self_test_gy, float *self_test_gz) {// Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
    u8 ret_code;
    u8 reg_value;
    short raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
    int avg_ax = 0, avg_ay = 0, avg_az = 0, avg_gx = 0, avg_gy = 0, avg_gz = 0;
    int avg_st_ax = 0, avg_st_ay = 0, avg_st_az = 0, avg_st_gx = 0, avg_st_gy = 0, avg_st_gz = 0;
    u8 st_ax, st_ay, st_az, st_gx, st_gy, st_gz;
    float ft_ax, ft_ay, ft_az, ft_gx, ft_gy, ft_gz;

    reg_value = 0x00;
    ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_SMPLRT_DIV, 1, &reg_value); // Set gyro sample rate to 1 kHz
    if (ret_code) {
        goto MPU_6500_SELFTEST_ERROR;
    }
    reg_value = MPU9250_GYRO_LPS_92HZ;
    ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_CONFIG, 1, &reg_value); // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
    if (ret_code) {
        goto MPU_6500_SELFTEST_ERROR;
    }
    reg_value = MPU9250_GYRO_FS_250DPS;
    ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_GYRO_CONFIG, 1, &reg_value); // Set full scale range for the gyro to 250 dps
    if (ret_code) {
        goto MPU_6500_SELFTEST_ERROR;
    }
    reg_value = MPU9250_ACCEL_LPS_92HZ;
    ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_ACCEL_CONFIG2, 1, &reg_value); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
    if (ret_code) {
        goto MPU_6500_SELFTEST_ERROR;
    }
    reg_value = MPU9250_ACCEL_FS_2G;
    ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_ACCEL_CONFIG, 1, &reg_value); // Set full scale range for the accelerometer to 2 g
    if (ret_code) {
        goto MPU_6500_SELFTEST_ERROR;
    }
    for (int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer
        ret_code = mpu_9250_read_accel_gyro_raw(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);
        if (ret_code) {
            goto MPU_6500_SELFTEST_ERROR;
        }
        avg_ax += raw_ax;
        avg_ay += raw_ay;
        avg_az += raw_az;
        avg_gx += raw_gx;
        avg_gy += raw_gy;
        avg_gz += raw_gz;
    }
    // Get average of 200 values and store as average current readings
    avg_ax /= 200;
    avg_ay /= 200;
    avg_az /= 200;
    avg_gx /= 200;
    avg_gy /= 200;
    avg_gz /= 200;

    // Sonfigure the accelerometer for self-test
    reg_value = 0xE0;
    ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_ACCEL_CONFIG, 1, &reg_value); // Enable self test on all three axes and set accelerometer range to +/- 2 g
    if (ret_code) {
        goto MPU_6500_SELFTEST_ERROR;
    }
    reg_value = 0xE0;
    ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_GYRO_CONFIG, 1, &reg_value); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
    if (ret_code) {
        goto MPU_6500_SELFTEST_ERROR;
    }
    delay_ms(25);  // Delay a while to let the device stabilize

    for (int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer
        ret_code = mpu_9250_read_accel_gyro_raw(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);
        if (ret_code) {
            goto MPU_6500_SELFTEST_ERROR;
        }
        avg_st_ax += raw_ax;
        avg_st_ay += raw_ay;
        avg_st_az += raw_az;
        avg_st_gx += raw_gx;
        avg_st_gy += raw_gy;
        avg_st_gz += raw_gz;
    }

    // Get average of 200 values and store as average self-test readings
    avg_st_ax /= 200;
    avg_st_ay /= 200;
    avg_st_az /= 200;
    avg_st_gx /= 200;
    avg_st_gy /= 200;
    avg_st_gz /= 200;

    // Configure the gyro and accelerometer for normal operation
    reg_value = 0x00;
    ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_ACCEL_CONFIG, 1, &reg_value);
    if (ret_code) {
        goto MPU_6500_SELFTEST_ERROR;
    }
    reg_value = 0x00;
    ret_code = mpu_9250_write(MPU6500_I2C_ADDR, MPU6500_GYRO_CONFIG, 1, &reg_value);
    if (ret_code) {
        goto MPU_6500_SELFTEST_ERROR;
    }
    delay_ms(25);  // Delay a while to let the device stabilize

    // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
    ret_code = mpu_9250_read(MPU6500_I2C_ADDR, MPU6500_SELF_TEST_X_ACCEL, 1, &st_ax); // X-axis accel self-test results
    if (ret_code) {
        goto MPU_6500_SELFTEST_ERROR;
    }
    ret_code = mpu_9250_read(MPU6500_I2C_ADDR, MPU6500_SELF_TEST_Y_ACCEL, 1, &st_ay); // Y-axis accel self-test results
    if (ret_code) {
        goto MPU_6500_SELFTEST_ERROR;
    }
    ret_code = mpu_9250_read(MPU6500_I2C_ADDR, MPU6500_SELF_TEST_Z_ACCEL, 1, &st_az); // Z-axis accel self-test results
    if (ret_code) {
        goto MPU_6500_SELFTEST_ERROR;
    }
    ret_code = mpu_9250_read(MPU6500_I2C_ADDR, MPU6500_SELF_TEST_X_GYRO, 1, &st_gx); // X-axis gyro self-test results
    if (ret_code) {
        goto MPU_6500_SELFTEST_ERROR;
    }
    ret_code = mpu_9250_read(MPU6500_I2C_ADDR, MPU6500_SELF_TEST_Y_GYRO, 1, &st_gy); // Y-axis gyro self-test results
    if (ret_code) {
        goto MPU_6500_SELFTEST_ERROR;
    }
    ret_code = mpu_9250_read(MPU6500_I2C_ADDR, MPU6500_SELF_TEST_Z_GYRO, 1, &st_gz); // Z-axis gyro self-test results
    if (ret_code) {
        goto MPU_6500_SELFTEST_ERROR;
    }

    // Retrieve factory self-test value from self-test code reads
    ft_ax = (float)(2620/1<<0)*(pow(1.01, ((float)st_ax - 1.0)));
    ft_ay = (float)(2620/1<<0)*(pow(1.01, ((float)st_ay - 1.0)));
    ft_az = (float)(2620/1<<0)*(pow(1.01, ((float)st_az - 1.0)));
    ft_gx = (float)(2620/1<<0)*(pow(1.01, ((float)st_gx - 1.0)));
    ft_gy = (float)(2620/1<<0)*(pow(1.01, ((float)st_gy - 1.0)));
    ft_gz = (float)(2620/1<<0)*(pow(1.01, ((float)st_gz - 1.0)));

    // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
    // To get percent, must multiply by 100
    *self_test_ax = 100.0 * ((float)(avg_st_ax - avg_ax)) / ft_ax - 100.; // Report percent differences
    *self_test_ay = 100.0 * ((float)(avg_st_ay - avg_ay)) / ft_ay - 100.;
    *self_test_az = 100.0 * ((float)(avg_st_az - avg_az)) / ft_az - 100.;
    *self_test_gx = 100.0 * ((float)(avg_st_gx - avg_gx)) / ft_gx - 100.;
    *self_test_gy = 100.0 * ((float)(avg_st_gy - avg_gy)) / ft_gy - 100.;
    *self_test_gz = 100.0 * ((float)(avg_st_gz - avg_gz)) / ft_gz - 100.;
    return 0;
MPU_6500_SELFTEST_ERROR:
    *self_test_ax = 0.0f;
    *self_test_ay = 0.0f;
    *self_test_az = 0.0f;
    *self_test_gx = 0.0f;
    *self_test_gy = 0.0f;
    *self_test_gz = 0.0f;
    printf("Gyro&Accel SelfTest: error occur, r/w register failed!\n");
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
    reg_value = 0x82; // 0x82, 0x22
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
        if (mpu_9250_prepare_mag_data()) {
            goto AK8963_CALIBRATE_ERROR;
        }
        if (CUR_MAG_LPS == MPU9250_MAG_LPS_8HZ) {
            delay_ms(135); // at 8 Hz ODR, new mag data is available every 125 ms
        } else if (CUR_MAG_LPS == MPU9250_MAG_LPS_100HZ) {
            delay_ms(12);  // at 100 Hz ODR, new mag data is available every 10 ms
        }
        // Read the mag data
        if (mpu_9250_read_mag_raw(&mag_raw[0], &mag_raw[1], &mag_raw[2])) {
            goto AK8963_CALIBRATE_ERROR;
        }
        for (int jj = 0; jj < 3; jj++) {
            if(mag_raw[jj] > mag_max[jj]) mag_max[jj] = mag_raw[jj];
            if(mag_raw[jj] < mag_min[jj]) mag_min[jj] = mag_raw[jj];
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
    float self_test_ax, self_test_ay, self_test_az, self_test_gx, self_test_gy, self_test_gz;
    u8 whoami = 0x0;
    mpu_i2c_init();
    if (mpu_9250_get_whoami(&whoami)) {
        return 1;
    }
    printf("device id: 0x%X\n", whoami);
    //if (whoami != MPU6500_DEVICE_ID) {
    //    return 2;
    //}
    delay_ms(1000);
    if (mpu_6500_selftest(&self_test_ax, &self_test_ay, &self_test_az, &self_test_gx, &self_test_gy, &self_test_gz)) {
        return 1;
    }
    printf("Gyro self test: %f, %f, %f Accel self test: %f, %f, %f\n", self_test_gx, self_test_gy, self_test_gz, self_test_ax, self_test_ay, self_test_az);
    if (mpu_9250_reset()) {
        return 1;
    }
    delay_ms(1000);
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
