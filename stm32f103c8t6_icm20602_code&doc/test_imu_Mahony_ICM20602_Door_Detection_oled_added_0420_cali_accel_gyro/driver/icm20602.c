#include "icm20602.h"
#include "delay.h"
#include "spi.h"
#include <math.h>

//========ICM20602寄存器地址========================
/********************************************
*复位后所有寄存器地址都为0，除了
*Register 26  CONFIG				= 0x80
*Register 107 Power Management 1 	= 0x41
*Register 117 WHO_AM_I 				= 0x12
*********************************************/
//陀螺仪温度补偿
#define	ICM20_XG_OFFS_TC_H				0x04
#define	ICM20_XG_OFFS_TC_L				0x05
#define	ICM20_YG_OFFS_TC_H				0x07
#define	ICM20_YG_OFFS_TC_L				0x08
#define	ICM20_ZG_OFFS_TC_H				0x0A
#define	ICM20_ZG_OFFS_TC_L				0x0B
//加速度自检输出(出产时设置，用于与用户的自检输出值比较）
#define	ICM20_SELF_TEST_X_ACCEL			0x0D
#define	ICM20_SELF_TEST_Y_ACCEL			0x0E
#define	ICM20_SELF_TEST_Z_ACCEL			0x0F
//陀螺仪静态偏移
#define	ICM20_XG_OFFS_USRH				0x13
#define	ICM20_XG_OFFS_USRL				0x14
#define	ICM20_YG_OFFS_USRH				0x15
#define	ICM20_YG_OFFS_USRL				0x16
#define	ICM20_ZG_OFFS_USRH				0x17
#define	ICM20_ZG_OFFS_USRL				0x18

#define	ICM20_SMPLRT_DIV				0x19
#define	ICM20_CONFIG					0x1A
#define	ICM20_GYRO_CONFIG				0x1B
#define	ICM20_ACCEL_CONFIG				0x1C
#define	ICM20_ACCEL_CONFIG2				0x1D
#define	ICM20_LP_MODE_CFG				0x1E

//运动唤醒加速度阈值
#define	ICM20_ACCEL_WOM_X_THR			0x20
#define	ICM20_ACCEL_WOM_Y_THR			0x21
#define	ICM20_ACCEL_WOM_Z_THR			0x22


#define	ICM20_FIFO_EN					0x23
//#define ICM20_
#define	ICM20_FSYNC_INT					0x36
#define	ICM20_INT_PIN_CFG				0x37
#define	ICM20_INT_ENABLE				0x38
#define	ICM20_FIFO_WM_INT_STATUS		0x39
#define	ICM20_INT_STATUS				0x3A

//加速度输出
#define	ICM20_ACCEL_XOUT_H				0x3B
#define	ICM20_ACCEL_XOUT_L				0x3C
#define	ICM20_ACCEL_YOUT_H				0x3D
#define	ICM20_ACCEL_YOUT_L				0x3E
#define	ICM20_ACCEL_ZOUT_H				0x3F
#define	ICM20_ACCEL_ZOUT_L				0x40
//温度输出
#define	ICM20_TEMP_OUT_H				0x41
#define	ICM20_TEMP_OUT_L				0x42
//角速度输出
#define	ICM20_GYRO_XOUT_H				0x43
#define	ICM20_GYRO_XOUT_L				0x44
#define	ICM20_GYRO_YOUT_H				0x45
#define	ICM20_GYRO_YOUT_L				0x46
#define	ICM20_GYRO_ZOUT_H				0x47
#define	ICM20_GYRO_ZOUT_L				0x48
//陀螺仪自检输出
#define	ICM20_SELF_TEST_X_GYRO			0x50
#define	ICM20_SELF_TEST_Y_GYRO			0x51
#define	ICM20_SELF_TEST_Z_GYRO			0x52

#define	ICM20_FIFO_WM_TH1				0x60
#define	ICM20_FIFO_WM_TH2				0x61
#define	ICM20_SIGNAL_PATH_RESET			0x68
#define	ICM20_ACCEL_INTEL_CTRL 			0x69
#define	ICM20_USER_CTRL					0x6A
//电源控制
#define	ICM20_PWR_MGMT_1				0x6B
#define	ICM20_PWR_MGMT_2				0x6C

#define	ICM20_I2C_IF					0x70
#define	ICM20_FIFO_COUNTH				0x72
#define	ICM20_FIFO_COUNTL				0x73
#define	ICM20_FIFO_R_W					0x74

#define	ICM20_WHO_AM_I 					0x75
//加速度静态偏移
#define	ICM20_XA_OFFSET_H				0x77
#define	ICM20_XA_OFFSET_L				0x78
#define	ICM20_YA_OFFSET_H				0x7A
#define	ICM20_YA_OFFSET_L				0x7B
#define	ICM20_ZA_OFFSET_H				0x7D
#define	ICM20_ZA_OFFSET_L 				0x7E
//===========================================================
float gyro_bias_x = 0.0f;
float gyro_bias_y = 0.0f;
float gyro_bias_z = 0.0f;
float accel_bias_x = 0.0f;
float accel_bias_y = 0.0f;
float accel_bias_z = 0.0f;

static float _accel_scale;
static float _gyro_scale;

#define ICM20602_ADDRESS	0xD2

#define GRAVITY_MSS 9.80665f
#define _ACCEL_SCALE_1G    (GRAVITY_MSS / 4096.0f) //量程8G
#define _DEG_TO_RAD    0.0174532f //量程8G

uint8_t icm20602_write_reg(uint8_t reg, uint8_t val) {
    //return myiic_write_reg(ICM20602_ADDRESS,reg,val);
    CS_ICM = 0;
    spi2_write_reg(reg, val);
    CS_ICM = 1;
    return 0;
}

uint8_t icm20602_read_reg(uint8_t reg) {
    uint8_t res;
    //return myiic_read_reg(ICM20602_ADDRESS,reg);
    CS_ICM = 0;
    res = spi2_read_reg(reg);
    CS_ICM = 1;
    return res;
}

uint8_t icm20602_read_buffer(uint8_t reg, void *buffer, uint8_t len) {
    //return myiic_read_buffer(ICM20602_ADDRESS,reg,len,buffer);
    CS_ICM = 0;
    spi2_read_reg_buffer(reg, buffer, len);
    CS_ICM = 1;
    return 0;
}

//ICM20602 Calibrate
u8 icm20602_calibrate(void) {
    u8 data[12];
    u16 i, packet_count = 3000;
    int gyro_bias[3] = {0}, accel_bias[3] = {0};
    printf("Keep still!Calibrate Gyro&Accel!\n");
    //reset device, reset all registers, cleaer gyro and accel bias registers  reg_val = 0x80;
    if(icm20602_write_reg(ICM20_PWR_MGMT_1, 0x80)) { //复位，复位后位0x41,睡眠模式
        puts("icm_20602 reset fail\r\n");
        return 1;
    }

    delay_ms(50);
    icm20602_write_reg(ICM20_PWR_MGMT_1, 0x01); //关闭睡眠，自动选择时钟
    delay_ms(50);

    //printf("icm_20602 id=%x\r\n",icm20602_read_reg(ICM20_WHO_AM_I)); //读取ID
    icm20602_write_reg(ICM20_SMPLRT_DIV, 0); //分频数=为0+1，数据输出速率为内部采样速率
    icm20602_write_reg(ICM20_CONFIG, DLPF_BW_20); //GYRO低通滤波设置
    icm20602_write_reg(ICM20_ACCEL_CONFIG2, ACCEL_AVER_4 | ACCEL_DLPF_BW_21); //ACCEL低通滤波设置

    icm20602_set_accel_fullscale(ICM20_ACCEL_FS_2G);
    icm20602_set_gyro_fullscale(ICM20_GYRO_FS_250);

    delay_ms(100);
    for(i = 0; i < packet_count; i++) {
        short fifo_accel[3] = {0, 0, 0}, fifo_gyro[3] = {0, 0, 0};
        if (icm20602_get_gyro_adc(fifo_gyro)) {
            goto icm20602_cal_error;
        }
        if (icm20602_get_accel_adc(fifo_accel)) {
            goto icm20602_cal_error;
        }
        accel_bias[0] += (int)fifo_accel[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int)fifo_accel[1];
        accel_bias[2] += (int)fifo_accel[2];
        gyro_bias[0] += (int)fifo_gyro[0];
        gyro_bias[1] += (int)fifo_gyro[1];
        gyro_bias[2] += (int)fifo_gyro[2];
    }
    accel_bias[0] /= (int) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int) packet_count;
    accel_bias[2] /= (int) packet_count;
    gyro_bias[0] /= (int) packet_count;
    gyro_bias[1] /= (int) packet_count;
    gyro_bias[2] /= (int) packet_count;


    if (abs(accel_bias[0]) > 15000) {

        if (accel_bias[0] > 0L) {
            accel_bias[0] -= (int)16384;   // Remove gravity from the z-axis accelerometer bias calculation
        }
        else {
            accel_bias[0] += (int)16384;
        }
    } else if (abs(accel_bias[1]) > 15000) {

        if (accel_bias[1] > 0L) {
            accel_bias[1] -= (int)16384;   // Remove gravity from the z-axis accelerometer bias calculation
        }
        else {
            accel_bias[1] += (int)16384;
        }
    } else  {

        if (accel_bias[2] > 0L) {
            accel_bias[2] -= (int)16384;   // Remove gravity from the z-axis accelerometer bias calculation
        }
        else {
            accel_bias[2] += (int)16384;
        }
    }

    gyro_bias_x = (float)gyro_bias[0] * _gyro_scale; // construct gyro bias in deg/s for later manual subtraction
    gyro_bias_y = (float)gyro_bias[1] * _gyro_scale;
    gyro_bias_z = (float)gyro_bias[2] * _gyro_scale;
    accel_bias_x = (float)accel_bias[0] * _accel_scale; // construct accel bias in deg/s for later manual subtraction
    accel_bias_y = (float)accel_bias[1] * _accel_scale;
    accel_bias_z = (float)accel_bias[2] * _accel_scale;
    return 0;
icm20602_cal_error:
    gyro_bias_x = 0.0f;
    gyro_bias_y = 0.0f;
    gyro_bias_z = 0.0f;
    accel_bias_x = 0.0f;
    accel_bias_y = 0.0f;
    accel_bias_z = 0.0f;
    printf("Gyro&Accel Calibration: error occur, can't read gyro&accel data!\n");
    return 1;
}


uint8_t icm20602_init() {
    //icm20602_calibrate();
    //printf("Gyro bias: %f, %f, %f Accel bias: %f, %f, %f\n", gyro_bias_x, gyro_bias_y, gyro_bias_z, accel_bias_x, accel_bias_y, accel_bias_z);

    if(icm20602_write_reg(ICM20_PWR_MGMT_1, 0x80)) { //复位，复位后位0x41,睡眠模式
        puts("icm_20602 reset fail\r\n");
        return 1;
    }
    delay_ms(50);
    icm20602_write_reg(ICM20_PWR_MGMT_1, 0x01); //关闭睡眠，自动选择时钟
    delay_ms(50);

    printf("icm_20602 id=%x\r\n", icm20602_read_reg(ICM20_WHO_AM_I)); //读取ID

    icm20602_write_reg(ICM20_SMPLRT_DIV, 0); //分频数=为0+1，数据输出速率为内部采样速率
    icm20602_write_reg(ICM20_CONFIG, DLPF_BW_20); //GYRO低通滤波设置
    icm20602_write_reg(ICM20_ACCEL_CONFIG2, ACCEL_AVER_4 | ACCEL_DLPF_BW_21); //ACCEL低通滤波设置

    icm20602_set_accel_fullscale(ICM20_ACCEL_FS_2G);
    icm20602_set_gyro_fullscale(ICM20_GYRO_FS_250);

    delay_ms(100);

    return 0;
}

uint8_t icm20602_set_gyro_fullscale(uint8_t fs) {
    switch(fs) {
    case ICM20_GYRO_FS_250:
        _gyro_scale = 1.0f / 131.068f;	//32767/250
        break;
    case ICM20_GYRO_FS_500:
        _gyro_scale = 1.0f / 65.534f;
        break;
    case ICM20_GYRO_FS_1000:
        _gyro_scale = 1.0f / 32.767f;
        break;
    case ICM20_GYRO_FS_2000:
        _gyro_scale = 1.0f / 16.4f;
        break;
    default:
        fs = ICM20_GYRO_FS_2000;
        _gyro_scale = 1.0f / 16.3835f;
        break;
    }
    _gyro_scale *= _DEG_TO_RAD;
    return icm20602_write_reg(ICM20_GYRO_CONFIG, fs);
}

uint8_t icm20602_set_accel_fullscale(uint8_t fs) {
    switch(fs) {
    case ICM20_ACCEL_FS_2G:
        _accel_scale = 1.0f / 16384.0f;
        break;
    case ICM20_ACCEL_FS_4G:
        _accel_scale = 1.0f / 8192.0f;
        break;
    case ICM20_ACCEL_FS_8G:
        _accel_scale = 1.0f / 4096.0f;
        break;
    case ICM20_ACCEL_FS_16G:
        _accel_scale = 1.0f / 2048.0f;
        break;
    default:
        fs = ICM20_ACCEL_FS_8G;
        _accel_scale = 1.0f / 4096.0f;
        break;
    }
    _accel_scale *= GRAVITY_MSS;
    return icm20602_write_reg(ICM20_ACCEL_CONFIG, fs);
}

uint8_t icm20602_get_accel_adc(int16_t *accel) {
    uint8_t buf[6];
    if(icm20602_read_buffer(ICM20_ACCEL_XOUT_H, buf, 6))return 1;
    accel[0] = ((int16_t)buf[0] << 8) + buf[1];
    accel[1] = ((int16_t)buf[2] << 8) + buf[3];
    accel[2] = ((int16_t)buf[4] << 8) + buf[5];
    return 0;
}

uint8_t icm20602_get_gyro_adc(int16_t *gyro) {
    uint8_t buf[6];
    if(icm20602_read_buffer(ICM20_GYRO_XOUT_H, buf, 6))return 1;
    gyro[0] = (buf[0] << 8) + buf[1];
    gyro[1] = (buf[2] << 8) + buf[3];
    gyro[2] = (buf[4] << 8) + buf[5];
    return 0;
}

uint8_t icm20602_get_gyro(float *gyro) {
    int16_t gyro_adc[3];
    if(icm20602_get_gyro_adc(gyro_adc))return 1;

    gyro[0] = _gyro_scale * gyro_adc[0] - gyro_bias_x;
    gyro[1] = _gyro_scale * gyro_adc[1] - gyro_bias_y;
    gyro[2] = _gyro_scale * gyro_adc[2] - gyro_bias_z;
    return 0;
}

uint8_t icm20602_get_accel(float *accel) {
    int16_t accel_adc[3];
    if(icm20602_get_accel_adc(accel_adc))return 1;
    accel[0] = _accel_scale * accel_adc[0] - accel_bias_x;
    accel[1] = _accel_scale * accel_adc[1] - accel_bias_y;
    accel[2] = _accel_scale * accel_adc[2] - accel_bias_z;
    return 0;
}

float icm20602_get_temp() {
    int16_t temp_adc;
    uint8_t buf[2];
    if(icm20602_read_buffer(ICM20_TEMP_OUT_H, buf, 2))return 0.0f;
    temp_adc = (buf[0] << 8) + buf[1];
    return (25.0f + (float)temp_adc / 326.8f);
}

//if sensor is still not update, else update

u8 icm20602_get_accel_gyro_statistic(
    float *ax, float *ay, float *az, float *gx, float *gy, float *gz,
    float *mean_ax, float *mean_ay, float *mean_az, float *mean_gx, float *mean_gy, float *mean_gz,
    float *var_ax, float *var_ay, float *var_az, float *var_gx, float *var_gy, float *var_gz) 
{
    const u8 STATISTIC_NUM = 20;
    static u8 statistic_cnt = 0, statistic_full = 0;
    static float axs[STATISTIC_NUM], ays[STATISTIC_NUM], azs[STATISTIC_NUM];
    static float sum_ax = 0, sum_ay = 0, sum_az = 0;
    //float d_ax, d_ay, d_az;
    float d_accel[3];
    static float gxs[STATISTIC_NUM], gys[STATISTIC_NUM], gzs[STATISTIC_NUM];
    static float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    //float d_gx, d_gy, d_gz;
    float d_gyro[3];
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
    if (icm20602_get_accel(d_accel)) {
        return 1; // invalid
    }
    if(icm20602_get_gyro(d_gyro)) {
        return 1; //invalid
    }
    *ax = d_accel[0];
    *ay = d_accel[1];
    *az = d_accel[2];
    *gx = d_gyro[0];
    *gy = d_gyro[1];
    *gz = d_gyro[2];
    axs[statistic_cnt] = d_accel[0];
    ays[statistic_cnt] = d_accel[1];
    azs[statistic_cnt] = d_accel[2];
    gxs[statistic_cnt] = d_gyro[0];
    gys[statistic_cnt] = d_gyro[1];
    gzs[statistic_cnt] = d_gyro[2];
    sum_ax += d_accel[0];
    sum_ay += d_accel[1];
    sum_az += d_accel[2];
    sum_gx += d_gyro[0];
    sum_gy += d_gyro[1];
    sum_gz += d_gyro[2];
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
void icm20602_set_gyro_bias(float gx, float gy, float gz) {
    gyro_bias_x = gx;
    gyro_bias_y = gy;
    gyro_bias_z = gz;
}
void icm20602_set_accel_gyro_bias(float ax, float ay, float az,
                                  float gx, float gy, float gz) {
    accel_bias_x = ax;
    accel_bias_y = ay;
    accel_bias_z = az;
    gyro_bias_x = gx;
    gyro_bias_y = gy;
    gyro_bias_z = gz;
}
float low_filter(float acc_cur, float acc_prev){
    float sample_value;
    float x = 0.01;
    sample_value = (1 - x) * acc_cur + x * acc_prev;
    return sample_value;
}