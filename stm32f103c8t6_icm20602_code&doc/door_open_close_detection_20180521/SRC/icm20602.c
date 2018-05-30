#include <math.h>
#include "icm20602.h"
#include "delay.h"
#include "spi.h"
/* 寄存器地址 */
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
#define	ICM20_FSYNC_INT					0x36
#define	ICM20_INT_PIN_CFG				0x37
//#define	ICM20_INT_ENABLE				0x38
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

/* 量程设置 */
//加速度量程
#define ICM20_ACCEL_FS_2G			(0<<3)
#define ICM20_ACCEL_FS_4G			(1<<3)
#define ICM20_ACCEL_FS_8G			(2<<3)
#define ICM20_ACCEL_FS_16G			(3<<3)
//角速度量程
#define ICM20_GYRO_FS_250			(0<<3)
#define ICM20_GYRO_FS_500			(1<<3)
#define ICM20_GYRO_FS_1000			(2<<3)
#define ICM20_GYRO_FS_2000			(3<<3)
//CONFIG DPF
#define DLPF_BW_250         	0x00	//Rate=8k
#define DLPF_BW_176         	0x01
#define DLPF_BW_92          	0x02
#define DLPF_BW_41          	0x03
#define DLPF_BW_20          	0x04
#define DLPF_BW_10          	0x05
#define DLPF_BW_5           	0x06
#define DLPF_BW_328           	0x06	//Rate=8k
//ACCEL_CONFIG2
#define ACCEL_AVER_4         	(0x00<<4)	//Rate=8k
#define ACCEL_AVER_8			(0x01<<4)
#define ACCEL_AVER_16			(0x02<<4)
#define ACCEL_AVER_32			(0x03<<4)
//ACCEL_DLPF
#define ACCEL_DLPF_BW_218         	0x00
//#define ACCEL_DLPF_BW_218         	0x01
#define ACCEL_DLPF_BW_99          	0x02
#define ACCEL_DLPF_BW_44          	0x03
#define ACCEL_DLPF_BW_21          	0x04
#define ACCEL_DLPF_BW_10          	0x05
#define ACCEL_DLPF_BW_5           	0x06
#define ACCEL_DLPF_BW_420           0x06

static float _accel_scale;
static float _gyro_scale;

#define GRAVITY_MSS 9.80665f
#define _ACCEL_SCALE_1G    (GRAVITY_MSS / 4096.0f) //量程8G
#define _DEG_TO_RAD    0.0174532f //量程8G

uint8_t icm20602_write_reg(uint8_t reg,uint8_t val) {
    CS_ICM = 0;
    spi2_write_reg(reg,val);
    CS_ICM =1;
    return 0;
}

uint8_t icm20602_read_reg(uint8_t reg) {
    uint8_t res;
    CS_ICM = 0;
    res = spi2_read_reg(reg);
    CS_ICM =1;
    return res;
}

uint8_t icm20602_read_buffer(uint8_t reg,void *buffer,uint8_t len) {
    CS_ICM = 0;
    spi2_read_reg_buffer(reg,buffer,len);
    CS_ICM = 1;
    return 0;
}

uint8_t icm20602_init() {
    if (icm20602_write_reg(ICM20_PWR_MGMT_1, 0x80)) { // 复位，复位后位0x41,睡眠模式
        puts("icm_20602 reset fail\r\n");
        return 1;
    }

    delay_ms(50);
    icm20602_write_reg(ICM20_PWR_MGMT_1, 0x01); // 关闭睡眠，自动选择时钟
    delay_ms(50);

    printf("icm_20602 id=%x\r\n",icm20602_read_reg(ICM20_WHO_AM_I)); // 读取ID

    icm20602_write_reg(ICM20_SMPLRT_DIV, 0); // 分频数=为0+1，数据输出速率为内部采样速率
    icm20602_write_reg(ICM20_CONFIG, DLPF_BW_20); // GYRO低通滤波设置
    icm20602_write_reg(ICM20_ACCEL_CONFIG2, ACCEL_AVER_4 | ACCEL_DLPF_BW_21); // ACCEL低通滤波设置

    icm20602_set_accel_fullscale(ICM20_ACCEL_FS_2G);
    icm20602_set_gyro_fullscale(ICM20_GYRO_FS_2000);

    delay_ms(100);
    printf("icm20602 init pass\r\n\r\n");

    return 0;
}

uint8_t icm20602_set_gyro_fullscale(uint8_t fs) {
    switch(fs) {
    case ICM20_GYRO_FS_250:
        _gyro_scale = 1.0f/131.068f;	//32767/250
        break;
    case ICM20_GYRO_FS_500:
        _gyro_scale = 1.0f/65.534f;
        break;
    case ICM20_GYRO_FS_1000:
        _gyro_scale = 1.0f/32.767f;
        break;
    case ICM20_GYRO_FS_2000:
        _gyro_scale = 1.0f/16.4f;
        break;
    default:
        fs = ICM20_GYRO_FS_2000;
        _gyro_scale = 1.0f/16.3835f;
        break;
    }
    _gyro_scale *= _DEG_TO_RAD;
    return icm20602_write_reg(ICM20_GYRO_CONFIG, fs);
}

uint8_t icm20602_set_accel_fullscale(uint8_t fs) {
    switch(fs) {
    case ICM20_ACCEL_FS_2G:
        _accel_scale = 1.0f/16348.0f;
        break;
    case ICM20_ACCEL_FS_4G:
        _accel_scale = 1.0f/8192.0f;
        break;
    case ICM20_ACCEL_FS_8G:
        _accel_scale = 1.0f/4096.0f;
        break;
    case ICM20_ACCEL_FS_16G:
        _accel_scale = 1.0f/2048.0f;
        break;
    default:
        fs = ICM20_ACCEL_FS_8G;
        _accel_scale = 1.0f/4096.0f;
        break;
    }
    _accel_scale *= GRAVITY_MSS;
    return icm20602_write_reg(ICM20_ACCEL_CONFIG, fs);
}

uint8_t icm20602_get_acc_gyro_adc(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz) {
    uint8_t buf[14];
    if (icm20602_read_buffer(ICM20_ACCEL_XOUT_H, buf, 14)) {
        return 1;
    }
    *ax = ((int16_t)buf[0]<<8) + buf[1];
    *ay = ((int16_t)buf[2]<<8) + buf[3];
    *az = ((int16_t)buf[4]<<8) + buf[5];
    *gx = ((int16_t)buf[8]<<8) + buf[9];
    *gy = ((int16_t)buf[10]<<8) + buf[11];
    *gz = ((int16_t)buf[12]<<8) + buf[13];
    return 0;
}

uint8_t icm20602_get_acc_gyro_raw(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) {
    int16_t ax_adc, ay_adc, az_adc, gx_adc, gy_adc, gz_adc;
    if (icm20602_get_acc_gyro_adc(&ax_adc, &ay_adc, &az_adc, &gx_adc, &gy_adc, &gz_adc)) {
        return 1;
    }
    *ax = _accel_scale * (float)ax_adc;
    *ay = _accel_scale * (float)ay_adc;
    *az = _accel_scale * (float)az_adc;
    *gx = _gyro_scale * (float)gx_adc;
    *gy = _gyro_scale * (float)gy_adc;
    *gz = _gyro_scale * (float)gz_adc;
    return 0;
}

uint8_t MATH_UpdateFIFOBufferXYZ(int16_t dataX, int16_t dataY, int16_t dataZ, int16_t *x_Fifo, int16_t *y_Fifo, int16_t *z_Fifo, int16_t *index, uint16_t length) {
    int16_t idx = *index;
    x_Fifo[idx] = dataX;
    y_Fifo[idx] = dataY;
    z_Fifo[idx] = dataZ;
    *index = (int16_t)(idx + 1) % length;
    return 0;
}

uint8_t MATH_GetMinMax(int16_t *fifo, int16_t start, int16_t length, int16_t *min_value, int16_t *max_value) {
    int32_t i;
    int16_t minv = fifo[start];
    int16_t maxv = minv;
    for (i = 1; i < length; i++) {
        int16_t v = fifo[start + i];
        if (v >= maxv) {
            maxv = v;
        }
        if (v < minv) {
            minv = v;
        }
    }
    *min_value = minv;
    *max_value = maxv;
    return 0;
}

uint8_t MATH_GetMax(int16_t *fifo, int16_t start, int16_t length, int16_t *max_value) {
    int32_t i;
    int16_t maxv = fifo[start];
    for (i = 1; i < length; i++) {
        int16_t v = fifo[start + i];
        if (v >= maxv) {
            maxv = v;
        }
    }
    *max_value = maxv;
    return 0;
}

uint8_t MATH_GetMin(int16_t *fifo, int16_t start, int16_t length, int16_t *min_value) {
    int32_t i;
    int16_t minv = fifo[start];
    for (i = 1; i < length; i++) {
        int16_t v = fifo[start + i];
        if (v < minv) {
            minv = v;
        }
    }
    *min_value = minv;
    return 0;
}

float MATH_Mean(int16_t *fifo, int16_t start, int16_t length) {
    int32_t i;
    float sum = (float)fifo[start];
    for (i = 1; i < length; i++) {
        int16_t val = fifo[start + i];
        sum += (float)val;
    }
    return sum / (float)length;
}

uint8_t UpdateOffsetStepwise(float *offsetValue, float meanValue, int16_t offsetStep) {
    float new_offsetValue;
    float old_offsetValue = *offsetValue;
    float diff_offsetValue_meanValue = old_offsetValue - meanValue;
    if (fabsf(diff_offsetValue_meanValue) >= (float)offsetStep) {
        if (diff_offsetValue_meanValue > -(float)offsetStep) {
            new_offsetValue = old_offsetValue - offsetStep;
        } else {
            new_offsetValue = old_offsetValue + offsetStep;
        }
    } else {
        new_offsetValue = meanValue;
    }
    *offsetValue = new_offsetValue * 0.1f + old_offsetValue * 0.9f;
    return 0;
}

uint8_t icm20602_get_acc_gyro_with_calib(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, int8_t *is_dynamic, int8_t *is_calibrated) {
    // acc
    const int16_t ACC_FIFO_SIZE = 5, ACC_DYN_DET_TH = 25, ACC_DYN_BWIDTH = 4, ACC_NO_DYN_DET_TIME = 3;
    static int16_t acc_dyn_det_count = 0;
    static int16_t ax_fifo[ACC_FIFO_SIZE], ay_fifo[ACC_FIFO_SIZE], az_fifo[ACC_FIFO_SIZE];
    static int16_t acc_fifo_idx = 0;
    int16_t acc_ref_idx;
    int8_t is_acc_dyn;
    // gyro
    const int16_t GYRO_FIFO_SIZE = 25, GYRO_DYN_DET_TH = /*9*//*18*/19, GYRO_OFFSET_TH_MAX = 163, GYRO_OFFSET_TH_STEP = 4;
    const int16_t GYRO_NO_DYN_DET_TIME = 50/*25*/;
    static int16_t gyro_dyn_det_count = 0;
    static int16_t gx_fifo[GYRO_FIFO_SIZE], gy_fifo[GYRO_FIFO_SIZE], gz_fifo[GYRO_FIFO_SIZE];
    static int16_t gyro_fifo_idx = 0;
    static int16_t gx_min = 0, gy_min = 0, gz_min = 0, gx_max = 0, gy_max = 0, gz_max = 0;
    static float gx_bias = 0, gy_bias = 0, gz_bias = 0;
    static int8_t is_gyro_calibrated = 0, gyro_calibrated_accuracy = 0;
    int8_t is_gyro_dyn;
    // read acc&gyro adc value
    int16_t cur_ax_adc, cur_ay_adc, cur_az_adc, cur_gx_adc, cur_gy_adc, cur_gz_adc;
    int16_t ref_ax_adc, ref_ay_adc, ref_az_adc, prev_gx_adc, prev_gy_adc, prev_gz_adc;
    if (icm20602_get_acc_gyro_adc(&cur_ax_adc, &cur_ay_adc, &cur_az_adc, &cur_gx_adc, &cur_gy_adc, &cur_gz_adc)) {
        return 1;
    }
    // check whether acc is dynamic ?
    if (acc_fifo_idx < ACC_DYN_BWIDTH) {
        acc_ref_idx = 5 + acc_fifo_idx - ACC_DYN_BWIDTH;
    } else {
        acc_ref_idx = acc_fifo_idx - ACC_DYN_BWIDTH;
    }
    MATH_UpdateFIFOBufferXYZ(cur_ax_adc, cur_ay_adc, cur_az_adc, ax_fifo, ay_fifo, az_fifo, &acc_fifo_idx, ACC_FIFO_SIZE);
    ref_ax_adc = ax_fifo[acc_ref_idx];
    ref_ay_adc = ay_fifo[acc_ref_idx];
    ref_az_adc = az_fifo[acc_ref_idx];
    if (fabsf((float)cur_ax_adc - (float)ref_ax_adc) > (float)ACC_DYN_DET_TH
     || fabsf((float)cur_ay_adc - (float)ref_ay_adc) > (float)ACC_DYN_DET_TH
     || fabsf((float)cur_az_adc - (float)ref_az_adc) > (float)ACC_DYN_DET_TH ) {
        acc_dyn_det_count = 0;
    } else {
        acc_dyn_det_count++;
        if (acc_dyn_det_count >= 100) {
            acc_dyn_det_count = 100;
        }
    }
    is_acc_dyn = acc_dyn_det_count < ACC_NO_DYN_DET_TIME;
    // check whether gyro is dynamic ?
    prev_gx_adc = gx_fifo[gyro_fifo_idx];
    prev_gy_adc = gy_fifo[gyro_fifo_idx];
    prev_gz_adc = gz_fifo[gyro_fifo_idx];
    MATH_UpdateFIFOBufferXYZ(cur_gx_adc, cur_gy_adc, cur_gz_adc, gx_fifo, gy_fifo, gz_fifo, &gyro_fifo_idx, GYRO_FIFO_SIZE);
    // gx max&min
    if (cur_gx_adc <= gx_max) {
        if (prev_gx_adc == gx_max) {
            MATH_GetMax(gx_fifo, 0, GYRO_FIFO_SIZE, &gx_max);
        }
    } else {
        gx_max = cur_gx_adc;
    }
    if (cur_gx_adc >= gx_min) {
        if (prev_gx_adc == gx_min) {
            MATH_GetMin(gx_fifo, 0, GYRO_FIFO_SIZE, &gx_min);
        }
    } else {
        gx_min = cur_gx_adc;
    }
    // gy max&min
    if (cur_gy_adc <= gy_max) {
        if (prev_gy_adc == gy_max) {
            MATH_GetMax(gy_fifo, 0, GYRO_FIFO_SIZE, &gy_max);
        }
    } else {
        gy_max = cur_gy_adc;
    }
    if (cur_gy_adc >= gy_min) {
        if (prev_gy_adc == gy_min) {
            MATH_GetMin(gy_fifo, 0, GYRO_FIFO_SIZE, &gy_min);
        }
    } else {
        gy_min = cur_gy_adc;
    }
    // gz max&min
    if (cur_gz_adc <= gz_max) {
        if (prev_gz_adc == gz_max) {
            MATH_GetMax(gz_fifo, 0, GYRO_FIFO_SIZE, &gz_max);
        }
    } else {
        gz_max = cur_gz_adc;
    }
    if (cur_gz_adc >= gz_min) {
        if (prev_gz_adc == gz_min) {
            MATH_GetMin(gz_fifo, 0, GYRO_FIFO_SIZE, &gz_min);
        }
    } else {
        gz_min = cur_gz_adc;
    }
    // diff of max,min of gx, gy, gz
    if (fabsf((float)gx_max - (float)gx_min) >= (float)GYRO_DYN_DET_TH
     || fabsf((float)gy_max - (float)gy_min) >= (float)GYRO_DYN_DET_TH
     || fabsf((float)gz_max - (float)gz_min) >= (float)GYRO_DYN_DET_TH) {
        gyro_dyn_det_count = 0;
    } else {
        gyro_dyn_det_count++;
        if (gyro_dyn_det_count >= 100) {
            gyro_dyn_det_count = 100;
        }
    }
    is_gyro_dyn = gyro_dyn_det_count < GYRO_NO_DYN_DET_TIME;
    // calc gyro offset if it is still
    if (!is_acc_dyn && !is_gyro_dyn) {
        float gx_fifo_mean, gy_fifo_mean, gz_fifo_mean;
        int16_t calc_mean_fifo_size = GYRO_NO_DYN_DET_TIME;
        gyro_dyn_det_count = 0;
        if (calc_mean_fifo_size > GYRO_FIFO_SIZE) {
            calc_mean_fifo_size = GYRO_FIFO_SIZE;
        }
        gx_fifo_mean = MATH_Mean(gx_fifo, 0, calc_mean_fifo_size);
        gy_fifo_mean = MATH_Mean(gy_fifo, 0, calc_mean_fifo_size);
        gz_fifo_mean = MATH_Mean(gz_fifo, 0, calc_mean_fifo_size);
        if (fabsf(gx_fifo_mean) < (float)GYRO_OFFSET_TH_MAX
         && fabsf(gy_fifo_mean) < (float)GYRO_OFFSET_TH_MAX
         && fabsf(gz_fifo_mean) < (float)GYRO_OFFSET_TH_MAX ) {
            if (gyro_calibrated_accuracy == 1
             || gyro_calibrated_accuracy == 2
             || gyro_calibrated_accuracy == 3) {
                // use iir-filter 1ord to update gyro bias
                UpdateOffsetStepwise(&gx_bias, gx_fifo_mean, GYRO_OFFSET_TH_STEP);
                UpdateOffsetStepwise(&gy_bias, gy_fifo_mean, GYRO_OFFSET_TH_STEP);
                UpdateOffsetStepwise(&gz_bias, gz_fifo_mean, GYRO_OFFSET_TH_STEP);
                if (gyro_calibrated_accuracy == 3) {
                    gyro_calibrated_accuracy = 1;
                } else {
                    gyro_calibrated_accuracy++;
                }
            } else {
                gx_bias = gx_fifo_mean;
                gy_bias = gy_fifo_mean;
                gz_bias = gz_fifo_mean;
                gyro_calibrated_accuracy= 1;
            }
            is_gyro_calibrated = 1;
        }
    }
    // calc data
    *ax = _accel_scale * (float)cur_ax_adc;
    *ay = _accel_scale * (float)cur_ay_adc;
    *az = _accel_scale * (float)cur_az_adc;
    *gx = _gyro_scale * ((float)cur_gx_adc - gx_bias);
    *gy = _gyro_scale * ((float)cur_gy_adc - gy_bias);
    *gz = _gyro_scale * ((float)cur_gz_adc - gz_bias);
    *is_dynamic = is_acc_dyn;
    *is_calibrated = is_gyro_calibrated;
    //printf("acc_dyn:%d ref/cur x %d %d y %d %d z %d %d\n", is_acc_dyn, ref_ax_adc, cur_ax_adc, ref_ay_adc, cur_ay_adc, ref_az_adc, cur_az_adc);
    //printf("gyro_dyn:%d max/min x %d %d y %d %d z %d %d\n", is_acc_dyn, gx_max, gx_min, gy_max, gy_min, gz_max, gz_min);
    return 0;
}
//ekf for icm20602
uint8_t icm20602_ekf(){
	
}
