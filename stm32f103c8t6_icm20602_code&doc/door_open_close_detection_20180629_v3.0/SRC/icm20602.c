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

float _accel_scale;
float _gyro_scale;

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
    //_gyro_scale *= _DEG_TO_RAD;
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
