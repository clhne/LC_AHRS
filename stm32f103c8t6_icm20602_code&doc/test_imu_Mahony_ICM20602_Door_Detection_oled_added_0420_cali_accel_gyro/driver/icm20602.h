#ifndef _ICM20602_H_
#define _ICM20602_H_

#include "sys.h"


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


uint8_t icm20602_init(void);

uint8_t icm20602_set_gyro_fullscale(uint8_t fs);
uint8_t icm20602_set_accel_fullscale(uint8_t fs);
uint8_t icm20602_get_accel_adc(int16_t *accel);
uint8_t icm20602_get_gyro_adc(int16_t *gyro);
uint8_t icm20602_get_gyro(float *gyro);
uint8_t icm20602_get_accel(float *accel);
float icm20602_get_temp(void);
void icm20602_set_accel_gyro_bias(float ax,float ay,float az,float gx,float gy,float gz);
void icm20602_set_gyro_bias(float gx, float gy, float gz);
u8 icm20602_get_accel_gyro_statistic(
    float *ax, float *ay, float *az, float *gx, float *gy, float *gz,
    float *mean_ax, float *mean_ay, float *mean_az, float *mean_gx, float *mean_gy, float *mean_gz,
    float *var_ax, float *var_ay, float *var_az, float *var_gx, float *var_gy, float *var_gz
);
#endif

