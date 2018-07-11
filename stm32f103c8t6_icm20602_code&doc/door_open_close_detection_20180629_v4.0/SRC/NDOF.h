#ifndef __NDOF_H
#define __NDOF_H
#define NDOF_ACC_RANGE_2G        (0U)
#define NDOF_ACC_RANGE_4G        (1U)
#define NDOF_ACC_RANGE_8G        (2U)
#define NDOF_ACC_RANGE_16G       (3U)
#define NDOF_GYRORANGE_2048DPS   (0U)
#define NDOF_GYRORANGE_2000DPS   (1U)
#define NDOF_GYRORANGE_1000DPS   (2U)
#define NDOF_GYRORANGE_500DPS    (3U)
#define NDOF_GYRORANGE_250DPS    (4U)
void NDOF_Init(unsigned char acc_range_idx, unsigned char acc_res_bits, unsigned char gyro_range_idx, unsigned char gyro_res_bits);
void NDOF_Reset(void);
int NDOF_DoStep(short ax_adc, short ay_adc, short az_adc, short gx_adc, short gy_adc, short gz_adc, long long ts_ms);
void NDOF_GetRawAccData(float *raw_ax, float *raw_ay, float *raw_az);
void NDOF_GetRawGyroData(float *raw_gx, float *raw_gy, float *raw_gz);
int NDOF_GetFiltAccData(float *filt_ax, float *filt_ay, float *filt_az);
int NDOF_GetCorAccData(float *cor_ax, float *cor_ay, float *cor_az);
int NDOF_GetCorGyroData(float *cor_gx, float *cor_gy, float *cor_gz);
int NDOF_IsAccCalibrated(void);
int NDOF_IsGyroCalibrated(void);
int NDOF_IsAccDynamic(void);
int NDOF_IsGyroDynamic(void);
int NDOF_GetAccBias(float *ax_bias, float *ay_bias, float *az_bias);
int NDOF_GetGyroBias(float *gx_bias, float *gy_bias, float *gz_bias);
int NDOF_GetQuat(float *qw, float *qx, float *qy, float *qz);
int NDOF_GetEulerAngle(float *roll, float *pitch, float *yaw);
#endif
