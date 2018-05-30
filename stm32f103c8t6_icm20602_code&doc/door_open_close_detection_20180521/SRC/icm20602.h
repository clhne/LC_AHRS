#ifndef _ICM20602_H_
#define _ICM20602_H_
#include "sys.h"
uint8_t icm20602_init(void);
uint8_t icm20602_set_gyro_fullscale(uint8_t fs);
uint8_t icm20602_set_accel_fullscale(uint8_t fs);
uint8_t icm20602_get_acc_gyro_adc(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);
uint8_t icm20602_get_acc_gyro_raw(float *ax, float *ay, float *az, float *gx, float *gy, float *gz);
uint8_t icm20602_get_acc_gyro_with_calib(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, int8_t *is_dynamic, int8_t *is_calibrated);
uint8_t icm20602_ekf();
#endif

