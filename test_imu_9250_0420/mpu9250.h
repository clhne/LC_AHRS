#ifndef __MPU9250_H
#define __MPU9250_H
u8 mpu_9250_init(void);
u8 mpu_9250_is_dry(void);
u8 mpu_9250_read_accel_gyro(float *ax, float *ay, float *az, float *gx, float *gy, float *gz);
u8 mpu_9250_read_mag(float *mx, float *my, float *mz);
u8 mpu_9250_read_accel_gyro_raw(short *ax, short *ay, short *az, short *gx, short *gy, short *gz);
u8 mpu_9250_read_mag_raw(short *mx, short *my, short *mz);
#endif
