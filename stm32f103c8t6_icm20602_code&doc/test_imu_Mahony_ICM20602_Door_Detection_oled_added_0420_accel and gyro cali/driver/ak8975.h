#ifndef _AK8975_H_
#define	_AK8975_H_

#include "sys.h"




#define AK8975_MODE_PowerDown		0x00
#define AK8975_MODE_SignalMeasure	0x01
#define AK8975_MODE_SelfTest		0x08
#define AK8975_MODE_POWR_DOWN		0x0F


uint8_t ak8975_init(void);
uint8_t ak8975_start(void);
uint8_t ak8975_get_mag_adc(int16_t *mag);
uint8_t ak8975_get_mag(float *mag);
void magcalMPU9250(float * dest1, float * dest2,float *mag_bias_x, float *mag_bias_y, float *mag_bias_z);
#endif

