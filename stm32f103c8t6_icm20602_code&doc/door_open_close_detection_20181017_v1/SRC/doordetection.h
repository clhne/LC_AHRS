#ifndef __DOORDETECTION_H
#define __DOORDETECTION_H
#define DOOR_STATUS_NOT_READY -1
#define DOOR_STATUS_OPEN 1
#define DOOR_STATUS_CLOSE 2
#define DOOR_STATUS_DETECTING 3
#include "NDOF.h"
void Door_Detection_Init(unsigned char acc_range_idx, unsigned char acc_res_bits, unsigned char gyro_range_idx, unsigned char gyro_res_bits);
void Door_Detection_Reset(void);
int Door_Detection(short ax_adc, short ay_adc, short az_adc, short gx_adc, short gy_adc, short gz_adc, float *pitch, float *roll, float *yaw, float *cur_cor_gx, float *cor_gy, float *cor_gz, 
	  float *cor_ax, float *cor_ay, int *peak_trough_index, int *peak_trough_count,long long ts,float *Door_angle);
#endif
