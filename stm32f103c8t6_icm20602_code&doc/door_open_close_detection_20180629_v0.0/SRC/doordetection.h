#ifndef __DOORDETECTION_H
#define __DOORDETECTION_H
#define DOOR_PEAK_TROUGH_VALUE_LITTLE -2
#define NOT_READY -1
#define READY 0
#define DOOR_STATUS_OPEN 1
#define DOOR_STATUS_CLOSE 2
#define DOOR_STATUS_DETECTING 3
int door_detection(int *door_status, float *cur_roll, float * pitch, float *cur_yaw, float *cur_cor_gx, float *cor_gy, float *cor_gz);
#endif