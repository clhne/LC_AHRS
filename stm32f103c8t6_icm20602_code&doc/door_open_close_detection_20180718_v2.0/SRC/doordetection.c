#include "doordetection.h"
#include <math.h>
#define IS_TROUGH 1
#define IS_PEAK 0
#define IS_NOT_INIT -1
#define MAX(a,b) ((a)>(b)?(a):(b))

typedef struct {
	  //Sensor detection param
    float prev_cor_gx;
    int prev_monotonicity;
	  int cur_monotonicity;
    int peak_trough_init;
    int peak_trough_index;
    int count_peak_trough;
    int prev_flag_peak_trough;  // IS_TROUGH, IS_PEAK, IS_INIT=-1
    int prev_prev_flag_peak_trough;
    float prev_prev_peak_trough_value;
    float prev_peak_trough_value;
	  //Door param
	  float Opened_angle;
	  float Closed_angle;
	  float Detection_factor;
} Door;

Door door;
void Door_Detection_Init(unsigned char acc_range_idx, unsigned char acc_res_bits, unsigned char gyro_range_idx, unsigned char gyro_res_bits)
{
	  //Door param
	  door.Opened_angle = 3.0;
	  door.Detection_factor = 0.5;
	  door.Closed_angle = 0.5;
	  NDOF_Init(acc_range_idx, acc_res_bits, gyro_range_idx, gyro_res_bits);
    //Reset Door Detection
    Door_Detection_Reset();
}

void Door_Detection_Reset()
{
    door.peak_trough_init = 0;
    door.peak_trough_index = 0;
    door.count_peak_trough = 0;
    door.prev_flag_peak_trough = IS_NOT_INIT;
    door.prev_prev_flag_peak_trough = IS_NOT_INIT;
    door.prev_peak_trough_value = 0;
    door.prev_prev_peak_trough_value = 0;
	  NDOF_Reset();
}
//Input ax_adc,ay_adc,az_adc,gx_adc,gy_adc,gz_adc
//Output door_status, pitch,cur_cor_gx,cor_gy,cor_gz
int Door_Detection(short ax_adc, short ay_adc, short az_adc, short gx_adc, short gy_adc, short gz_adc, float *pitch, float *cor_gx, int *peak_trough_index, int *peak_trough_count, long long ts)
{
	  int door_status;
    float peak_trough_value;
    int is_gyro_dyn, is_acc_dyn;
    //Check if NDOF_IsGyroCalibrated
    if (NDOF_DoStep(ax_adc, ay_adc, az_adc, gx_adc, gy_adc, gz_adc, ts)) {
        float roll, yaw;
        float cor_gy, cor_gz;
        NDOF_GetEulerAngle(&roll, pitch, &yaw);
        NDOF_GetCorGyroData(cor_gx, &cor_gy, &cor_gz);
        is_gyro_dyn = NDOF_IsGyroDynamic();
        is_acc_dyn = NDOF_IsAccDynamic();

        //seq/array monotonicity detection
        if(fabs(*pitch) >= door.Opened_angle) {		//Opened_angle need modified!
					  door.peak_trough_init = 0;
            door.peak_trough_index = 0;
            door.count_peak_trough = 0;
						*peak_trough_index = door.peak_trough_index;
						*peak_trough_count = door.count_peak_trough;
            door.prev_flag_peak_trough = IS_NOT_INIT;
            door.prev_prev_flag_peak_trough = IS_NOT_INIT;
            door.prev_peak_trough_value = 0;
            door.prev_prev_peak_trough_value = 0;
            door_status = DOOR_STATUS_OPEN;
        } else if(fabs(*pitch) < door.Opened_angle ) {
            static int status = DOOR_STATUS_DETECTING;
            if(door.prev_cor_gx < *cor_gx)
                door.cur_monotonicity = 1;
            else if(door.prev_cor_gx > *cor_gx)
                door.cur_monotonicity = 0;
            if(door.peak_trough_init == 1) {
                if(door.cur_monotonicity != door.prev_monotonicity) {
                    if(door.cur_monotonicity == 0 && door.prev_monotonicity == 1 && door.prev_cor_gx > 0.125f) {
                        peak_trough_value = door.prev_cor_gx;
                        if(door.prev_flag_peak_trough == IS_PEAK) {
                            door.prev_peak_trough_value = MAX(door.prev_peak_trough_value, peak_trough_value);
                        } else if (door.prev_flag_peak_trough == IS_TROUGH) {
                            if(door.prev_prev_flag_peak_trough != IS_NOT_INIT) {
                                if(fabs(door.prev_peak_trough_value) <= fabs(door.prev_prev_peak_trough_value)) {
                                    door.count_peak_trough++;
                                }
                                door.peak_trough_index ++;
                            }
                            door.prev_prev_flag_peak_trough = door.prev_flag_peak_trough;
                            door.prev_prev_peak_trough_value = door.prev_peak_trough_value;
                            door.prev_flag_peak_trough = IS_PEAK;
                            door.prev_peak_trough_value = peak_trough_value;
                        } else {
                            // IS_NOT_INIT
                            door.prev_flag_peak_trough = IS_PEAK;
                            door.prev_peak_trough_value = peak_trough_value;
                        }
                    } else if(door.cur_monotonicity == 1 && door.prev_monotonicity == 0 && door.prev_cor_gx <= -0.125f) {
                        peak_trough_value = fabs(door.prev_cor_gx);
                        if(door.prev_flag_peak_trough == IS_TROUGH) {
                            door.prev_peak_trough_value = MAX(door.prev_peak_trough_value, peak_trough_value);
                        } else if (door.prev_flag_peak_trough == IS_PEAK)  {
                            if(door.prev_prev_flag_peak_trough != IS_NOT_INIT) {
                                if(fabs(door.prev_peak_trough_value) <= fabs(door.prev_prev_peak_trough_value)) {
                                    door.count_peak_trough++;
                                }
                                door.peak_trough_index++;
                            }
                            door.prev_prev_flag_peak_trough = door.prev_flag_peak_trough;
                            door.prev_prev_peak_trough_value = door.prev_peak_trough_value;
                            door.prev_flag_peak_trough = IS_TROUGH;
                            door.prev_peak_trough_value = peak_trough_value;
                        } else {
                            // IS_NOT_INIT
                            door.prev_flag_peak_trough = IS_TROUGH;
                            door.prev_peak_trough_value = peak_trough_value;
                        }
                    }
										*peak_trough_index = door.peak_trough_index;
										*peak_trough_count = door.count_peak_trough;
                }
								//acc and gyro static?
                if(is_gyro_dyn == 0 && is_acc_dyn == 0) {
                    if(door.count_peak_trough >= floor(door.peak_trough_index  * door.Detection_factor )) {   //Detection_factor need modified!
                        if(fabs(*pitch) < door.Closed_angle) {   //Closed_angle need modified!
                            status = DOOR_STATUS_CLOSE;
                        } else {
                            status = DOOR_STATUS_OPEN;
                        }
                    } else {
                        status = DOOR_STATUS_OPEN;
                    }
                }
            } else {
                door.peak_trough_init = 1;
                status = DOOR_STATUS_DETECTING;
            }
            door.prev_monotonicity = door.cur_monotonicity;
            door_status = status;
        }

    } else {
        door_status = DOOR_STATUS_NOT_READY;
    }
    door.prev_cor_gx = *cor_gx;
				
    return door_status;
}
