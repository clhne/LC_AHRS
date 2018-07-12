#include "doordetection.h"
#include "NDOF.h"
#include <math.h>
#define IS_TROUGH 1
#define IS_PEAK 0
#define IS_NOT_INIT -1
#define PEAK_TROUGH_SIZE (128)
#define SHOW_STRING_SIZE (128)
#define FLAG_PEAK_TROUGH (20)
#define MAX(a,b) ((a)>(b)?(a):(b))

typedef struct {
    float prev_cor_gx;
    int prev_monotonicity, cur_monotonicity;
    int cur_ts;
    int peak_trough_init;
    int peak_trough_index ;
    int count_peak_trough ;
    int prev_flag_peak_trough;  // IS_TROUGH, IS_PEAK, IS_INIT=-1
    int prev_prev_flag_peak_trough;
    float prev_prev_peak_trough_value;
    float prev_peak_trough_value;
    long long ts;

} Door;

Door door;
void Door_Detection_Init()
{
    //State param
    door.cur_ts = 0;

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
    door.ts = 0;
}
//Input ax_adc,ay_adc,az_adc,gx_adc,gy_adc,gz_adc
//Output door_status, pitch,cur_cor_gx,cor_gy,cor_gz
int Door_Detection(short ax_adc, short ay_adc, short az_adc, short gx_adc, short gy_adc, short gz_adc, int *door_status, float *pitch, float *cor_gx, long long ts)
{
    char show_string[SHOW_STRING_SIZE];
    float raw_gx, raw_gy, raw_gz;
    float peak_trough_value;
    int prev_is_gyro_dyn, cur_is_gyro_dyn, prev_is_acc_dyn, cur_is_acc_dyn;
    door.cur_ts = millis();
    //Check if NDOF_IsGyroCalibrated
    if (NDOF_DoStep(ax_adc, ay_adc, az_adc, gx_adc, gy_adc, gz_adc, door.cur_ts)) {
        float roll, yaw;
        float cor_gy, cor_gz;
        NDOF_GetEulerAngle(&roll, pitch, &yaw);
        NDOF_GetCorGyroData(cor_gx, &cor_gy, &cor_gz);
        cur_is_gyro_dyn = NDOF_IsGyroDynamic();
        cur_is_acc_dyn = NDOF_IsAccDynamic();

        //seq/array monotonicity detection
        if(fabs(*pitch) >= 4) {
            door.peak_trough_index = 0;
            door.count_peak_trough = 0;
            door.peak_trough_init = 0;
            door.prev_flag_peak_trough = IS_NOT_INIT;
            door.prev_prev_flag_peak_trough = IS_NOT_INIT;
            door.prev_peak_trough_value = 0;
            door.prev_prev_peak_trough_value = 0;
            *door_status = DOOR_STATUS_OPEN;
        } else if(fabs(*pitch) < 4 ) {
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
                }
                if(cur_is_gyro_dyn == 0 && cur_is_acc_dyn == 0) {
                    if(door.count_peak_trough >= floor(door.peak_trough_index  * 0.5)) {
                        //acc and gyro static? gyro stable?
                        if(fabs(*pitch) < 1.2) {
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
            *door_status = status;
        }

    } else {
        *door_status =  NOT_READY;
    }
    door.prev_cor_gx = *cor_gx;
    prev_is_gyro_dyn = cur_is_gyro_dyn;
    prev_is_acc_dyn = cur_is_acc_dyn;

    door.ts = ts;
    return *door_status;
}
