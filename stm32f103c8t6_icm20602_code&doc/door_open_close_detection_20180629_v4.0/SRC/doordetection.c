#include "doordetection.h"
#include "icm20602.h"
#include "NDOF.h"
#include "oled.h"
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
    float dt, roll, prev_pitch, cur_pitch, yaw;
    int prev_monotonicity, cur_monotonicity;
    float dt_count;
    u32 prev_ts, cur_ts;
    u32 cur_dt, prev_dt;
    int peak_trough_init;
    int peak_trough_index ;
    int count_peak_trough ;
    int prev_flag_peak_trough;  // IS_TROUGH, IS_PEAK, IS_INIT=-1
    int prev_prev_flag_peak_trough;
    float prev_prev_peak_trough_value;
    float prev_peak_trough_value;

} DOOR_DETECTION;

DOOR_DETECTION door;
void DOOR_DETECTION_Init()
{
    door.prev_ts = 0;
    door.cur_ts = 0;
    door.cur_dt = 0;
    door.prev_dt = 0;
    door.peak_trough_init = 0;
    door.peak_trough_index = 0;
    door.count_peak_trough = 0;
    door.dt_count = 0;
    door.prev_flag_peak_trough = IS_NOT_INIT;
    door.prev_prev_flag_peak_trough = IS_NOT_INIT;
    door.prev_peak_trough_value = 0;
    door.prev_prev_peak_trough_value = 0;
}
int door_detection(int *door_status, float *cur_roll, float * pitch, float *cur_yaw, float *cur_cor_gx, float *cor_gy, float *cor_gz)
{
	  char show_string[SHOW_STRING_SIZE];
	  short ax_adc, ay_adc, az_adc, gx_adc, gy_adc, gz_adc;
    float raw_ax, raw_ay, raw_az;
    float cor_ax, cor_ay, cor_az;
    float filt_ax, filt_ay, filt_az;
    float raw_gx, raw_gy, raw_gz;
    float gx_bias, gy_bias, gz_bias;
    float peak_trough_value;
    int oscillation_det;
	  float cur_cor_gx1;
    u32 prev_is_gyro_dyn, cur_is_gyro_dyn, prev_is_acc_dyn, cur_is_acc_dyn;
    door.cur_ts = millis();
	    if (icm20602_get_acc_gyro_adc(&ax_adc, &ay_adc, &az_adc, &gx_adc, &gy_adc, &gz_adc)) {
        delay_ms(2);
    } else {
			if (NDOF_DoStep(ax_adc, ay_adc, az_adc, gx_adc, gy_adc, gz_adc, door.cur_ts)) {
        door.dt = (float)(door.cur_ts - door.prev_ts) / 1000.0f;
        NDOF_GetEulerAngle(&door.roll, &door.cur_pitch, &door.yaw);
        NDOF_GetRawAccData(&raw_ax, &raw_ay, &raw_az);
        NDOF_GetCorAccData(&cor_ax, &cor_ay, &cor_az);
        NDOF_GetFiltAccData(&filt_ax, &filt_ay, &filt_az);
        NDOF_GetRawGyroData(&raw_gx, &raw_gy, &raw_gz);
        NDOF_GetCorGyroData(&cur_cor_gx1, cor_gy, cor_gz);
        NDOF_GetGyroBias(&gx_bias, &gy_bias, &gz_bias);

				
        sprintf(show_string, "%.3f %d %d %d %d \n\n%.3f  ", door.dt, NDOF_IsGyroDynamic(), NDOF_IsGyroCalibrated(), NDOF_IsAccDynamic(), NDOF_IsAccCalibrated(), door.cur_pitch);//, yaw);
        oled_show_string(0, 0, show_string);
        cur_is_gyro_dyn = NDOF_IsGyroDynamic();
        cur_is_acc_dyn = NDOF_IsAccDynamic();

        //seq/array monotonicity detection
        if(fabs(door.cur_pitch) >= 10) {
            oscillation_det = 0;
            door.peak_trough_index = 0;
            door.dt_count = 0;
            door.count_peak_trough = 0;

            door.peak_trough_init = 0;
            door.prev_flag_peak_trough = IS_NOT_INIT;
            door.prev_prev_flag_peak_trough = IS_NOT_INIT;
            door.prev_peak_trough_value = 0;
            door.prev_prev_peak_trough_value = 0;

            sprintf(show_string, "Door opened.   %d\n", oscillation_det);
            oled_show_string(0, 6, show_string);
            *door_status = DOOR_STATUS_OPEN;
        } else if(fabs(door.cur_pitch) < 10 /***&& door.dt_count < 128 * door.dt **/) {
            static int status = DOOR_STATUS_DETECTING;
            door.dt_count += door.dt;
            if(door.prev_cor_gx < cur_cor_gx1)
                door.cur_monotonicity = 1;
            else if(door.prev_cor_gx > cur_cor_gx1)
                door.cur_monotonicity = 0;
            if(door.peak_trough_init == 1) {
                if(door.cur_monotonicity != door.prev_monotonicity) {
                    if(door.cur_monotonicity == 0 && door.prev_monotonicity == 1 && door.prev_cor_gx > 0.125f) {
                        peak_trough_value = door.prev_cor_gx;
                        if(door.prev_flag_peak_trough == IS_PEAK) {
                            door.prev_peak_trough_value = MAX(door.prev_peak_trough_value, peak_trough_value);
													printf("value1 %f\n",door.prev_peak_trough_value);
                            if(door.prev_peak_trough_value < 0.25) {
                                door.prev_dt = millis();
                            }
                        } else if (door.prev_flag_peak_trough == IS_TROUGH) {
                            if(door.prev_prev_flag_peak_trough != IS_NOT_INIT) {
                                if(fabs(door.prev_peak_trough_value) <= fabs(door.prev_prev_peak_trough_value)) {
                                    door.count_peak_trough++;
                                }
//                               door.peak_trough_index ++;
                            }
                            door.peak_trough_index ++;
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
													printf("value2 %f\n",door.prev_peak_trough_value);
                            if(door.prev_peak_trough_value < 0.25) {
                                door.prev_dt = millis();
                            }
                        } else if (door.prev_flag_peak_trough == IS_PEAK)  {
                            if(door.prev_prev_flag_peak_trough != IS_NOT_INIT) {
                                if(fabs(door.prev_peak_trough_value) <= fabs(door.prev_prev_peak_trough_value)) {	
                                    door.count_peak_trough++;
                                }
//                                  door.peak_trough_index++;
                            }
                            door.peak_trough_index ++;
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
                    door.cur_dt = millis();
                    if(door.count_peak_trough >= floor(door.peak_trough_index  * 0.5)) {
                        //acc and gyro static?
                        //gyro stable?
                        if(fabs(door.cur_pitch) < 1.2) {
                            oscillation_det = 1;
                            sprintf(show_string, "Door closed.   %d\n", oscillation_det);
                            oled_show_string(0, 6, show_string);
                            status = DOOR_STATUS_CLOSE;
                        } else {
                            oscillation_det = 0;
                            sprintf(show_string, "Door opened.   %d\n", oscillation_det);
                            oled_show_string(0, 6, show_string);
                            status = DOOR_STATUS_OPEN;
                        }
                    } else {
                        oscillation_det = 0;
                        sprintf(show_string, "Door opened.   %d\n", oscillation_det);
                        oled_show_string(0, 6, show_string);
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
				//printf("value %f\n",door.prev_peak_trough_value);
				
    } else {
        *door_status =  NOT_READY;
        delay_ms(2);
    }
   }
    door.prev_ts = door.cur_ts;
    door.prev_cor_gx = cur_cor_gx1;
    prev_is_gyro_dyn = cur_is_gyro_dyn;
    prev_is_acc_dyn = cur_is_acc_dyn;
    door.prev_pitch = door.cur_pitch;
		printf("[1]%f %f %f\n",door.cur_pitch, door.prev_peak_trough_value, cur_cor_gx1);
	  //printf("ts %d, value %f\n",door.cur_ts, door.prev_peak_trough_value);
    //printf("%d %d %f %d %d %f %d %d %d %d %f %f\n", door.prev_dt, door.cur_dt, door.prev_peak_trough_value, door.count_peak_trough, door.peak_trough_index, door.cur_pitch, door.cur_ts, NDOF_IsGyroDynamic(),  NDOF_IsAccDynamic(), NDOF_IsAccCalibrated(), raw_gx, *cur_cor_gx);
    return *door_status;
}
