#include "doordetection.h"
#include "icm20602.h"
#include "NDOF.h"
#include "oled.h"
#include <math.h>
#define PEAK_TROUGH_SIZE (20)
#define SHOW_STRING_SIZE (128)

typedef struct {
    short ax_adc, ay_adc, az_adc, gx_adc, gy_adc, gz_adc;
    float dt, roll, prev_pitch, cur_pitch, yaw;
    float prev_monotonicity, cur_monotonicity;
		float dt_count;
    char show_string[SHOW_STRING_SIZE];
    u32 prev_ts, cur_ts;
    u32 peak_trough_init;
    u32 peak_trough_index ;
    u32 count_peak_trough ;
    float peak_trough_value[PEAK_TROUGH_SIZE];
} DOOR_DETECTION;

DOOR_DETECTION door;
void DOOR_DETECTION_Init()
{
    door.peak_trough_init = 0;
    door.peak_trough_index = 0;
    door.count_peak_trough = 0;
	  door.dt_count = 0;
}
int door_detection(int *door_status, float *cur_roll, float * pitch, float *cur_yaw, float *cur_cor_gx, float *cor_gy, float *cor_gz)
{
    float raw_ax, raw_ay, raw_az;
    float cor_ax, cor_ay, cor_az;
    float filt_ax, filt_ay, filt_az;
    float raw_gx, raw_gy, raw_gz;
    float prev_cor_gx;
    float gx_bias, gy_bias, gz_bias;
    int oscillation_det;
    u32 prev_is_gyro_dyn, cur_is_gyro_dyn, prev_is_acc_dyn, cur_is_acc_dyn;
    u32 cur_dt, prev_dt;
    door.cur_ts = millis();
    if (icm20602_get_acc_gyro_adc(&door.ax_adc, &door.ay_adc, &door.az_adc, &door.gx_adc, &door.gy_adc, &door.gz_adc)) {
        delay_ms(2);
    } else {
        if (NDOF_DoStep(door.ax_adc, door.ay_adc, door.az_adc, door.gx_adc, door.gy_adc, door.gz_adc, door.cur_ts)) {
            door.dt = (float)(door.cur_ts - door.prev_ts) / 1000.0f;
            NDOF_GetEulerAngle(&door.roll, &door.cur_pitch, &door.yaw);
            NDOF_GetRawAccData(&raw_ax, &raw_ay, &raw_az);
            NDOF_GetCorAccData(&cor_ax, &cor_ay, &cor_az);
            NDOF_GetFiltAccData(&filt_ax, &filt_ay, &filt_az);
            NDOF_GetRawGyroData(&raw_gx, &raw_gy, &raw_gz);
            NDOF_GetCorGyroData(cur_cor_gx, cor_gy, cor_gz);
            NDOF_GetGyroBias(&gx_bias, &gy_bias, &gz_bias);

            sprintf(door.show_string, "%.3f %d %d %d %d \n\n%.3f  ", door.dt, NDOF_IsGyroDynamic(), NDOF_IsGyroCalibrated(), NDOF_IsAccDynamic(), NDOF_IsAccCalibrated(), door.cur_pitch);//, yaw);
            oled_show_string(0, 0, door.show_string);
            cur_is_gyro_dyn = NDOF_IsGyroDynamic();
            cur_is_acc_dyn = NDOF_IsAccDynamic();
						printf("%d %d %d %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n", door.cur_ts, NDOF_IsGyroDynamic(), NDOF_IsGyroCalibrated(), NDOF_IsAccDynamic(), NDOF_IsAccCalibrated(), door.roll, door.cur_pitch, door.yaw, raw_ax, raw_ay, raw_az, filt_ax, filt_ay, filt_az, raw_gx, raw_gy, raw_gz, *cur_cor_gx, *cor_gy, *cor_gz);
            //seq/array monotonicity detection
            if(fabs(door.cur_pitch) >= 3) {
                oscillation_det = 0;
                sprintf(door.show_string, "Door opened.  %d\n", oscillation_det);
                oled_show_string(0, 6, door.show_string);
                *door_status = DOOR_STATUS_OPEN;
            }
            else if(fabs(door.cur_pitch) < 3) {
                if(prev_cor_gx < *cur_cor_gx)
                    door.cur_monotonicity = 1;
                else if(prev_cor_gx > *cur_cor_gx)
                    door.cur_monotonicity = 0;
                if(door.peak_trough_init == 1) {
                    if(door.cur_monotonicity != door.prev_monotonicity) {
                        if(door.cur_monotonicity == 0 && door.prev_monotonicity == 1 && prev_cor_gx > 0.125) {
                            door.peak_trough_value[door.peak_trough_index] = prev_cor_gx;
                            printf("value %f, ", door.peak_trough_value[door.peak_trough_index]);
                            if(door.peak_trough_value[door.peak_trough_index] < 0.25)
                                prev_dt = millis();
                            door.peak_trough_index ++;
                        } else if(door.cur_monotonicity == 1 && door.prev_monotonicity == 0 && prev_cor_gx <= -0.125) {
                            door.peak_trough_value[door.peak_trough_index] = prev_cor_gx;
                            printf("value %f, ", door.peak_trough_value[door.peak_trough_index]);
                            if(door.peak_trough_value[door.peak_trough_index] > -0.25)
                                prev_dt = millis();
                            door.peak_trough_index ++;
                        }

                        //acc and gyro static?
                        if(cur_is_gyro_dyn == 0 && cur_is_acc_dyn == 0) {
                            cur_dt = millis();
                        }
                        if(door.peak_trough_value[door.peak_trough_index] <= door.peak_trough_value[door.peak_trough_index - 1])
                            door.count_peak_trough++;
                        if(door.count_peak_trough >= floor(door.peak_trough_index * 0.5)) {
                            //gyro stable?
                            oscillation_det = 1;
                            sprintf(door.show_string, "Door closed.  %d\n", oscillation_det);
                            oled_show_string(0, 6, door.show_string);
                            *door_status = DOOR_STATUS_CLOSE;

                        } else
                            oscillation_det = 0;
                    }
                }else
                    door.peak_trough_init = 1;
    
                door.prev_monotonicity = door.cur_monotonicity;
                *door_status = DOOR_STATUS_DETECTING;
            } else {
                door.peak_trough_init = 0;
                door.peak_trough_index = 1;
            }
            *door_status = READY;
        } else {
            delay_ms(2);
        }
    }
    door.prev_ts = door.cur_ts;
    prev_cor_gx = *cur_cor_gx;
    prev_is_gyro_dyn = cur_is_gyro_dyn;
    prev_is_acc_dyn = cur_is_acc_dyn;
    door.prev_pitch = door.cur_pitch;

    *door_status =  NOT_READY;

		printf("value %f, count_peak_trough %d, peak_trough_index %d\n",door.peak_trough_value[door.peak_trough_index-1],door.count_peak_trough,door.peak_trough_index);
		printf("prev_dt %d, cur_dt %d, count_peak_trough %d,peak_trough_index %d\n", prev_dt,cur_dt,door.count_peak_trough,door.peak_trough_index);
		
    return *door_status;
}
