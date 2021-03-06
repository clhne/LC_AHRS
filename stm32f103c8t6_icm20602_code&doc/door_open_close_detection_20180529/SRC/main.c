#include <math.h>
#include "sys.h"
#include "time.h"
#include "delay.h"
#include "usart.h"
#include "spi.h"
#include "icm20602.h"
#include "oled.h"
#include "NDOF.h"
int main()
{
    char show_string[128];
    u32 prev_ts, cur_ts;
    short ax_adc, ay_adc, az_adc, gx_adc, gy_adc, gz_adc;
    float dt, roll, prev_pitch, cur_pitch, yaw;
    //door detection declear
    u32 count_peak = 0;
    u32 count_trough = 0;
    u32 count_peak_trough = 0;
    u32 count_peak_equal = 0;
    u32 count_trough_equal = 0;
    u32 peak_trough_init = 0;
    u32 peak_index = 1;
    u32 trough_index = 1;
    u32 i = 0, j = 0;
    float peak_value[10];
    float trough_value[10];
    float peak_trough_value[20];
    u32 peak_trough_index = 1;
    //int oscillation_det;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0);
    delay_init();
    time_init();
    usart1_init(115200);
    spi2_init();
    icm20602_init();
    oled_init();
    oled_clear();
    sprintf(show_string, "init...\n");
    oled_show_string(0, 0, show_string);
    NDOF_Init(NDOF_ACC_RANGE_2G, 16, NDOF_GYRORANGE_2000DPS, 16);


    while(1) {
        float raw_ax, raw_ay, raw_az;
        float cor_ax, cor_ay, cor_az;
        float filt_ax, filt_ay, filt_az;
        float raw_gx, raw_gy, raw_gz;
        float prev_cor_gx, cur_cor_gx, cor_gy, cor_gz;
        float gx_bias, gy_bias, gz_bias;
        float prev_monotonicity, cur_monotonicity;
        float dt_count;
        int oscillation_det;
        u32 prev_is_gyro_dyn, cur_is_gyro_dyn, prev_is_acc_dyn, cur_is_acc_dyn;
        cur_ts = millis();

        if (icm20602_get_acc_gyro_adc(&ax_adc, &ay_adc, &az_adc, &gx_adc, &gy_adc, &gz_adc)) {
            delay_ms(5);
        } else {
            if (NDOF_DoStep(ax_adc, ay_adc, az_adc, gx_adc, gy_adc, gz_adc, cur_ts)) {

                dt = (float)(cur_ts - prev_ts) / 1000.0f;
                NDOF_GetEulerAngle(&roll, &cur_pitch, &yaw);
#if 0
                sprintf(show_string, "%.3f %d %d %d %d \n%.3f  \n%.3f  ", dt, NDOF_IsGyroDynamic(), NDOF_IsGyroCalibrated(), NDOF_IsAccDynamic(), NDOF_IsAccCalibrated(), roll, cur_pitch, yaw);
                oled_show_string(0, 0, show_string);
#else
                NDOF_GetRawAccData(&raw_ax, &raw_ay, &raw_az);
                NDOF_GetCorAccData(&cor_ax, &cor_ay, &cor_az);
                NDOF_GetFiltAccData(&filt_ax, &filt_ay, &filt_az);
                NDOF_GetRawGyroData(&raw_gx, &raw_gy, &raw_gz);
                NDOF_GetCorGyroData(&cur_cor_gx, &cor_gy, &cor_gz);
                NDOF_GetGyroBias(&gx_bias, &gy_bias, &gz_bias);
                //sprintf(show_string, "%.3f %d %d %d %d \n%.1f %.1f %.1f \n%.1f %.1f %.1f \n%.1f %.1f %.1f ", dt, NDOF_IsGyroDynamic(), NDOF_IsGyroCalibrated(), NDOF_IsAccDynamic(), NDOF_IsAccCalibrated(), roll, raw_gx, gx_bias, pitch, raw_gy, gy_bias, yaw, raw_gz, gz_bias);
                sprintf(show_string, "%.3f %d %d %d %d \n\n%.3f  ", dt, NDOF_IsGyroDynamic(), NDOF_IsGyroCalibrated(), NDOF_IsAccDynamic(), NDOF_IsAccCalibrated(), cur_pitch);//, yaw);
                oled_show_string(0, 0, show_string);
                cur_is_gyro_dyn = NDOF_IsGyroDynamic();
                cur_is_acc_dyn = NDOF_IsAccDynamic();
                //printf("%d %d %d %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n", cur_ts, NDOF_IsGyroDynamic(), NDOF_IsGyroCalibrated(), NDOF_IsAccDynamic(), NDOF_IsAccCalibrated(), roll, cur_pitch, yaw, raw_ax, raw_ay, raw_az, filt_ax, filt_ay, filt_az, raw_gx, raw_gy, raw_gz, cur_cor_gx, cor_gy, cor_gz);
#endif
            } else {
                delay_ms(2);
            }
        }
        prev_ts = cur_ts;
        prev_cor_gx = cur_cor_gx;
        prev_is_gyro_dyn = cur_is_gyro_dyn;
        prev_is_acc_dyn = cur_is_acc_dyn;
        prev_pitch = cur_pitch;
        /*
                //door detection of oscillation detection
                if(fabs(cur_pitch) >= 1.46) {
                    //printf("door opened\n");
        						oscillation_det = 0;
                    sprintf(show_string, "Door opened.  %d\n",oscillation_det);
                    oled_show_string(0, 6, show_string);
        //				continue;
                } else if(fabs(cur_pitch) < 1.46 && fabs(cur_pitch) > 0.63) {
                    //if(0 == cur_is_gyro_dyn && 0 == cur_is_acc_dyn) break;
                    if(prev_cor_gx < cur_cor_gx)
                        cur_monotonicity = 1;
                    else if(prev_cor_gx > cur_cor_gx)
                        cur_monotonicity = 0;
                    else
                        continue;
                    if(peak_trough_init == 1) {
                        if(cur_monotonicity != prev_monotonicity) {
                            if(cur_monotonicity == 0 && prev_monotonicity == 1 && prev_cor_gx > 0) {
                                peak_value[peak_index] = prev_cor_gx;
                                peak_index ++;
                                if(peak_value[peak_index] < 0.065) break;
                                //if(peak_value[peak_index] == peak_value[peak_index-1]) break;
                                //if(peak_index >= 6) break;
                            } else if(cur_monotonicity == 1 && prev_monotonicity == 0 && prev_cor_gx < 0) {
                                trough_value[peak_index] = prev_cor_gx;
                                trough_index ++;
                                if(fabs(trough_value[trough_index]) < 0.065) break;
                                //if(trough_value[trough_index] == trough_value[trough_index-1]) break;
                                //if(trough_index >= 6) break;
                            }
                        } else
                            peak_trough_init = 1;
                    }
                    prev_monotonicity = cur_monotonicity;
                } else {
                    //printf("door closed!\n");
                    peak_trough_init = 0;
                    peak_index = 1;
                    trough_index = 1;
                    oscillation_det = 1;
                    sprintf(show_string, "Door closed.  %d\n", oscillation_det);
                    oled_show_string(0, 6, show_string);
                }
                if(fabs(cur_pitch) < 1.46 && fabs(cur_pitch) > 0.63 && dt_count < 10 * dt) {
                    dt_count += dt;
                    dt_count = 0;
                    for(i = 1; i < peak_index - 1; i++) {
                        if(peak_value[i] < peak_value[i - 1]) {
                            count_peak++;
                        } else if(peak_value[i] == peak_value[i - 1]) {
                            count_peak_equal++;
                        } else
                            continue;
                    }
                    for(j = 1; j < trough_index - 1; j++) {
                        if(fabs(trough_value[i]) <= fabs(trough_value[i - 1])) {
                            count_trough++;
                        } else if(trough_value[i] == trough_value[i - 1]) {
                            count_trough_equal++;
                        } else
                            continue;
                    }
                    if(count_peak_equal >= 2) {
                        break;
                    }
                    if(count_trough_equal >= 2) {
                        break;
                    }
                    if(count_peak >= floor(peak_index / 4) && count_trough >= floor(trough_index / 4)) {
                        //printf("count_peak=%d,floor(peak_index/4)=%f,count_trough=%d,floor(trough_index/4)=%f,pitch=%f,door closed\n",count_peak, floor(peak_index/4), count_trough, floor(trough_index/4), cur_pitch);
                        oscillation_det = 1;
                        sprintf(show_string, "Door closed  %d\n", oscillation_det);
                        oled_show_string(0, 6, show_string);
                        break;
                    }
                } else
                    oscillation_det = 0;
                //dt_count = dt;
        				*/
        //seq/array monotonicity detection
        if(fabs(cur_pitch) >= 1.46) {
            oscillation_det = 0;
            sprintf(show_string, "Door opened.  %d\n", oscillation_det);
            oled_show_string(0, 6, show_string);
        } else if(fabs(cur_pitch) < 1.46 && fabs(cur_pitch) > 0.63) {
            if(prev_cor_gx < cur_cor_gx)
                cur_monotonicity = 1;
            else if(prev_cor_gx > cur_cor_gx)
                cur_monotonicity = 0;
            else
                continue;
            if(peak_trough_init == 1) {
                if(cur_monotonicity != prev_monotonicity) {
                    if(cur_monotonicity == 0 && prev_monotonicity == 1 && prev_cor_gx > 0) {
                        peak_trough_value[peak_trough_index] = prev_cor_gx;
                        peak_trough_index ++;
                        if(peak_trough_value[peak_index] < 0.125) break;
                    } else if(cur_monotonicity == 1 && prev_monotonicity == 0 && prev_cor_gx < 0) {
                        peak_trough_value[peak_trough_index] = prev_cor_gx;
                        peak_trough_index ++;
                        if(fabs(peak_trough_value[peak_trough_index]) < 0.125) break;
                    }
										for(i = 1; i < peak_trough_index - 1; i++) {
											if(peak_trough_value[i] <= peak_trough_value[i - 1]) {
													count_peak_trough++;
											} else
													continue;
									  }
										if(count_peak_trough >= floor(peak_trough_index * 0.5)) {
												oscillation_det = 1;
												sprintf(show_string, "Door closed  %d\n", oscillation_det);
												oled_show_string(0, 6, show_string);
												break;
										} else
												oscillation_det = 0;
                } else
                    peak_trough_init = 1;
            }
            prev_monotonicity = cur_monotonicity;
        } else {
            peak_trough_init = 0;
            peak_trough_index = 1;
            oscillation_det = 1;
            sprintf(show_string, "Door closed.  %d\n", oscillation_det);
            oled_show_string(0, 6, show_string);
        }
    }
}
