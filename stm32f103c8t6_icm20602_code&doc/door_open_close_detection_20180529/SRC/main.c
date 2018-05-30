#include <math.h>
#include "sys.h"
#include "time.h"
#include "delay.h"
#include "usart.h"
#include "spi.h"
#include "icm20602.h"
#include "oled.h"
#include "NDOF.h"
int main() {
    char show_string[128];
    u32 prev_ts, cur_ts;
    short ax_adc, ay_adc, az_adc, gx_adc, gy_adc, gz_adc;
    float dt, roll, pitch, yaw;
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
        cur_ts = millis();
        if (icm20602_get_acc_gyro_adc(&ax_adc, &ay_adc, &az_adc, &gx_adc, &gy_adc, &gz_adc)) {
            delay_ms(5);
        } else {
            if (NDOF_DoStep(ax_adc, ay_adc, az_adc, gx_adc, gy_adc, gz_adc, cur_ts)) {
                float raw_ax, raw_ay, raw_az;
                float cor_ax, cor_ay, cor_az;
                float filt_ax, filt_ay, filt_az;
                float raw_gx, raw_gy, raw_gz;
                float cor_gx, cor_gy, cor_gz;
                float gx_bias, gy_bias, gz_bias;
                dt = (float)(cur_ts - prev_ts) / 1000.0f;
                NDOF_GetEulerAngle(&roll, &pitch, &yaw);
#if 0
                sprintf(show_string, "%.3f %d %d %d %d \n%.3f  \n%.3f  \n%.3f  ", dt, NDOF_IsGyroDynamic(), NDOF_IsGyroCalibrated(), NDOF_IsAccDynamic(), NDOF_IsAccCalibrated(), roll, pitch, yaw);
                oled_show_string(0, 0, show_string);
#else
                NDOF_GetRawAccData(&raw_ax, &raw_ay, &raw_az);
                NDOF_GetCorAccData(&cor_ax, &cor_ay, &cor_az);
                NDOF_GetFiltAccData(&filt_ax, &filt_ay, &filt_az);
                NDOF_GetRawGyroData(&raw_gx, &raw_gy, &raw_gz);
                NDOF_GetCorGyroData(&cor_gx, &cor_gy, &cor_gz);
                NDOF_GetGyroBias(&gx_bias, &gy_bias, &gz_bias); 
                //sprintf(show_string, "%.3f %d %d %d %d \n%.1f %.1f %.1f \n%.1f %.1f %.1f \n%.1f %.1f %.1f ", dt, NDOF_IsGyroDynamic(), NDOF_IsGyroCalibrated(), NDOF_IsAccDynamic(), NDOF_IsAccCalibrated(), roll, raw_gx, gx_bias, pitch, raw_gy, gy_bias, yaw, raw_gz, gz_bias);
							  sprintf(show_string, "%.3f %d %d %d %d \n%.3f  \n%.3f  \n%.3f  ", dt, NDOF_IsGyroDynamic(), NDOF_IsGyroCalibrated(), NDOF_IsAccDynamic(), NDOF_IsAccCalibrated(), roll, pitch, yaw);
                oled_show_string(0, 0, show_string);
                printf("%d %d %d %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n", cur_ts, NDOF_IsGyroDynamic(), NDOF_IsGyroCalibrated(), NDOF_IsAccDynamic(), NDOF_IsAccCalibrated(), roll, pitch, yaw, raw_ax, raw_ay, raw_az, filt_ax, filt_ay, filt_az, raw_gx, raw_gy, raw_gz, cor_gx, cor_gy, cor_gz);
#endif
            } else {
                delay_ms(5);
            }
        }
        prev_ts = cur_ts;
    }
}
