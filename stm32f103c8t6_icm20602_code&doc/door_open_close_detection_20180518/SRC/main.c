#include <math.h>
#include "sys.h"
#include "time.h"
#include "delay.h"
#include "usart.h"
#include "spi.h"
#include "icm20602.h"
#include "oled.h"
#include "AHRS.h"
int main() {
    char show_string[128];
    u8 is_first_time = 1;
    u32 prev_ts, cur_ts;
    float prev_ax, prev_ay, prev_az, prev_gx, prev_gy, prev_gz;
    float cur_ax, cur_ay, cur_az, cur_gx, cur_gy, cur_gz;
    float deltaT, costT;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0);
    delay_init();
    time_init();
    usart1_init(115200);
    spi2_init();
    icm20602_init();
    oled_init();
    oled_clear();
    sprintf(show_string, "initializing...\n");
    oled_show_string(0, 0, show_string);
    while(1) {
        int8_t is_dynamic = 1, is_calibrated = 0;
        cur_ts = millis();
        if (icm20602_get_acc_gyro_with_calib(&cur_ax, &cur_ay, &cur_az, &cur_gx, &cur_gy, &cur_gz, &is_dynamic, &is_calibrated) == 0) {
            if (is_first_time) {
                is_first_time = 0;
                delay_ms(5);
            } else {
              if (is_dynamic && is_calibrated) {
                deltaT = (float)(cur_ts - prev_ts) / 1000.0;
                costT = millis();
                MadgwickAHRSUpdateIMU(prev_gx, prev_gy, prev_gz, prev_ax, prev_ay, prev_az, deltaT);
                Quat2Angle();
                costT = millis() - costT;
                //printf("%f %f %f %f %f %f %f\n", deltaT, prev_ax, prev_ay, prev_az, prev_gx, prev_gy, prev_gz);
                //printf("%f %f %f %f %f\n", costT / 1000.0f, deltaT, roll, pitch, yaw);
                sprintf(show_string, "dt:%.3f\nroll:%.3f\npitch:%.3f\nyaw:%.3f\n", deltaT, roll, pitch, yaw);
                oled_show_string(0, 0, show_string);
              } else {
                delay_ms(5);
              }
            }
            prev_ts = cur_ts;
            prev_ax = cur_ax;
            prev_ay = cur_ay;
            prev_az = cur_az;
            prev_gx = cur_gx;
            prev_gy = cur_gy;
            prev_gz = cur_gz;
        }
    }
}
