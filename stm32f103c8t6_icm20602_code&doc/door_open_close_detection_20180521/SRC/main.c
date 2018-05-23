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
            if (is_calibrated) {
              if (is_first_time) {
                is_first_time = 0;
                delay_ms(4);
              } else {
                deltaT = (float)(cur_ts - prev_ts) / 1000.0;
                if (/*is_dynamic && */1) {
                  costT = millis();
                  //MadgwickAHRSUpdateIMU(prev_gx, prev_gy, prev_gz, prev_ax, prev_ay, prev_az, deltaT);
                  //MadgwickAHRSUpdateIMU(prev_gy, prev_gx, prev_gz, prev_ay, prev_ax, prev_az, deltaT);
//                  MahonyAHRSUpdateIMU(prev_gx, prev_gy, prev_gz, prev_ax, prev_ay, prev_az, deltaT);
                  //MahonyAHRSUpdateIMU(prev_gy, prev_gx, prev_gz, prev_ay, prev_ax, prev_az, deltaT);
                  MahonyAHRSUpdateIMU(0, 0, prev_gz, 0, 0, 0, deltaT);
                  Quat2Angle();
                  costT = millis() - costT;
                  //printf("%f %f %f %f %f %f %f\n", deltaT, prev_ax, prev_ay, prev_az, prev_gx, prev_gy, prev_gz);
                  //printf(show_string, "dt:%.3f dyn:%d\nroll:%.3f\npitch:%.3f\nyaw:%.3f\n", deltaT, is_dynamic, roll, pitch, yaw);
                  sprintf(show_string, "dt:%.3f dyn:%d\nroll:%.3f\npitch:%.3f\nyaw:%.3f\n", deltaT, is_dynamic, roll, pitch, yaw);
                  oled_show_string(0, 0, show_string);
                } else {
                  sprintf(show_string, "dt:%.3f dyn:%d\n", deltaT, is_dynamic);
                  oled_show_string(0, 0, show_string);
                  delay_ms(2);
                }
              }
              prev_ts = cur_ts;
              prev_ax = cur_ax;
              prev_ay = cur_ay;
              prev_az = cur_az;
              prev_gx = cur_gx;
              prev_gy = cur_gy;
              prev_gz = cur_gz;
            } else {
              delay_ms(5);
            }
        }
    }
}
