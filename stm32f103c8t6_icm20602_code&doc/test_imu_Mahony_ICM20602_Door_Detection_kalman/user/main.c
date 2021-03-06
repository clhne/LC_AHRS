#include "sys.h"
#include "time.h"
#include "delay.h"
#include "usart.h"
#include "spi.h"
#include "icm20602.h"
#include "ak8975.h"
#include "oled.h"
//#include "MahonyAHRS.h"
#include "AHRS.h"
#include <math.h>

//extern float q0,q1,q2,q3;

int main() {
    char show_string[128];
    float acc[3] = {0.0f, 0.0f, 0.0f};
    float gyro[3] = {0.0f, 0.0f, 0.0f};
    float mag[3] = {0.0f, 0.0f, 0.0f};
    float dest1[3] = {0, 0, 0};
    float dest2[3] = {0, 0, 0};
    u8 is_first_time = 1;
    u8 is_calibrated = 0;
    static u8 is_acc_reverse = 1;
    float accel_fused_std = 0;
    u32 prev_ts, cur_ts;
    float dt, cost;
    float mag_bias_x = 10.0, mag_bias_y = 5.5, mag_bias_z = 38;   //calibrate 2018.4.13

    float prev_roll = 0, cur_roll = 0;
    float prev_pitch = 0, cur_pitch = 0;
    float prev_yaw = 0, cur_yaw = 0;
    float prev_acc[3] = {0}, prev_gyro[3] = {0}, cur_acc[3] = {0}, cur_gyro[3] = {0}, cur_mag[3] = {0};
    float cur_mean_ax, cur_mean_ay, cur_mean_az, cur_mean_gx, cur_mean_gy, cur_mean_gz;
    float cur_var_ax, cur_var_ay, cur_var_az, cur_var_gx, cur_var_gy, cur_var_gz;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0);
    JTAG_Set(SWD_ENABLE);

    delay_init();
    time_init();
    usart1_init(115200);
    spi2_init();
    //ak8975_init();
    icm20602_init();
    oled_init();
    oled_clear();
#if 0
    while(1) {
        u8 is_invalid = 0;
        cur_ts = millis();
        is_invalid |= icm20602_get_accel_gyro_statistic(
                          &cur_acc[0], &cur_acc[1], &cur_acc[2],
                          &cur_gyro[0], &cur_gyro[1], &cur_gyro[2],
                          &cur_mean_ax, &cur_mean_ay, &cur_mean_az,
                          &cur_mean_gx, &cur_mean_gy, &cur_mean_gz,
                          &cur_var_ax, &cur_var_ay, &cur_var_az, &cur_var_gx, &cur_var_gy, &cur_var_gz);
//     sprintf(show_string,"is_invalid=%d\n",is_invalid);
//     oled_show_string(0,0, show_string);
        if(is_first_time) {
            //magcalMPU9250(dest1, dest2, &mag_bias_x, &mag_bias_y, &mag_bias_z);
            is_first_time = 0;
        } else {
            dt = (float)(cur_ts - prev_ts) / 1000.0;
            cost = millis();
            if(!is_invalid) {
                accel_fused_std = sqrt(cur_var_ax + cur_var_ay + cur_var_az);
                if(is_calibrated && accel_fused_std > 0.03) {
                    MahonyAHRSUpdate(cur_gyro[1], cur_gyro[0], cur_gyro[2], cur_acc[1], cur_acc[0], cur_acc[2], cur_mag[1], -cur_mag[0], -cur_mag[2], dt);
                    MahonyAHRSupdateIMU(cur_gyro[1], cur_gyro[0], cur_gyro[2], cur_acc[1], cur_acc[0], cur_acc[2], dt);
                    Quat2Angle();
                    cur_roll = roll;
                    cur_pitch = pitch;
                    cur_yaw = yaw;
                }
                else if(!is_calibrated && accel_fused_std <= 0.03) {
                    icm20602_set_accel_gyro_bias(0, 0, 0, cur_mean_gx, cur_mean_gy, cur_mean_gz);
                    //icm20602_set_gyro_bias(cur_mean_gx, cur_mean_gy, cur_mean_gz);
                    printf("Gyro bias: %f, %f, %f\n", cur_mean_gx, cur_mean_gy, cur_mean_gz);
                    is_calibrated = 1;
                }
            }
            cost = millis() - cost;
            printf("%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", cur_roll, cur_pitch, cur_yaw, cur_acc[0], cur_acc[1], cur_acc[2], cur_gyro[0], cur_gyro[1], cur_gyro[2]);
            //printf("%f %f %f %f %f %f\n", prev_acc[0], prev_acc[1], prev_acc[2], prev_gyro[0], prev_gyro[1], prev_gyro[2]);
        }
        if(is_calibrated) {
            //sprintf(show_string, "roll=%.3f\npitch=%.3f\nyaw=%.3f\n", cur_roll, cur_pitch, cur_yaw);
            sprintf(show_string, "Roll=%.6f\nPitch=%.6f\nYaw=%.6f\n", cur_roll, cur_pitch, cur_yaw);
            oled_show_string(0, 0, show_string);
            //sprintf(show_string,"%f\n%f\n%f\n",cur_acc[0],cur_acc[1],cur_acc[2]);
//            sprintf(show_string, "%f %f      ", cost, accel_fused_std);
//            oled_show_string(0, 3, show_string);
            //Door Open_Close detection
            //false close
//          if(fabs(cur_yaw) > 1.0 && fabs(cur_yaw) < 1.8)
//              oled_show_string(0,6, "Door is open    ");
//            sprintf(show_string,"gradient = %f\n",fabs(fabs(cur_acc[1])-fabs(prev_acc[1]))/dt);
//            oled_show_string(0,6,show_string);
//            printf("%f\n",fabs(fabs(cur_acc[1])-fabs(prev_acc[1]))/dt);
            cur_acc[1] = low_filter(cur_acc[1], prev_acc[1]);
            //printf("%f\n", fabs(fabs(cur_acc[1]) - fabs(prev_acc[1])) / dt);
//            sprintf(show_string,"%f\n",fabs(fabs(cur_acc[1]) - fabs(prev_acc[1])) / dt);
//            oled_show_string(0,2,show_string);
            if(cur_yaw >= 1.60 ) {
                oled_show_string(0, 6, "Door is open    ");
            }
//            else if((cur_yaw>0.5 &&cur_yaw < 3.6 && fabs(fabs(cur_acc[1]) - fabs(prev_acc[1])) / dt > 5.0) || fabs(cur_gyro[0]) >= 0.1) {
//                //if(fabs(fabs(cur_acc[1]) - fabs(prev_acc[1])) / dt > 5.0 || fabs(cur_gyro[0]) >= 0.1) {
//                    oled_show_string(0, 6, "Door is open    ");
//                //}
////            if(fabs(cur_acc[1]) > 0.065 | fabs(cur_gyro[0]) >= 0.1) {
////                oled_show_string(0, 6, "Door is open    ");
////            }
////                else
////                    oled_show_string(0, 6, "Door is closed    ");
//            }
            //else if(cur_yaw < 1.6 && fabs(fabs(cur_acc[1]) - fabs(prev_acc[1])) / dt > 5.0) {
            else if(cur_yaw < 1.6 && fabs(fabs(cur_acc[1]) - fabs(prev_acc[1])) / dt > 5.0) {
                oled_show_string(0, 6, "Door is Open     ");
            }
            else
                oled_show_string(0, 6, "Door is closed    ");

        }
        prev_ts = cur_ts;
        prev_roll = cur_roll;
        prev_pitch = cur_pitch;
        prev_yaw = cur_yaw;
        prev_acc[0] = cur_acc[0];
        prev_acc[1] = cur_acc[1];
        prev_acc[2] = cur_acc[2];
        prev_gyro[0] = cur_gyro[0];
        prev_gyro[1] = cur_gyro[1];
        prev_gyro[2] = cur_gyro[2];
        delay_ms(5);
    }
#endif
#if 0
    while(1) {
        cur_ts = millis();
        if (icm20602_get_accel(cur_acc) == 0 && icm20602_get_gyro(cur_gyro) == 0) { // && ak8975_get_mag(cur_mag) == 0) {
            //ak8975_start();
            if(is_first_time) {
                //magcalMPU9250(dest1, dest2, &mag_bias_x, &mag_bias_y, &mag_bias_z);
                is_first_time = 0;
            } else {
//                cur_mag[0] -= mag_bias_x;
//                cur_mag[1] -= mag_bias_y;
//                cur_mag[2] -= mag_bias_z;
                dt = (float)(cur_ts - prev_ts) / 1000.0;

                cost = millis();
                //still or not
                //                if (fabs(cur_gyro[0]) < 0.005)
                //                {
                //                    cur_gyro[0] = 0.0f;
                //                }
                //                if (fabs(cur_gyro[1]) < 0.005)
                //                {
                //                    cur_gyro[1] = 0.0f;
                //                }
                //                if (fabs(cur_gyro[2]) < 0.005)
                //                {
                //                    cur_gyro[2] = 0.0f;
                //                }
                //printf("%f,%f,%f\n",cur_gyro[0],cur_gyro[1],cur_gyro[2]);
                //update(cur_gyro[0], cur_gyro[1], cur_gyro[2], cur_acc[0], cur_acc[1], cur_acc[2], -cur_mag[0], cur_mag[1], -cur_mag[2],dt);    // better
                //update(cur_gyro[1], -cur_gyro[0], -cur_gyro[2], cur_acc[1], -cur_acc[0], -cur_acc[2], 0, 0, 0, dt); //no mag
#if 0
                update(cur_gyro[0], cur_gyro[1], cur_gyro[2], cur_acc[0], cur_acc[1], cur_acc[2], 0, 0, 0, dt); //no mag
                //Door close/open detection
                cur_roll = getRoll();
                cur_pitch = getPitch();
                cur_yaw = getYaw();
#else
                //MahonyAHRSupdateIMU(cur_gyro[2], cur_gyro[1], cur_gyro[0], cur_acc[2], cur_acc[1], cur_acc[0], dt);
                MahonyAHRSupdateIMU(cur_gyro[1], cur_gyro[0], cur_gyro[2], cur_acc[1], cur_acc[0], cur_acc[2], dt);
                Quat2Angle();
                cur_roll = roll;
                cur_pitch = pitch;
                cur_yaw = yaw;
#endif
                cost = millis() - cost;
                printf("%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", cur_roll, cur_pitch, cur_yaw,
                       cur_acc[0], cur_acc[1], cur_acc[2], cur_gyro[0], cur_gyro[1], cur_gyro[2]);
                //sprintf(show_string, "Yaw=%.3f\n", cur_yaw);
                sprintf(show_string, "Roll=%.6f\nPitch=%.6f\nYaw=%.6f\n", cur_roll, cur_pitch, cur_yaw);
                //sprintf(show_string,"%f\n%f\n%f\n",cur_acc[0],cur_acc[1],cur_acc[2]);
                oled_show_string(0, 0, show_string);
                //Door Open_Close detection
                //false close
                //                if(fabs(cur_yaw) > 1.0 && fabs(cur_yaw) < 1.8)
                //                oled_show_string(0,6, "Door is open    ");
                if(cur_yaw >= 1.60 ) {
                    //printf("Door is open\n");
                    oled_show_string(0, 6, "Door is open    ");
                }
                else {
                    //printf("Door is closed\n");
                    if(fabs(cur_acc[2]) > 0.065 | fabs(cur_gyro[0]) >= 0.1) {
                        oled_show_string(0, 6, "Door is open    ");
                    }
                    else
                        oled_show_string(0, 6, "Door is closed    ");
                }

            }
        }
        prev_ts = cur_ts;
        prev_roll = cur_roll;
        prev_pitch = cur_pitch;
        prev_yaw = cur_yaw;
        prev_acc[0] = cur_acc[0];
        prev_acc[1] = cur_acc[1];
        prev_acc[2] = cur_acc[2];
        prev_gyro[0] = cur_gyro[0];
        prev_gyro[1] = cur_gyro[1];
        prev_gyro[2] = cur_gyro[2];
        //delay_ms(5);
    }
#endif

#if 1
    while(1) {
        u8 is_invalid = 0;
        cur_ts = millis();
        if (icm20602_get_accel(cur_acc) == 0 && icm20602_get_gyro(cur_gyro) == 0) {
            if(is_first_time) {
                is_first_time = 0;
            }
            else {
                dt = (float)(cur_ts - prev_ts) / 1000.0;
                kalman_filter(&prev_gyro[0], &cur_gyro[0], &prev_acc[2], &cur_acc[2], &dt);
                printf("%f,%f,%f,%f,%f\n",prev_gyro[0], cur_gyro[0], prev_acc[2], cur_acc[2], dt);
            } 
        }
				prev_ts = cur_ts;
        prev_gyro[0] = cur_gyro[0];
        prev_acc[2] = cur_acc[2];
    }
#endif
}