#include "sys.h"
#include "time.h"
#include "delay.h"
#include "usart.h"
#include "spi.h"
#include "icm20602.h"
#include "ak8975.h"
#include "oled.h"
#include "MahonyAHRS.h"

extern float q0,q1,q2,q3;
int main() {
    char show_string[128];
    float acc[3] = {0.0f, 0.0f, 0.0f};
    float gyro[3] = {0.0f, 0.0f, 0.0f};
    float mag[3] = {0.0f, 0.0f, 0.0f};
    float dest1[3] = {0,0,0};
    float dest2[3] = {0,0,0};
    u8 is_first_time = 1;
    u32 prev_ts, cur_ts;
    float dt, cost;
    float mag_bias_x = 10.0, mag_bias_y = 5.5, mag_bias_z = 38;   //calibrate 2018.4.13

    float prev_roll = 0, cur_roll =0;
    float prev_pitch = 0, cur_pitch = 0;
    float prev_yaw = 0, cur_yaw = 0;
    float prev_acc[3] = {0}, prev_gyro[3]= {0}, cur_acc[3]= {0}, cur_gyro[3]= {0}, cur_mag[3]= {0};
    //Kalman filter
    float aax=0, aay=0,aaz=0, agx=0, agy=0, agz=0;   
    float axo = 0, ayo = 0, azo = 0;   
    float gxo = 0, gyo = 0, gzo = 0;
    float pi = 3.1415926;
    float AcceRatio = 16384.0;                 
    float GyroRatio = 131.0;                    
    uint8_t n_sample = 8;    
    int i;        
    float aaxs[8] = {0}, aays[8] = {0}, aazs[8] = {0};   
    long aax_sum, aay_sum, aaz_sum;    
    float a_x[10]={0}, a_y[10]={0},a_z[10]={0} ,g_x[10]={0} ,g_y[10]={0},g_z[10]={0};
    float Px=1, Rx, Kx, Sx, Vx, Qx; 
    float Py=1, Ry, Ky, Sy, Vy, Qy;
    float Pz=1, Rz, Kz, Sz, Vz, Qz; 
    float gyrox, gyroy, gyroz;
    float accx, accy, accz;
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0);
    JTAG_Set(SWD_ENABLE);

    delay_init();
    time_init();
    usart1_init(115200);
    spi2_init();
    ak8975_init();
    icm20602_init();
    oled_init();
    oled_clear();
        
    while(1) {
        cur_ts = millis();
        if (icm20602_get_accel(cur_acc) == 0 && icm20602_get_gyro(cur_gyro) == 0 && ak8975_get_mag(cur_mag) == 0) {
            ak8975_start();
            if(is_first_time) {
                //magcalMPU9250(dest1, dest2, &mag_bias_x, &mag_bias_y, &mag_bias_z);
                is_first_time = 0;
                prev_ts = cur_ts;
            } else {
                cur_mag[0] -= mag_bias_x;
                cur_mag[1] -= mag_bias_y;
                cur_mag[2] -= mag_bias_z;
                dt = (float)(cur_ts - prev_ts) / 1000.0;
                prev_ts = cur_ts;
                cost = millis();
                //update(cur_gyro[0], cur_gyro[1], cur_gyro[2], cur_acc[0], cur_acc[1], cur_acc[2], -cur_mag[0], cur_mag[1], -cur_mag[2],dt);    // better
                update(cur_gyro[1], -cur_gyro[0], -cur_gyro[2], cur_acc[1], -cur_acc[0], -cur_acc[2], 0, 0, 0, dt); //no mag
                //Door close/open detection
                cur_roll = getRoll();
                cur_pitch = getPitch();
                cur_yaw = getYaw();
                cost = millis() - cost;
                printf("cost=%f dt=%f roll=%f pitch=%f yaw=%f\n", cost / 1000.0f, dt, cur_roll, cur_pitch, cur_yaw);
                sprintf(show_string, "roll=%.3f\npitch=%.3f\nyaw=%.3f\n", cur_roll, cur_pitch, cur_yaw);
                oled_show_string(0,0, show_string);
                if(cur_roll >= 1.0 | fabs(cur_acc[1]-prev_acc[1])>=0.15 | fabs(cur_gyro[0]-prev_gyro[0]) >=0.09) {
                    //printf("Door is open\n");
                    oled_show_string(0,6, "Door is open\n");
                }
                else {
                    //printf("Door is closed\n");
                    oled_show_string(0,6, "Door is closed\n");
                }
            }
        }
        prev_roll = cur_roll;
        prev_pitch = cur_pitch;
        prev_yaw = cur_yaw;
        prev_acc[0] = cur_acc[0];
        prev_acc[1] = cur_acc[1];
        prev_acc[2] = cur_acc[2];
        prev_gyro[0] = cur_gyro[0];
        prev_gyro[1] = cur_gyro[1];
        prev_gyro[2] = cur_gyro[2];
        //delay_ms(10);
    }
}
/*
    while(1) {
        cur_ts = millis();
        if (icm20602_get_accel(cur_acc) == 0 && icm20602_get_gyro(cur_gyro) == 0 && ak8975_get_mag(cur_mag) == 0) {
            ak8975_start();
            if(is_first_time) {
                //magcalMPU9250(dest1, dest2, &mag_bias_x, &mag_bias_y, &mag_bias_z);
                is_first_time = 0;
                prev_ts = cur_ts;
            } else {
                cur_mag[0] -= mag_bias_x;
                cur_mag[1] -= mag_bias_y;
                cur_mag[2] -= mag_bias_z;
                dt = (float)(cur_ts - prev_ts) / 1000.0;
                prev_ts = cur_ts;
                cost = millis();
                accx = cur_acc[0] / AcceRatio; 
                accy = cur_acc[1] / AcceRatio; 
                accz = cur_acc[2] / AcceRatio; 
                aax = atan(accy / accz) * (-180) / pi;
                aay = atan(accx / accz) * 180 / pi;
                aaz = atan(accz / accy) * 180 / pi;
                aax_sum = 0;                              
                aay_sum = 0;
                aaz_sum = 0;
              
                for(i=1;i<n_sample;i++)
                {
                    aaxs[i-1] = aaxs[i];
                    aax_sum += aaxs[i] * i;
                    aays[i-1] = aays[i];
                    aay_sum += aays[i] * i;
                    aazs[i-1] = aazs[i];
                    aaz_sum += aazs[i] * i;
                
                }
                
                aaxs[n_sample-1] = aax;
                aax_sum += aax * n_sample;
                aax = (aax_sum / (11*n_sample/2.0)) * 9 / 7.0; 
                aays[n_sample-1] = aay;                        
                aay_sum += aay * n_sample;                     
                aay = (aay_sum / (11*n_sample/2.0)) * 9 / 7.0;
                aazs[n_sample-1] = aaz; 
                aaz_sum += aaz * n_sample;
                aaz = (aaz_sum / (11*n_sample/2.0)) * 9 / 7.0;

                gyrox = - (cur_gyro[0]-gxo) / GyroRatio * dt; 
                gyroy = - (cur_gyro[1]-gyo) / GyroRatio * dt; 
                gyroz = - (cur_gyro[2]-gzo) / GyroRatio * dt; 
                agx += gyrox;                             
                agy += gyroy;                             
                agz += gyroz;
                
                // kalman start 
                Sx = 0; Rx = 0;
                Sy = 0; Ry = 0;
                Sz = 0; Rz = 0;
                
                for(i=1;i<10;i++)
                {                 
                    a_x[i-1] = a_x[i];                      
                    Sx += a_x[i];
                    a_y[i-1] = a_y[i];
                    Sy += a_y[i];
                    a_z[i-1] = a_z[i];
                    Sz += a_z[i];
                
                }
                
                a_x[9] = aax;
                Sx += aax;
                Sx /= 10;                                
                a_y[9] = aay;
                Sy += aay;
                Sy /= 10;                             
                a_z[9] = aaz;
                Sz += aaz;
                Sz /= 10;
///////////////////////////
                for(i=0;i<10;i++)
                {
                    Rx += (a_x[i] - Sx)*(a_x[i] - Sx);
                    Ry += (a_y[i] - Sy)*(a_y[i] - Sy);
                    Rz += (a_z[i] - Sz)*(a_z[i] - Sz);
                
                }
                printf("Rx=%f,Ry=%f,Rz=%f\n",Rx,Ry,Rz);
                Rx = Rx / 9;                            
                Ry = Ry / 9;                        
                Rz = Rz / 9;
              
                Px = Px + 0.0025;                        
                Kx = Px / (Px + Rx);                      
                agx = agx + Kx * (aax - agx);             
                Px = (1 - Kx) * Px;                      

                Py = Py + 0.0025;
                Ky = Py / (Py + Ry);
                agy = agy + Ky * (aay - agy); 
                Py = (1 - Ky) * Py;
              
                Pz = Pz + 0.0025;
                Kz = Pz / (Pz + Rz);
                agz = agz + Kz * (aaz - agz); 
                Pz = (1 - Kz) * Pz;
                //Door close/open detection
                cur_roll = agx;
                cur_pitch = agy;
                cur_yaw = agz;
                cost = millis() - cost;
                printf("cost=%f dt=%f roll=%f pitch=%f yaw=%f\n", cost / 1000.0f, dt, cur_roll, cur_pitch, cur_yaw);
                sprintf(show_string, "roll=%.3f\npitch=%.3f\nyaw=%.3f\n", cur_roll, cur_pitch, cur_yaw);
                oled_show_string(0,0, show_string);
                if(cur_roll >= 1.0 | fabs(cur_acc[1]-prev_acc[1])>=0.15 | fabs(cur_gyro[0]-prev_gyro[0]) >=0.09) {
                    //printf("Door is open\n");
                    oled_show_string(0,6, "Door is open");
                }
                else {
                    //printf("Door is closed\n");
                    oled_show_string(0,6, "Door is closed");
                }
            }
        }
        prev_roll = cur_roll;
        prev_pitch = cur_pitch;
        prev_yaw = cur_yaw;
        prev_acc[0] = cur_acc[0];
        prev_acc[1] = cur_acc[1];
        prev_acc[2] = cur_acc[2];
        prev_gyro[0] = cur_gyro[0];
        prev_gyro[1] = cur_gyro[1];
        prev_gyro[2] = cur_gyro[2];
        //delay_ms(10);
    }
}
*/