#include <math.h>
#include "sys.h"
#include "time.h"
#include "delay.h"
#include "usart.h"
#include "spi.h"
#include "icm20602.h"
#include "oled.h"
#include "doordetection.h"
int main()
{
    char show_string[128];
    short ax_adc, ay_adc, az_adc, gx_adc, gy_adc, gz_adc;
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

    //Door param
    int door_status;
    float pitch;
	  float cor_gx;
	  long long Detection_dt;
	  long long Total_dt = 0;
    long long prev_ts;
		long long cur_ts;
    Door_Detection_Init(NDOF_ACC_RANGE_2G, 16, NDOF_GYRORANGE_2000DPS, 16);
		//Debug param
		float cor_ax;
		float cor_ay;
		float cor_az;
		float cor_gy;
	  float cor_gz;
    while(1) {
        //Detection Door Status
        if (icm20602_get_acc_gyro_adc(&ax_adc, &ay_adc, &az_adc, &gx_adc, &gy_adc, &gz_adc)) {
            delay_ms(5);
        } else {
					  //sample time must greater than 5ms!
            prev_ts = millis();
            door_status = Door_Detection(ax_adc, ay_adc, az_adc, gx_adc, gy_adc, gz_adc, &pitch, &cor_gx, prev_ts);
					  cur_ts = millis();
					  Detection_dt = cur_ts - prev_ts;
					/*
            sprintf(show_string, "%lld %lld %d %d %d %d \n%.3f\n", Detection_dt, Total_dt, NDOF_IsGyroDynamic(), NDOF_IsGyroCalibrated(), NDOF_IsAccDynamic(), NDOF_IsAccCalibrated(), pitch);
            oled_show_string(0, 0, show_string);
            if(door_status == DOOR_STATUS_OPEN) {
                sprintf(show_string, "Door opened.   \n");
                oled_show_string(0, 6, show_string);
            } else if(door_status == DOOR_STATUS_CLOSE) {
                sprintf(show_string, "Door closed.   \n");
                oled_show_string(0, 6, show_string);
            } else if(door_status == DOOR_STATUS_DETECTING) {
                sprintf(show_string, "Door detecting. \n");
                oled_show_string(0, 6, show_string);
            } else {
                sprintf(show_string, "Door not ready. \n");
                oled_show_string(0, 6, show_string);
                delay_ms(2);
            }
						*/
						NDOF_GetCorAccData(&cor_ax, &cor_ay, &cor_az);
						NDOF_GetCorGyroData(&cor_gx, &cor_gy, &cor_gz);
						sprintf(show_string,"%.3f %.3f\n%.3f\n", cor_ax, cor_ay, cor_az);
						oled_show_string(0,0,show_string);
						sprintf(show_string,"%.3f %.3f\n%.3f\n",cor_gx,cor_gy,cor_gz);
						oled_show_string(0,4,show_string);
						Total_dt = millis() - prev_ts;
        }
    }
}
