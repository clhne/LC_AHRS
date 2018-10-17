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
	  int peak_trough_index = 0;
	  int peak_trough_count = 0;
    float pitch = 0;
	  float roll = 0;
	  float cor_gx = 0;
	  float cor_gy = 0;
		float cor_gz = 0;
	  long long Detection_dt = 0;
	  long long Total_dt = 0;
    long long prev_ts;
		long long cur_ts;
    Door_Detection_Init(NDOF_ACC_RANGE_2G, 16, NDOF_GYRORANGE_2000DPS, 16);
	  
		//Debug param
		float cor_ax;
		float cor_ay;
		float cor_az;			  
		
    while(1) {
        //Detection Door Status
        if (icm20602_get_acc_gyro_adc(&ax_adc, &ay_adc, &az_adc, &gx_adc, &gy_adc, &gz_adc)) {
            delay_ms(5);
        } else {
					  //sample time must greater than 5ms!
            prev_ts = millis();
					  NDOF_GetCorAccData(&cor_ax, &cor_ay,&cor_az);
											
						door_status = Door_Detection(ax_adc, ay_adc, az_adc, gx_adc, gy_adc, gz_adc, &pitch, &roll,&cor_gx, &cor_gy, &cor_gz, &cor_ax,&cor_ay,&peak_trough_index, &peak_trough_count,prev_ts);		
						//door_status = Door_Detection(-ax_adc, ay_adc, az_adc, gx_adc, gy_adc, gz_adc, &pitch, &cor_gx, &cor_gy, &cor_gz, &peak_trough_index, &peak_trough_count,prev_ts);		
						//door_status = Door_Detection(ay_adc, -ax_adc, az_adc, gx_adc, gy_adc, gz_adc, &pitch, &cor_gx, &cor_gy, &cor_gz, &peak_trough_index, &peak_trough_count,prev_ts);
	            
						//door_status = Door_Detection(-ay_adc, ax_adc, az_adc, gx_adc, gy_adc, gz_adc, &pitch, &roll, &yaw,&cor_gx, &cor_gy, &cor_gz, &peak_trough_index, &peak_trough_count,prev_ts);
					  printf("%f,%f,%f,pitch %f,roll %f\n",cor_ax,cor_ay,cor_az,pitch,roll);
            //door_status = Door_Detection(ax_adc, ay_adc, az_adc, gx_adc, gy_adc, gz_adc, &pitch, &cor_gx, &cor_gy, &cor_gz, &peak_trough_index, &peak_trough_count,prev_ts);
					  cur_ts = millis();
					  Detection_dt = cur_ts - prev_ts;
//            sprintf(show_string, "%lld %lld %d %d %d %d \n%.3f\n%.3f %d %d\n", Detection_dt, Total_dt, NDOF_IsGyroDynamic(), 
//					      NDOF_IsGyroCalibrated(), NDOF_IsAccDynamic(), NDOF_IsAccCalibrated(), cor_gx, pitch, peak_trough_index, peak_trough_count);
					sprintf(show_string, "%lld %lld %d %d %d %d \n%d %d %.3f\n%.3f\n", Detection_dt, Total_dt, NDOF_IsGyroDynamic(), 
					      NDOF_IsGyroCalibrated(), NDOF_IsAccDynamic(), NDOF_IsAccCalibrated(), peak_trough_index, peak_trough_count, cor_gx, pitch);
					/*
				  printf("%lld %lld %d %d %d %d, %d %d %.3f, %.3f, %.3f, %.3f\n", Detection_dt, Total_dt, NDOF_IsGyroDynamic(), 
					      NDOF_IsGyroCalibrated(), NDOF_IsAccDynamic(), NDOF_IsAccCalibrated(), peak_trough_index, peak_trough_count, cor_gx, cor_gy, cor_gz, pitch);
					*/
					
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
						Total_dt = millis() - prev_ts;
        }
    }
}
