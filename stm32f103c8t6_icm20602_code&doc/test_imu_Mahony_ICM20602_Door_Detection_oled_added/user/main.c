#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "icm20602.h"
#include "spl06.h"
#include "ak8975.h"
#include "MahonyAHRS.h"
#include "ano.h"
#include "spi.h"
#include "time.h"
#include "oled.h"

extern float q0,q1,q2,q3;
int main()
{
  char show_string[128];
	float acc[3] = {0.0f, 0.0f, 0.0f};
	float gyro[3] = {0.0f, 0.0f, 0.0f};
	float mag[3] = {0.0f, 0.0f, 0.0f};
	float dest1[3] = {0,0,0};
    float dest2[3] = {0,0,0};
	u8 is_first_time = 1;
    u32 prev_ts, cur_ts, time_begin, time_end;
    float dt;
    float mag_bias_x = 10.0, mag_bias_y = 5.5, mag_bias_z = 38;   //calibrate 2018.4.13

	float prev_roll = 0, cur_roll =0;
	float prev_pitch = 0, cur_pitch = 0;
	float prev_yaw = 0, cur_yaw = 0;
    float prev_acc[3] = {0}, prev_gyro[3]={0}, cur_acc[3]={0}, cur_gyro[3]={0}, cur_mag[3]={0};
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0);
	JTAG_Set(SWD_ENABLE);
	
	delay_init();
    time_init();
	usart1_init(500000);
	spi2_init();
	
	ak8975_init();
	icm20602_init();
	spl06_init();
  oled_init();
  oled_clear(); 
	while(1)	
	{
        cur_ts = millis();
        time_begin = millis();
		if (icm20602_get_accel(cur_acc) == 0 && icm20602_get_gyro(cur_gyro) == 0 && ak8975_get_mag(cur_mag) == 0) {		
            if(is_first_time){
                //magcalMPU9250(dest1, dest2, &mag_bias_x, &mag_bias_y, &mag_bias_z);
                is_first_time = 0;
                prev_ts = cur_ts;
            }
            else{
            cur_mag[0] -= mag_bias_x;
            cur_mag[1] -= mag_bias_y;
            cur_mag[2] -= mag_bias_z;           
            dt = (float)(cur_ts - prev_ts) / 1000.0;
            //update(cur_gyro[0], cur_gyro[1], cur_gyro[2], cur_acc[0], cur_acc[1], cur_acc[2], -cur_mag[0], -cur_mag[1], -cur_mag[2],dt);    // better
                        
            update(-cur_gyro[2], -cur_gyro[1], cur_gyro[0], -cur_acc[2], -cur_acc[1], cur_acc[0], 0, 0, 0, dt); //no mag
            prev_ts = cur_ts;
                
            //Door close/open detection
            cur_roll = getRoll();
            cur_pitch = getPitch();
            cur_yaw = getYaw();
            printf("roll = %f,pitch = %f,yaw = %f\n",cur_roll, cur_pitch, cur_yaw); 
       //     printf("%f, %f, %f, %f\n",q0,q1,q2,q3);
            sprintf(show_string, "roll=%.3f\npitch=%.3f\nyaw=%.3f\n", cur_roll, cur_pitch, cur_yaw);
            oled_show_string(0,0, show_string);              
            if(cur_roll >= 1.0 | fabs(cur_acc[1]-prev_acc[1])>=0.15 | fabs(cur_gyro[0]-prev_gyro[0]) >=0.09){
                //printf("Door is open\n");
                oled_show_string(0,6,"Door is open");
            }
            else
            {
                //printf("Door is closed\n");
                oled_show_string(0,6,"Door is closed");
            }            
            }
		}
		prev_roll = cur_roll; prev_pitch = cur_pitch; prev_yaw = cur_yaw;
        prev_acc[0] = cur_acc[0]; prev_acc[1] = cur_acc[1]; prev_acc[2] = cur_acc[2];
        prev_gyro[0] = cur_gyro[0]; prev_gyro[1] = cur_gyro[1]; prev_gyro[2] = cur_gyro[2];
		ak8975_start();
		delay_ms(10);
        time_end = millis();
        //printf("run time = %d ms\n",time_end - time_begin);
    
	}
}

