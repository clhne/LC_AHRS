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

extern float q0,q1,q2,q3;
int main()
{
	float acc[3] = {0.0f, 0.0f, 0.0f};
	float gyro[3] = {0.0f, 0.0f, 0.0f};
	float mag[3] = {0.0f, 0.0f, 0.0f};
	float dest1[3] = {0,0,0};
    float dest2[3] = {0,0,0};
	u8 is_first_time = 1;
    //统计一次while循环时间int32
    u32 prev_ts, cur_ts, time_begin, time_end;
    float dt;
    float mag_bias_x = 10.0, mag_bias_y = 5.5, mag_bias_z = 38;
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
    
	while(1)	
	{
        cur_ts = millis();
        time_begin = millis();
		if (icm20602_get_accel(cur_acc) == 0 && icm20602_get_gyro(cur_gyro) == 0 && ak8975_get_mag(cur_mag) == 0) {
			/*
			gyro[0] = 0.008514; gyro[1] = -0.004257; gyro[2] = 0.025541;
			acc[0] = -5.765238; acc[1] = -3.926491;  acc[2] = 7.077260;
			mag[0] = 46.800003; mag[1] = 28.800001;  mag[2] = 68.400002;
			*/
			//printf("acc: %f %f %f\ngyro: %f %f %f\nmag: %f %f %f\n", acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2]);
			//ANO_DT_Send_Sensor(accadc[0],accadc[1],accadc[2],gyroadc[0],gyroadc[1],gyroadc[2],magadc[0],magadc[1],magadc[2]);

            if(is_first_time){
                //magcalMPU9250(dest1, dest2, &mag_bias_x, &mag_bias_y, &mag_bias_z);
                is_first_time = 0;
                prev_ts = cur_ts;
            }
            else{
            mag[0] -= mag_bias_x;
            mag[1] -= mag_bias_y;
            mag[2] -= mag_bias_z;           
            dt = (float)(cur_ts - prev_ts) / 1000.0;
            //update(cur_gyro[0], cur_gyro[1], cur_gyro[2], cur_acc[0], cur_acc[1], cur_acc[2], -cur_mag[0], cur_mag[1], -cur_mag[2],dt);     // Initial 
            //update(cur_gyro[0], cur_gyro[1], cur_gyro[2], cur_acc[0], cur_acc[1], cur_acc[2], -cur_mag[0], -cur_mag[1], -cur_mag[2],dt);    // better
            
            //printf("dt = %f,cur_ts = %d\n",dt,cur_ts);
            
            update(cur_gyro[0], cur_gyro[1], cur_gyro[2], cur_acc[0], cur_acc[1], cur_acc[2], 0, 0, 0, dt); //no mag
            prev_ts = cur_ts;
			//printf("q0 %f, q1 %f,q2 %f, q3 %f\n",q0,q1,q2,q3);
           // printf("acc_x=%f\tacc_y=%f\tacc_z=%f\t\n",cur_acc[0]-prev_acc[0],cur_acc[1]-prev_acc[1],cur_acc[2]-prev_acc[2]);
           // printf("gyro_x=%f\tgyro_y=%f\tgyro_z=%f\t\n",cur_gyro[0]-prev_gyro[0],cur_gyro[1]-prev_gyro[1],cur_gyro[2]-prev_gyro[2]);
            //printf("%f\n",mag[1]);
                
		    //printf("%f, %f, %f\n",cur_mag[0],cur_mag[1],cur_mag[2]);                
           
            //Door close/open detection
            cur_roll = getRoll();
            cur_pitch = getPitch();
            cur_yaw = getYaw();
            printf("roll = %f,pitch = %f,yaw = %f\n",cur_roll, cur_pitch, cur_yaw);  
            if(cur_roll >= 1.0 | fabs(cur_acc[1]-prev_acc[1])>=0.15 | fabs(cur_gyro[0]-prev_gyro[0]) >=0.09){
                printf("Door is open\n");
                //delay_ms(10);
            }
            else
            {
                printf("Door is closed\n");
                //delay_ms(10);
            }            
            }
		}
		prev_roll = cur_roll; prev_pitch = cur_pitch; prev_yaw = cur_yaw;
        prev_acc[0] = cur_acc[0]; prev_acc[1] = cur_acc[1]; prev_acc[2] = cur_acc[2];
        prev_gyro[0] = cur_gyro[0]; prev_gyro[1] = cur_gyro[1]; prev_gyro[2] = cur_gyro[2];
		//ak8975_start();
        time_end = millis();
        printf("run time = %d ms\n",time_end - time_begin);
		delay_ms(10);
//        time_end = millis();
//        printf("run time = %d ms\n",time_end - time_begin);
	}
}

