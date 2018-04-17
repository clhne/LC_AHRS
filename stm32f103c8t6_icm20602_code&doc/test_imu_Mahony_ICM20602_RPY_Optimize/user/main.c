#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "icm20602.h"
#include "spl06.h"
#include "ak8975.h"
#include "MahonyAHRS.h"
#include "ano.h"
#include "spi.h"
extern float q0,q1,q2,q3;
int main()
{
	float acc[3] = {0.0f, 0.0f, 0.0f};
	float gyro[3] = {0.0f, 0.0f, 0.0f};
	float mag[3] = {0.0f, 0.0f, 0.0f};
	float dest1[3] = {0,0,0};
    float dest2[3] = {0,0,0};
	u8 is_first_time = 1;
    float mag_bias_x = 22, mag_bias_y = 5.5, mag_bias_z = 30;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0);
	JTAG_Set(SWD_ENABLE);
	
	delay_init();
	usart1_init(500000);
	spi2_init();
	
	ak8975_init();
	icm20602_init();
	spl06_init();
    
	while(1)	
	{
		float roll = getRoll();
		float pitch = getPitch();
		float yaw = getYaw();
		if (icm20602_get_accel(acc) == 0 && icm20602_get_gyro(gyro) == 0 && ak8975_get_mag(mag) == 0) {
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
            }
            else{
            mag[0] -= mag_bias_x;
            mag[1] -= mag_bias_y;
            mag[2] -= mag_bias_z;           
            //update(gyro[1],-gyro[0],-gyro[2],acc[1],-acc[0],-acc[2],mag[1],mag[0],mag[2]);
            update(gyro[0], -gyro[1], -gyro[2], acc[0], -acc[1], -acc[2], -mag[0], -mag[1], mag[2]);
			//printf("q0 %f, q1 %f,q2 %f, q3 %f\n",q0,q1,q2,q3);
            printf("acc_x=%f\tacc_y=%f\tacc_z=%f\t\n",acc[0],acc[1],acc[2]);
            //printf("%f\n",mag[1]);
			//printf("%f, %f, %f\n",mag[0],mag[1],mag[2]);
         //   printf("roll = %f,pitch = %f,yaw = %f\n",roll, pitch, yaw);
            }
		}
		
		ak8975_start();
		delay_ms(10);
	}
}

