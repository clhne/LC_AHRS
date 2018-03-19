#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "icm20602.h"
#include "spl06.h"
#include "ak8975.h"

#include "ano.h"
#include "spi.h"

int main()
{
	float acc[3] = {0.0f, 0.0f, 0.0f};
	float gyro[3] = {0.0f, 0.0f, 0.0f};
	float mag[3] = {0.0f, 0.0f, 0.0f};
	
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
		if (icm20602_get_accel(acc) == 0 && icm20602_get_gyro(gyro) == 0 && ak8975_get_mag(mag) == 0) {
			printf("acc: %f %f %f\ngyro: %f %f %f\nmag: %f %f %f\n", acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2]);
			//ANO_DT_Send_Sensor(accadc[0],accadc[1],accadc[2],gyroadc[0],gyroadc[1],gyroadc[2],magadc[0],magadc[1],magadc[2]);
		}
		ak8975_start();
		delay_ms(10);
	}
}

