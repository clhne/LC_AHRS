#include <math.h>
#include <stdio.h>
#include "usart.h"
#include "delay.h"
#include "time.h"
//#include "mpu6050.h"
#include "mpu9250.h"
#include "led.h"
struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};

struct hal_s {
    unsigned char lp_accel_mode;
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned char motion_int_mode;
    unsigned long no_dmp_hz;
    unsigned long next_pedo_ms;
    unsigned long next_temp_ms;
	
    unsigned long next_compass_ms;
    unsigned int report;
    unsigned short dmp_features;
    struct rx_s rx;
};
static struct hal_s hal = {0};

// Every time new gyro data is available, this function is called in an
// ISR context. In this example, it sets a flag protecting the FIFO read
// function.

void gyro_data_ready_cb(void)
{
    hal.new_gyro = 1;
}
 u8 abuffer[6];
u16 ax,ay,az,gx,gy,gz,mx,my,mz,XA;
float sum=0,sum1=0,sum2=0;
int main(void) {
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP,ENABLE);
	PWR_BackupAccessCmd(ENABLE);
	RCC_BackupResetCmd(ENABLE);
	RCC_BackupResetCmd(DISABLE);
  SystemInit();
	delay_init(1000);
	NVIC_Configuration();
	mpu_iic_init();
	uart_init(115200);
	LED_Init();	 //LED¶Ë¿Ú³õÊ¼»¯115200
	mpu_9250_init();
	MPU9250_testConnection();
	delay_ms(1000);
  time_init();
  
	//mpu_6050_init();
  mpu_9250_read(ACCEL_ADDRESS,ACCEL_XOUT_H,2,abuffer); 
  XA = (((int16_t)abuffer[0]) << 8) | abuffer[1];
  while(1) {
    mpu_9250_read(ACCEL_ADDRESS,ACCEL_XOUT_H,6,abuffer);
		ax=(((int16_t)abuffer[0]) << 8) | abuffer[1];
		ay=(((int16_t)abuffer[2]) << 8) | abuffer[3];
		az=(((int16_t)abuffer[4]) << 8) | abuffer[5];
		sum=(39.2/65536.0)*(ax-XA);
		sum1=ax*0.0025;
		sum2+=sum1;
		
		mpu_9250_read(GYRO_ADDRESS, GYRO_XOUT_H, 6, abuffer);
		gx=(((int16_t)abuffer[0]) << 8) | abuffer[1];
		gy=(((int16_t)abuffer[2]) << 8) | abuffer[3];
		gz=(((int16_t)abuffer[4]) << 8) | abuffer[5];
		sum=(39.2/65536.0)*gx;
		sum1=gx*0.0025;
		sum2+=sum1;
		
		printf("ax=%d,  ay=%d,  az=%d  \n",ax,ay,az);
		printf("gx=%d,  gy=%d,  gz=%d  \n",gx,gy,gz);
    delay_ms(50);
    //printf("xxxxx %d\n", millis());
  }
}