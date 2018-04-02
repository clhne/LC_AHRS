#include <math.h>
#include <stdio.h>
#include "usart.h"
#include "delay.h"
#include "time.h"
#include "mpu9250.h"
#include "MahonyAHRS.h"
 
int main(void) {
  u8 ret_code;
  u8 is_first_time = 1;
  u32 prev_ts, cur_ts;
  float prev_ax, prev_ay, prev_az, prev_gx, prev_gy, prev_gz, prev_mx, prev_my, prev_mz;
  float cur_ax, cur_ay, cur_az, cur_gx, cur_gy, cur_gz, cur_mx, cur_my, cur_mz;
  float dt, pitch, roll, yaw;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP,ENABLE);
	PWR_BackupAccessCmd(ENABLE);
	RCC_BackupResetCmd(ENABLE);
	RCC_BackupResetCmd(DISABLE);
  SystemInit();
  NVIC_Configuration();
	uart_init(115200);
	delay_init();
  time_init();
  ret_code = mpu_9250_init();
  while(1) {
    //if(mpu_9250_is_dry()) {
      cur_ts = millis();
      ret_code |= mpu_9250_read_accel_gyro(&cur_ax, &cur_ay, &cur_az, &cur_gx, &cur_gy, &cur_gz);
      ret_code |= mpu_9250_read_mag(&cur_mx, &cur_my, &cur_mz);
      if (is_first_time) {
        is_first_time = 0;
        prev_ts = cur_ts;
      } else {
        dt = (float)(cur_ts - prev_ts) / 1000.0;
        // https://github.com/kriswiner/MPU9250/issues/220#
				// https://github.com/kriswiner/MPU9250/issues/51
				//MahonyAHRSupdate(prev_gx, prev_gy, prev_gz, prev_ax, prev_ay, prev_az, prev_my, prev_mx, -prev_mz, dt);
			  MahonyAHRSupdate(prev_gx, prev_gy, prev_gz, prev_ax, prev_ay, prev_az, 0.0, 0.0, 0.0, dt);
        prev_ts = cur_ts;
				yaw   = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.32484076433121;   
        pitch = -asin(2.0f * (q1 * q3 - q0 * q2)) * 57.32484076433121;
        roll  = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * 57.32484076433121; // https://github.com/kriswiner/MPU9250/blob/master/STM32F401/main.cpp
        //printf("%d %f %f %f %f\n", ret_code, dt, roll, pitch, yaw);
        printf("%d %f %f %f %f %f %f %f %f %f %f\n", ret_code, dt, prev_gx, prev_gy, prev_gz, prev_ax, prev_ay, prev_az, prev_mx, prev_my, prev_mz);
      }
      prev_ax = cur_ax;
      prev_ay = cur_ay;
      prev_az = cur_az;
      prev_gx = cur_gx;
      prev_gy = cur_gy;
      prev_gz = cur_gz;
      prev_mx = cur_mx,
      prev_my = cur_my;
      prev_mz = cur_mz;
    //}
  }
}
