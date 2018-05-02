#include <stdio.h>
#include <math.h>
#include "led.h"
#include "key.h"
#include "usart.h"
#include "delay.h"
#include "time.h"
#include "encoder.h"
#include "adc.h"
#include "oled.h"
#include "mpu9250.h"
#include "motor.h"
#include "exti.h"
#include "AHRS.h"

//#define ENABLE_MAGNETIC_FILED

// ȫ�ֱ���
u8 delay_flag, delay_50ms, delay_10ms = 0;
float battary_voltage = 12.0f;
int left_target_pwm = 0, right_target_pwm = 0; // ���PWM����
int left_pid_pwm, right_pid_pwm; // ���PWM����
int left_encoder, right_encoder;  // ���������������

u8 is_calibrated = 0;
float accel_fused_std;
float cost;

u8 is_low_power() {
    return battary_voltage < 11.2f ? 1 : 0;
}

/**************************************************************************
�������ܣ�����PI������
��ڲ���������������ֵ��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)��������ƫ��
e(k-1)������һ�ε�ƫ��  �Դ�����
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/
#define MOTOR_KP 12
#define MOTOR_KI 12
int left_PID_control(int encoder, int target_pwm) {
    static int pid_pwm = 0, last_bias = 0;
    int bias = encoder - target_pwm; //����ƫ��
    pid_pwm += MOTOR_KP * (bias - last_bias) + MOTOR_KI * bias; // ����ʽPI������
    last_bias = bias; // ������һ��ƫ��
    return pid_pwm; // �������
}

int right_PID_control(int encoder, int target_pwm) {
    static int pid_pwm = 0, last_bias = 0;
    int bias = encoder - target_pwm; // ����ƫ��
    pid_pwm += MOTOR_KP * (bias - last_bias) + MOTOR_KI * bias; // ����ʽPI������
    last_bias = bias; // ������һ��ƫ��
    return pid_pwm; // �������
}

/**************************************************************************
�������ܣ����еĿ��ƴ��붼��������
         5ms��ʱ�ж���MPU6050��INT���Ŵ���
         �ϸ�֤���������ݴ�����ʱ��ͬ��
**************************************************************************/
int EXTI9_5_IRQHandler(void) {
    if (PBin(5) == 0) { // PB5���ӵ�MPU6050���ж�����
        EXTI_ClearITPendingBit(EXTI_Line5);
        if (delay_flag == 1) {
            if (++delay_50ms == 10)	 { //���������ṩ50ms�ľ�׼��ʱ
                delay_50ms = 0;
                delay_flag = 0;
            }
        }
        if (delay_10ms == 3) {
            // 10ms��ȡһ�������ǡ����ٶȼƺʹ����Ƶ�ֵ��������̬
            static u8 is_first_time = 1;
            static u32 prev_ts;
            static float prev_ax, prev_ay, prev_az, prev_gx, prev_gy, prev_gz;
            float cur_ax, cur_ay, cur_az, cur_gx, cur_gy, cur_gz;
            float cur_mean_ax, cur_mean_ay, cur_mean_az, cur_mean_gx, cur_mean_gy, cur_mean_gz;
            float cur_var_ax, cur_var_ay, cur_var_az, cur_var_gx, cur_var_gy, cur_var_gz;
            u8 is_invalid = 0;
            u32 cur_ts = millis();
#ifdef ENABLE_MAGNETIC_FILED
            static float prev_mx, prev_my, prev_mz;
            float cur_mx, cur_my, cur_mz;
            float cur_mean_mx, cur_mean_my, cur_mean_mz;
            float cur_var_mx, cur_var_my, cur_var_mz;
            is_invalid |= mpu_9250_read_mag_and_statistic(
                            &cur_mx, &cur_my, &cur_mz,
                            &cur_mean_mx, &cur_mean_my, &cur_mean_mz,
                            &cur_var_mx, &cur_var_my, &cur_var_mz,
                        );
#endif
            is_invalid |= mpu_9250_read_accel_gyro_and_statistic(
                            &cur_ax, &cur_ay, &cur_az,
                            &cur_gx, &cur_gy, &cur_gz,
                            &cur_mean_ax, &cur_mean_ay, &cur_mean_az,
                            &cur_var_ax, &cur_var_ay, &cur_var_az,
                            &cur_mean_gx, &cur_mean_gy, &cur_mean_gz,
                            &cur_var_gx, &cur_var_gy, &cur_var_gz
                        );
            if (is_first_time) {
                is_first_time = 0;
            } else {
                float dt = (float)(cur_ts - prev_ts) / 1000.0;
                if (!is_invalid) {
                    accel_fused_std = sqrt(cur_var_ax + cur_var_ay + cur_var_az);
                    if (is_calibrated && accel_fused_std > 0.030) {
                        // Pass gyro rate as rad/s
                        // https://github.com/kriswiner/MPU9250/issues/220#
                        // https://github.com/kriswiner/MPU9250/issues/51
                        //MadgwickAHRSUpdate(prev_gy, prev_gx, -prev_gz, prev_ay, prev_ax, -prev_az, prev_mx, prev_my, prev_mz, dt);
                        //MahonyAHRSUpdate(prev_gy, prev_gx, -prev_gz, prev_ay, prev_ax, -prev_az, prev_mx, prev_my, prev_mz, dt);
                        //MadgwickAHRSUpdateIMU(prev_gy, prev_gx, -prev_gz, prev_ay, prev_ax, -prev_az, dt);
                        //MahonyAHRSUpdateIMU(prev_gy, prev_gx, -prev_gz, prev_ay, prev_ax, -prev_az, dt);
                        //MahonyAHRSUpdateIMU(prev_gx, prev_gy, prev_gz, prev_ax, prev_ay, prev_az, dt);
                        //MahonyAHRSUpdateIMU(prev_gx, prev_gy, prev_gz, prev_ax, prev_ay, prev_az, dt);
                        MahonyAHRSUpdateIMU(prev_gx, prev_gy, prev_gz, prev_ax, prev_ay, prev_az, dt);
                        Quat2Angle();
                    } else if (!is_calibrated && accel_fused_std <= 0.030) {
                        mpu_9250_set_accel_gyro_bias(0, 0, 0, cur_mean_gx, cur_mean_gy, cur_mean_gz);
                        printf("Accel bias: %f, %f, %f Gyro bias: %f, %f, %f\n", 0.0f, 0.0f, 0.0f, cur_mean_gx, cur_mean_gy, cur_mean_gz);
                        is_calibrated = 1;
                    }
                }
                //printf("%d %f %f %f %f\n", ret_code, dt, roll, pitch, yaw);
                //printf("%f %f %f %f %f %f %f %f %f %f\n", dt, prev_ax, prev_ay, prev_az, prev_gx, prev_gy, prev_gz, prev_mx, prev_my, prev_mz);
                printf("%f %f %f %f %f %f\n", prev_ax, prev_ay, prev_az,prev_gx, prev_gy, prev_gz);
            }
            prev_ts = cur_ts;
            prev_ax = cur_ax;
            prev_ay = cur_ay;
            prev_az = cur_az;
            prev_gx = cur_gx;
            prev_gy = cur_gy;
            prev_gz = cur_gz;
#ifdef ENABLE_MAGNETIC_FILED
            prev_mx = cur_mx,
            prev_my = cur_my;
            prev_mz = cur_mz;
#endif
            cost = (millis() - cur_ts) / 1000.0f;
            // 10ms����һ�ε�ص�ѹֵ
            battary_voltage = get_battery_volt() / 100.0f;
            // LED��˸: ����ģʽ1s��˸һ��, �͵�ѹ0.1s��˸һ��
            led_toggle(is_low_power() ? 10 : 100);
#if 0
            // 10ms���Ƶ��һ�Σ�Ϊ�˱�֤M�����ٵ�ʱ���׼�����ȶ�ȡ����������
            left_target_pwm = is_low_power() ? 0 : 3;
            right_target_pwm = is_low_power() ? 0 : -3;
            // ��ȡ���ҵ��������ֵ
            left_encoder = encoder_read(2);
            right_encoder = encoder_read(4);
            // PID����
            left_pid_pwm = left_PID_control(left_encoder, left_target_pwm); // �ٶȱջ����Ƽ�����1����PWM
            right_pid_pwm = right_PID_control(right_encoder, right_target_pwm); // �ٶȱջ����Ƽ�����2����PWM
            // �������ҵ��PWMֵ
            motor_set_pwm(&left_pid_pwm, &right_pid_pwm);
#endif
            delay_10ms = 0;
        }
        if (delay_10ms == 0) {
            //mpu_9250_prepare_mag_data();
        }
        delay_10ms++;
    }
    return 0;
}

int main(void) {
    u8 ret_code;
    char show_string[255];
    //SystemInit();
    NVIC_Configuration();
    led_init();
    key_init();
    usart_init(115200);
    delay_init();
    time_init();
    encoder_init();
    adc_init();
    oled_init();
    oled_clear();
    oled_show_string(0, 0, "calibrating...\nkeep still.");
    ret_code = mpu_9250_init();
    motor_init(7199, 0); // PWM=10KHZ
    exti_init();
    while(1) {
        delay_flag = 1;
        delay_50ms = 0;
        while(delay_flag); // ͨ��MPU6050��INT�ж�ʵ�ֵ�50ms��׼��ʱ
        sprintf(show_string, "%.1f %.3f %f      ", battary_voltage, cost, accel_fused_std);
        oled_show_string(0, 6, show_string);
        if (is_calibrated) {
          sprintf(show_string, "roll:%.3f      \npitch:%.3f           \nyaw:%.3f         ", roll, pitch, yaw);
          oled_show_string(0, 0, show_string);
        }
    }
}