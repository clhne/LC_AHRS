#include <math.h>
#include "sys.h"
#include "time.h"
#include "delay.h"
#include "usart.h"
#include "spi.h"
#include "icm20602.h"
#include "oled.h"
#include "NDOF.h"
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
    NDOF_Init(NDOF_ACC_RANGE_2G, 16, NDOF_GYRORANGE_2000DPS, 16);

    //Door param
    int door_status;
    float cur_roll, pitch, cur_yaw, cor_gx, cor_gy, cor_gz;
    long long ts;
    Door_Detection_Init();
    while(1) {
        //Detection Door Status
        if (icm20602_get_acc_gyro_adc(&ax_adc, &ay_adc, &az_adc, &gx_adc, &gy_adc, &gz_adc)) {
            delay_ms(5);
        } else {
            ts = millis();
            Door_Detection(ax_adc, ay_adc, az_adc, gx_adc, gy_adc, gz_adc, &door_status, &pitch, &cor_gx, ts);
            sprintf(show_string, "%lld %d %d %d %d \n\n%.3f  ", ts, NDOF_IsGyroDynamic(), NDOF_IsGyroCalibrated(), NDOF_IsAccDynamic(), NDOF_IsAccCalibrated(), pitch);
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
        }
    }
}
