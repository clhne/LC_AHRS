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

    while(1) {
        //Detection Door Status
        door_detection(&door_status, &cur_roll, &pitch, &cur_yaw, &cor_gx, &cor_gy, &cor_gz);
    }
}
