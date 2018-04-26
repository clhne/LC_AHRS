#include "key.h"
#define KEY PAin(15)
/**************************************************************************
函数功能：按键初始化
入口参数：无
返回  值：无
**************************************************************************/
void key_init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // 使能PA端口时钟
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; // 端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 上拉输入
    GPIO_Init(GPIOA, &GPIO_InitStructure); // 根据设定参数初始化GPIOA
}

/**************************************************************************
函数功能：按键扫描
入口参数：双击等待时间
返回  值：按键状态 0：无动作 1：单击 2：双击
**************************************************************************/
u8 double_click(u8 time) {
    static u8 flag_key, count_key, double_key;
    static u16 count_single, forever_count;
    if (KEY == 0) { //长按标志位未置1
        forever_count++;
    } else {
        forever_count = 0;
    }
    if (KEY == 0 && flag_key == 0) {
        flag_key = 1;
    }
    if (count_key == 0) {
        if (flag_key == 1) {
            double_key++;
            count_key = 1;
        }
        if (double_key == 2) {
            double_key = 0;
            count_single = 0;
            return 2; // 双击执行的指令
        }
    }
    if (KEY == 1)	{
        flag_key = 0;
        count_key = 0;
    }
    if (double_key == 1) {
        count_single++;
        if (count_single > time && forever_count < time) {
            double_key = 0;
            count_single = 0;
            return 1; // 单击执行的指令
        }
        if(forever_count > time) {
            double_key = 0;
            count_single = 0;
        }
    }
    return 0;
}

/**************************************************************************
函数功能：按键扫描
入口参数：无
返回  值：按键状态 0：无动作 1：单击
**************************************************************************/
u8 click(void) {
    static u8 flag_key = 1; // 按键按松开标志
    if (flag_key && KEY == 0) {
        flag_key = 0;
        return 1;	// 按键按下
    } else if (KEY == 1) {
        flag_key = 1;
    }
    return 0; // 无按键按下
}

/**************************************************************************
函数功能：长按检测
入口参数：无
返回  值：按键状态 0：无动作 1：长按2s
**************************************************************************/
u8 long_press(void) {
    static u16 long_press_count, long_press_flag;
    if (long_press_flag == 0 && KEY == 0) { // 长按标志位未置1
        long_press_count++;
    } else {
        long_press_count = 0;
    }
    if (long_press_count > 200) {
        long_press_flag = 1;
        long_press_count = 0;
        return 1;
    }
    if (long_press_flag == 1) { // 长按标志位置1
        long_press_flag = 0;
    }
    return 0;
}
