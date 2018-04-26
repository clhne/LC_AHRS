#ifndef __KEY_H
#define __KEY_H
#include "sys.h"
void key_init(void); // 按键初始化
u8 double_click(u8 time); // 单击按键扫描和双击按键扫描
u8 click(void); // 单击按键扫描
u8 long_press(void); // 长按扫描
#endif
