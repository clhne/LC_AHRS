#ifndef __OLED_H
#define __OLED_H
#include "sys.h"
#include "stdlib.h"

#define FONT_SIZE 1 // 设置字体大小 0, 6x8 pixel font; 1, 8x16 pixel font;
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

void oled_init(void);
void oled_display_on(void);
void oled_display_off(void);
void oled_clear(void);
void oled_show_string(u8 pos_x, u8 pos_y, char *show_string); // 显示字符串
#endif




