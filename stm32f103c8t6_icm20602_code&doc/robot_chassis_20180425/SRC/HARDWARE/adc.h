#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"
void adc_init(void);
u16 get_adc(u8 ch);
int get_battery_volt(void); 
#endif
