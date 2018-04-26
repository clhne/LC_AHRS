#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>
void motor_init(u16 arr, u16 psc);
void motor_set_pwm(int *left_pwm_ptr, int *right_pwm_ptr);
#endif
