#ifndef GKEARHS_h
#define GKEARHS_h

#include<math.h>
#include "quaternion.h"

float Sqr(float a);
void DoMadgwickAHRS(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

#endif
