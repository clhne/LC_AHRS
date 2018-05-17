#ifndef __GYROCONFIG_H__
#define __GYROCONFIG_H__
#include "InternalDataTypes.h"
#include "GyroCalibConfig.h"
typedef struct {
  BSX_U8 opMode;
  BSX_U8 range;
  BSX_U8 datarate;
  ts_preProc preProc;
  ts_gyrocalibconfig gyroCalibConfig;
} ts_gyroconfig;
BSX_S8 gyroconfig_setDefaultConfig(ts_gyroconfig *gyroConfig);
#endif