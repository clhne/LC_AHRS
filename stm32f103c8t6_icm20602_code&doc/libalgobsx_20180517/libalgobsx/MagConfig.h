#ifndef __BSX_MAGCONFIG_H__
#define __BSX_MAGCONFIG_H__
#include "InternalDataTypes.h"
#include "MagCalibConfig.h"
typedef struct {
  BSX_U8 opMode;
  BSX_U8 range;
  BSX_U8 datarate;
  ts_preProc preProc;
  ts_magcalibconfig magCalibConfig;
} ts_magconfig;
BSX_S8 magconfig_setDefaultConfig(ts_magconfig *magConfig);
#endif