#ifndef __BSX_COMPASSCONFIG_H__
#define __BSX_COMPASSCONFIG_H__
#include "BsxLibraryDataTypes.h"
typedef struct {
  BSX_F32 baseCoef[4];
  BSX_F32 dynCoef[4];
  BSX_F32 accNoise;
  BSX_F32 magNoise;
  BSX_U8 mode;
  BSX_U8 coeffcalmode;
} ts_filt;
typedef struct {
  ts_filt filt;
  BSX_U8 opMode;
} ts_compassconfig;
BSX_S8 compassconfig_setDefaultConfig(ts_compassconfig *compassConfig);
#endif