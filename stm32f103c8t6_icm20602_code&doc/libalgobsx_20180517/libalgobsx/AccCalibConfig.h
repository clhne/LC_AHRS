#ifndef __BSX_ACCCALIBCONFIG_H__
#define __BSX_ACCCALIBCONFIG_H__
#include "BsxLibraryDataTypes.h"
typedef struct {
  BSX_F32 radius;
  BSX_F32 calibCovariance[16];
  BSX_F32 calibProcessNoise[4];
  BSX_F32 calibMeasurementNoise;
  BSX_S16 dynBufferSize;
  BSX_S16 dynBufferSizeMax;
  BSX_S16 dynamicThres;
  BSX_S16 maxGThres;
  BSX_S16 minGThres;
  BSX_S16 orientThres;
  BSX_S8 orientCntThres;
  BSX_S16 accurRefThres;
  BSX_U8 opMode;
} ts_acccalibconfig;
BSX_S8 acccalibconfig_setDefaultConfig(ts_acccalibconfig *accCalibConfig);
#endif