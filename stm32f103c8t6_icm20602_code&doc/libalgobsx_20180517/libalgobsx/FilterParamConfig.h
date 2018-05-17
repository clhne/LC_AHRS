#ifndef __BSX_FILTERPARAMCONFIG_H__
#define __BSX_FILTERPARAMCONFIG_H__
#include "BsxLibraryDataTypes.h"
typedef struct {
  BSX_F32 magnoise;
  BSX_F32 headsens_magnoise[6];
  BSX_F32 headsens_magbasecoeff[6];
  BSX_F32 headsens_magdynproc[6];
  BSX_F32 ndoforientcorspeed_magnoise[6];
  BSX_F32 magfiltcoeff;
  BSX_F32 magfiltnoiselvl;
  BSX_F32 magfilt_magcoeff[6];
  BSX_F32 magfilt_magnoiselvl[6];
  BSX_F32 magcalibaccstly_magcalibaccthrsldinit;
  BSX_F32 magcalibaccstly_magcalibaccthrsldinitary[6];
  BSX_F32 magcalibaccstly_magcalibaccthrsld;
  BSX_F32 magcalibaccstly_magcalibaccthrsldary[6];
} ts_filterparamconfig;
BSX_S8 filterparamconfig_setDefaultConfig(ts_filterparamconfig *filterparamConfig);
#endif