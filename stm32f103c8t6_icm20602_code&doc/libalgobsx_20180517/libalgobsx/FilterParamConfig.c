#include "FilterParamConfig.h"
const BSX_F32 FILTERPARAMCONFIG_DEFAULT_HEADINGSENS_MAGNOISE[] = {2.0f, 0.0f, -1.0f, -2.0f, -3.0f, -7.0f};
const BSX_F32 FILTERPARAMCONFIG_DEFAULT_HEADINGSENS_MAGBASECOEFF[] = {-0.1f, 0.0f, 0.05f, 0.1f, 0.2f, 0.3f};
const BSX_F32 FILTERPARAMCONFIG_DEFAULT_HEADINGSENS_MAGDYNPROC[] = {0.1f, 0.0f, -0.1f, -0.15f, -0.2f, -0.2f};
const BSX_F32 FILTERPARAMCONFIG_DEFAULT_NDOFORIENTCORSPEED_MAGNOISE[] = {-0.1f, 0.0f, 0.1f, 0.2f, 0.3f, 0.4f};
const BSX_F32 FILTERPARAMCONFIG_DEFAULT_MAGFILT_MAGCOEFF[] = {-10.0f, 0.0f, 10.0f, 20.0f, 30.0f, 40.0f};
const BSX_F32 FILTERPARAMCONFIG_DEFAULT_MAGFILT_MAGNOISELVL[] = {-5.0f, 0.0f, 1.0f, 2.0f, 3.0f, 4.0f};
const BSX_F32 FILTERPARAMCONFIG_DEFAULT_MAGCALIBACCSTLY_MAGNCALIBACCTHRSLDINITARY[] = {-0.2f, 0.0f, 0.1f, 0.2f, 0.3f, 0.4f};
const BSX_F32 FILTERPARAMCONFIG_DEFAULT_MAGCALIBACCSTLY_MAGNCALIBACCTHRSLDARY[] = {-1.0f, 0.0f, 1.0f, 2.0f, 3.0f, 5.0f};
BSX_S8 filterparamconfig_setDefaultConfig(ts_filterparamconfig *filterparamConfig) {
  BSX_S32 i;
  filterparamConfig->magnoise = 0.8f;
  for (i = 0; i < 6; i++) {
    filterparamConfig->headsens_magnoise[i] = FILTERPARAMCONFIG_DEFAULT_HEADINGSENS_MAGNOISE[i];
    filterparamConfig->headsens_magbasecoeff[i] = FILTERPARAMCONFIG_DEFAULT_HEADINGSENS_MAGBASECOEFF[i];
    filterparamConfig->headsens_magdynproc[i] = FILTERPARAMCONFIG_DEFAULT_HEADINGSENS_MAGDYNPROC[i];
    filterparamConfig->ndoforientcorspeed_magnoise[i] = FILTERPARAMCONFIG_DEFAULT_NDOFORIENTCORSPEED_MAGNOISE[i];
    filterparamConfig->magfilt_magcoeff[i] = FILTERPARAMCONFIG_DEFAULT_MAGFILT_MAGCOEFF[i];
    filterparamConfig->magfilt_magnoiselvl[i] = FILTERPARAMCONFIG_DEFAULT_MAGFILT_MAGNOISELVL[i];
    filterparamConfig->magcalibaccstly_magcalibaccthrsldinitary[i] = FILTERPARAMCONFIG_DEFAULT_MAGCALIBACCSTLY_MAGNCALIBACCTHRSLDINITARY[i];
    filterparamConfig->magcalibaccstly_magcalibaccthrsldary[i] = FILTERPARAMCONFIG_DEFAULT_MAGCALIBACCSTLY_MAGNCALIBACCTHRSLDARY[i];

  }
  filterparamConfig->magcalibaccstly_magcalibaccthrsldinit = 0.6f;
  filterparamConfig->magfiltcoeff = 20.0f;
  filterparamConfig->magfiltnoiselvl = 7.0f;
  filterparamConfig->magcalibaccstly_magcalibaccthrsld = 0.6f;
  return 0;
}
