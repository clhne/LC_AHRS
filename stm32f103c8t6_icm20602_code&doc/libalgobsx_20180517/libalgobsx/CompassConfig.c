#include "CompassConfig.h"
BSX_S8 compassconfig_setDefaultConfig(ts_compassconfig *compassConfig) {
  compassConfig->filt.dynCoef[1] = 0.25f;
  compassConfig->filt.baseCoef[0] = 0.3f;
  compassConfig->filt.dynCoef[0] = 0.3f;
  compassConfig->filt.baseCoef[1] = 0.3f;
  compassConfig->filt.dynCoef[2] = 0.3f;
  compassConfig->opMode = 1;
  compassConfig->filt.baseCoef[2] = 0.4f;
  compassConfig->filt.baseCoef[3] = 0.6f;
  compassConfig->filt.coeffcalmode = 1;
  compassConfig->filt.dynCoef[3] = 0.36f;
  compassConfig->filt.mode = 2;
  compassConfig->filt.accNoise = 12.0f;
  compassConfig->filt.magNoise = 15.0f;
  return 0;
}