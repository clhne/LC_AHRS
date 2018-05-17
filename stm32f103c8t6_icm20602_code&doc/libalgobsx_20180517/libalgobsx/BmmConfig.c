#include "BmmConfig.h"
BSX_S8 bmmconfig_setDefaultConfig(ts_magsensorspec *magSensorSpec, BSX_U8 SensorId) {
  if (SensorId == 1) {
    magSensorSpec->range.x = 1100;
    magSensorSpec->range.y = 1100;
    magSensorSpec->range.z = 1100;
    magSensorSpec->dataRate = 20;
    magSensorSpec->maxDatarate = 100;
    magSensorSpec->measCurrent = 6000;
    magSensorSpec->name = 1;
    magSensorSpec->opMode = 1;
    magSensorSpec->res = 0.0625f;
    magSensorSpec->rmsNoise = 0.8f;
    magSensorSpec->noise = 7.0f;
    magSensorSpec->noisefactorarray[0] = 1;
    magSensorSpec->noisefactorarray[1] = 1;
    magSensorSpec->noisefactorarray[2] = 1;
    magSensorSpec->noisefactorarray[3] = 1;
    magSensorSpec->noisefactorarray[4] = 1;
    magSensorSpec->noisefactorarray[5] = 1;
    magSensorSpec->noisefactorarray[6] = 1;
    magSensorSpec->noisefactorarray[7] = 1;
    magSensorSpec->noisefactorarray[8] = 1;
    magSensorSpec->version = 1;
    magSensorSpec->vendor = 1;
  }
  return 0;
}