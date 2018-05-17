#include "BmaConfig.h"
BSX_S8 bmaconfig_setDefaultConfig(ts_accsensorspec *accSensorSpec, BSX_U8 SensorId) {
  if (SensorId == 1) {
    accSensorSpec->range = 0;
    accSensorSpec->maxDatarate = 9;
    accSensorSpec->resBits = 10;
    accSensorSpec->gVecErrMax = 1300;
    accSensorSpec->dataRate = 7;
    accSensorSpec->name = 1;
    accSensorSpec->noise = 12.0f;
    accSensorSpec->measCurrent = 150;
    accSensorSpec->opMode = 1;
    accSensorSpec->offErr = 180;
    accSensorSpec->sensErr = 5;
    accSensorSpec->gVecErrMin = 800;
    accSensorSpec->noise1s = 1;
    accSensorSpec->noisefactorarray[3] = 2;
    accSensorSpec->noisefactorarray[4] = 2;
    accSensorSpec->noisefactorarray[5] = 2;
    accSensorSpec->noisefactorarray[0] = 1;
    accSensorSpec->noisefactorarray[1] = 1;
    accSensorSpec->noisefactorarray[2] = 1;
    accSensorSpec->noisefactorarray[6] = 3;
    accSensorSpec->noisefactorarray[7] = 4;
    accSensorSpec->noisefactorarray[8] = 4;
    accSensorSpec->version = 1;
    accSensorSpec->vendor = 1;
  }
  return 0;
}