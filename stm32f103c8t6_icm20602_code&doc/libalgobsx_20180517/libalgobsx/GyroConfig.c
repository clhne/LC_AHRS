#include "GyroConfig.h"
const BSX_F32 GYROCONFIG_DEFAULT_DIGITALFILTER1IIRCOEF[] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
BSX_S8 gyroconfig_setDefaultConfig(ts_gyroconfig *gyroConfig) {
  BSX_S32 i;
  gyroConfig->opMode = 1;
  gyroConfig->range = 1;
  gyroConfig->datarate = 7;
  gyroConfig->preProc.mode.general = 1;
  gyroConfig->preProc.mode.dataCorrection = 1;
  gyroConfig->preProc.mode.digitalFilter1 = 0;
  gyroConfig->preProc.mode.digitalFilter2 = 0;
  gyroConfig->preProc.axisConfig.axisConfig = 0;
  gyroConfig->preProc.axisConfig.axisSign = 0;
  gyroConfig->preProc.off.x = 0;
  gyroConfig->preProc.off.y = 0;
  gyroConfig->preProc.off.z = 0;
  gyroConfig->preProc.sensM.x.y = 0.0;
  gyroConfig->preProc.sensM.x.x = 1.0;
  gyroConfig->preProc.sensM.y.y = 1.0;
  gyroConfig->preProc.sensM.z.z = 1.0;
  gyroConfig->preProc.sensM.y.x = 0.0;
  gyroConfig->preProc.sensM.y.z = 0.0;
  gyroConfig->preProc.sensM.z.y = 0.0;
  gyroConfig->preProc.sensM.z.x = 0.0;
  gyroConfig->preProc.sensM.x.z = 0.0;
  for (i = 0; i < 10; i++) {
    gyroConfig->preProc.digFilt1ConstiirCoeff[i] = GYROCONFIG_DEFAULT_DIGITALFILTER1IIRCOEF[i];
  }
  gyroConfig->preProc.digFilt2Param = 0.1f;
  gyrocalibconfig_setDefaultParam(&gyroConfig->gyroCalibConfig);
  gyroConfig->gyroCalibConfig.opMode = 2;
  return 0;
}