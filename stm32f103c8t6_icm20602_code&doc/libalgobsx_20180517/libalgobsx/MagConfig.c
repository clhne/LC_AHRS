#include "MagConfig.h"
const BSX_F32 MAGCONFIG_DEFAULT_DIGITALFILTER1IIRCOEF[] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.35f, 0.55f, 0.0f, 0.7f};
BSX_S8 magconfig_setDefaultConfig(ts_magconfig *magConfig) {
  BSX_S32 i;
  magConfig->opMode = 1;
  magConfig->datarate = 2;
  magConfig->preProc.mode.general = 1;
  magConfig->preProc.mode.dataCorrection = 1;
  magConfig->preProc.mode.digitalFilter1 = 1;
  magConfig->preProc.mode.digitalFilter2 = 0;
  magConfig->preProc.axisConfig.axisConfig = 0;
  magConfig->preProc.axisConfig.axisSign = 0;
  magConfig->preProc.off.x = 0;
  magConfig->preProc.off.y = 0;
  magConfig->preProc.off.z = 0;
  magConfig->preProc.sensM.x.y = 0.0f;
  magConfig->preProc.sensM.x.x = 1.0f;
  magConfig->preProc.sensM.y.y = 1.0f;
  magConfig->preProc.sensM.z.z = 1.0f;
  magConfig->preProc.sensM.y.x = 0.0f;
  magConfig->preProc.sensM.y.z = 0.0f;
  magConfig->preProc.sensM.z.y = 0.0f;
  magConfig->preProc.sensM.z.x = 0.0f;
  magConfig->preProc.sensM.x.z = 0.0f;
  magConfig->preProc.digFilt2Param = 0.2f;
  for (i = 0; i < 10; i++) {
    magConfig->preProc.digFilt1ConstiirCoeff[i] = MAGCONFIG_DEFAULT_DIGITALFILTER1IIRCOEF[i];
  }
  magcalibconfig_setDefaultConfig(&magConfig->magCalibConfig);
  magConfig->magCalibConfig.opMode = 2;
  magConfig->magCalibConfig.source = 2;
  return 0;
}