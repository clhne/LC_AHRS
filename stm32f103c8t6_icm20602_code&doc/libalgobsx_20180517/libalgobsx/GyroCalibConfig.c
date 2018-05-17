#include "GyroCalibConfig.h"
BSX_S8 gyrocalibconfig_setDefaultParam(ts_gyrocalibconfig *gyroCalibConfig) {
  gyroCalibConfig->S_GyroObserv.DynamicDetectionThreshold = 19;
  gyroCalibConfig->S_GyroObserv.NoDynamicDetectionTime = 50;
  gyroCalibConfig->S_MagObserv.DynamicDetectionThreshold = 40;
  gyroCalibConfig->S_MagObserv.NoDynamicDetectionTime = 4;
  gyroCalibConfig->S_GyroOffsetThreshold.Step = 4;
  gyroCalibConfig->S_GyroOffsetThreshold.OffsetThreshold = -93;
  gyroCalibConfig->S_GyroObserv.S_ShakeDetection.DetectionThreshold = 3276;
  gyroCalibConfig->S_GyroObserv.S_ShakeDetection.DetectionTime = 100;
  return 0;
}
