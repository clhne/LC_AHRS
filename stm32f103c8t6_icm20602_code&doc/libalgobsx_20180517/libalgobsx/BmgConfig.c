#include "BmgConfig.h"
BSX_S8 bmgconfig_setDefaultConfig(ts_gyrosensorspec *gyroSensorSpec, BSX_U8 SensorId) {
  if (SensorId == 1) {
    gyroSensorSpec->dataRate = 7;
    gyroSensorSpec->name = 1;
    gyroSensorSpec->opMode = 1;
    gyroSensorSpec->range = 1;
    gyroSensorSpec->maxDatarate = 9;
    gyroSensorSpec->measCurrent = 5000;
    gyroSensorSpec->resBits = 16;
    gyroSensorSpec->version = 1;
    gyroSensorSpec->vendor = 1;
  }
  return 0;
}