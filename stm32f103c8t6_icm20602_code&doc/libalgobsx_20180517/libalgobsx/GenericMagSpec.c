#include <stdio.h>
#include "BsxLibraryConfiguration.h"
#include "GenericMagSpec.h"

const BSX_U8 GENERICMAGSPEC_DATARATEARRAY[] = {0x01, 0x05, 0x0A, 0x14, 0x19, 0x28, 0x32, 0x64, 0x7D, 0xC8};

BSX_U8 genericmagspec_setSensorSpec(ts_magsensorspec *magSensorSpec, BSX_U8* magdata) {
  int i;
  bsx_copyBytefromMemory(&magSensorSpec->name, magdata + 2, 1);
  bsx_copyBytefromMemory(&magSensorSpec->opMode, magdata + 3, 1);
  bsx_copyBytefromMemory((BSX_U8 *)&magSensorSpec->range.x, magdata + 4, 2);
  bsx_copyBytefromMemory((BSX_U8 *)&magSensorSpec->range.y, magdata + 6, 2);
  bsx_copyBytefromMemory((BSX_U8 *)&magSensorSpec->range.z, magdata + 8, 2);
  bsx_copyBytefromMemory(&magSensorSpec->dataRate, magdata + 10, 1);
  bsx_copyBytefromMemory(&magSensorSpec->maxDatarate, magdata + 11, 1);
  bsx_copyBytefromMemory((BSX_U8 *)&magSensorSpec->measCurrent, magdata + 12, 2);
  bsx_copyBytefromMemory((BSX_U8 *)&magSensorSpec->res, magdata + 14, 4);
  bsx_copyBytefromMemory((BSX_U8 *)&magSensorSpec->rmsNoise, magdata + 18, 4);
  bsx_copyBytefromMemory((BSX_U8 *)&magSensorSpec->noise, magdata + 22, 4);
  bsx_copyBytefromMemory(magSensorSpec->noisefactorarray, magdata + 26, 9);
  bsx_copyBytefromMemory(&magSensorSpec->version, magdata + 35, 1);
  bsx_copyBytefromMemory(&magSensorSpec->vendor, magdata + 36, 1);
  printf("MAG spec\n");
  printf("name: %d\n", magSensorSpec->name);
  printf("opMode: %d\n", magSensorSpec->opMode);
  printf("range: x=%d y=%d z=%d\n", magSensorSpec->range.x, magSensorSpec->range.y, magSensorSpec->range.z);
  printf("dataRate: %d", magSensorSpec->dataRate);
  switch (magSensorSpec->dataRate) {
  case BSX_DATARATE_1HZ:
    printf(" 1Hz\n");
    break;
  case BSX_DATARATE_5HZ:
    printf(" 5Hz\n");
    break;
  case BSX_DATARATE_10HZ:
    printf(" 10Hz\n");
    break;
  case BSX_DATARATE_20HZ:
    printf(" 20Hz\n");
    break;
  case BSX_DATARATE_25HZ:
    printf(" 25Hz\n");
    break;
  case BSX_DATARATE_40HZ:
    printf(" 40Hz\n");
    break;
  case BSX_DATARATE_50HZ:
    printf(" 50Hz\n");
    break;
  case BSX_DATARATE_100HZ:
    printf(" 100Hz\n");
    break;
  case BSX_DATARATE_125HZ:
    printf(" 125Hz\n");
    break;
  case BSX_DATARATE_200HZ:
    printf(" 200Hz\n");
    break;
  case BSX_DATARATE_400HZ:
    printf(" 400Hz\n");
    break;
  default:
    printf(" unknown\n");
    break;
  }
  printf("maxDatarate: %d\n", magSensorSpec->maxDatarate);
  printf("measCurrent: %d\n", magSensorSpec->measCurrent);
  printf("res: %f\n", magSensorSpec->res);
  printf("rmsNoise: %f\n", magSensorSpec->rmsNoise);
  printf("noise: %f\n", magSensorSpec->noise);
  printf("noisefactorarray:");
  for (i = 0; i < 9; i++) {
    printf(" %d", magSensorSpec->noisefactorarray[i]);
  }
  printf("\n");
  printf("version: %d\n", magSensorSpec->version);
  printf("vendor: %d\n", magSensorSpec->vendor);
  return 0;
}

BSX_S8 genericmagspec_setRange(ts_magsensorspec *magSensorSpec, ts_dataxyz range) {
  magSensorSpec->range.x = range.x;
  magSensorSpec->range.y = range.y;
  magSensorSpec->range.z = range.z;
  return 0;
}

BSX_S8 genericmagspec_setOpMode(ts_magsensorspec *magSensorSpec, BSX_U8 opmode) {
  magSensorSpec->opMode = opmode;
  return 0;
}

BSX_S8 genericmagspec_setDatarate(ts_magsensorspec *magSensorSpec, BSX_U8 datarate) {
  magSensorSpec->dataRate = datarate;
  return 0;
}

BSX_S8 genericmagspec_getVersion(ts_magsensorspec *magSensorSpec, BSX_U8 *version) {
  *version = magSensorSpec->version;
  return 0;
}

BSX_S8 genericmagspec_getVendor(ts_magsensorspec *magSensorSpec, BSX_U8 *vendor) {
  *vendor = magSensorSpec->vendor;
  return 0;
}

BSX_S8 genericmagspec_getType(BSX_U8 *type) {
  *type = 2;
  return 0;
}

BSX_S8 genericmagspec_getSensorRmsNoise(ts_magsensorspec *magSensorSpec, BSX_F32 *rmsNoise) {
  *rmsNoise = magSensorSpec->rmsNoise;
  return 0;
}

BSX_S8 genericmagspec_getResolution(ts_magsensorspec *magSensorSpec, BSX_F32 *res) {
  *res = magSensorSpec->res;
  return 0;
}

BSX_S8 genericmagspec_getRange(ts_magsensorspec *magSensorSpec, ts_dataxyz *range) {
  range->x = magSensorSpec->range.x;
  range->y = magSensorSpec->range.y;
  range->z = magSensorSpec->range.z;
  return 0;
}

BSX_S8 genericmagspec_getPower(ts_magsensorspec *magSensorSpec, BSX_F32 *powerConsumption) {
  if (magSensorSpec->opMode == 1) {
    BSX_U8 data_rate = GENERICMAGSPEC_DATARATEARRAY[magSensorSpec->dataRate];
    BSX_U8 max_data_rate = GENERICMAGSPEC_DATARATEARRAY[magSensorSpec->maxDatarate];
    *powerConsumption = (float)magSensorSpec->measCurrent * (float)data_rate / ((float)data_rate + (float)max_data_rate);
  } else {
    *powerConsumption = 1.0f;
  }
  return 0;
}

BSX_S8 genericmagspec_getOpMode(ts_magsensorspec *magSensorSpec, BSX_U8 *opMode) {
  *opMode = magSensorSpec->opMode;
  return 0;
}

BSX_S8 genericmagspec_getNoiseFactor(ts_magsensorspec *magSensorSpec, BSX_U8 factor) {
  return magSensorSpec->noisefactorarray[factor];
}

BSX_S8 genericmagspec_getNoise(ts_magsensorspec *magSensorSpec, BSX_F32 *noise) {
  *noise = magSensorSpec->noise;
  return 0;
}

BSX_S8 genericmagspec_getName(ts_magsensorspec *magSensorSpec, BSX_S8 *name) {
  *name = magSensorSpec->name;
  return 0;
}

BSX_S8 genericmagspec_getMinDelay(ts_magsensorspec *magSensorSpec, BSX_F32 *minDelay) {
  *minDelay = 1.0f / (BSX_F32)GENERICMAGSPEC_DATARATEARRAY[magSensorSpec->dataRate];
  return 0;
}

BSX_S8 genericmagspec_getMaximumRange(ts_magsensorspec *magSensorSpec, ts_dataxyz *maxRange) {
  maxRange->x = magSensorSpec->range.x;
  maxRange->y = magSensorSpec->range.y;
  maxRange->z = magSensorSpec->range.z;
  return 0;
}

BSX_S8 genericmagspec_getDatarate(ts_magsensorspec *magSensorSpec, BSX_U8 *datarate) {
  *datarate = magSensorSpec->dataRate;
  return 0;
}
