#include <math.h>
#include <stdio.h>
#include "BsxLibraryConstants.h"
#include "BsxLibraryConfiguration.h"
#include "GenericAccSpec.h"

const BSX_U8 GENERICACCSPEC_RANGEARRAY[] = {0x02, 0x04, 0x8, 0x10};
const BSX_U8 GENERICACCSPEC_DATARATEARRAY[] = {0x01, 0x05, 0x0A, 0x14, 0x19, 0x28, 0x32, 0x64, 0x7D, 0xC8};

struct ts_GenericAccSpecInternalStates {
  BSX_F32 rmsNoise;
  BSX_F32 res;
};

struct ts_GenericAccSpecInternalStates s_GenericAccSpecInternalStates;
BSX_S8 genericaccspec_updateStates(ts_accsensorspec *accSensorSpec) {
  s_GenericAccSpecInternalStates.res = (BSX_F32)GENERICACCSPEC_RANGEARRAY[accSensorSpec->range] / (BSX_F32)(1 << (accSensorSpec->resBits - 1)) * 1000.0f;
  s_GenericAccSpecInternalStates.rmsNoise = sqrtf(1.0f / (BSX_F32)GENERICACCSPEC_DATARATEARRAY[accSensorSpec->dataRate]) * (BSX_F32)accSensorSpec->noise1s * 1.57f;
  return 0;
}

BSX_S8 genericaccspec_setSensorSpec(ts_accsensorspec *accSensorSpec, BSX_U8* accdata) {
  int i;
  bsx_copyBytefromMemory(&accSensorSpec->name, accdata + 2, 1);
  bsx_copyBytefromMemory(&accSensorSpec->opMode, accdata + 3, 1);
  bsx_copyBytefromMemory(&accSensorSpec->range, accdata + 4, 1);
  bsx_copyBytefromMemory(&accSensorSpec->dataRate, accdata + 5, 1);
  bsx_copyBytefromMemory(&accSensorSpec->maxDatarate, accdata + 6, 1);
  bsx_copyBytefromMemory((BSX_U8 *)&accSensorSpec->measCurrent, accdata + 7, 2);
  bsx_copyBytefromMemory((BSX_U8 *)&accSensorSpec->resBits, accdata + 9, 1);
  bsx_copyBytefromMemory((BSX_U8 *)&accSensorSpec->offErr, accdata + 10, 2);
  bsx_copyBytefromMemory((BSX_U8 *)&accSensorSpec->sensErr, accdata + 12, 2);
  bsx_copyBytefromMemory((BSX_U8 *)&accSensorSpec->noise1s, accdata + 14, 2);
  bsx_copyBytefromMemory((BSX_U8 *)&accSensorSpec->gVecErrMax, accdata + 16, 2);
  bsx_copyBytefromMemory((BSX_U8 *)&accSensorSpec->gVecErrMin, accdata + 18, 2);
  bsx_copyBytefromMemory((BSX_U8 *)&accSensorSpec->noise, accdata + 20, 4);
  bsx_copyBytefromMemory(accSensorSpec->noisefactorarray, accdata + 24, 9);
  bsx_copyBytefromMemory(&accSensorSpec->version, accdata + 33, 1);
  bsx_copyBytefromMemory(&accSensorSpec->vendor, accdata + 34, 1);
  genericaccspec_updateStates(accSensorSpec);
  printf("ACC spec\n");
  printf("name: %d\n", accSensorSpec->name);
  printf("opMode: %d\n", accSensorSpec->opMode);
  printf("range: %d", accSensorSpec->range);
  switch (accSensorSpec->range) {
  case BSX_ACCRANGE_2G:
    printf(" 2G\n");
    break;
  case BSX_ACCRANGE_4G:
    printf(" 4G\n");
    break;
  case BSX_ACCRANGE_8G:
    printf(" 8G\n");
    break;
  case BSX_ACCRANGE_16G:
    printf(" 16G\n");
    break;
  default:
    printf(" unknown\n");
    break;
  }
  printf("dataRate: %d", accSensorSpec->dataRate);
  switch (accSensorSpec->dataRate) {
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
  printf("maxDatarate: %d\n", accSensorSpec->maxDatarate);
  printf("measCurrent: %d\n", accSensorSpec->measCurrent);
  printf("resBits: %d\n", accSensorSpec->resBits);
  printf("offErr: %d\n", accSensorSpec->offErr);
  printf("sensErr: %d\n", accSensorSpec->sensErr);
  printf("noise1s: %d\n", accSensorSpec->noise1s);
  printf("gVecErrMax: %d\n", accSensorSpec->gVecErrMax);
  printf("gVecErrMin: %d\n", accSensorSpec->gVecErrMin);
  printf("noise: %f\n", accSensorSpec->noise);
  printf("noisefactorarray:");
  for (i = 0; i < 9; i++) {
    printf(" %d", accSensorSpec->noisefactorarray[i]);
  }
  printf("\n");
  printf("version: %d\n", accSensorSpec->version);
  printf("vendor: %d\n", accSensorSpec->vendor);
  return 0;
}

BSX_S8 genericaccspec_setRange(ts_accsensorspec *accSensorSpec, BSX_U8 range) {
  accSensorSpec->range = range;
  genericaccspec_updateStates(accSensorSpec);
  return 0;
}

BSX_S8 genericaccspec_setOpMode(ts_accsensorspec *accSensorSpec, BSX_S8 opmode) {
  accSensorSpec->opMode = opmode;
  return 0;
}

BSX_S8 genericaccspec_setDatarate(ts_accsensorspec *accSensorSpec, BSX_U8 datarate) {
  accSensorSpec->dataRate = datarate;
  genericaccspec_updateStates(accSensorSpec);
  return 0;
}

BSX_S8 genericaccspec_getVersion(ts_accsensorspec *accSensorSpec, BSX_U8 *version) {
  *version = accSensorSpec->version;
  return 0;
}

BSX_S8 genericaccspec_getVendor(ts_accsensorspec *accSensorSpec, BSX_U8 *vendor) {
  *vendor = accSensorSpec->vendor;
  return 0;
}

BSX_S8 genericaccspec_getType(BSX_U8 *type) {
  *type = 1;
  return 0;
}

BSX_S8 genericaccspec_getSensorRmsNoise(BSX_F32 *rmsNoise) {
  *rmsNoise = s_GenericAccSpecInternalStates.rmsNoise;
  return 0;
}

BSX_S8 genericaccspec_getSensitivityErr(ts_accsensorspec *accSensorSpec, BSX_S16 *maxSensErr) {
  *maxSensErr = accSensorSpec->sensErr;
  return 0;
}

BSX_S8 genericaccspec_getResolution(BSX_F32 *res) {
  *res = s_GenericAccSpecInternalStates.res;
  return 0;
}

BSX_S8 genericaccspec_getRange(ts_accsensorspec *accSensorSpec, BSX_U8 *range) {
  *range = accSensorSpec->range;
  return 0;
}

BSX_S8 genericaccspec_getPower(ts_accsensorspec *accSensorSpec, BSX_S16 *powerConsumption) {
  if (accSensorSpec->opMode == 1) {
    *powerConsumption = accSensorSpec->measCurrent;
  } else {
    *powerConsumption = 1;
  }
  return 0;
}

BSX_S8 genericaccspec_getOpMode(ts_accsensorspec *accSensorSpec, BSX_U8 *opmode) {
  *opmode = accSensorSpec->opMode;
  return 0;
}

BSX_S8 genericaccspec_getOffsetErr(ts_accsensorspec *accSensorSpec, BSX_S16 *maxOffErr) {
  *maxOffErr = accSensorSpec->offErr;
  return 0;
}

BSX_U8 genericaccspec_getNoiseFactor(ts_accsensorspec *accSensorSpec, BSX_U8 factor) {
  return accSensorSpec->noisefactorarray[factor];
}

BSX_S8 genericaccspec_getNoise(ts_accsensorspec *accSensorSpec, BSX_F32 *noise) {
  *noise = accSensorSpec->noise;
  return 0;
}

BSX_S8 genericaccspec_getName(ts_accsensorspec *accSensorSpec, BSX_S8 *name) {
  *name = accSensorSpec->name;
  return 0;
}

BSX_S8 genericaccspec_getMinDelay(ts_accsensorspec *accSensorSpec, BSX_F32 *minDelay) {
  *minDelay = 1.0f / (BSX_F32)GENERICACCSPEC_DATARATEARRAY[accSensorSpec->maxDatarate];
  return 0;
}

BSX_S8 genericaccspec_getMaximumRange(ts_accsensorspec *accSensorSpec, BSX_S16 *maxRange) {
  *maxRange = GENERICACCSPEC_RANGEARRAY[accSensorSpec->range];
  return 0;
}

BSX_S8 genericaccspec_getGVectorErrors(ts_accsensorspec *accSensorSpec, BSX_S16 *gVecErrMax, BSX_S16 *gVecErrMin) {
  *gVecErrMax = accSensorSpec->gVecErrMax;
  *gVecErrMin = accSensorSpec->gVecErrMin;
  return 0;
}

BSX_S8 genericaccspec_getDatarate(ts_accsensorspec *accSensorSpec, BSX_U8 *datarate) {
  *datarate = accSensorSpec->dataRate;
  return 0;
}
