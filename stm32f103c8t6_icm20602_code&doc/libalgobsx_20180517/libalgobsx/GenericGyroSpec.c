#include <stdio.h>
#include "BsxLibraryConstants.h"
#include "BsxLibraryConfiguration.h"
#include "GenericGyroSpec.h"

const BSX_S16 GENERICGYROSPEC_RANGEARRAY[] = {0x800, 0x7D0, 0x3E8, 0x1F4, 0xFA};
const BSX_U8 GENERICGYROSPEC_DATARATEARRAY[] = {0x01, 0x05, 0x0A, 0x14, 0x19, 0x28, 0x32, 0x64, 0x7D, 0xC8};

BSX_F32 genericgyro_res;
BSX_S8 genericgyrospec_updateStates(BSX_U8 range, BSX_S8 resBits) {
  genericgyro_res = (BSX_F32)GENERICGYROSPEC_RANGEARRAY[range] / (BSX_F32)((1 << (resBits - 1)) - 1);
  return 0;
}

BSX_S8 genericgyrospec_setSensorSpec(ts_gyrosensorspec *gyroSensorSpec, BSX_U8 *gyrodata) {
  bsx_copyBytefromMemory(&gyroSensorSpec->name, gyrodata + 2, 1);
  bsx_copyBytefromMemory(&gyroSensorSpec->opMode, gyrodata + 3, 1);
  bsx_copyBytefromMemory(&gyroSensorSpec->range, gyrodata + 4, 1);
  bsx_copyBytefromMemory(&gyroSensorSpec->dataRate, gyrodata + 5, 1);
  bsx_copyBytefromMemory(&gyroSensorSpec->maxDatarate, gyrodata + 6, 1);
  bsx_copyBytefromMemory((BSX_U8 *)&gyroSensorSpec->measCurrent, gyrodata + 7, 2);
  bsx_copyBytefromMemory(&gyroSensorSpec->resBits, gyrodata + 9, 1);
  bsx_copyBytefromMemory(&gyroSensorSpec->version, gyrodata + 10, 1);
  bsx_copyBytefromMemory((BSX_U8 *)&gyroSensorSpec->vendor, gyrodata + 11, 1);
  genericgyrospec_updateStates(gyroSensorSpec->range, gyroSensorSpec->resBits);
  printf("GYRO spec\n");
  printf("name: %d\n", gyroSensorSpec->name);
  printf("opMode: %d\n", gyroSensorSpec->opMode);
  printf("range: %d", gyroSensorSpec->range);
  switch (gyroSensorSpec->range) {
  case BSX_GYRORANGE_2048DPS:
    printf(" 2048dps\n");
    break;
  case BSX_GYRORANGE_2000DPS:
    printf(" 2000dps\n");
    break;
  case BSX_GYRORANGE_1000DPS:
    printf(" 1000dps\n");
    break;
  case BSX_GYRORANGE_500DPS:
    printf(" 500dps\n");
    break;
  case BSX_GYRORANGE_250DPS:
    printf(" 250dps\n");
    break;
  default:
    printf(" unknown\n");
    break;
  }
  printf("dataRate: %d", gyroSensorSpec->dataRate);
  switch (gyroSensorSpec->dataRate) {
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
  printf("maxDatarate: %d\n", gyroSensorSpec->maxDatarate);
  printf("measCurrent: %d\n", gyroSensorSpec->measCurrent);
  printf("resBits: %d\n", gyroSensorSpec->resBits);
  printf("version: %d\n", gyroSensorSpec->version);
  printf("vendor: %d\n", gyroSensorSpec->vendor);
  return 0;
}

BSX_S8 genericgyrospec_setRange(ts_gyrosensorspec *gyroSensorSpec, BSX_U8 range) {
  gyroSensorSpec->range = range;
  genericgyrospec_updateStates(range, gyroSensorSpec->resBits);
  return 0;
}

BSX_S8 genericgyrospec_setOpMode(ts_gyrosensorspec *gyroSensorSpec, BSX_U8 opmode) {
  gyroSensorSpec->opMode = opmode;
  return 0;
}

BSX_S8 genericgyrospec_setDatarate(ts_gyrosensorspec *gyroSensorSpec, BSX_U8 datarate) {
  gyroSensorSpec->dataRate = datarate;
  genericgyrospec_updateStates(gyroSensorSpec->range, gyroSensorSpec->resBits);
  return 0;
}

BSX_S8 genericgyrospec_getVersion(ts_gyrosensorspec *gyroSensorSpec, BSX_U8 *version) {
  *version = gyroSensorSpec->version;
  return 0;
}

BSX_S8 genericgyrospec_getVendor(ts_gyrosensorspec *gyroSensorSpec, BSX_U8 *vendor) {
  *vendor = gyroSensorSpec->vendor;
  return 0;
}

BSX_S8 genericgyrospec_getType(BSX_U8 *type) {
  *type = 4;
  return 0;
}

BSX_S8 genericgyrospec_getResolution(BSX_F32 *res) {
  *res = genericgyro_res;
  return 0;
}

BSX_S8 genericgyrospec_getRange(ts_gyrosensorspec *gyroSensorSpec, BSX_U8 *range) {
  *range = gyroSensorSpec->range;
  return 0;
}

BSX_S8 genericgyrospec_getPower(ts_gyrosensorspec *gyroSensorSpec, BSX_S16 *powerConsumption) {
  if (gyroSensorSpec->opMode == 1) {
    *powerConsumption = gyroSensorSpec->measCurrent;
  } else {
    *powerConsumption = 1;
  }
  return 0;
}

BSX_S8 genericgyrospec_getOpMode(ts_gyrosensorspec *gyroSensorSpec, BSX_U8 *opmode) {
  *opmode = gyroSensorSpec->opMode;;
  return 0;
}

BSX_S8 genericgyrospec_getName(ts_gyrosensorspec *gyroSensorSpec, BSX_S8 *name) {
  *name = gyroSensorSpec->name;
  return 0;
}

BSX_S8 genericgyrospec_getMinDelay(ts_gyrosensorspec *gyroSensorSpec, BSX_F32 *minDelay) {
  *minDelay = 1.0f / (BSX_F32)GENERICGYROSPEC_DATARATEARRAY[gyroSensorSpec->maxDatarate];
  return 0;
}

BSX_S8 genericgyrospec_getMaximumRange(ts_gyrosensorspec *gyroSensorSpec, BSX_S16 *maxRange) {
  *maxRange = GENERICGYROSPEC_RANGEARRAY[gyroSensorSpec->range];
  return 0;
}

BSX_S8 genericgyrospec_getDatarate(ts_gyrosensorspec *gyroSensorSpec, BSX_U8 *datarate) {
  *datarate = gyroSensorSpec->dataRate;
  return 0;
}
