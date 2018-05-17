#include "MatrixOperations.h"
#include "AccCalibConfig.h"
const BSX_F32 ACCELCALIBCONFIG_DEFAULT_CALIBCOVARIANCEDiag[] = {0.0001f, 0.0001f, 0.0001f, 0.0f};
const BSX_F32 ACCELCALIBCONFIG_DEFAULT_CALIBPROCESSNOISEDiag[] = {0.000001f, 0.000001f, 0.000001f, 0.0f};
BSX_S8 acccalibconfig_setDefaultConfig(ts_acccalibconfig *accCalibConfig) {
  accCalibConfig->dynBufferSize = 5;
  accCalibConfig->dynBufferSizeMax = 10;
  accCalibConfig->dynamicThres = 45;
  accCalibConfig->orientThres = 150;
  accCalibConfig->minGThres = 900;
  accCalibConfig->maxGThres = 1200;
  accCalibConfig->orientCntThres = 8;
  accCalibConfig->accurRefThres = 150;
  accCalibConfig->radius = 1000.0;
  MATRIX_Diagonal((BSX_F32 *)ACCELCALIBCONFIG_DEFAULT_CALIBCOVARIANCEDiag, accCalibConfig->calibCovariance, 1, 4);
  MATRIX_Copy((BSX_F32 *)ACCELCALIBCONFIG_DEFAULT_CALIBPROCESSNOISEDiag, accCalibConfig->calibProcessNoise, 1, 4);
  accCalibConfig->calibMeasurementNoise = 36.0;
  return 0;
}
