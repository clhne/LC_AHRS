#include "MatrixOperations.h"
#include "MagCalibConfig.h"
const BSX_F32 MAGCALIB_DEFAULTCONFIG_CALIBPROCESSNOISEDiag[] = {0.000009f, 0.000009f, 0.000009f, 0.000081f};
const BSX_F32 MAGCALIB_DEFAULTCONFIG_CALIBCOVARIANCEDiag[] = {0.25f, 0.25f, 0.25f, 4.0f};
BSX_S8 magcalibconfig_setDefaultConfig(ts_magcalibconfig *magCalibConfig) {
  magCalibConfig->opMode = 2;
  magCalibConfig->dynBufferSize = 5;
  magCalibConfig->dynReferenceThres = 321;
  magCalibConfig->magDistGapCnt = 2;
  magCalibConfig->maxMagField = 320;
  magCalibConfig->magDistGapCntThres = 2;
  magCalibConfig->accurQuiteCntThres = 5;
  magCalibConfig->minMagField = 36;
  magCalibConfig->typMagField = 120;
  magCalibConfig->magDistDynThres = 260;
  magCalibConfig->outOfRangeCnt = 20;
  magCalibConfig->outOfRangeCntThres = 20;
  magCalibConfig->magDistDynCntThres = 4;
  magCalibConfig->V_magDistInitCountThres = 4;
  magCalibConfig->cosphi = 1.0f;
  magCalibConfig->cosPhiThresMin = 0.1f;
  magCalibConfig->cosPhiThresMax = 1.2f;
  magCalibConfig->cosPhiDeltaUp = 0.4f;
  magCalibConfig->cosPhiDeltaDown = 0.1f;
  magCalibConfig->cosPhiCntThres = 20;
  magCalibConfig->orientThres = 16;
  magCalibConfig->orientCntThres = 4;
  magCalibConfig->radius = 120;
  magCalibConfig->accurCosPhiMaxThres = 0.6f;
  magCalibConfig->accurBufferSize = 8;
  magCalibConfig->accurCosPhiBreakDownThres = 0.6f;
  magCalibConfig->magDistDynCnt = 0;
  magCalibConfig->V_magDistInitCount = 0;
  magCalibConfig->cosPhiThres = 1;
  magCalibConfig->cosphiFlag = 0;
  magCalibConfig->cosPhiCnt = 0;
  magCalibConfig->accurQuiteCnt = 0;
  magCalibConfig->constMaxMagField = 80;
  magCalibConfig->constTypMagField = 30;
  magCalibConfig->constMinMagField = 9;
  magCalibConfig->constMeasRange = 1000;
  magCalibConfig->constDynReferenceThres = 80;
  magCalibConfig->constMagDistDynThres = 65;
  magCalibConfig->constOrientThres = 4;
  MATRIX_Diagonal((BSX_F32 *)MAGCALIB_DEFAULTCONFIG_CALIBCOVARIANCEDiag, magCalibConfig->calibCovariance, 1, 4);
  MATRIX_Copy((BSX_F32 *)MAGCALIB_DEFAULTCONFIG_CALIBPROCESSNOISEDiag, magCalibConfig->calibProcessNoise, 1, 4);
  magCalibConfig->calibMeasurementNoise = 7.0;
  return 0;
}