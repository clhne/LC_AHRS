#include "BsxLibraryConstants.h"
#include "GenericClockDomain.h"

const BSX_U32 C_GENERICCLOCKDOMAIN__REFTIMEARRAY_U32X[] = {0xF4240, 0x30D40, 0x186A0, 0xC350, 0x9C40, 0x61A8, 0x4E20, 0x2710, 0x1F40, 0x1388};

BSX_S8 GENERICCLOCKDOMAIN_SetSystemDatarate(ts_GenClockDomain *p_genClockDomain, BSX_U8 v_dataRate_u8r) {
  if (v_dataRate_u8r > BSX_DATARATE_200HZ) {
    return -1;
  }
  p_genClockDomain->V_systemDatarate_U8R = v_dataRate_u8r;
  p_genClockDomain->V_trigSysTimerVal_U32R = 9999999;
  return 0;
}

BSX_S8 GENERICCLOCKDOMAIN_SetOrientDatarate(ts_GenClockDomain *p_genClockDomain, BSX_U8 v_dataRate_u8r) {
  if (v_dataRate_u8r > BSX_DATARATE_200HZ) {
    return -1;
  }
  p_genClockDomain->V_orientDatarate_U8R = v_dataRate_u8r;
  p_genClockDomain->V_triOrientTimerVal_U32R = 9999999;
  return 0;
}

BSX_S8 GENERICCLOCKDOMAIN_SetOperationMode(ts_GenClockDomain *p_genClockDomain, BSX_U8 v_mode_u8r) {
  if (v_mode_u8r <= BSX_OPMODE_FIFO) {
    p_genClockDomain->V_opmode_U8R = v_mode_u8r;
    if (v_mode_u8r == BSX_OPMODE_SLEEP) {
      p_genClockDomain->V_sysProcTick_U8R = 0;
      p_genClockDomain->V_calProcTick_U8R = 0;
      p_genClockDomain->V_observerProcTick_u8r = 0;
      p_genClockDomain->V_orientProcTick_U8R = 0;
    }
    return 0;
  }
  return -1;
}

BSX_S8 GENERICCLOCKDOMAIN_SetObserverDatarate(ts_GenClockDomain *p_genClockDomain, BSX_U8 v_dataRate_u8r) {
  if (v_dataRate_u8r > BSX_DATARATE_200HZ) {
    return -1;
  }
  p_genClockDomain->V_observerDatarate_U8R = v_dataRate_u8r;
  p_genClockDomain->V_triObserverTimerVal_U32R = 9999999;
  return 0;
}

BSX_S8 GENERICCLOCKDOMAIN_SetCalibDatarate(ts_GenClockDomain *p_genClockDomain, BSX_U8 v_dataRate_u8r) {
  if (v_dataRate_u8r > BSX_DATARATE_200HZ) {
    return -1;
  }
  p_genClockDomain->V_calibDatarate_U8R = v_dataRate_u8r;
  p_genClockDomain->V_trigCalTimerVal_U32R = 9999999;
  return 0;
}

BSX_S8 GENERICCLOCKDOMAIN_Reset(ts_GenClockDomain *p_genClockDomain) {
  p_genClockDomain->V_systemDatarate_U8R = BSX_DATARATE_10HZ;
  p_genClockDomain->V_opmode_U8R = BSX_OPMODE_SLEEP;
  p_genClockDomain->V_calibDatarate_U8R = BSX_DATARATE_10HZ;
  p_genClockDomain->V_orientDatarate_U8R = BSX_DATARATE_10HZ;
  p_genClockDomain->V_observerDatarate_U8R = BSX_DATARATE_10HZ;
  p_genClockDomain->V_sysProcTick_U8R = 0;
  p_genClockDomain->V_calProcTick_U8R = 0;
  p_genClockDomain->V_orientProcTick_U8R = 0;
  p_genClockDomain->V_observerProcTick_u8r = 0;
  p_genClockDomain->V_trigSysTimerVal_U32R = 9999999;
  p_genClockDomain->V_trigCalTimerVal_U32R = 9999999;
  p_genClockDomain->V_triOrientTimerVal_U32R = 9999999;
  p_genClockDomain->V_triObserverTimerVal_U32R = 9999999;
  return 0;
}

BSX_S8 GENERICCLOCKDOMAIN_Init(ts_GenClockDomain *p_genClockDomain) {
  GENERICCLOCKDOMAIN_SetOperationMode(p_genClockDomain, BSX_OPMODE_SLEEP);
  GENERICCLOCKDOMAIN_SetSystemDatarate(p_genClockDomain, BSX_DATARATE_10HZ);
  GENERICCLOCKDOMAIN_SetCalibDatarate(p_genClockDomain, BSX_DATARATE_10HZ);
  GENERICCLOCKDOMAIN_SetOrientDatarate(p_genClockDomain, BSX_DATARATE_10HZ);
  GENERICCLOCKDOMAIN_SetObserverDatarate(p_genClockDomain, BSX_DATARATE_10HZ);
  p_genClockDomain->V_sysProcTick_U8R = 0;
  p_genClockDomain->V_calProcTick_U8R = 0;
  p_genClockDomain->V_observerProcTick_u8r = 0;
  p_genClockDomain->V_orientProcTick_U8R = 0;
  p_genClockDomain->V_currentTimerVal_U32R = 9999999;
  p_genClockDomain->V_trigCalTimerVal_U32R = 9999999;
  p_genClockDomain->V_trigSysTimerVal_U32R = 9999999;
  p_genClockDomain->V_triOrientTimerVal_U32R = 9999999;
  p_genClockDomain->V_triObserverTimerVal_U32R = 9999999;
  return 0;
}

BSX_S8 GENERICCLOCKDOMAIN_GetVersion(ts_version *p_version){
  p_version->major = 0;
  p_version->minor = 9;
  p_version->majorbugFix = 0;
  p_version->minorbugFix = 0;
  return 0;
}

BSX_S8 GENERICCLOCKDOMAIN_GetSystemProcessingTimerTick(ts_GenClockDomain *p_genClockDomain, BSX_U8 *p_tickValue_u8r) {
  *p_tickValue_u8r = p_genClockDomain->V_sysProcTick_U8R;
  return 0;
}

BSX_S8 GENERICCLOCKDOMAIN_GetOrientProcessingTimerTick(ts_GenClockDomain *p_genClockDomain, BSX_U8 *p_tickValue_u8r) {
  *p_tickValue_u8r = p_genClockDomain->V_orientProcTick_U8R;
  return 0;
}

BSX_S8 GENERICCLOCKDOMAIN_GetCalibProcessingTimerTick(ts_GenClockDomain *p_genClockDomain, BSX_U8 *p_tickValue_u8r) {
  *p_tickValue_u8r = p_genClockDomain->V_calProcTick_U8R;
  return 0;
}

BSX_S8 GENERICCLOCKDOMAIN_DoStep(ts_GenClockDomain *p_genClockDomain, BSX_U32 v_timerValue_U32r, BSX_U8 *p_sysProcTick_u8r, BSX_U8 *p_calProc_u8r, BSX_U8 *p_orientProcTick_u8r, BSX_U8 *p_observerProcTick_u8r, BSX_U32 *p_dT_U32r) {
  if (p_genClockDomain->V_opmode_U8R) {
    p_genClockDomain->V_currentTimerVal_U32R = v_timerValue_U32r;
    if (p_genClockDomain->V_opmode_U8R == BSX_OPMODE_REGULAR) {
      if (100 * (v_timerValue_U32r - p_genClockDomain->V_trigSysTimerVal_U32R) < 95 * C_GENERICCLOCKDOMAIN__REFTIMEARRAY_U32X[p_genClockDomain->V_systemDatarate_U8R]) {
        p_genClockDomain->V_sysProcTick_U8R = 0;
      } else {
        p_genClockDomain->V_trigSysTimerVal_U32R = v_timerValue_U32r;
        p_genClockDomain->V_sysProcTick_U8R = 1;
      }
      if (100 * (v_timerValue_U32r - p_genClockDomain->V_trigCalTimerVal_U32R) < 95 * C_GENERICCLOCKDOMAIN__REFTIMEARRAY_U32X[p_genClockDomain->V_calibDatarate_U8R]) {
        p_genClockDomain->V_calProcTick_U8R = 0;
      } else {
        p_genClockDomain->V_trigCalTimerVal_U32R = v_timerValue_U32r;
        p_genClockDomain->V_calProcTick_U8R = 1;
      }
      if (100 * (v_timerValue_U32r - p_genClockDomain->V_triOrientTimerVal_U32R) < 95 * C_GENERICCLOCKDOMAIN__REFTIMEARRAY_U32X[p_genClockDomain->V_orientDatarate_U8R]){
        p_genClockDomain->V_orientProcTick_U8R = 0;
      } else {
        p_genClockDomain->V_triOrientTimerVal_U32R = v_timerValue_U32r;
        p_genClockDomain->V_orientProcTick_U8R = 1;
      }
      if (100 * (v_timerValue_U32r - p_genClockDomain->V_triObserverTimerVal_U32R) < 95 * C_GENERICCLOCKDOMAIN__REFTIMEARRAY_U32X[p_genClockDomain->V_observerDatarate_U8R]) {
        p_genClockDomain->V_observerProcTick_u8r = 0;
      } else {
        p_genClockDomain->V_triObserverTimerVal_U32R = v_timerValue_U32r;
        p_genClockDomain->V_observerProcTick_u8r = 1;
      }
    } else {
      p_genClockDomain->V_sysProcTick_U8R = 0;
      p_genClockDomain->V_calProcTick_U8R = 0;
      p_genClockDomain->V_dT_U32R = 0;
      p_genClockDomain->V_observerProcTick_u8r = 0;
      p_genClockDomain->V_orientProcTick_U8R = 0;
    }
  }
  *p_sysProcTick_u8r = p_genClockDomain->V_sysProcTick_U8R;
  *p_calProc_u8r = p_genClockDomain->V_calProcTick_U8R;
  *p_orientProcTick_u8r = p_genClockDomain->V_orientProcTick_U8R;
  *p_observerProcTick_u8r = p_genClockDomain->V_observerProcTick_u8r;
  *p_dT_U32r = p_genClockDomain->V_dT_U32R;
  return 0;
}
