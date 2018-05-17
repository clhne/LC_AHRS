#include "BSX_ObserverGyro.h"
#include "MathOperations.h"
BSX_U8 GYROOBSERVER_SetShakeDetectionParam(ts_GYROBSERVER *p_GyroObserver, BSX_U16 v_detectThreshold_u16r, BSX_S8 v_detectionTime_s8r) {
  p_GyroObserver->V_ShakeDetectThresh_U16R = v_detectThreshold_u16r;
  p_GyroObserver->V_ShakeDetectTime_S8R = v_detectionTime_s8r;
  return 0;
}

BSX_S8 GYROOBSERVER_SetNoDynamicDetectionTime(ts_GYROBSERVER *p_GyroObserver, BSX_U8 v_noDynDetTime_u8r) {
  p_GyroObserver->V_NoDynDetTime_U8R = v_noDynDetTime_u8r;
  return 0;
}

BSX_S8 GYROOBSERVER_SetDynamicDetectionThreshold(ts_GYROBSERVER *p_GyroObserver, BSX_U8 v_gyrodynthres_u8r) {
  p_GyroObserver->V_DynDetThresh_U8R = v_gyrodynthres_u8r;
  return 0;
}

BSX_U8 GYROOBSERVER_Reset(ts_GYROBSERVER *p_GyroObserver) {
  p_GyroObserver->V_DynDetCounter_U8R = 0;
  p_GyroObserver->V_DynamicStatus_U8R = 0;
  p_GyroObserver->V_ShakeDetectCounter_S8R = 0;
  p_GyroObserver->V_ShakeStatus_U8R = 0;
  return 0;
}

BSX_U8 GYROOBSERVER_Init(ts_GYROBSERVER *p_GyroObserver, BSX_U8 v_gyrohwid_u8) {
  BSX_S32 i;
  p_GyroObserver->V_DynamicStatus_U8R = 0;
  p_GyroObserver->V_DynDetCounter_U8R = 0;
  p_GyroObserver->V_ShakeDetectCounter_S8R = 0;
  p_GyroObserver->V_ShakeStatus_U8R = 0;
  p_GyroObserver->V_NoDynDetTime_U8R = 25;
  p_GyroObserver->V_FifoSize_U8R = 25;
  p_GyroObserver->V_ShakeDetectTime_S8R = 100;
  p_GyroObserver->V_ShakeDetectThresh_U16R = 3276;
  if (v_gyrohwid_u8 <= 2) {
    p_GyroObserver->V_DynDetThresh_U8R = 9;
  } else if (v_gyrohwid_u8 == 3) {
    p_GyroObserver->V_DynDetThresh_U8R = 13;
  }
  for (i = 0; i < 25; i++) {
    p_GyroObserver->S_GyroRatesBuffer.Wx_buf[i] = 0;
    p_GyroObserver->S_GyroRatesBuffer.Wy_buf[i] = 0;
    p_GyroObserver->S_GyroRatesBuffer.Wz_buf[i] = 0;
  }
  return 0;
}

BSX_U8 GYROOBSERVER_GetShakeStatus(ts_GYROBSERVER *p_GyroObserver, BSX_U8 *v_status_u8r) {
  *v_status_u8r = p_GyroObserver->V_ShakeStatus_U8R;
  return 0;
}

BSX_S8 GYROOBSERVER_GetNoDynamicDetectionTime(ts_GYROBSERVER *p_GyroObserver, BSX_U8 *v_noDynDetTime_u8r) {
  *v_noDynDetTime_u8r = p_GyroObserver->V_NoDynDetTime_U8R;
  return 0;
}

BSX_S8 GYROOBSERVER_GetDynamicDetectionThreshold(ts_GYROBSERVER *p_GyroObserver, BSX_U8 *v_gyrodynthres_u8r) {
  *v_gyrodynthres_u8r = p_GyroObserver->V_DynDetThresh_U8R;
  return 0;
}

BSX_U8 GYROOBSERVER_DynamicDetection(ts_dataxyzf32 *Gyro_Data_Raw, ts_GYROBSERVER *p_GyroObserver) {
  ts_dataxyz fifoInXYZ;
  ts_dataxyz fifoOutXYZ;
  ts_dataxyzs32 delatXYZ;
  BSX_S8 shakeDetectCounter = 0;
  if (!p_GyroObserver->S_GyroRatesBuffer.iteration) {
    MATH_GetMinMax(p_GyroObserver->S_GyroRatesBuffer.Wx_buf, 0, p_GyroObserver->V_FifoSize_U8R, &p_GyroObserver->MinValX, &p_GyroObserver->MaxValX);
    MATH_GetMinMax(p_GyroObserver->S_GyroRatesBuffer.Wy_buf, 0, p_GyroObserver->V_FifoSize_U8R, &p_GyroObserver->MinValY, &p_GyroObserver->MaxValY);
    MATH_GetMinMax(p_GyroObserver->S_GyroRatesBuffer.Wz_buf, 0, p_GyroObserver->V_FifoSize_U8R, &p_GyroObserver->MinValY, &p_GyroObserver->MaxValY);
    p_GyroObserver->S_GyroRatesBuffer.iteration = 1;
  }
  fifoInXYZ.x = (BSX_S16)Gyro_Data_Raw->x;
  fifoInXYZ.y = (BSX_S16)Gyro_Data_Raw->y;
  fifoInXYZ.z = (BSX_S16)Gyro_Data_Raw->z;
  fifoOutXYZ.x = p_GyroObserver->S_GyroRatesBuffer.Wx_buf[p_GyroObserver->S_GyroRatesBuffer.insert_index];
  fifoOutXYZ.y = p_GyroObserver->S_GyroRatesBuffer.Wy_buf[p_GyroObserver->S_GyroRatesBuffer.insert_index];
  fifoOutXYZ.z = p_GyroObserver->S_GyroRatesBuffer.Wz_buf[p_GyroObserver->S_GyroRatesBuffer.insert_index];
  MATH_UpdateFIFOBufferXYZ(
    fifoInXYZ.x,
    fifoInXYZ.y,
    fifoInXYZ.z,
    p_GyroObserver->S_GyroRatesBuffer.Wx_buf,
    p_GyroObserver->S_GyroRatesBuffer.Wy_buf,
    p_GyroObserver->S_GyroRatesBuffer.Wz_buf,
    &p_GyroObserver->S_GyroRatesBuffer.insert_index,
    25);
  if (p_GyroObserver->S_GyroRatesBuffer.iteration == 1) {
    // x
    if (fifoInXYZ.x <= p_GyroObserver->MaxValX) {
      if (fifoOutXYZ.x == p_GyroObserver->MaxValX) {
        MATH_GetMax(p_GyroObserver->S_GyroRatesBuffer.Wx_buf, 0, p_GyroObserver->V_FifoSize_U8R, &p_GyroObserver->MaxValX);
      }
    } else {
      p_GyroObserver->MaxValX = fifoInXYZ.x;
    }
    if (fifoInXYZ.x >= p_GyroObserver->MinValX) {
      if (fifoOutXYZ.x == p_GyroObserver->MinValX) {
        MATH_GetMin(p_GyroObserver->S_GyroRatesBuffer.Wx_buf, 0, p_GyroObserver->V_FifoSize_U8R, &p_GyroObserver->MinValX);
      }
    } else {
      p_GyroObserver->MinValX = fifoInXYZ.x;
    }
    // y
    if (fifoInXYZ.y <= p_GyroObserver->MaxValY) {
      if (fifoOutXYZ.y == p_GyroObserver->MaxValY) {
        MATH_GetMax(p_GyroObserver->S_GyroRatesBuffer.Wy_buf, 0, p_GyroObserver->V_FifoSize_U8R, &p_GyroObserver->MaxValY);
      }
    } else {
      p_GyroObserver->MaxValY = fifoInXYZ.y;
    }
    if (fifoInXYZ.y >= p_GyroObserver->MinValY) {
      if (fifoOutXYZ.y == p_GyroObserver->MinValY) {
        MATH_GetMin(p_GyroObserver->S_GyroRatesBuffer.Wy_buf, 0, p_GyroObserver->V_FifoSize_U8R, &p_GyroObserver->MinValY);
      }
    } else {
      p_GyroObserver->MinValY = fifoInXYZ.y;
    }
    // z
    if (fifoInXYZ.z <= p_GyroObserver->MaxValZ) {
      if (fifoOutXYZ.z == p_GyroObserver->MaxValZ) {
        MATH_GetMax(p_GyroObserver->S_GyroRatesBuffer.Wz_buf, 0, p_GyroObserver->V_FifoSize_U8R, &p_GyroObserver->MaxValZ);
      }
    } else {
      p_GyroObserver->MaxValZ = fifoInXYZ.z;
    }
    if (fifoInXYZ.z >= p_GyroObserver->MinValZ) {
      if (fifoOutXYZ.z == p_GyroObserver->MinValZ) {
        MATH_GetMin(p_GyroObserver->S_GyroRatesBuffer.Wz_buf, 0, p_GyroObserver->V_FifoSize_U8R, &p_GyroObserver->MinValZ);
      }
    } else {
      p_GyroObserver->MinValZ = fifoInXYZ.z;
    }
  }
  delatXYZ.x = p_GyroObserver->MaxValX - p_GyroObserver->MinValX;
  delatXYZ.y = p_GyroObserver->MaxValY - p_GyroObserver->MinValY;
  delatXYZ.z = p_GyroObserver->MaxValZ - p_GyroObserver->MinValZ;
  if (abs(delatXYZ.x) >= p_GyroObserver->V_DynDetThresh_U8R
   || abs(delatXYZ.y) >= p_GyroObserver->V_DynDetThresh_U8R
   || abs(delatXYZ.z) >= p_GyroObserver->V_DynDetThresh_U8R) {
    p_GyroObserver->V_DynDetCounter_U8R = 0;
  } else {
    p_GyroObserver->V_DynDetCounter_U8R++;
    if (p_GyroObserver->V_DynDetCounter_U8R >= 100) {
      p_GyroObserver->V_DynDetCounter_U8R = 100;
    }
  }
  if (delatXYZ.x > p_GyroObserver->V_ShakeDetectThresh_U16R) {
    shakeDetectCounter++;
  }
  if (delatXYZ.y > p_GyroObserver->V_ShakeDetectThresh_U16R) {
    shakeDetectCounter++;
  }
  if (delatXYZ.z > p_GyroObserver->V_ShakeDetectThresh_U16R) {
    shakeDetectCounter++;
  }
  if (shakeDetectCounter) {
    if (p_GyroObserver->V_ShakeDetectCounter_S8R >= p_GyroObserver->V_ShakeDetectTime_S8R) {
      shakeDetectCounter = p_GyroObserver->V_ShakeDetectTime_S8R + 1;
    } else {
      shakeDetectCounter = p_GyroObserver->V_ShakeDetectCounter_S8R + 1;
    }
  } else {
    if (p_GyroObserver->V_ShakeDetectCounter_S8R > 1) {
      shakeDetectCounter = p_GyroObserver->V_ShakeDetectCounter_S8R - 1;
    }
  }
  p_GyroObserver->V_ShakeDetectCounter_S8R = shakeDetectCounter;
  p_GyroObserver->V_ShakeStatus_U8R = p_GyroObserver->V_ShakeDetectTime_S8R < shakeDetectCounter;
  return p_GyroObserver->V_DynDetCounter_U8R < p_GyroObserver->V_NoDynDetTime_U8R;
}

BSX_U8 GYROOBSERVER_DoStep(ts_dataxyzf32 *p_gyrodataraw, ts_GYROBSERVER *p_GyroObserver) {
  p_GyroObserver->V_DynamicStatus_U8R = GYROOBSERVER_DynamicDetection(p_gyrodataraw, p_GyroObserver);
  return 0;
}
