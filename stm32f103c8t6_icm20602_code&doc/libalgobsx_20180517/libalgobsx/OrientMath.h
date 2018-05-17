#ifndef __ORIENTMATH_H__
#define __ORIENTMATH_H__
#include "BsxLibraryDataTypes.h"
typedef enum {
  AXIS_UNKNOWN = 0x0,
  AXIS_UNIT_XPOSITIVE = 0x1,
  AXIS_UNIT_XNEGATIVE = 0x2,
  AXIS_UNIT_YPOSITIVE = 0x4,
  AXIS_UNIT_YNEGATIVE = 0x8,
  AXIS_UNIT_ZPOSITIVE = 0x10,
  AXIS_UNIT_ZNEGATIVE = 0x20,
  AXIS_INVALID = 0x40,
} axis_unit;
BSX_S8 orientmath_get_PitchRollFromGravity(ts_dataquatf32 gravity, ts_dataxyzf32 sqrG, BSX_F32 *pitch, BSX_F32 *roll);
BSX_S8 orientmath_getHeading(ts_dataquatf32 magData, BSX_F32 *heading);
BSX_S8 orientmath_crossProductS(axis_unit axis, ts_dataquatf32 vec, ts_dataquatf32 *vectOut);
BSX_S8 orientmath_crossProduct(ts_dataquatf32 vector1, ts_dataquatf32 vector2, ts_dataquatf32 *vectOut);
BSX_S8 orientmath_calcAngleS(axis_unit axis, ts_dataquatf32 vec, BSX_F32 *angle);
BSX_S8 orientmath_calcAngle(ts_dataquatf32 vector1, ts_dataquatf32 vector2, BSX_F32 *angle);
BSX_S8 orientmath_axisAng2Quat(ts_dataquatf32 axis, BSX_F32 angle, ts_dataquatf32 *quatOut);
BSX_S8 orientmath_getVersion(ts_version *version);
#endif