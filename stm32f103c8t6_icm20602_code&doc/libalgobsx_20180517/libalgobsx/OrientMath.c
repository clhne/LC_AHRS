#include <math.h>
#include "invSqrt.h"
#include "OrientMath.h"
BSX_S8 orientmath_get_PitchRollFromGravity(ts_dataquatf32 gravity, ts_dataxyzf32 sqrG, BSX_F32 *pitch, BSX_F32 *roll) {
  if (sqrG.z + sqrG.y >= sqrG.x * 0.0144f) {
    *pitch = atan2f(-gravity.y, gravity.z);
  } else {
    if (gravity.z <= 0.0) {
      *pitch = atan2f(-gravity.y, -sqrtf(sqrG.x + sqrG.z)); 
    } else {
      *pitch = atan2f(-gravity.y, sqrtf(sqrG.x + sqrG.z)); 
    }
  }
  *roll = atan2f(gravity.x, sqrtf(sqrG.z + sqrG.y));
  return 0;
}

BSX_S8 orientmath_getHeading(ts_dataquatf32 magData, BSX_F32 *heading) {
  BSX_F32 angle = atan2f(-magData.x, magData.y);
  if (angle < 0.0f) {
    angle = angle + 6.2832f;
  }
  *heading = angle;
  return 0;
}

BSX_S8 orientmath_crossProductS(axis_unit axis, ts_dataquatf32 vec, ts_dataquatf32 *vectOut) {
  switch (axis) {
  case AXIS_UNIT_XPOSITIVE:
    vectOut->x = 0.0f;
    vectOut->y = -vec.z;
    vectOut->z = vec.y;
    break;
  case AXIS_UNIT_XNEGATIVE:
    vectOut->x = 0.0f;
    vectOut->y = vec.z;
    vectOut->z = -vec.y;
    break;
  case AXIS_UNIT_YPOSITIVE:
    vectOut->x = vec.z;
    vectOut->y = 0.0f;
    vectOut->z = -vec.x;
    break;
  case AXIS_UNIT_YNEGATIVE:
    vectOut->x = -vec.z;
    vectOut->y = 0.0f;
    vectOut->z = vec.x;
    break;
  case AXIS_UNIT_ZPOSITIVE:
    vectOut->x = -vec.y;
    vectOut->y = vec.x;
    vectOut->z = 0.0f;
    break;
  case AXIS_UNIT_ZNEGATIVE:
    vectOut->x = vec.y;
    vectOut->y = -vec.x;
    vectOut->z = 0.0f;
    break;
  default:
    vectOut->x = 0.0f;
    vectOut->y = 0.0f;
    vectOut->z = 0.0f;
    break;
  }
  return 0;
}

BSX_S8 orientmath_crossProduct(ts_dataquatf32 vector1, ts_dataquatf32 vector2, ts_dataquatf32 *vectOut) {
  vectOut->x = vector1.y * vector2.z - vector2.y * vector1.z;
  vectOut->y = vector1.z * vector2.x - vector2.z * vector1.x;
  vectOut->z = vector1.x * vector2.y - vector2.x * vector1.y;
  return 0;
}

BSX_S8 orientmath_calcAngleS(axis_unit axis, ts_dataquatf32 vec, BSX_F32 *angle) {
  BSX_F32 length = vec.x * vec.x + vec.y * vec.y + vec.z * vec.z;
  if (length == 0.0f) {
    *angle = 1.5708f;
  } else {
    BSX_F32 cos_angle;
    BSX_F32 inv_sqrt_length = invSqrtF(length);
    switch (axis) {
    case AXIS_UNIT_XPOSITIVE:
      cos_angle = vec.z * inv_sqrt_length;
      break;
    case AXIS_UNIT_XNEGATIVE:
      cos_angle = -vec.x * inv_sqrt_length;
      break;
    case AXIS_UNIT_YPOSITIVE:
      cos_angle = vec.y * inv_sqrt_length;
      break;
    case AXIS_UNIT_YNEGATIVE:
      cos_angle = -vec.y * inv_sqrt_length;
      break;
    case AXIS_UNIT_ZPOSITIVE:
      cos_angle = vec.z * inv_sqrt_length;
      break;
    case AXIS_UNIT_ZNEGATIVE:
      cos_angle = -vec.z * inv_sqrt_length;
      break;
    default:
      cos_angle = 0.0f;
      break;
    }
    *angle = acosf(cos_angle);
  }
  return 0;
}

BSX_S8 orientmath_calcAngle(ts_dataquatf32 vector1, ts_dataquatf32 vector2, BSX_F32 *angle) {
  BSX_F32 length1 = vector1.x * vector1.x + vector1.y * vector1.y + vector1.z * vector1.z;
  BSX_F32 length2 = vector2.x * vector2.x + vector2.y * vector2.y + vector2.z * vector2.z;
  if (length1 == 0.0f || length2 == 0.0f) {
    *angle = 1.5708f;
  } else {
    BSX_F32 inv_sqrt_length1 = invSqrtF(length1);
    BSX_F32 inv_sqrt_length2 = invSqrtF(length2);
    *angle = acosf(vector1.y * inv_sqrt_length1 * vector2.y * inv_sqrt_length2 + vector1.x * inv_sqrt_length1 * vector2.x * inv_sqrt_length2 + vector1.z * inv_sqrt_length1 * vector2.z * inv_sqrt_length2);
  }
  return 0;
}

BSX_S8 orientmath_axisAng2Quat(ts_dataquatf32 axis, BSX_F32 angle, ts_dataquatf32 *quatOut) {
  BSX_F32 length = axis.x * axis.x + axis.y * axis.y + axis.z * axis.z;
  BSX_F32 sin_half_angle = sinf(angle * 0.5f);
  BSX_F32 cos_half_angle = cosf(angle * 0.5f);
  quatOut->w = cos_half_angle;
  if (length <= 1.0e-10f) {
    quatOut->x = 0.0f;
    quatOut->y = 0.0f;
    quatOut->z = 0.0f;
  } else {
    BSX_F32 inv_sqrt_length = invSqrtF(length);
    quatOut->x = axis.x * inv_sqrt_length * sin_half_angle;
    quatOut->y = axis.y * inv_sqrt_length * sin_half_angle;
    quatOut->z = axis.z * inv_sqrt_length * sin_half_angle;
  }
  return 0;
}

BSX_S8 orientmath_getVersion(ts_version *version) {
  version->minorbugFix = 0;
  version->major = 1;
  version->majorbugFix = 0;
  version->minor = 0;
  return 0;
}
