#include <math.h>
#include "NDOF.h"

// helper functions
float Round(float x)
{
    float result;
    if (x < 0.0f) {
        result = ceilf(x - 0.5f);
    } else {
        result = floorf(x + 0.5f);
    }
    return result;
}

void AddXYZDataToFIFO(short data_x, short data_y, short data_z, short *fifo_x, short *fifo_y, short *fifo_z, short *index, short length)
{
    short idx = *index;
    fifo_x[idx] = data_x;
    fifo_y[idx] = data_y;
    fifo_z[idx] = data_z;
    *index = (short)(idx + 1) % length;
}

short GetMaxFromFIFO(short *fifo, short start, short length)
{
    int i;
    short maxv = fifo[start];
    for (i = 1; i < length; i++) {
        short v = fifo[start + i];
        if (v >= maxv) {
            maxv = v;
        }
    }
    return maxv;
}

short GetMinFromFIFO(short *fifo, short start, short length)
{
    int i;
    short minv = fifo[start];
    for (i = 1; i < length; i++) {
        short v = fifo[start + i];
        if (v < minv) {
            minv = v;
        }
    }
    return minv;
}

float GetMeanFromFIFO(short *fifo, short start, short length)
{
    short i;
    float sum = (float)fifo[start];
    for (i = 1; i < length; i++) {
        short val = fifo[start + i];
        sum += (float)val;
    }
    return sum / (float)length;
}

unsigned char UpdateOffsetStepwise(float *offsetValue, float meanValue, short offsetStep)
{
    float new_offsetValue;
    float old_offsetValue = *offsetValue;
    float diff_offsetValue_meanValue = old_offsetValue - meanValue;
    if (fabsf(diff_offsetValue_meanValue) >= (float)offsetStep) {
        if (diff_offsetValue_meanValue > -(float)offsetStep) {
            new_offsetValue = old_offsetValue - offsetStep;
        } else {
            new_offsetValue = old_offsetValue + offsetStep;
        }
    } else {
        new_offsetValue = meanValue;
    }
    *offsetValue = new_offsetValue * 0.1f + old_offsetValue * 0.9f;
    return 0;
}

void UpdateOrientation(float *quat_in, float gyro_in_x, float gyro_in_y, float gyro_in_z, float samplingtimegyro, float threshold, float *quat_out)
{
    float quat_rot[4];
    float half_dt = samplingtimegyro * 0.0087267f; // 0.0087267f = 0.5 * 3.1415926 / 180.0
    if (quat_in == quat_out) {
        return;
    }
    quat_rot[0] = 1.0f;
    if (fabsf(gyro_in_x) < threshold) {
        quat_rot[1] = 0.0f;
    } else {
        quat_rot[1] = gyro_in_x * half_dt;
    }
    if (fabsf(gyro_in_y) < threshold) {
        quat_rot[2] = 0.0f;
    } else {
        quat_rot[2] = gyro_in_y * half_dt;
    }
    if (fabsf(gyro_in_z) < threshold) {
        quat_rot[3] = 0.0f;
    } else {
        quat_rot[3] = gyro_in_z * half_dt;
    }
    quat_out[0] = quat_in[0] - quat_in[1] * quat_rot[1] - quat_in[2] * quat_rot[2] - quat_in[3] * quat_rot[3];
    quat_out[1] = quat_in[1] + quat_in[0] * quat_rot[1] - quat_in[3] * quat_rot[2] + quat_in[2] * quat_rot[3];
    quat_out[2] = quat_in[2] + quat_in[3] * quat_rot[1] + quat_in[0] * quat_rot[2] - quat_in[1] * quat_rot[3];
    quat_out[3] = quat_in[3] - quat_in[2] * quat_rot[1] + quat_in[1] * quat_rot[2] + quat_in[0] * quat_rot[3];
}

void GetGravity(const float *quat_in, float *gravity_out)
{
    gravity_out[2] = quat_in[3] * quat_in[3] - quat_in[2] * quat_in[2] - quat_in[1] * quat_in[1] + quat_in[0] * quat_in[0];
    gravity_out[0] = quat_in[1] * quat_in[3] - quat_in[0] * quat_in[2] + quat_in[1] * quat_in[3] - quat_in[0] * quat_in[2];
    gravity_out[1] = quat_in[2] * quat_in[3] + quat_in[0] * quat_in[1] + quat_in[2] * quat_in[3] + quat_in[0] * quat_in[1];
}

void QuatMultiplication(const float* quatIn1, const float* quatIn2, float *quatOut)
{
    quatOut[0] = quatIn1[0] * quatIn2[0] - quatIn1[1] * quatIn2[1] - quatIn1[2] * quatIn2[2] - quatIn1[3] * quatIn2[3];
    quatOut[1] = quatIn1[0] * quatIn2[1] + quatIn1[1] * quatIn2[0] - quatIn1[3] * quatIn2[2] + quatIn1[2] * quatIn2[3];
    quatOut[2] = quatIn1[3] * quatIn2[1] + quatIn1[2] * quatIn2[0] + quatIn1[0] * quatIn2[2] - quatIn1[1] * quatIn2[3];
    quatOut[3] = quatIn1[3] * quatIn2[0] - quatIn1[2] * quatIn2[1] + quatIn1[1] * quatIn2[2] + quatIn1[0] * quatIn2[3];
}

void QuatNormalization(const float* quat_in, float *quat_out)
{
    float vector_length = quat_in[0] * quat_in[0] + quat_in[1] * quat_in[1] + quat_in[2] * quat_in[2] + quat_in[3] * quat_in[3];
    if (vector_length <= 0.000001f) {
        quat_out[0] = 1.0f;
        quat_out[1] = 0.0f;
        quat_out[2] = 0.0f;
        quat_out[3] = 0.0f;
    } else {
        if (vector_length == 1.0f) {
            quat_out[0] = quat_in[0];
            quat_out[1] = quat_in[1];
            quat_out[2] = quat_in[2];
            quat_out[3] = quat_in[3];
        } else {
            vector_length = 1.0f / sqrtf(vector_length);
            quat_out[0] = quat_in[0] * vector_length;
            quat_out[1] = quat_in[1] * vector_length;
            quat_out[2] = quat_in[2] * vector_length;
            quat_out[3] = quat_in[3] * vector_length;
        }
    }
}

void QuatPointRot(const float* quat_in, const float* point_in, float* quat_out)
{
    float qw = point_in[1] * quat_in[1] - quat_in[0] * 0.0f + point_in[2] * quat_in[2] + point_in[3] * quat_in[3];
    float qx = point_in[1] * quat_in[0] + quat_in[1] * 0.0f - point_in[2] * quat_in[3] + point_in[3] * quat_in[2];
    float qy = point_in[2] * quat_in[0] + quat_in[2] * 0.0f + point_in[1] * quat_in[3] - point_in[3] * quat_in[1];
    float qz = quat_in[3] * 0.0f - point_in[1] * quat_in[2] + point_in[2] * quat_in[1] + point_in[3] * quat_in[0];
    quat_out[0] = quat_in[1] * qx - quat_in[0] * qw + quat_in[2] * qy + quat_in[3] * qz;
    quat_out[1] = quat_in[1] * qw + quat_in[0] * qx + quat_in[2] * qz - quat_in[3] * qy;
    quat_out[2] = quat_in[0] * qy - quat_in[1] * qz + quat_in[2] * qw + quat_in[3] * qx;
    quat_out[3] = quat_in[1] * qy + quat_in[0] * qz - quat_in[2] * qx + quat_in[3] * qw;
}

void GetEulerAngle(const float *quat_in, float *roll, float *pitch, float *yaw)
{
    float angle;
    float sin_angle;
    float cos_angle;
    float vector_length;
    float inv_sqrt_vector_length;
    float quatG[4];
    float sqrG[3];
    float quat1[4];
    float quat2[4];
    GetGravity(quat_in, &quatG[1]);
    quatG[0] = 0.0f;
    sqrG[0] = quatG[1] * quatG[1];
    sqrG[1] = quatG[2] * quatG[2];
    sqrG[2] = quatG[3] * quatG[3];
    // calc roll and pitch
    if ((sqrG[2] + sqrG[1]) >= sqrG[0] * 0.0144f) {
        *pitch = atan2f(-quatG[2], quatG[3]);
    } else {
        if (quatG[3] <= 0.0f) {
            *pitch = atan2f(-quatG[2], -sqrtf(sqrG[0] + sqrG[2]));
        } else {
            *pitch = atan2f(-quatG[2], sqrtf(sqrG[0] + sqrG[2]));
        }
    }
    *roll = atan2f(quatG[1], sqrtf(sqrG[1] + sqrG[2]));
    // calc yaw
    if (quatG[3] < 0.0f && (sqrG[0] + sqrG[1]) < 0.5f) {
        // Z negative
        angle = 0.0f;
        vector_length = quatG[1] * quatG[1] + quatG[2] * quatG[2] + quatG[3] * quatG[3];
        if (vector_length == 0.0f) {
            angle = 1.5708f;
        } else {
            inv_sqrt_vector_length = 1.0f / sqrt(vector_length);
            cos_angle = -quatG[3] * inv_sqrt_vector_length;
            angle = acosf(cos_angle);
        }
        quat1[1] = quatG[2];
        quat1[2] = -quatG[1];
        quat1[3] = 0.0f;
        vector_length = quat1[1] * quat1[1] + quat1[2] * quat1[2] + quat1[3] * quat1[3];
        sin_angle = sinf(angle * 0.5f);
        cos_angle = cosf(angle * 0.5f);
        quat2[0] = cos_angle;
        if (vector_length <= 1.0e-10f) {
            quat2[1] = 0.0f;
            quat2[2] = 0.0f;
            quat2[3] = 0.0f;
        } else {
            inv_sqrt_vector_length = 1.0f / sqrt(vector_length) * sin_angle;
            quat2[1] = quat1[1] * inv_sqrt_vector_length;
            quat2[2] = quat1[2] * inv_sqrt_vector_length;
            quat2[3] = quat1[3] * inv_sqrt_vector_length;
        }
        // rot roll 180deg
        quat1[2] = quat2[0];
        quat1[3] = quat2[1];
        quat1[0] = -quat2[2];
        quat1[1] = -quat2[3];
    } else {
        // Z positive
        angle = 0.0f;
        vector_length = quatG[1] * quatG[1] + quatG[2] * quatG[2] + quatG[3] * quatG[3];
        if (vector_length == 0.0f) {
            angle = 1.5708f;
        } else {
            inv_sqrt_vector_length = 1.0f / sqrt(vector_length);
            cos_angle = quatG[3] * inv_sqrt_vector_length;
            angle = acosf(cos_angle);
        }
        quat1[1] = -quatG[2];
        quat1[2] = quatG[1];
        quat1[3] = 0.0f;
        vector_length = quat1[1] * quat1[1] + quat1[2] * quat1[2] + quat1[3] * quat1[3];
        sin_angle = sinf(angle * 0.5f);
        cos_angle = cosf(angle * 0.5f);
        quat2[0] = cos_angle;
        if (vector_length <= 1.0e-10f) {
            quat2[1] = 0.0f;
            quat2[2] = 0.0f;
            quat2[3] = 0.0f;
        } else {
            inv_sqrt_vector_length = 1.0f / sqrt(vector_length) * sin_angle;
            quat2[1] = quat1[1] * inv_sqrt_vector_length;
            quat2[2] = quat1[2] * inv_sqrt_vector_length;
            quat2[3] = quat1[3] * inv_sqrt_vector_length;
        }
        // no rot
        quat1[2] = quat2[2];
        quat1[3] = quat2[3];
        quat1[0] = quat2[0];
        quat1[1] = quat2[1];
    }
    QuatMultiplication(quat_in, quat1, quat2);
    *yaw = atan2f(quat2[3], quat2[0]) * -2.0f;
    if (*yaw < 0.0f) {
        *yaw += 6.2832f;
    }
}

// NDOF interfaces

#define ACC_FIFO_SIZE        (5)
#define GYRO_FIFO_SIZE       (25)
#define GRAVITY              (9.80665f)

typedef struct {
    // acc param
    float acc_res;
    float acc_iir_coef;
    short acc_dyn_det_th;
    short acc_no_dyn_det_time;
    short acc_dyn_det_bwidth;
    int acc_oneG_th_min;
    int acc_oneG_th_max;
    // gyro param
    float gyro_res;
    short gyro_dyn_det_th;
    short gyro_no_dyn_det_time;
    short gyro_offset_th_max;
    short gyro_offset_th_step;
    short gyro_shake_det_th;
    short gyro_shake_det_time;
    // orientation param
    float ori_motion_threshold;
    float ori_acc_gyro_rate_threshold;
    float ori_acc_noise_threshold;
    int ori_acc_coupling_timer_threshold;
    float ori_acc_coupling_factor;
    float ori_acc_coupling_threshold;
    // acc states
    short ax_adc;
    short ay_adc;
    short az_adc;
    float raw_ax;
    float raw_ay;
    float raw_az;
    float iir_ax;
    float iir_ay;
    float iir_az;
    float cor_ax;
    float cor_ay;
    float cor_az;
    short ax_fifo[ACC_FIFO_SIZE];
    short ay_fifo[ACC_FIFO_SIZE];
    short az_fifo[ACC_FIFO_SIZE];
    short acc_fifo_idx;
    short acc_dyn_det_count;
    unsigned char acc_dyn_status;
    unsigned char acc_oneG_status;
    unsigned char acc_is_calibrated;
    unsigned char acc_calibrated_accuracy;
    float ax_bias;
    float ay_bias;
    float az_bias;
    // gyro states
    short gx_adc;
    short gy_adc;
    short gz_adc;
    float raw_gx;
    float raw_gy;
    float raw_gz;
    float cor_gx;
    float cor_gy;
    float cor_gz;
    short gx_fifo[GYRO_FIFO_SIZE];
    short gy_fifo[GYRO_FIFO_SIZE];
    short gz_fifo[GYRO_FIFO_SIZE];
    short gyro_fifo_idx;
    short gyro_dyn_det_count;
    unsigned char gyro_dyn_status;
    unsigned char gyro_is_calibrated;
    unsigned char gyro_calibrated_accuracy;
    float iir_gx_bias;
    float iir_gy_bias;
    float iir_gz_bias;
    float gx_bias;
    float gy_bias;
    float gz_bias;
    short gx_min;
    short gy_min;
    short gz_min;
    short gx_max;
    short gy_max;
    short gz_max;
    short gyro_shake_det_count;
    unsigned char gyro_shake_status;
    // orientation states
    unsigned char ori_can_update;
    float ori_quat[4];
    unsigned char ori_shake_status;
    unsigned char ori_motion_status;
    int ori_acc_coupling_timer;
    unsigned char ori_acc_coupling_restart_flag;
    // other states
    long long ts;
} NDOF;

NDOF ndof;

const unsigned char NDOF_ACC_RANGE_ARRAY[] = {0x02, 0x04, 0x8, 0x10};
const short NDOF_GYRO_RANGE_ARRAY[] = {0x800, 0x7D0, 0x3E8, 0x1F4, 0xFA};
void NDOF_Init(unsigned char acc_range_idx, unsigned char acc_res_bits, unsigned char gyro_range_idx, unsigned char gyro_res_bits)
{
    // acc param
    ndof.acc_res = (float)NDOF_ACC_RANGE_ARRAY[acc_range_idx] * 1000.0f / (float)(1L << (acc_res_bits - 1));
    ndof.acc_iir_coef = 0.55f;
    ndof.acc_dyn_det_th = 25/*13*/;
    ndof.acc_no_dyn_det_time = 3;
    ndof.acc_dyn_det_bwidth = 4;
    ndof.acc_oneG_th_min = 553536;
    ndof.acc_oneG_th_max = 1577536;
    // gyro param
    ndof.gyro_res = (float)NDOF_GYRO_RANGE_ARRAY[gyro_range_idx] / (float)((1L << (gyro_res_bits - 1)) - 1) / 0.061f;
    ndof.gyro_dyn_det_th = /*9*//*18*/19;
    ndof.gyro_no_dyn_det_time = 50/*25*/;
    ndof.gyro_offset_th_max = 163;
    ndof.gyro_offset_th_step = 4;
    ndof.gyro_shake_det_th = 3276;
    ndof.gyro_shake_det_time = 100;
    // other param
    ndof.ori_motion_threshold = 35;
    ndof.ori_acc_gyro_rate_threshold = 11085.125f;
    ndof.ori_acc_noise_threshold = 0.02f;
    ndof.ori_acc_coupling_timer_threshold = 100;
    ndof.ori_acc_coupling_factor = 0.25f /*2.5f*/;
    ndof.ori_acc_coupling_threshold = 0.03f /*0.3f*/;
    // reset acc&gyro states
    NDOF_Reset();
}

void NDOF_Reset(void)
{
    int i;
    // acc states
    ndof.ax_adc = 0;
    ndof.ay_adc = 0;
    ndof.az_adc = 0;
    ndof.raw_ax = 0.0f;
    ndof.raw_ay = 0.0f;
    ndof.raw_az = 0.0f;
    ndof.iir_ax = 0.0f;
    ndof.iir_ay = 0.0f;
    ndof.iir_az = 0.0f;
    ndof.cor_ax = 0.0f;
    ndof.cor_ay = 0.0f;
    ndof.cor_az = 0.0f;
    for (i = 0; i < ACC_FIFO_SIZE; i++) {
        ndof.ax_fifo[i] = 0;
        ndof.ay_fifo[i] = 0;
        ndof.az_fifo[i] = 0;
    }
    ndof.acc_fifo_idx = 0;
    ndof.acc_dyn_det_count = 0;
    ndof.acc_dyn_status = 0;
    ndof.acc_oneG_status = 0;
    //ndof.acc_is_calibrated = 0;
    ndof.acc_is_calibrated = 1; // TODO .........
    ndof.acc_calibrated_accuracy = 0;
    ndof.ax_bias = 0.0f;
    ndof.ay_bias = 0.0f;
    ndof.az_bias = 0.0f;
    // gyro states
    ndof.gx_adc = 0;
    ndof.gy_adc = 0;
    ndof.gz_adc = 0;
    ndof.raw_gx = 0.0f;
    ndof.raw_gy = 0.0f;
    ndof.raw_gz = 0.0f;
    ndof.cor_gx = 0.0f;
    ndof.cor_gy = 0.0f;
    ndof.cor_gz = 0.0f;
    for (i = 0; i < GYRO_FIFO_SIZE; i++) {
        ndof.gx_fifo[i] = 0;
        ndof.gy_fifo[i] = 0;
        ndof.gz_fifo[i] = 0;
    }
    ndof.gyro_fifo_idx = 0;
    ndof.gyro_dyn_det_count = 0;
    ndof.gyro_dyn_status = 0;
    ndof.gyro_is_calibrated = 0;
    ndof.gyro_calibrated_accuracy = 0;
    ndof.iir_gx_bias = 0.0f;
    ndof.iir_gy_bias = 0.0f;
    ndof.iir_gz_bias = 0.0f;
    ndof.gx_bias = 0.0f;
    ndof.gy_bias = 0.0f;
    ndof.gz_bias = 0.0f;
    ndof.gx_min = 0;
    ndof.gy_min = 0;
    ndof.gz_min = 0;
    ndof.gx_max = 0;
    ndof.gy_max = 0;
    ndof.gz_max = 0;
    ndof.gyro_shake_det_count = 0;
    ndof.gyro_shake_status = 0;
    // orientation states
    ndof.ori_can_update = 0;
    ndof.ori_quat[0] = 1.0f;
    ndof.ori_quat[1] = 0.0f;
    ndof.ori_quat[2] = 0.0f;
    ndof.ori_quat[3] = 0.0f;
    ndof.ori_shake_status = 0;
    ndof.ori_motion_status = 0;
    ndof.ori_acc_coupling_timer = 0;
    ndof.ori_acc_coupling_restart_flag = 1;
    // other states
    ndof.ts = 0;
}

int NDOF_DoStep(short ax_adc, short ay_adc, short az_adc, short gx_adc, short gy_adc, short gz_adc, long long ts)
{
    short acc_dyn_ref_idx, acc_dyn_ref_x, acc_dyn_ref_y, acc_dyn_ref_z, acc_dyn_cur_x, acc_dyn_cur_y, acc_dyn_cur_z;
    float acc_vector_length;
    short gyro_dyn_ref_idx, gyro_dyn_ref_x, gyro_dyn_ref_y, gyro_dyn_ref_z, gyro_dyn_cur_x, gyro_dyn_cur_y, gyro_dyn_cur_z;
    float delta_x, delta_y, delta_z;
    // acc preprocess
    ndof.ax_adc = ax_adc;
    ndof.ay_adc = ay_adc;
    ndof.az_adc = az_adc;
    ndof.raw_ax = (float)ndof.ax_adc * ndof.acc_res;
    ndof.raw_ay = (float)ndof.ay_adc * ndof.acc_res;
    ndof.raw_az = (float)ndof.az_adc * ndof.acc_res;
    ndof.cor_ax = ndof.raw_ax - ndof.ax_bias;
    ndof.cor_ay = ndof.raw_ay - ndof.ay_bias;
    ndof.cor_az = ndof.raw_az - ndof.az_bias;
    // iir 1-Ord filter
    ndof.iir_ax = ndof.cor_ax * (1.0f - ndof.acc_iir_coef) + ndof.iir_ax * ndof.acc_iir_coef;
    ndof.iir_ay = ndof.cor_ay * (1.0f - ndof.acc_iir_coef) + ndof.iir_ay * ndof.acc_iir_coef;
    ndof.iir_az = ndof.cor_az * (1.0f - ndof.acc_iir_coef) + ndof.iir_az * ndof.acc_iir_coef;
    // gyro preprocess
    ndof.gx_adc = gx_adc;
    ndof.gy_adc = gy_adc;
    ndof.gz_adc = gz_adc;
    ndof.raw_gx = (float)ndof.gx_adc * ndof.gyro_res;
    ndof.raw_gy = (float)ndof.gy_adc * ndof.gyro_res;
    ndof.raw_gz = (float)ndof.gz_adc * ndof.gyro_res;
    ndof.cor_gx = ndof.raw_gx - ndof.gx_bias;
    ndof.cor_gy = ndof.raw_gy - ndof.gy_bias;
    ndof.cor_gz = ndof.raw_gz - ndof.gz_bias;
    // check if acc is dynamic ?
    if (ndof.acc_fifo_idx < ndof.acc_dyn_det_bwidth) {
        acc_dyn_ref_idx = 5 + ndof.acc_fifo_idx - ndof.acc_dyn_det_bwidth;
    } else {
        acc_dyn_ref_idx = ndof.acc_fifo_idx - ndof.acc_dyn_det_bwidth;
    }
    acc_dyn_cur_x = (short)ndof.iir_ax;
    acc_dyn_cur_y = (short)ndof.iir_ay;
    acc_dyn_cur_z = (short)ndof.iir_az;
    AddXYZDataToFIFO(acc_dyn_cur_x, acc_dyn_cur_y, acc_dyn_cur_z, ndof.ax_fifo, ndof.ay_fifo, ndof.az_fifo, &ndof.acc_fifo_idx, ACC_FIFO_SIZE);
    acc_dyn_ref_x = ndof.ax_fifo[acc_dyn_ref_idx];
    acc_dyn_ref_y = ndof.ay_fifo[acc_dyn_ref_idx];
    acc_dyn_ref_z = ndof.az_fifo[acc_dyn_ref_idx];
    if (fabsf((float)acc_dyn_cur_x - (float)acc_dyn_ref_x) > (float)ndof.acc_dyn_det_th
            || fabsf((float)acc_dyn_cur_y - (float)acc_dyn_ref_y) > (float)ndof.acc_dyn_det_th
            || fabsf((float)acc_dyn_cur_z - (float)acc_dyn_ref_z) > (float)ndof.acc_dyn_det_th) {
        ndof.acc_dyn_det_count = 0;
    } else {
        ndof.acc_dyn_det_count++;
        if (ndof.acc_dyn_det_count >= 100) {
            ndof.acc_dyn_det_count = 100;
        }
    }
    ndof.acc_dyn_status = ndof.acc_dyn_det_count < ndof.acc_no_dyn_det_time;
    acc_vector_length = ndof.iir_ax * ndof.iir_ax + ndof.iir_ay * ndof.iir_ay + ndof.iir_az * ndof.iir_az;
    ndof.acc_oneG_status = acc_vector_length > ndof.acc_oneG_th_min && acc_vector_length < ndof.acc_oneG_th_max;
    // check if gyro is dynamic and shaked ? shook
    gyro_dyn_ref_idx = ndof.gyro_fifo_idx;
    gyro_dyn_ref_x = ndof.gx_fifo[gyro_dyn_ref_idx];
    gyro_dyn_ref_y = ndof.gy_fifo[gyro_dyn_ref_idx];
    gyro_dyn_ref_z = ndof.gz_fifo[gyro_dyn_ref_idx];
    gyro_dyn_cur_x = (short)ndof.raw_gx;
    gyro_dyn_cur_y = (short)ndof.raw_gy;
    gyro_dyn_cur_z = (short)ndof.raw_gz;
    AddXYZDataToFIFO(gyro_dyn_cur_x, gyro_dyn_cur_y, gyro_dyn_cur_z, ndof.gx_fifo, ndof.gy_fifo, ndof.gz_fifo, &ndof.gyro_fifo_idx, GYRO_FIFO_SIZE);
    // gx max&min
    if (gyro_dyn_cur_x <= ndof.gx_max) {
        if (gyro_dyn_ref_x == ndof.gx_max) {
            ndof.gx_max = GetMaxFromFIFO(ndof.gx_fifo, 0, GYRO_FIFO_SIZE);
        }
    } else {
        ndof.gx_max = gyro_dyn_cur_x;
    }
    if (gyro_dyn_cur_x >= ndof.gx_min) {
        if (gyro_dyn_ref_x == ndof.gx_min) {
            ndof.gx_min = GetMinFromFIFO(ndof.gx_fifo, 0, GYRO_FIFO_SIZE);
        }
    } else {
        ndof.gx_min = gyro_dyn_cur_x;
    }
    // gy max&min
    if (gyro_dyn_cur_y <= ndof.gy_max) {
        if (gyro_dyn_ref_y == ndof.gy_max) {
            ndof.gy_max = GetMaxFromFIFO(ndof.gy_fifo, 0, GYRO_FIFO_SIZE);
        }
    } else {
        ndof.gy_max = gyro_dyn_cur_y;
    }
    if (gyro_dyn_cur_y >= ndof.gy_min) {
        if (gyro_dyn_ref_y == ndof.gy_min) {
            ndof.gy_min = GetMinFromFIFO(ndof.gy_fifo, 0, GYRO_FIFO_SIZE);
        }
    } else {
        ndof.gy_min = gyro_dyn_cur_y;
    }
    // gz max&min
    if (gyro_dyn_cur_z <= ndof.gz_max) {
        if (gyro_dyn_ref_z == ndof.gz_max) {
            ndof.gz_max = GetMaxFromFIFO(ndof.gz_fifo, 0, GYRO_FIFO_SIZE);
        }
    } else {
        ndof.gz_max = gyro_dyn_cur_z;
    }
    if (gyro_dyn_cur_z >= ndof.gz_min) {
        if (gyro_dyn_ref_z == ndof.gz_min) {
            ndof.gz_min = GetMinFromFIFO(ndof.gz_fifo, 0, GYRO_FIFO_SIZE);
        }
    } else {
        ndof.gz_min = gyro_dyn_cur_z;
    }
    // diff of max,min of gx, gy, gz
    delta_x = (float)ndof.gx_max - (float)ndof.gx_min;
    delta_y = (float)ndof.gy_max - (float)ndof.gy_min;
    delta_z = (float)ndof.gz_max - (float)ndof.gz_min;
    if (fabsf(delta_x) >= (float)ndof.gyro_dyn_det_th
            || fabsf(delta_y) >= (float)ndof.gyro_dyn_det_th
            || fabsf(delta_z) >= (float)ndof.gyro_dyn_det_th) {
        ndof.gyro_dyn_det_count = 0;
    } else {
        ndof.gyro_dyn_det_count++;
        if (ndof.gyro_dyn_det_count >= 100) {
            ndof.gyro_dyn_det_count = 100;
        }
    }
    ndof.gyro_dyn_status = ndof.gyro_dyn_det_count < ndof.gyro_no_dyn_det_time;
    if (delta_x > (float)ndof.gyro_shake_det_th
            || delta_y > (float)ndof.gyro_shake_det_th
            || delta_z > (float)ndof.gyro_shake_det_th) {
        if (ndof.gyro_shake_det_count >= ndof.gyro_shake_det_time) {
            ndof.gyro_shake_det_count = ndof.gyro_shake_det_time + 1;
        } else {
            ndof.gyro_shake_det_count++;
        }
    } else {
        if (ndof.gyro_shake_det_count > 1) {
            ndof.gyro_shake_det_count--;
        }
    }
    ndof.gyro_shake_status = ndof.gyro_shake_det_count > ndof.gyro_shake_det_time;
    // calc gyro offset if it is still
    if (!ndof.acc_dyn_status && !ndof.gyro_dyn_status) {
        float gx_fifo_mean, gy_fifo_mean, gz_fifo_mean;
        short calc_mean_fifo_size = ndof.gyro_no_dyn_det_time;
        ndof.gyro_dyn_det_count = 0;
        if (calc_mean_fifo_size > GYRO_FIFO_SIZE) {
            calc_mean_fifo_size = GYRO_FIFO_SIZE;
        }
        gx_fifo_mean = GetMeanFromFIFO(ndof.gx_fifo, 0, calc_mean_fifo_size);
        gy_fifo_mean = GetMeanFromFIFO(ndof.gy_fifo, 0, calc_mean_fifo_size);
        gz_fifo_mean = GetMeanFromFIFO(ndof.gz_fifo, 0, calc_mean_fifo_size);
        if (fabsf(gx_fifo_mean) < (float)ndof.gyro_offset_th_max
                && fabsf(gy_fifo_mean) < (float)ndof.gyro_offset_th_max
                && fabsf(gz_fifo_mean) < (float)ndof.gyro_offset_th_max ) {
            if (!ndof.gyro_calibrated_accuracy) {
                ndof.iir_gx_bias = gx_fifo_mean;
                ndof.iir_gy_bias = gy_fifo_mean;
                ndof.iir_gz_bias = gz_fifo_mean;
                ndof.gyro_calibrated_accuracy = 1;
            } else {
                // use iir-filter 1ord to update gyro bias
                UpdateOffsetStepwise(&ndof.iir_gx_bias, gx_fifo_mean, ndof.gyro_offset_th_step);
                UpdateOffsetStepwise(&ndof.iir_gy_bias, gy_fifo_mean, ndof.gyro_offset_th_step);
                UpdateOffsetStepwise(&ndof.iir_gz_bias, gz_fifo_mean, ndof.gyro_offset_th_step);
                if (ndof.gyro_calibrated_accuracy < 3) {
                    ndof.gyro_calibrated_accuracy++;
                }
            }
            ndof.gx_bias = (short)Round(ndof.iir_gx_bias);
            ndof.gy_bias = (short)Round(ndof.iir_gy_bias);
            ndof.gz_bias = (short)Round(ndof.iir_gz_bias);
            ndof.gyro_is_calibrated = 1;
        }
    }
    // orentation quaternion
    if (!ndof.ori_shake_status) {
        ndof.ori_shake_status = ndof.gyro_shake_status;
    }
    if (ndof.gyro_is_calibrated) {
        if (ndof.ori_can_update) {
            float gyro_quat[4];
            float acc_quat[4];
            float tmp_quat[4];
            float cor_gx = ndof.cor_gx * 0.061f;
            float cor_gy = ndof.cor_gy * 0.061f;
            float cor_gz = ndof.cor_gz * 0.061f;
            UpdateOrientation(ndof.ori_quat, cor_gx, cor_gy, cor_gz, (ts - ndof.ts) / 1000.0f, 0.6f, gyro_quat);
            if (fabsf(cor_gx) > (float)ndof.ori_motion_threshold
                    || fabsf(cor_gy) > (float)ndof.ori_motion_threshold
                    || fabsf(cor_gz) > (float)ndof.ori_motion_threshold) {
                ndof.ori_motion_status = 1;
            } else {
                ndof.ori_motion_status = 0;
            }
            if (0/*!ndof.acc_dyn_status && ndof.acc_oneG_status*/) {
                float cor_gyro_vector_length;
                acc_quat[0] = 0.0f;
                acc_quat[1] = ndof.iir_ax * 0.001f;
                acc_quat[2] = ndof.iir_ay * 0.001f;
                acc_quat[3] = ndof.iir_az * 0.001f;
                QuatPointRot(gyro_quat, acc_quat, tmp_quat);
                QuatNormalization(tmp_quat, acc_quat);
                cor_gyro_vector_length = cor_gx * cor_gx + cor_gy * cor_gy + cor_gz * cor_gz;
                if (cor_gyro_vector_length < ndof.ori_acc_gyro_rate_threshold) {
                    if (fabs(acc_quat[1]) >= ndof.ori_acc_noise_threshold
                            || fabs(acc_quat[2]) >= ndof.ori_acc_noise_threshold
                            || ndof.ori_motion_status) {
                        ndof.ori_acc_coupling_timer = 0;
                    } else {
                        ndof.ori_acc_coupling_timer++;
                    }
                    if (ndof.ori_acc_coupling_timer <= ndof.ori_acc_coupling_timer_threshold) {
                        if (ndof.ori_acc_coupling_restart_flag != 1 && ndof.ori_shake_status != 1) {
                            float acc_coupling_value = (fabsf(acc_quat[1]) + fabsf(acc_quat[2])) * ndof.ori_acc_coupling_factor * (1.0f - cor_gyro_vector_length / ndof.ori_acc_gyro_rate_threshold);
                            if (acc_coupling_value >= ndof.ori_acc_coupling_threshold) {
                                acc_coupling_value = ndof.ori_acc_coupling_threshold;
                            }
                            tmp_quat[1] = sinf(acc_quat[2] * acc_coupling_value * 0.5f);
                            tmp_quat[2] = sinf(-acc_quat[1] * acc_coupling_value * 0.5f);
                            tmp_quat[0] = sqrtf(1.0f - tmp_quat[1] * tmp_quat[1] - tmp_quat[2] * tmp_quat[2]);
                            tmp_quat[3] = 0.0f;
                        } else {
                            float theta = acosf(acc_quat[3]) * 0.5f;
                            float sin_theta = sinf(theta);
                            tmp_quat[0] = acc_quat[0];
                            tmp_quat[1] = acc_quat[1];
                            tmp_quat[2] = acc_quat[2];
                            tmp_quat[3] = 0.0f;
                            QuatNormalization(tmp_quat, acc_quat);
                            tmp_quat[0] = cosf(theta);
                            tmp_quat[1] = acc_quat[2] * sin_theta;
                            tmp_quat[2] = -acc_quat[1] * sin_theta;
                            tmp_quat[3] = 0.0f;
                            ndof.ori_acc_coupling_restart_flag = 0;
                            ndof.ori_shake_status = 0;
                        }
                        QuatMultiplication(tmp_quat, gyro_quat, acc_quat);
                        gyro_quat[0] = acc_quat[0];
                        gyro_quat[1] = acc_quat[1];
                        gyro_quat[2] = acc_quat[2];
                        gyro_quat[3] = acc_quat[3];
                    }
                }
            }
            QuatNormalization(gyro_quat, ndof.ori_quat);
        } else {
            ndof.ori_can_update = 1;
        }
    }
    ndof.ts = ts;

    return NDOF_IsGyroCalibrated();
}

void NDOF_GetRawAccData(float *raw_ax, float *raw_ay, float *raw_az)
{
    *raw_ax = ndof.raw_ax * GRAVITY * 0.001f;
    *raw_ay = ndof.raw_ay * GRAVITY * 0.001f;
    *raw_az = ndof.raw_az * GRAVITY * 0.001f;
}

void NDOF_GetRawGyroData(float *raw_gx, float *raw_gy, float *raw_gz)
{
    *raw_gx = ndof.raw_gx * 0.061f;
    *raw_gy = ndof.raw_gy * 0.061f;
    *raw_gz = ndof.raw_gz * 0.061f;
}

int NDOF_GetCorAccData(float *cor_ax, float *cor_ay, float *cor_az)
{
    *cor_ax = ndof.cor_ax * GRAVITY * 0.001f;
    *cor_ay = ndof.cor_ay * GRAVITY * 0.001f;
    *cor_az = ndof.cor_az * GRAVITY * 0.001f;
    return NDOF_IsAccCalibrated();
}

int NDOF_GetFiltAccData(float *filt_ax, float *filt_ay, float *filt_az)
{
    *filt_ax = ndof.iir_ax * GRAVITY * 0.001f;
    *filt_ay = ndof.iir_ay * GRAVITY * 0.001f;
    *filt_az = ndof.iir_az * GRAVITY * 0.001f;
    return NDOF_IsAccCalibrated();
}

int NDOF_GetCorGyroData(float *cor_gx, float *cor_gy, float *cor_gz)
{
    *cor_gx = ndof.cor_gx * 0.061f;
    *cor_gy = ndof.cor_gy * 0.061f;
    *cor_gz = ndof.cor_gz * 0.061f;
    return NDOF_IsGyroCalibrated();
}

int NDOF_IsAccCalibrated(void)
{
    return ndof.acc_is_calibrated;
}

int NDOF_IsGyroCalibrated(void)
{
    return ndof.gyro_is_calibrated;
}

int NDOF_IsAccDynamic(void)
{
    return ndof.acc_dyn_status;
}

int NDOF_IsGyroDynamic(void)
{
    return ndof.gyro_dyn_status;
}

int NDOF_GetAccBias(float *ax_bias, float *ay_bias, float *az_bias)
{
    *ax_bias = ndof.ax_bias * GRAVITY * 0.001f;
    *ay_bias = ndof.ay_bias * GRAVITY * 0.001f;
    *az_bias = ndof.az_bias * GRAVITY * 0.001f;
    return NDOF_IsAccCalibrated();
}

int NDOF_GetGyroBias(float *gx_bias, float *gy_bias, float *gz_bias)
{
    *gx_bias = ndof.gx_bias * 0.061f;
    *gy_bias = ndof.gy_bias * 0.061f;
    *gz_bias = ndof.gz_bias * 0.061f;
    return NDOF_IsGyroCalibrated();
}

int NDOF_GetQuat(float *qw, float *qx, float *qy, float *qz)
{
    *qw = ndof.ori_quat[0];
    *qx = ndof.ori_quat[1];
    *qy = ndof.ori_quat[2];
    *qz = ndof.ori_quat[3];
    return NDOF_IsGyroCalibrated();
}

int NDOF_GetEulerAngle(float *roll, float *pitch, float *yaw)
{
    GetEulerAngle(ndof.ori_quat, roll, pitch, yaw);
    *roll *= 57.296f;
    *pitch *= 57.296f;
    *yaw *= 57.296f;
    return NDOF_IsGyroCalibrated();
}
