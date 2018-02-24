//_________________________________________________________________________________

// AHRS.c
// S.O.H. Madgwick
// 25th August 2010

// Description:

// Quaternion implementation of the 'DCM filter' [Mahoney et al].  Incorporates the magnetic distortion
// compensation algorithms from my filter [Madgwick] which eliminates the need for a reference
// direction of flux (bx bz) to be predefined and limits the effect of magnetic distortions to yaw
// a only.

// User must define 'dTOn2' as the (sample period / 2), and the filter gains 'MKp' and 'MKi'.

// Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elements representing the estimated
// orientation.  See my report for an overview of the use of quaternions in this application.

// User must call 'AHRSupdate()' every sample period and parse calibrated gyroscope ('gx', 'gy', 'gz'),
// accelerometer ('ax', 'ay', 'az') and magnetometer ('mx', 'my', 'mz') data.  Gyroscope units are
// radians/second, accelerometer and magnetometer units are irrelevant as the vector is normalised.
#include "GKEARHS.h"
//#include "quaternion.h"
#define dTOn2 1/512
#define dT 1/256

//void  MadgwickEulerAngles(int S);
const float MKp = 2.0;           // proportional gain governs rate of convergence to accelerometer/magnetometer
const float MKi = 1.0;          // integral gain governs rate of convergence of gyroscope biases // 0.005
float exInt = 0.0, eyInt = 0.0, ezInt = 0.0;   // scaled integral error
//float q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;  // quaternion elements representing the estimated orientation

void DoMadgwickAHRS(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {

	static int g;
	static float ReNorm;
	static float hx, hy, hz, bx2, bz2, mx2, my2, mz2;
	static float vx, vy, vz, wx, wy, wz;
	static float ex, ey, ez;
	static float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	if (1) {

		// auxiliary variables to reduce number of repeated operations
		q0q0 = q0*q0;
		q0q1 = q0*q1;
		q0q2 = q0*q2;
		q0q3 = q0*q3;
		q1q1 = q1*q1;
		q1q2 = q1*q2;
		q1q3 = q1*q3;
		q2q2 = q2*q2;
		q2q3 = q2*q3;
		q3q3 = q3*q3;

		ReNorm = 1.0 / sqrt(Sqr(mx) + Sqr(my) + Sqr(mz));
		mx *= ReNorm;
		my *= ReNorm;
		mz *= ReNorm;
		mx2 = 2.0*mx;
		my2 = 2.0*my;
		mz2 = 2.0*mz;

		// compute reference direction of flux
		hx = mx2*(0.5 - q2q2 - q3q3) + my2*(q1q2 - q0q3) + mz2*(q1q3 + q0q2);
		hy = mx2*(q1q2 + q0q3) + my2*(0.5 - q1q1 - q3q3) + mz2*(q2q3 - q0q1);
		hz = mx2*(q1q3 - q0q2) + my2*(q2q3 + q0q1) + mz2*(0.5 - q1q1 - q2q2);
		bx2 = 2.0*sqrt(Sqr(hx) + Sqr(hy));
		bz2 = 2.0*hz;

		// estimated direction of gravity and flux (v and w)
		vx = 2.0*(q1q3 - q0q2);
		vy = 2.0*(q0q1 + q2q3);
		vz = q0q0 - q1q1 - q2q2 + q3q3;

		wx = bx2*(0.5 - q2q2 - q3q3) + bz2*(q1q3 - q0q2);
		wy = bx2*(q1q2 - q0q3) + bz2*(q0q1 + q2q3);
		wz = bx2*(q0q2 + q1q3) + bz2*(0.5 - q1q1 - q2q2);

		// error is sum of cross product between reference direction of fields and direction measured by sensors
		ex = (ay*vz - az*vy) + (my*wz - mz*wy);
		ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
		ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

		// integral error scaled integral gain
		exInt += ex*MKi*dT;
		eyInt += ey*MKi*dT;
		ezInt += ez*MKi*dT;

		// adjusted gyroscope measurements
		gx += MKp*ex + exInt;
		gy += MKp*ey + eyInt;
		gz += MKp*ez + ezInt;

		// integrate quaternion rate and normalise
		q0 += (-q1*gx - q2*gy - q3*gz)*dTOn2;
		q1 += (q0*gx + q2*gz - q3*gy)*dTOn2;
		q2 += (q0*gy - q1*gz + q3*gx)*dTOn2;
		q3 += (q0*gz + q1*gy - q2*gx)*dTOn2;

		// normalise quaternion
		ReNorm = 1.0 / sqrt(Sqr(q0) + Sqr(q1) + Sqr(q2) + Sqr(q3));
		q0 *= ReNorm;
		q1 *= ReNorm;
		q2 *= ReNorm;
		q3 *= ReNorm;

		//MadgwickEulerAngles(MadgwickAHRS);

	}
	//else
	//	for (g = 0; g <(int)3; g++) {
	//		EstRate[g][MadgwickAHRS] = Gyro[g];
	//		EstAngle[g][MadgwickAHRS] += EstRate[g][MadgwickAHRS] * dT;
	//	}

} // DoMadgwickAHRS

//void  MadgwickEulerAngles(int S) {
//
//	EstAngle[Roll][S] = atan2(2.0*q2*q3 - 2.0*q0*q1, 2.0*Sqr(q0) + 2.0*Sqr(q3) - 1.0);
//	EstAngle[Pitch][S] = asin(2.0*q1*q2 - 2.0*q0*q2);
//	EstAngle[Yaw][S] = atan2(2.0*q1*q2 - 2.0*q0*q3, 2.0*Sqr(q0) + 2.0*Sqr(q1) - 1.0);
//
//} // MadgwickEulerAngles

float Sqr(float a)
{
	a *= a;
	return a;
}