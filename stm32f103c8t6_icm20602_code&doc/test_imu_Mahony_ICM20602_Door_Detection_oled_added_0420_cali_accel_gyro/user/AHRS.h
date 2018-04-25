#ifndef AHRS_h
#define AHRS_h
#define PI 3.14159265358979323846f
extern volatile float q0, q1, q2, q3; // quaternion of sensor frame relative to auxiliary frame
extern volatile float roll, pitch, yaw;
void MadgwickAHRSUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void MahonyAHRSUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void Quat2Angle(void);
#endif
