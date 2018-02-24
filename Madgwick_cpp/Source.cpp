#include <iostream>
#include "MadgwickAHRS.h"
#include "Eigen/Dense"
#include "Eigen/Geometry"

using namespace std;
using namespace Eigen;

int main()
{
	float a[3] = { -0.3162f, 0.609f, 0.7267f };
	float g[3] = { 0.0f, 0.0f, 0.0f };

	for (int i = 0; i < 10000; i++) MadgwickAHRSupdateIMU(g[0], g[1], g[2], a[0], a[1], a[2]);
	cout << q0 << q1 << q2 << q3 << endl;

	//Quaternionf Quat = AHRS.getQuaternion();
	//cout << Quat.w() << " " << Quat.x() << " " << Quat.y() << " " << Quat.z() << endl;

	return 0;
}