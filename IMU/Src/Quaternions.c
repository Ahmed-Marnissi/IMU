
#include "Inc/Quaternions.h"


QuatFilterSel FilterSelecter=NONE ;



size_t NFilterIteration =1;

double DeltaT=0.025;
float Kp = 30.0;
float Ki = 0.0;

void UpdateQuaternion (float A[],float G[] , float M[],float Q[])
{
	/* 		newTime = micros();
            deltaT = newTime - oldTime;
            oldTime = newTime;
            deltaT = fabs(deltaT * 0.001 * 0.001);
            */

	switch (FilterSelecter)
	{
	case MADGWICK:
		Mdgwick();
		break;
	case MAHONY:
		Mahony(A,G,M,Q);
		break;
	default:
		NoFilter(A,G,M,Q);
		break;
	}
}

void NoFilter (float A[],float G[] , float M[],float Q[])
{
	// Madgwick function needs to be fed North, East, and Down direction like
	// (AN, AE, AD, GN, GE, GD, MN, ME, MD)
	// Accel and Gyro direction is Right-Hand, X-Forward, Z-Up
	// Magneto direction is Right-Hand, Y-Forward, Z-Down
	// So to adopt to the general Aircraft coordinate system (Right-Hand, X-Forward, Z-Down),
	// we need to feed (ax, -ay, -az, gx, -gy, -gz, my, -mx, mz)
	// but we pass (-ax, ay, az, gx, -gy, -gz, my, -mx, mz)
	// because gravity is by convention positive down, we need to ivnert the accel data

	// get quaternion based on aircraft coordinate (Right-Hand, X-Forward, Z-Down)
	// acc[mg], gyro[deg/s], mag [mG]
	// gyro will be convert from [deg/s] to [rad/s] inside of this function
	// quat_filter.update(-a[0], a[1], a[2], g[0] * DEG_TO_RAD, -g[1] * DEG_TO_RAD, -g[2] * DEG_TO_RAD, m[1], -m[0], m[2], q);

	float an = -A[0];
	float ae = +A[1];
	float ad = +A[2];
	float gn = +G[0] * (M_PI/180.0);
	float ge = -G[1] * (M_PI/180.0);
	float gd = -G[2] * (M_PI/180.0);
	float mn = +M[1];
	float me = -M[0];
	float md = +M[2];

	for (size_t i = 0; i < NFilterIteration; ++i)
	{
		float q0 = Q[0];
		float q1 = Q[1];
		float q2 = Q[2];
		float q3 = Q[3];


		Q[0] += 0.5f * (-q1 * gn - q2 * ge - q3 * gd) * DeltaT;
		Q[1] += 0.5f * (q0 * gn + q2 * gd - q3 * ge) * DeltaT;
		Q[2] += 0.5f * (q0 * ge - q1 * gd + q3 * gn) * DeltaT;
		Q[3] += 0.5f * (q0 * gd + q1 * ge - q2 * gn) * DeltaT;

		float  Norm = 1.0 / sqrtf(Q[0] * Q[0] + Q[1] * Q[1] + Q[2] * Q[2] + Q[3] * Q[3]);

		Q[0] *= Norm;
		Q[1] *= Norm;
		Q[2] *= Norm;
		Q[3] *= Norm;
	}



}

void Mdgwick()
{
}

void Mahony(float A[],float G[] , float M[],float Q[])
{
}




