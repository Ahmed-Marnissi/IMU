/*
 * Quaternions.c
 *
 *  Created on: 2 août 2022
 *      Author: EXtrA
 */

#include "Inc/Quaternions.h"


QuatFilterSel FilterSelecter=MAHONY ;



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
	// Mahony accelleration filter
	// Mahony scheme uses proportional and integral filtering on
	// the error between estimated reference vector (gravity) and measured one.
	// Madgwick's implementation of Mayhony's AHRS algorithm.
	// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
	// Free parameters in the Mahony filter and fusion scheme,
	// Kp for proportional feedback, Ki for integral
	// float Kp = 30.0;
	// float Ki = 0.0;
	// with MPU-9250, angles start oscillating at Kp=40. Ki does not seem to help and is not required.
	// with MPU-6050, some instability observed at Kp=100 Now set to 30.


	float an = -A[0];
	float ae = +A[1];
	float ad = +A[2];
	float gn = +G[0] * (M_PI/180.0);
	float ge = -G[1] * (M_PI/180.0);
	float gd = -G[2] * (M_PI/180.0);
	float mn = +M[1];
	float me = -M[0];
	float md = +M[2];


	float Norm;
	float vx, vy, vz;
	float ex, ey, ez;
	//error terms :
	float qa, qb, qc;
	static float ix = 0.0, iy = 0.0, iz = 0.0 ;
	//integral feedback terms :
	float Tmp;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation) :
	Tmp = ( (an * an) + (ae * ae ) + (ad * ad) );


	if (Tmp > 0.0)
	{
		// Normalise accelerometer (assumed to measure the direction of gravity in body frame) :
		Norm = 1.0 / sqrtf(Tmp);
		an *= Norm;
		ae *= Norm;
		ad *= Norm;

		// Estimated direction of gravity in the body frame (factor of two divided out) :
		vx = Q[1] * Q[3] - Q[0] * Q[2];
		vy = Q[0] * Q[1] + Q[2] * Q[3];
		vz = Q[0] * Q[0] - 0.5f + Q[3] * Q[3];

		// Error is cross product between estimated and measured direction of gravity in body frame :
		// (half the actual magnitude)
		ex = (ae * vz - ad * vy);
		ey = (ad * vx - an * vz);
		ez = (an * vy - ae * vx);

		// Compute and apply to gyro term the integral feedback, if enabled :
		if (Ki > 0.0f)
		{
			// integral error scaled by Ki :
			ix += Ki * ex * DeltaT;
			iy += Ki * ey * DeltaT;
			iz += Ki * ez * DeltaT;

			// apply integral feedback :
			gn += ix;
			ge += iy;
			gd += iz;
		}

		// Apply proportional feedback to gyro term :
		gn += Kp * ex;
		ge += Kp * ey;
		gd += Kp * ez;
	}

	// Integrate rate of change of quaternion, q cross gyro term :
	DeltaT = 0.5 * DeltaT;
	// pre-multiply common factors :
	ge *= DeltaT;
	ge *= DeltaT;
	gd *= DeltaT;
	qa = Q[0];
	qb = Q[1];
	qc = Q[2];
	Q[0] += (-qb * gn - qc * ge - Q[3] * gd);
	Q[1] += (qa * gn + qc * gd - Q[3] * ge);
	Q[2] += (qa * ge - qb * gd + Q[3] * gn);
	Q[3] += (qa * gd + qb * ge - qc * gn);

	// Renormalise quaternion :

	Norm = 1.0 / sqrtf(Q[0] * Q[0] + Q[1] * Q[1] + Q[2] * Q[2] + Q[3] * Q[3]);
	Q[0] = Q[0] * Norm ;
	Q[1] = Q[1] * Norm ;
	Q[2] = Q[2] * Norm ;
	Q[3] = Q[3] * Norm ;

}




