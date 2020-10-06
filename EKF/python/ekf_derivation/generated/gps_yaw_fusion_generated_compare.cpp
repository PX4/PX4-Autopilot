#include <math.h>
#include <stdio.h>
#include <cstdlib>
#include "../../../../../matrix/matrix/math.hpp"

typedef matrix::Vector<float, 24> Vector24f;
typedef matrix::SquareMatrix<float, 24> SquareMatrix24f;
template<int ... Idxs>
using SparseVector24f = matrix::SparseVectorf<24, Idxs...>;

float sq(float in) {
	return in * in;
}

int main()
{
	// Compare calculation of observation Jacobians and Kalman gains for sympy and matlab generated equations

	float H_YAW[24];
	Vector24f Kfusion;
	float _heading_innov_var;

	const float R_YAW = sq(0.3f);

	const float _gps_yaw_offset = 1.5f;

	// quaternion inputs must be normalised
	float q0 = 2.0f * ((float)rand() - 0.5f);
	float q1 = 2.0f * ((float)rand() - 0.5f);
	float q2 = 2.0f * ((float)rand() - 0.5f);
	float q3 = 2.0f * ((float)rand() - 0.5f);
	const float length = sqrtf(sq(q0) + sq(q1) + sq(q2) + sq(q3));
	q0 /= length;
	q1 /= length;
	q2 /= length;
	q3 /= length;

	// create a symmetrical positive dfinite matrix with off diagonals between -1 and 1 and diagonals between 0 and 1
	SquareMatrix24f P;
	for (int col=0; col<=23; col++) {
		for (int row=0; row<=col; row++) {
			if (row == col) {
				P(row,col) = (float)rand();
			} else {
				P(col,row) = P(row,col) = 2.0f * ((float)rand() - 0.5f);
			}
		}
	}

	// First calculate observationjacobians and Kalman gains using sympy generated equations

	// calculate intermediate variables
	const float HK0 = sinf(_gps_yaw_offset);
	const float HK1 = q0*q3;
	const float HK2 = q1*q2;
	const float HK3 = 2*HK0*(HK1 - HK2);
	const float HK4 = cosf(_gps_yaw_offset);
	const float HK5 = powf(q1, 2);
	const float HK6 = powf(q2, 2);
	const float HK7 = powf(q0, 2) - powf(q3, 2);
	const float HK8 = HK4*(HK5 - HK6 + HK7);
	const float HK9 = HK3 - HK8;
	if (fabsf(HK9) < 1e-3f) {
		return 0;
	}
	const float HK10 = 1.0F/HK9;
	const float HK11 = HK4*q0;
	const float HK12 = HK0*q3;
	const float HK13 = HK0*(-HK5 + HK6 + HK7) + 2*HK4*(HK1 + HK2);
	const float HK14 = HK10*HK13;
	const float HK15 = HK0*q0 + HK4*q3;
	const float HK16 = HK10*(HK14*(HK11 - HK12) + HK15);
	const float HK17 = powf(HK13, 2)/powf(HK9, 2) + 1;
	if (fabsf(HK17) < 1e-3f) {
		return 0;
	}
	const float HK18 = 2/HK17;
	// const float HK19 = 1.0F/(-HK3 + HK8);
	const float HK19_inverse = -HK3 + HK8;
	if (fabsf(HK19_inverse) < 1e-6f) {
		return 0;
	}
	const float HK19 = 1.0F/HK19_inverse;
	const float HK20 = HK4*q1;
	const float HK21 = HK0*q2;
	const float HK22 = HK13*HK19;
	const float HK23 = HK0*q1 - HK4*q2;
	const float HK24 = HK19*(HK22*(HK20 + HK21) + HK23);
	const float HK25 = HK19*(-HK20 - HK21 + HK22*HK23);
	const float HK26 = HK10*(-HK11 + HK12 + HK14*HK15);
	const float HK27 = -HK16*P(0,0) - HK24*P(0,1) - HK25*P(0,2) + HK26*P(0,3);
	const float HK28 = -HK16*P(0,1) - HK24*P(1,1) - HK25*P(1,2) + HK26*P(1,3);
	const float HK29 = 4/powf(HK17, 2);
	const float HK30 = -HK16*P(0,2) - HK24*P(1,2) - HK25*P(2,2) + HK26*P(2,3);
	const float HK31 = -HK16*P(0,3) - HK24*P(1,3) - HK25*P(2,3) + HK26*P(3,3);
	const float HK32 = HK18/(-HK16*HK27*HK29 - HK24*HK28*HK29 - HK25*HK29*HK30 + HK26*HK29*HK31 + R_YAW);

	// calculate observation jacobian
	// Observation jacobian and Kalman gain vectors
	SparseVector24f<0,1,2,3> Hfusion;
	Hfusion.at<0>() = -HK16*HK18;
	Hfusion.at<1>() = -HK18*HK24;
	Hfusion.at<2>() = -HK18*HK25;
	Hfusion.at<3>() = HK18*HK26;

	// calculate the Kalman gains
	// only calculate gains for states we are using
	Kfusion(0) = HK27*HK32;
	Kfusion(1) = HK28*HK32;
	Kfusion(2) = HK30*HK32;
	Kfusion(3) = HK31*HK32;
	for (unsigned row = 4; row <= 23; row++) {
		Kfusion(row) = HK32*(-HK16*P(0,row) - HK24*P(1,row) - HK25*P(2,row) + HK26*P(3,row));
	}

	// save output and repeat calculation using legacy matlab generated code
	float Hfusion_sympy[24] = {};
	Vector24f Kfusion_sympy;
	Hfusion_sympy[0] = Hfusion.at<0>();
	Hfusion_sympy[1] = Hfusion.at<1>();
	Hfusion_sympy[2] = Hfusion.at<2>();
	Hfusion_sympy[3] = Hfusion.at<3>();

	for (int row=0; row<24; row++) {
		Kfusion_sympy(row) = Kfusion(row);
	}

	// repeat calculation using matlab generated equations
	// calculate observation jacobian
	float t2 = sinf(_gps_yaw_offset);
	float t3 = cosf(_gps_yaw_offset);
	float t4 = q0*q3*2.0f;
	float t5 = q0*q0;
	float t6 = q1*q1;
	float t7 = q2*q2;
	float t8 = q3*q3;
	float t9 = q1*q2*2.0f;
	float t10 = t5+t6-t7-t8;
	float t11 = t3*t10;
	float t12 = t4+t9;
	float t13 = t3*t12;
	float t14 = t5-t6+t7-t8;
	float t15 = t2*t14;
	float t16 = t13+t15;
	float t17 = t4-t9;
	float t19 = t2*t17;
	float t20 = t11-t19;
	float t18 = (t20*t20);

	t18 = 1.0f / t18;
	float t21 = t16*t16;
	float t22 = sq(t11-t19);

	t22 = 1.0f/t22;
	float t23 = q1*t3*2.0f;
	float t24 = q2*t2*2.0f;
	float t25 = t23+t24;
	float t26 = 1.0f/t20;
	float t27 = q1*t2*2.0f;
	float t28 = t21*t22;
	float t29 = t28+1.0f;

	float t30 = 1.0f/t29;
	float t31 = q0*t3*2.0f;
	float t32 = t31-q3*t2*2.0f;
	float t33 = q3*t3*2.0f;
	float t34 = q0*t2*2.0f;
	float t35 = t33+t34;

	memset(&H_YAW, 0, sizeof(H_YAW));
	H_YAW[0] = (t35/(t11-t2*(t4-q1*q2*2.0f))-t16*t18*t32)/(t18*t21+1.0f);
	H_YAW[1] = -t30*(t26*(t27-q2*t3*2.0f)+t16*t22*t25);
	H_YAW[2] = t30*(t25*t26-t16*t22*(t27-q2*t3*2.0f));
	H_YAW[3] = t30*(t26*t32+t16*t22*t35);

	// Calculate innovation variance and Kalman gains, taking advantage of the fact that only the first 3 elements in H are non zero
	// calculate the innovation variance
	float PH[4];
	_heading_innov_var = R_YAW;

	for (unsigned row = 0; row <= 3; row++) {
		PH[row] = 0.0f;

		for (uint8_t col = 0; col <= 3; col++) {
			PH[row] += P(row,col) * H_YAW[col];
		}

		_heading_innov_var += H_YAW[row] * PH[row];
	}

	const float heading_innov_var_inv = 1.f / _heading_innov_var;

	// calculate the Kalman gains
	// only calculate gains for states we are using
	memset(&Kfusion, 0, sizeof(Kfusion));

	for (uint8_t row = 0; row <= 15; row++) {
		for (uint8_t col = 0; col <= 3; col++) {
			Kfusion(row) += P(row,col) * H_YAW[col];
		}
		Kfusion(row) *= heading_innov_var_inv;
	}

	if (true) {
		for (uint8_t row = 22; row <= 23; row++) {
			for (uint8_t col = 0; col <= 3; col++) {
				Kfusion(row) += P(row,col) * H_YAW[col];
			}
			Kfusion(row) *= heading_innov_var_inv;
		}
	}

	// find largest observation variance difference as a fraction of the matlab value
	float max_diff_fraction = 0.0f;
	int max_row;
	float max_old, max_new;
	for (int row=0; row<24; row++) {
		float diff_fraction;
		if (H_YAW[row] != 0.0f) {
			diff_fraction = fabsf(Hfusion_sympy[row] - H_YAW[row]) / fabsf(H_YAW[row]);
		} else if (Hfusion_sympy[row] != 0.0f) {
			diff_fraction = fabsf(Hfusion_sympy[row] - H_YAW[row]) / fabsf(Hfusion_sympy[row]);
		} else {
			diff_fraction = 0.0f;
		}
		if (diff_fraction > max_diff_fraction) {
			max_diff_fraction = diff_fraction;
			max_row = row;
			max_old = H_YAW[row];
			max_new = Hfusion_sympy[row];
		}
	}

	if (max_diff_fraction > 1e-5f) {
		printf("Fail: GPS yaw Hfusion max diff fraction = %e , old = %e , new = %e , location index = %i\n",max_diff_fraction, max_old, max_new, max_row);
	} else {
		printf("Pass: GPS yaw Hfusion max diff fraction = %e\n",max_diff_fraction);
	}

	// find largest Kalman gain difference as a fraction of the matlab value
	max_diff_fraction = 0.0f;
	for (int row=0; row<4; row++) {
		float diff_fraction;
		if (Kfusion(row) != 0.0f) {
			diff_fraction = fabsf(Kfusion_sympy(row) - Kfusion(row)) / fabsf(Kfusion(row));
		} else if (Kfusion_sympy(row) != 0.0f) {
			diff_fraction = fabsf(Kfusion_sympy(row) - Kfusion(row)) / fabsf(Kfusion_sympy(row));
		} else {
			diff_fraction = 0.0f;
		}
		if (diff_fraction > max_diff_fraction) {
			max_diff_fraction = diff_fraction;
			max_row = row;
			max_old = Kfusion(row);
			max_new = Kfusion_sympy(row);
		}
	}

	if (max_diff_fraction > 1e-5f) {
		printf("Fail: GPS yaw Kfusion max diff fraction = %e , old = %e , new = %e , location index = %i\n",max_diff_fraction, max_old, max_new, max_row);
	} else {
		printf("Pass: GPS yaw Kfusion max diff fraction = %e\n",max_diff_fraction);
	}

    return 0;
}
