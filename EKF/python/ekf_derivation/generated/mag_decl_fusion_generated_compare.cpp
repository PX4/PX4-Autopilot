#include <math.h>
#include <stdio.h>
#include <cstdlib>
#include "../../../../../matrix/matrix/math.hpp"

typedef matrix::Vector<float, 24> Vector24f;
typedef matrix::SquareMatrix<float, 24> SquareMatrix24f;

float sq(float in) {
    return in * in;
}

int main()
{
    // Compare calculation of observation Jacobians and Kalman gains for sympy and matlab generated equations

	float Hfusion[24] = {};
    Vector24f H_DECL;
    Vector24f Kfusion;
    float decl_innov_var;

	const float R_DECL = sq(0.3f);

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

	const float magN = 0.04f;
	const float magE = -0.03f;

    const float h_field_min = 1e-3f;

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

	// Calculate intermediate variables
	const float magN_sq = sq(magN);
	if (magN_sq < sq(h_field_min)) {
		printf("bad numerical conditioning\n");
        return 0;
	}
	const float HK0 = 1.0F / magN_sq;
	const float HK1 = HK0*sq(magE) + 1.0F;
	const float HK2 = 1.0F/HK1;
	const float HK3 = 1.0F/magN;
	const float HK4 = HK2*HK3;
	const float HK5 = HK3*magE;
	const float HK6 = HK5*P(16,17) - P(17,17);
	const float HK7 = 1.0F/sq(HK1);
	const float HK8 = HK5*P(16,16) - P(16,17);
	const float innovation_variance = -HK0*HK6*HK7 + HK7*HK8*magE/(magN * magN_sq) + R_DECL;
	float HK9;
	if (innovation_variance > R_DECL) {
		HK9 = HK4/innovation_variance;
	} else {
		printf("bad numerical conditioning\n");
	}

	// Calculate the observation Jacobian
	// Note only 2 terms are non-zero which can be used in matrix operations for calculation of Kalman gains and covariance update to significantly reduce cost
	Hfusion[16] = -HK0*HK2*magE;
	Hfusion[17] = HK4;

	// Calculate the Kalman gains
	for (unsigned row = 0; row <= 15; row++) {
		Kfusion(row) = -HK9*(HK5*P(row,16) - P(row,17));
	}

	Kfusion(16) = -HK8*HK9;
	Kfusion(17) = -HK6*HK9;

	for (unsigned row = 18; row <= 23; row++) {
		Kfusion(row) = -HK9*(HK5*P(16,row) - P(17,row));
	}

    // save output and repeat calculation using legacy matlab generated code
    float Hfusion_sympy[24];
    Vector24f Kfusion_sympy;
    for (int row=0; row<24; row++) {
        Hfusion_sympy[row] = Hfusion[row];
        Kfusion_sympy(row) = Kfusion(row);
    }

    // repeat calculation using matlab generated equations
	// Calculate intermediate variables
	float t2 = magE*magE;
	float t3 = magN*magN;
	float t4 = t2+t3;
	// if the horizontal magnetic field is too small, this calculation will be badly conditioned
	if (t4 < h_field_min*h_field_min) {
		printf("bad numerical conditioning\n");
        return 0;
	}
	float t5 = P(16,16)*t2;
	float t6 = P(17,17)*t3;
	float t7 = t2*t2;
	float t8 = R_DECL*t7;
	float t9 = t3*t3;
	float t10 = R_DECL*t9;
	float t11 = R_DECL*t2*t3*2.0f;
	float t14 = P(16,17)*magE*magN;
	float t15 = P(17,16)*magE*magN;
	float t12 = t5+t6+t8+t10+t11-t14-t15;
	float t13;
	if (fabsf(t12) > 1e-6f) {
		t13 = 1.0f / t12;
	} else {
		printf("bad numerical conditioning\n");
        return 0;
	}
	float t18 = magE*magE;
	float t19 = magN*magN;
	float t20 = t18+t19;
	float t21;
	if (fabsf(t20) > 1e-6f) {
		t21 = 1.0f/t20;
	} else {
		printf("bad numerical conditioning\n");
        return 0;
	}

	// Calculate the observation Jacobian
	// Note only 2 terms are non-zero which can be used in matrix operations for calculation of Kalman gains and covariance update to significantly reduce cost
    memset(&H_DECL, 0, sizeof(H_DECL));
	H_DECL(16) = -magE*t21;
	H_DECL(17) = magN*t21;

	// Calculate the Kalman gains
	Kfusion(0) = -t4*t13*(P(0,16)*magE-P(0,17)*magN);
	Kfusion(1) = -t4*t13*(P(1,16)*magE-P(1,17)*magN);
	Kfusion(2) = -t4*t13*(P(2,16)*magE-P(2,17)*magN);
	Kfusion(3) = -t4*t13*(P(3,16)*magE-P(3,17)*magN);
	Kfusion(4) = -t4*t13*(P(4,16)*magE-P(4,17)*magN);
	Kfusion(5) = -t4*t13*(P(5,16)*magE-P(5,17)*magN);
	Kfusion(6) = -t4*t13*(P(6,16)*magE-P(6,17)*magN);
	Kfusion(7) = -t4*t13*(P(7,16)*magE-P(7,17)*magN);
	Kfusion(8) = -t4*t13*(P(8,16)*magE-P(8,17)*magN);
	Kfusion(9) = -t4*t13*(P(9,16)*magE-P(9,17)*magN);
	Kfusion(10) = -t4*t13*(P(10,16)*magE-P(10,17)*magN);
	Kfusion(11) = -t4*t13*(P(11,16)*magE-P(11,17)*magN);
	Kfusion(12) = -t4*t13*(P(12,16)*magE-P(12,17)*magN);
	Kfusion(13) = -t4*t13*(P(13,16)*magE-P(13,17)*magN);
	Kfusion(14) = -t4*t13*(P(14,16)*magE-P(14,17)*magN);
	Kfusion(15) = -t4*t13*(P(15,16)*magE-P(15,17)*magN);
	Kfusion(16) = -t4*t13*(P(16,16)*magE-P(16,17)*magN);
	Kfusion(17) = -t4*t13*(P(17,16)*magE-P(17,17)*magN);
	Kfusion(18) = -t4*t13*(P(18,16)*magE-P(18,17)*magN);
	Kfusion(19) = -t4*t13*(P(19,16)*magE-P(19,17)*magN);
	Kfusion(20) = -t4*t13*(P(20,16)*magE-P(20,17)*magN);
	Kfusion(21) = -t4*t13*(P(21,16)*magE-P(21,17)*magN);
	Kfusion(22) = -t4*t13*(P(22,16)*magE-P(22,17)*magN);
	Kfusion(23) = -t4*t13*(P(23,16)*magE-P(23,17)*magN);

    // find largest observation variance difference as a fraction of the matlab value
    float max_diff_fraction = 0.0f;
    int max_row;
    float max_old, max_new;
    for (int row=0; row<24; row++) {
        float diff_fraction;
        if (H_DECL(row) != 0.0f) {
            diff_fraction = fabsf(Hfusion_sympy[row] - H_DECL(row)) / fabsf(H_DECL(row));
        } else if (Hfusion_sympy[row] != 0.0f) {
            diff_fraction = fabsf(Hfusion_sympy[row] - H_DECL(row)) / fabsf(Hfusion_sympy[row]);
        } else {
            diff_fraction = 0.0f;
        }
        if (diff_fraction > max_diff_fraction) {
            max_diff_fraction = diff_fraction;
            max_row = row;
            max_old = H_DECL(row);
            max_new = Hfusion_sympy[row];
        }
    }

	if (max_diff_fraction > 1e-5f) {
		printf("Fail: Mag Declination Hfusion max diff fraction = %e , old = %e , new = %e , location index = %i\n",max_diff_fraction, max_old, max_new, max_row);
	} else {
		printf("Pass: Mag Declination Hfusion max diff fraction = %e\n",max_diff_fraction);
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
		printf("Fail: Mag Declination Kfusion max diff fraction = %e , old = %e , new = %e , location index = %i\n",max_diff_fraction, max_old, max_new, max_row);
	} else {
		printf("Pass: Mag Declination Kfusion max diff fraction = %e\n",max_diff_fraction);
	}

    return 0;
}
