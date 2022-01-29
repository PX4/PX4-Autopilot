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
    // Compare calculation of observation Jacobian for sympy and matlab generated equations

    // observation Jacobians
	float H_YAW[4];

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


    // calculate 321 yaw observation matrix using two computational paths to work around singularities
    // in calculation of the Jacobians.

    {
        // This first comparison is for the 321 sequence option A equations that have a singularity when
        // yaw is at +- 90 deg

        // perform calculation using sympy generated equations
        const float SA0 = 2*q3;
        const float SA1 = 2*q2;
        const float SA2 = SA0*q0 + SA1*q1;
        const float SA3 = powf(q0, 2) + powf(q1, 2) - powf(q2, 2) - powf(q3, 2);
        const float SA4 = powf(SA3, -2);
        const float SA5 = 1.0F/(powf(SA2, 2)*SA4 + 1);
        const float SA6 = 1.0F/SA3;
        const float SA7 = SA2*SA4;
        const float SA8 = 2*SA7;
        const float SA9 = 2*SA6;

        H_YAW[0] = SA5*(SA0*SA6 - SA8*q0);
        H_YAW[1] = SA5*(SA1*SA6 - SA8*q1);
        H_YAW[2] = SA5*(SA1*SA7 + SA9*q1);
        H_YAW[3] = SA5*(SA0*SA7 + SA9*q0);

        // save output and repeat calculation using legacy matlab generated code
        float H_YAW_sympy[4];
        for (int row=0; row<4; row++) {
            H_YAW_sympy[row] = H_YAW[row];
        }

        // repeat calculation using matlab generated equations
        float t9 = q0*q3;
        float t10 = q1*q2;
        float t2 = t9+t10;
        float t3 = q0*q0;
        float t4 = q1*q1;
        float t5 = q2*q2;
        float t6 = q3*q3;
        float t7 = t3+t4-t5-t6;
        float t16 = q3*t3;
        float t17 = q3*t5;
        float t18 = q0*q1*q2*2.0f;
        float t19 = t16+t17+t18-q3*t4+q3*t6;
        float t24 = q2*t4;
        float t25 = q2*t6;
        float t26 = q0*q1*q3*2.0f;
        float t27 = t24+t25+t26-q2*t3+q2*t5;
        float t28 = q1*t3;
        float t29 = q1*t5;
        float t30 = q0*q2*q3*2.0f;
        float t31 = t28+t29+t30+q1*t4-q1*t6;
        float t32 = q0*t4;
        float t33 = q0*t6;
        float t34 = q1*q2*q3*2.0f;
        float t35 = t32+t33+t34+q0*t3-q0*t5;
        float t8 = t7*t7;
        t8 = 1.0f/t8;
        float t11 = t2*t2;
        float t12 = t8*t11*4.0f;
        float t13 = t12+1.0f;
        float t14 = 1.0f/t13;

        H_YAW[0] = t8*t14*t19*(-2.0f);
        H_YAW[1] = t8*t14*t27*(-2.0f);
        H_YAW[2] = t8*t14*t31*2.0f;
        H_YAW[3] = t8*t14*t35*2.0f;

        // find largest difference as a fraction of the matlab value
        float max_diff_fraction = 0.0f;
        int max_row;
        float max_old, max_new;
        for (int row=0; row<4; row++) {
            float diff_fraction = fabsf(H_YAW_sympy[row] - H_YAW[row]) / fabsf(H_YAW[row]);
            if (diff_fraction > max_diff_fraction) {
                max_diff_fraction = diff_fraction;
                max_row = row;
                max_old = H_YAW[row];
                max_new = H_YAW_sympy[row];
            }
        }

        if (max_diff_fraction > 1e-5f) {
            printf("Fail: 321 yaw option A Hfusion max diff fraction = %e , old = %e , new = %e , location index = %i\n",max_diff_fraction, max_old, max_new, max_row);
        } else {
            printf("Pass: 321 yaw option A Hfusion max diff fraction = %e\n",max_diff_fraction);
        }

        // This second comparison for the 321 sequence option B equations that have a singularity when
        // yaw is at 0 and +-190 deg

        // perform calculation using sympy generated equations
        const float SB0 = 2*q0;
        const float SB1 = 2*q1;
        const float SB2 = SB0*q3 + SB1*q2;
        const float SB3 = powf(SB2, -2);
        const float SB4 = powf(q0, 2) + powf(q1, 2) - powf(q2, 2) - powf(q3, 2);
        const float SB5 = 1.0F/(SB3*powf(SB4, 2) + 1);
        const float SB6 = 1.0F/SB2;
        const float SB7 = SB3*SB4;
        const float SB8 = 2*SB7;
        const float SB9 = 2*SB6;

        H_YAW[0] = -SB5*(SB0*SB6 - SB8*q3);
        H_YAW[1] = -SB5*(SB1*SB6 - SB8*q2);
        H_YAW[2] = -SB5*(-SB1*SB7 - SB9*q2);
        H_YAW[3] = -SB5*(-SB0*SB7 - SB9*q3);

        // save output and repeat calculation using legacy matlab generated code
        for (int row=0; row<4; row++) {
            H_YAW_sympy[row] = H_YAW[row];
        }

        float t15 = t2*t2;
        t15 = 1.0f/t15;
        float t20 = t7*t7;
        float t21 = t15*t20*0.25f;
        float t22 = t21+1.0f;
        float t23 = 1.0f/t22;

        H_YAW[0] = t15*t19*t23*(-0.5f);
        H_YAW[1] = t15*t23*t27*(-0.5f);
        H_YAW[2] = t15*t23*t31*0.5f;
        H_YAW[3] = t15*t23*t35*0.5f;

        // find largest difference as a fraction of the matlab value
        max_diff_fraction = 0.0f;
        for (int row=0; row<4; row++) {
            float diff_fraction = fabsf(H_YAW_sympy[row] - H_YAW[row]) / fabsf(H_YAW[row]);
            if (diff_fraction > max_diff_fraction) {
                max_diff_fraction = diff_fraction;
                max_row = row;
                max_old = H_YAW[row];
                max_new = H_YAW_sympy[row];
            }
        }

        if (max_diff_fraction > 1e-5f) {
            printf("Fail: 321 yaw option B Hfusion max diff fraction = %e , old = %e , new = %e , location index = %i\n",max_diff_fraction, max_old, max_new, max_row);
        } else {
            printf("Pass: 321 yaw option B Hfusion max diff fraction = %e\n",max_diff_fraction);
        }
    }

    // calculate 312 yaw observation matrix using two computational paths to work around singularities
    // in calculation of the Jacobians.

    {
        // This first comparison is for the 312 sequence option A equations that have a singularity when
        // yaw is at +- 90 deg

        // perform calculation using sympy generated equations
        const float SA0 = 2*q3;
        const float SA1 = 2*q2;
        const float SA2 = SA0*q0 - SA1*q1;
        const float SA3 = powf(q0, 2) - powf(q1, 2) + powf(q2, 2) - powf(q3, 2);
        const float SA4 = powf(SA3, -2);
        const float SA5 = 1.0F/(powf(SA2, 2)*SA4 + 1);
        const float SA6 = 1.0F/SA3;
        const float SA7 = SA2*SA4;
        const float SA8 = 2*SA7;
        const float SA9 = 2*SA6;

        H_YAW[0] = SA5*(SA0*SA6 - SA8*q0);
        H_YAW[1] = SA5*(-SA1*SA6 + SA8*q1);
        H_YAW[2] = SA5*(-SA1*SA7 - SA9*q1);
        H_YAW[3] = SA5*(SA0*SA7 + SA9*q0);

        // save output and repeat calculation using legacy matlab generated code
        float H_YAW_sympy[4];
        for (int row=0; row<4; row++) {
            H_YAW_sympy[row] = H_YAW[row];
        }

        // repeat calculation using matlab generated equations
        float t9 = q0*q3;
        float t10 = q1*q2;
        float t2 = t9-t10;
        float t3 = q0*q0;
        float t4 = q1*q1;
        float t5 = q2*q2;
        float t6 = q3*q3;
        float t7 = t3-t4+t5-t6;
        float t16 = q3*t3;
        float t17 = q3*t4;
        float t18 = t16+t17-q3*t5+q3*t6-q0*q1*q2*2.0f;
        float t23 = q2*t3;
        float t24 = q2*t4;
        float t25 = t23+t24+q2*t5-q2*t6-q0*q1*q3*2.0f;
        float t26 = q1*t5;
        float t27 = q1*t6;
        float t28 = t26+t27-q1*t3+q1*t4-q0*q2*q3*2.0f;
        float t29 = q0*t5;
        float t30 = q0*t6;
        float t31 = t29+t30+q0*t3-q0*t4-q1*q2*q3*2.0f;
        float t8 = t7*t7;
        float t15 = t2*t2;
		t8 = 1.0f/t8;
		float t11 = t2*t2;
		float t12 = t8*t11*4.0f;
		float t13 = t12+1.0f;
		float t14 = 1.0f/t13;

		H_YAW[0] = t8*t14*t18*(-2.0f);
		H_YAW[1] = t8*t14*t25*(-2.0f);
		H_YAW[2] = t8*t14*t28*2.0f;
		H_YAW[3] = t8*t14*t31*2.0f;

        // find largest difference as a fraction of the matlab value
        float max_diff_fraction = 0.0f;
        int max_row;
        float max_old, max_new;
        for (int row=0; row<4; row++) {
            float diff_fraction = fabsf(H_YAW_sympy[row] - H_YAW[row]) / fabsf(H_YAW[row]);
            if (diff_fraction > max_diff_fraction) {
                max_diff_fraction = diff_fraction;
                max_row = row;
                max_old = H_YAW[row];
                max_new = H_YAW_sympy[row];
            }
        }

        if (max_diff_fraction > 1e-5f) {
            printf("Fail: 312 yaw option A Hfusion max diff fraction = %e , old = %e , new = %e , location index = %i\n",max_diff_fraction, max_old, max_new, max_row);
        } else {
            printf("Pass: 312 yaw option A Hfusion max diff fraction = %e\n",max_diff_fraction);
        }

        // This second comparison for the 321 sequence option B equations that have a singularity when
        // yaw is at 0 and +-190 deg

        // perform calculation using sympy generated equations
        const float SB0 = 2*q0;
        const float SB1 = 2*q1;
        const float SB2 = -SB0*q3 + SB1*q2;
        const float SB3 = powf(SB2, -2);
        const float SB4 = -powf(q0, 2) + powf(q1, 2) - powf(q2, 2) + powf(q3, 2);
        const float SB5 = 1.0F/(SB3*powf(SB4, 2) + 1);
        const float SB6 = 1.0F/SB2;
        const float SB7 = SB3*SB4;
        const float SB8 = 2*SB7;
        const float SB9 = 2*SB6;

        H_YAW[0] = -SB5*(-SB0*SB6 + SB8*q3);
        H_YAW[1] = -SB5*(SB1*SB6 - SB8*q2);
        H_YAW[2] = -SB5*(-SB1*SB7 - SB9*q2);
        H_YAW[3] = -SB5*(SB0*SB7 + SB9*q3);

        // save output and repeat calculation using legacy matlab generated code
        for (int row=0; row<4; row++) {
            H_YAW_sympy[row] = H_YAW[row];
        }

		t15 = 1.0f/t15;
		float t19 = t7*t7;
		float t20 = t15*t19*0.25f;
		float t21 = t20+1.0f;
		float t22 = 1.0f/t21;

        H_YAW[0] = t15*t18*t22*(-0.5f);
        H_YAW[1] = t15*t22*t25*(-0.5f);
        H_YAW[2] = t15*t22*t28*0.5f;
        H_YAW[3] = t15*t22*t31*0.5f;

        // find largest difference as a fraction of the matlab value
        max_diff_fraction = 0.0f;
        for (int row=0; row<4; row++) {
            float diff_fraction = fabsf(H_YAW_sympy[row] - H_YAW[row]) / fabsf(H_YAW[row]);
            if (diff_fraction > max_diff_fraction) {
                max_diff_fraction = diff_fraction;
                max_row = row;
                max_old = H_YAW[row];
                max_new = H_YAW_sympy[row];
            }
        }

        if (max_diff_fraction > 1e-5f) {
            printf("Fail: 312 yaw option B Hfusion max diff fraction = %e , old = %e , new = %e , location index = %i\n",max_diff_fraction, max_old, max_new, max_row);
        } else {
            printf("Pass: 312 yaw option B Hfusion max diff fraction = %e\n",max_diff_fraction);
        }
    }

    return 0;
}
