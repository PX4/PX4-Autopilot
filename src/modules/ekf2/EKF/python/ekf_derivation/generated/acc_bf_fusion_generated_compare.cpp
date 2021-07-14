#include <math.h>
#include <stdio.h>
#include <cstdlib>
#include "../../../../../matrix/matrix/math.hpp"
#include "util.h"

typedef matrix::Vector<float, 24> Vector24f;
typedef matrix::SquareMatrix<float, 24> SquareMatrix24f;
template<int ... Idxs>
using SparseVector24f = matrix::SparseVectorf<24, Idxs...>;

int main()
{
    // Compare calculation of observation Jacobians and Kalman gains for sympy and matlab generated equations

	SparseVector24f<0,1,2,3,4,5,6,22,23> Hfusion;
    Vector24f H_ACC;
    Vector24f Kfusion;
    float drag_innov_var;

    Vector24f Hfusion_sympy;
    Vector24f Kfusion_sympy;

    Vector24f Hfusion_matlab;
    Vector24f Kfusion_matlab;

	float SH_ACC[4] = {}; // Variable used to optimise calculations of measurement jacobian
	float SK_ACC[9] = {}; // Variable used to optimise calculations of the Kalman gain vector
	const float R_ACC = sq(2.5f); // observation noise variance in specific force drag (m/sec**2)**2

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

	// get latest velocity in earth frame
	const float vn = 8.0f;
	const float ve = 6.0f;
	const float vd = 1.0f;

	// get latest wind velocity in earth frame
	const float vwn = 4.0f;
	const float vwe = 3.0f;

	const float BC_inv_x = 1.0f / 35.0f;
	const float BC_inv_y = 1.0f / 25.0f;

    const float airSpd = 5.0f;

    const float rho = 1.225f;

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

    // Compare X axis equations
    {
        // Estimate the derivative of specific force wrt airspeed along the X axis
        // Limit lower value to prevent arithmetic exceptions
        const float Kaccx = fmaxf(1e-1f, rho * BC_inv_x * airSpd);

        // intermediate variables
        const float HK0 = vn - vwn;
        const float HK1 = ve - vwe;
        const float HK2 = HK0*q0 + HK1*q3 - q2*vd;
        const float HK3 = 2*Kaccx;
        const float HK4 = HK0*q1 + HK1*q2 + q3*vd;
        const float HK5 = HK0*q2 - HK1*q1 + q0*vd;
        const float HK6 = -HK0*q3 + HK1*q0 + q1*vd;
        const float HK7 = ecl::powf(q0, 2) + ecl::powf(q1, 2) - ecl::powf(q2, 2) - ecl::powf(q3, 2);
        const float HK8 = HK7*Kaccx;
        const float HK9 = q0*q3 + q1*q2;
        const float HK10 = HK3*HK9;
        const float HK11 = q0*q2 - q1*q3;
        const float HK12 = 2*HK9;
        const float HK13 = 2*HK11;
        const float HK14 = 2*HK4;
        const float HK15 = 2*HK2;
        const float HK16 = 2*HK5;
        const float HK17 = 2*HK6;
        const float HK18 = -HK12*P(0,23) + HK12*P(0,5) - HK13*P(0,6) + HK14*P(0,1) + HK15*P(0,0) - HK16*P(0,2) + HK17*P(0,3) - HK7*P(0,22) + HK7*P(0,4);
        const float HK19 = HK12*P(5,23);
        const float HK20 = -HK12*P(23,23) - HK13*P(6,23) + HK14*P(1,23) + HK15*P(0,23) - HK16*P(2,23) + HK17*P(3,23) + HK19 - HK7*P(22,23) + HK7*P(4,23);
        const float HK21 = ecl::powf(Kaccx, 2);
        const float HK22 = HK12*HK21;
        const float HK23 = HK12*P(5,5) - HK13*P(5,6) + HK14*P(1,5) + HK15*P(0,5) - HK16*P(2,5) + HK17*P(3,5) - HK19 + HK7*P(4,5) - HK7*P(5,22);
        const float HK24 = HK12*P(5,6) - HK12*P(6,23) - HK13*P(6,6) + HK14*P(1,6) + HK15*P(0,6) - HK16*P(2,6) + HK17*P(3,6) + HK7*P(4,6) - HK7*P(6,22);
        const float HK25 = HK7*P(4,22);
        const float HK26 = -HK12*P(4,23) + HK12*P(4,5) - HK13*P(4,6) + HK14*P(1,4) + HK15*P(0,4) - HK16*P(2,4) + HK17*P(3,4) - HK25 + HK7*P(4,4);
        const float HK27 = HK21*HK7;
        const float HK28 = -HK12*P(22,23) + HK12*P(5,22) - HK13*P(6,22) + HK14*P(1,22) + HK15*P(0,22) - HK16*P(2,22) + HK17*P(3,22) + HK25 - HK7*P(22,22);
        const float HK29 = -HK12*P(1,23) + HK12*P(1,5) - HK13*P(1,6) + HK14*P(1,1) + HK15*P(0,1) - HK16*P(1,2) + HK17*P(1,3) - HK7*P(1,22) + HK7*P(1,4);
        const float HK30 = -HK12*P(2,23) + HK12*P(2,5) - HK13*P(2,6) + HK14*P(1,2) + HK15*P(0,2) - HK16*P(2,2) + HK17*P(2,3) - HK7*P(2,22) + HK7*P(2,4);
        const float HK31 = -HK12*P(3,23) + HK12*P(3,5) - HK13*P(3,6) + HK14*P(1,3) + HK15*P(0,3) - HK16*P(2,3) + HK17*P(3,3) - HK7*P(3,22) + HK7*P(3,4);
        const float HK32 = Kaccx/(-HK13*HK21*HK24 + HK14*HK21*HK29 + HK15*HK18*HK21 - HK16*HK21*HK30 + HK17*HK21*HK31 - HK20*HK22 + HK22*HK23 + HK26*HK27 - HK27*HK28 + R_ACC);

        // Observation Jacobians
        Hfusion.at<0>() = -HK2*HK3;
        Hfusion.at<1>() = -HK3*HK4;
        Hfusion.at<2>() = HK3*HK5;
        Hfusion.at<3>() = -HK3*HK6;
        Hfusion.at<4>() = -HK8;
        Hfusion.at<5>() = -HK10;
        Hfusion.at<6>() = HK11*HK3;
        Hfusion.at<22>() = HK8;
        Hfusion.at<23>() = HK10;

        // Kalman gains
        Kfusion(0) = -HK18*HK32;
        Kfusion(1) = -HK29*HK32;
        Kfusion(2) = -HK30*HK32;
        Kfusion(3) = -HK31*HK32;
        Kfusion(4) = -HK26*HK32;
        Kfusion(5) = -HK23*HK32;
        Kfusion(6) = -HK24*HK32;
        Kfusion(7) = -HK32*(HK12*P(5,7) - HK12*P(7,23) - HK13*P(6,7) + HK14*P(1,7) + HK15*P(0,7) - HK16*P(2,7) + HK17*P(3,7) + HK7*P(4,7) - HK7*P(7,22));
        Kfusion(8) = -HK32*(HK12*P(5,8) - HK12*P(8,23) - HK13*P(6,8) + HK14*P(1,8) + HK15*P(0,8) - HK16*P(2,8) + HK17*P(3,8) + HK7*P(4,8) - HK7*P(8,22));
        Kfusion(9) = -HK32*(HK12*P(5,9) - HK12*P(9,23) - HK13*P(6,9) + HK14*P(1,9) + HK15*P(0,9) - HK16*P(2,9) + HK17*P(3,9) + HK7*P(4,9) - HK7*P(9,22));
        Kfusion(10) = -HK32*(-HK12*P(10,23) + HK12*P(5,10) - HK13*P(6,10) + HK14*P(1,10) + HK15*P(0,10) - HK16*P(2,10) + HK17*P(3,10) - HK7*P(10,22) + HK7*P(4,10));
        Kfusion(11) = -HK32*(-HK12*P(11,23) + HK12*P(5,11) - HK13*P(6,11) + HK14*P(1,11) + HK15*P(0,11) - HK16*P(2,11) + HK17*P(3,11) - HK7*P(11,22) + HK7*P(4,11));
        Kfusion(12) = -HK32*(-HK12*P(12,23) + HK12*P(5,12) - HK13*P(6,12) + HK14*P(1,12) + HK15*P(0,12) - HK16*P(2,12) + HK17*P(3,12) - HK7*P(12,22) + HK7*P(4,12));
        Kfusion(13) = -HK32*(-HK12*P(13,23) + HK12*P(5,13) - HK13*P(6,13) + HK14*P(1,13) + HK15*P(0,13) - HK16*P(2,13) + HK17*P(3,13) - HK7*P(13,22) + HK7*P(4,13));
        Kfusion(14) = -HK32*(-HK12*P(14,23) + HK12*P(5,14) - HK13*P(6,14) + HK14*P(1,14) + HK15*P(0,14) - HK16*P(2,14) + HK17*P(3,14) - HK7*P(14,22) + HK7*P(4,14));
        Kfusion(15) = -HK32*(-HK12*P(15,23) + HK12*P(5,15) - HK13*P(6,15) + HK14*P(1,15) + HK15*P(0,15) - HK16*P(2,15) + HK17*P(3,15) - HK7*P(15,22) + HK7*P(4,15));
        Kfusion(16) = -HK32*(-HK12*P(16,23) + HK12*P(5,16) - HK13*P(6,16) + HK14*P(1,16) + HK15*P(0,16) - HK16*P(2,16) + HK17*P(3,16) - HK7*P(16,22) + HK7*P(4,16));
        Kfusion(17) = -HK32*(-HK12*P(17,23) + HK12*P(5,17) - HK13*P(6,17) + HK14*P(1,17) + HK15*P(0,17) - HK16*P(2,17) + HK17*P(3,17) - HK7*P(17,22) + HK7*P(4,17));
        Kfusion(18) = -HK32*(-HK12*P(18,23) + HK12*P(5,18) - HK13*P(6,18) + HK14*P(1,18) + HK15*P(0,18) - HK16*P(2,18) + HK17*P(3,18) - HK7*P(18,22) + HK7*P(4,18));
        Kfusion(19) = -HK32*(-HK12*P(19,23) + HK12*P(5,19) - HK13*P(6,19) + HK14*P(1,19) + HK15*P(0,19) - HK16*P(2,19) + HK17*P(3,19) - HK7*P(19,22) + HK7*P(4,19));
        Kfusion(20) = -HK32*(-HK12*P(20,23) + HK12*P(5,20) - HK13*P(6,20) + HK14*P(1,20) + HK15*P(0,20) - HK16*P(2,20) + HK17*P(3,20) - HK7*P(20,22) + HK7*P(4,20));
        Kfusion(21) = -HK32*(-HK12*P(21,23) + HK12*P(5,21) - HK13*P(6,21) + HK14*P(1,21) + HK15*P(0,21) - HK16*P(2,21) + HK17*P(3,21) - HK7*P(21,22) + HK7*P(4,21));
        Kfusion(22) = -HK28*HK32;
        Kfusion(23) = -HK20*HK32;

        // save output
        Hfusion_sympy(0) = Hfusion.at<0>();
        Hfusion_sympy(1) = Hfusion.at<1>();
        Hfusion_sympy(2) = Hfusion.at<2>();
        Hfusion_sympy(3) = Hfusion.at<3>();
        Hfusion_sympy(4) = Hfusion.at<4>();
        Hfusion_sympy(5) = Hfusion.at<5>();
        Hfusion_sympy(6) = Hfusion.at<6>();
        Hfusion_sympy(22) = Hfusion.at<22>();
        Hfusion_sympy(23) = Hfusion.at<23>();
        Kfusion_sympy = Kfusion;

        // repeat calculation using matlab generated equations

        const float Kacc = Kaccx;

        // Estimate the derivative of specific force wrt airspeed along the X axis
        SH_ACC[0] = sq(q0) + sq(q1) - sq(q2) - sq(q3);
        SH_ACC[1] = vn - vwn;
        SH_ACC[2] = ve - vwe;
        SH_ACC[3] = 2.0f*q0*q3 + 2.0f*q1*q2;

        H_ACC.setZero();
        H_ACC(0) = -Kacc*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd);
        H_ACC(1) = -Kacc*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd);
        H_ACC(2) = Kacc*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd);
        H_ACC(3) = -Kacc*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd);
        H_ACC(4) = -Kacc*SH_ACC[0];
        H_ACC(5) = -Kacc*SH_ACC[3];
        H_ACC(6) = Kacc*(2.0f*q0*q2 - 2.0f*q1*q3);
        H_ACC(22) = Kacc*SH_ACC[0];
        H_ACC(23) = Kacc*SH_ACC[3];

        drag_innov_var = (R_ACC + Kacc*SH_ACC[0]*(Kacc*P(4,4)*SH_ACC[0] + Kacc*P(5,4)*SH_ACC[3] - Kacc*P(22,4)*SH_ACC[0] - Kacc*P(23,4)*SH_ACC[3] - Kacc*P(6,4)*(2.0f*q0*q2 - 2.0f*q1*q3) + Kacc*P(0,4)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd) + Kacc*P(1,4)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(2,4)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(3,4)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd)) + Kacc*SH_ACC[3]*(Kacc*P(4,5)*SH_ACC[0] + Kacc*P(5,5)*SH_ACC[3] - Kacc*P(22,5)*SH_ACC[0] - Kacc*P(23,5)*SH_ACC[3] - Kacc*P(6,5)*(2.0f*q0*q2 - 2.0f*q1*q3) + Kacc*P(0,5)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd) + Kacc*P(1,5)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(2,5)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(3,5)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd)) - Kacc*SH_ACC[0]*(Kacc*P(4,22)*SH_ACC[0] + Kacc*P(5,22)*SH_ACC[3] - Kacc*P(22,22)*SH_ACC[0] - Kacc*P(23,22)*SH_ACC[3] - Kacc*P(6,22)*(2.0f*q0*q2 - 2.0f*q1*q3) + Kacc*P(0,22)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd) + Kacc*P(1,22)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(2,22)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(3,22)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd)) - Kacc*SH_ACC[3]*(Kacc*P(4,23)*SH_ACC[0] + Kacc*P(5,23)*SH_ACC[3] - Kacc*P(22,23)*SH_ACC[0] - Kacc*P(23,23)*SH_ACC[3] - Kacc*P(6,23)*(2.0f*q0*q2 - 2.0f*q1*q3) + Kacc*P(0,23)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd) + Kacc*P(1,23)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(2,23)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(3,23)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd)) - Kacc*(2.0f*q0*q2 - 2.0f*q1*q3)*(Kacc*P(4,6)*SH_ACC[0] + Kacc*P(5,6)*SH_ACC[3] - Kacc*P(22,6)*SH_ACC[0] - Kacc*P(23,6)*SH_ACC[3] - Kacc*P(6,6)*(2.0f*q0*q2 - 2.0f*q1*q3) + Kacc*P(0,6)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd) + Kacc*P(1,6)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(2,6)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(3,6)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd)) + Kacc*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd)*(Kacc*P(4,0)*SH_ACC[0] + Kacc*P(5,0)*SH_ACC[3] - Kacc*P(22,0)*SH_ACC[0] - Kacc*P(23,0)*SH_ACC[3] - Kacc*P(6,0)*(2.0f*q0*q2 - 2.0f*q1*q3) + Kacc*P(0,0)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd) + Kacc*P(1,0)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(2,0)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(3,0)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd)) + Kacc*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd)*(Kacc*P(4,1)*SH_ACC[0] + Kacc*P(5,1)*SH_ACC[3] - Kacc*P(22,1)*SH_ACC[0] - Kacc*P(23,1)*SH_ACC[3] - Kacc*P(6,1)*(2.0f*q0*q2 - 2.0f*q1*q3) + Kacc*P(0,1)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd) + Kacc*P(1,1)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(2,1)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(3,1)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd)) - Kacc*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd)*(Kacc*P(4,2)*SH_ACC[0] + Kacc*P(5,2)*SH_ACC[3] - Kacc*P(22,2)*SH_ACC[0] - Kacc*P(23,2)*SH_ACC[3] - Kacc*P(6,2)*(2.0f*q0*q2 - 2.0f*q1*q3) + Kacc*P(0,2)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd) + Kacc*P(1,2)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(2,2)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(3,2)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd)) + Kacc*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd)*(Kacc*P(4,3)*SH_ACC[0] + Kacc*P(5,3)*SH_ACC[3] - Kacc*P(22,3)*SH_ACC[0] - Kacc*P(23,3)*SH_ACC[3] - Kacc*P(6,3)*(2.0f*q0*q2 - 2.0f*q1*q3) + Kacc*P(0,3)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd) + Kacc*P(1,3)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(2,3)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(3,3)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd)));

        SK_ACC[0] = 1.0f/drag_innov_var;
        SK_ACC[1] = 2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd;
        SK_ACC[2] = 2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd;
        SK_ACC[3] = 2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd;
        SK_ACC[4] = 2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd;
        SK_ACC[5] = 2.0f*q0*q2 - 2.0f*q1*q3;
        SK_ACC[6] = SH_ACC[3];

        Kfusion(0) = -SK_ACC[0]*(Kacc*P(0,4)*SH_ACC[0] - Kacc*P(0,22)*SH_ACC[0] + Kacc*P(0,0)*SK_ACC[3] - Kacc*P(0,2)*SK_ACC[2] + Kacc*P(0,3)*SK_ACC[1] + Kacc*P(0,1)*SK_ACC[4] + Kacc*P(0,5)*SK_ACC[6] - Kacc*P(0,6)*SK_ACC[5] - Kacc*P(0,23)*SK_ACC[6]);
        Kfusion(1) = -SK_ACC[0]*(Kacc*P(1,4)*SH_ACC[0] - Kacc*P(1,22)*SH_ACC[0] + Kacc*P(1,0)*SK_ACC[3] - Kacc*P(1,2)*SK_ACC[2] + Kacc*P(1,3)*SK_ACC[1] + Kacc*P(1,1)*SK_ACC[4] + Kacc*P(1,5)*SK_ACC[6] - Kacc*P(1,6)*SK_ACC[5] - Kacc*P(1,23)*SK_ACC[6]);
        Kfusion(2) = -SK_ACC[0]*(Kacc*P(2,4)*SH_ACC[0] - Kacc*P(2,22)*SH_ACC[0] + Kacc*P(2,0)*SK_ACC[3] - Kacc*P(2,2)*SK_ACC[2] + Kacc*P(2,3)*SK_ACC[1] + Kacc*P(2,1)*SK_ACC[4] + Kacc*P(2,5)*SK_ACC[6] - Kacc*P(2,6)*SK_ACC[5] - Kacc*P(2,23)*SK_ACC[6]);
        Kfusion(3) = -SK_ACC[0]*(Kacc*P(3,4)*SH_ACC[0] - Kacc*P(3,22)*SH_ACC[0] + Kacc*P(3,0)*SK_ACC[3] - Kacc*P(3,2)*SK_ACC[2] + Kacc*P(3,3)*SK_ACC[1] + Kacc*P(3,1)*SK_ACC[4] + Kacc*P(3,5)*SK_ACC[6] - Kacc*P(3,6)*SK_ACC[5] - Kacc*P(3,23)*SK_ACC[6]);
        Kfusion(4) = -SK_ACC[0]*(Kacc*P(4,4)*SH_ACC[0] - Kacc*P(4,22)*SH_ACC[0] + Kacc*P(4,0)*SK_ACC[3] - Kacc*P(4,2)*SK_ACC[2] + Kacc*P(4,3)*SK_ACC[1] + Kacc*P(4,1)*SK_ACC[4] + Kacc*P(4,5)*SK_ACC[6] - Kacc*P(4,6)*SK_ACC[5] - Kacc*P(4,23)*SK_ACC[6]);
        Kfusion(5) = -SK_ACC[0]*(Kacc*P(5,4)*SH_ACC[0] - Kacc*P(5,22)*SH_ACC[0] + Kacc*P(5,0)*SK_ACC[3] - Kacc*P(5,2)*SK_ACC[2] + Kacc*P(5,3)*SK_ACC[1] + Kacc*P(5,1)*SK_ACC[4] + Kacc*P(5,5)*SK_ACC[6] - Kacc*P(5,6)*SK_ACC[5] - Kacc*P(5,23)*SK_ACC[6]);
        Kfusion(6) = -SK_ACC[0]*(Kacc*P(6,4)*SH_ACC[0] - Kacc*P(6,22)*SH_ACC[0] + Kacc*P(6,0)*SK_ACC[3] - Kacc*P(6,2)*SK_ACC[2] + Kacc*P(6,3)*SK_ACC[1] + Kacc*P(6,1)*SK_ACC[4] + Kacc*P(6,5)*SK_ACC[6] - Kacc*P(6,6)*SK_ACC[5] - Kacc*P(6,23)*SK_ACC[6]);
        Kfusion(7) = -SK_ACC[0]*(Kacc*P(7,4)*SH_ACC[0] - Kacc*P(7,22)*SH_ACC[0] + Kacc*P(7,0)*SK_ACC[3] - Kacc*P(7,2)*SK_ACC[2] + Kacc*P(7,3)*SK_ACC[1] + Kacc*P(7,1)*SK_ACC[4] + Kacc*P(7,5)*SK_ACC[6] - Kacc*P(7,6)*SK_ACC[5] - Kacc*P(7,23)*SK_ACC[6]);
        Kfusion(8) = -SK_ACC[0]*(Kacc*P(8,4)*SH_ACC[0] - Kacc*P(8,22)*SH_ACC[0] + Kacc*P(8,0)*SK_ACC[3] - Kacc*P(8,2)*SK_ACC[2] + Kacc*P(8,3)*SK_ACC[1] + Kacc*P(8,1)*SK_ACC[4] + Kacc*P(8,5)*SK_ACC[6] - Kacc*P(8,6)*SK_ACC[5] - Kacc*P(8,23)*SK_ACC[6]);
        Kfusion(9) = -SK_ACC[0]*(Kacc*P(9,4)*SH_ACC[0] - Kacc*P(9,22)*SH_ACC[0] + Kacc*P(9,0)*SK_ACC[3] - Kacc*P(9,2)*SK_ACC[2] + Kacc*P(9,3)*SK_ACC[1] + Kacc*P(9,1)*SK_ACC[4] + Kacc*P(9,5)*SK_ACC[6] - Kacc*P(9,6)*SK_ACC[5] - Kacc*P(9,23)*SK_ACC[6]);
        Kfusion(10) = -SK_ACC[0]*(Kacc*P(10,4)*SH_ACC[0] - Kacc*P(10,22)*SH_ACC[0] + Kacc*P(10,0)*SK_ACC[3] - Kacc*P(10,2)*SK_ACC[2] + Kacc*P(10,3)*SK_ACC[1] + Kacc*P(10,1)*SK_ACC[4] + Kacc*P(10,5)*SK_ACC[6] - Kacc*P(10,6)*SK_ACC[5] - Kacc*P(10,23)*SK_ACC[6]);
        Kfusion(11) = -SK_ACC[0]*(Kacc*P(11,4)*SH_ACC[0] - Kacc*P(11,22)*SH_ACC[0] + Kacc*P(11,0)*SK_ACC[3] - Kacc*P(11,2)*SK_ACC[2] + Kacc*P(11,3)*SK_ACC[1] + Kacc*P(11,1)*SK_ACC[4] + Kacc*P(11,5)*SK_ACC[6] - Kacc*P(11,6)*SK_ACC[5] - Kacc*P(11,23)*SK_ACC[6]);
        Kfusion(12) = -SK_ACC[0]*(Kacc*P(12,4)*SH_ACC[0] - Kacc*P(12,22)*SH_ACC[0] + Kacc*P(12,0)*SK_ACC[3] - Kacc*P(12,2)*SK_ACC[2] + Kacc*P(12,3)*SK_ACC[1] + Kacc*P(12,1)*SK_ACC[4] + Kacc*P(12,5)*SK_ACC[6] - Kacc*P(12,6)*SK_ACC[5] - Kacc*P(12,23)*SK_ACC[6]);
        Kfusion(13) = -SK_ACC[0]*(Kacc*P(13,4)*SH_ACC[0] - Kacc*P(13,22)*SH_ACC[0] + Kacc*P(13,0)*SK_ACC[3] - Kacc*P(13,2)*SK_ACC[2] + Kacc*P(13,3)*SK_ACC[1] + Kacc*P(13,1)*SK_ACC[4] + Kacc*P(13,5)*SK_ACC[6] - Kacc*P(13,6)*SK_ACC[5] - Kacc*P(13,23)*SK_ACC[6]);
        Kfusion(14) = -SK_ACC[0]*(Kacc*P(14,4)*SH_ACC[0] - Kacc*P(14,22)*SH_ACC[0] + Kacc*P(14,0)*SK_ACC[3] - Kacc*P(14,2)*SK_ACC[2] + Kacc*P(14,3)*SK_ACC[1] + Kacc*P(14,1)*SK_ACC[4] + Kacc*P(14,5)*SK_ACC[6] - Kacc*P(14,6)*SK_ACC[5] - Kacc*P(14,23)*SK_ACC[6]);
        Kfusion(15) = -SK_ACC[0]*(Kacc*P(15,4)*SH_ACC[0] - Kacc*P(15,22)*SH_ACC[0] + Kacc*P(15,0)*SK_ACC[3] - Kacc*P(15,2)*SK_ACC[2] + Kacc*P(15,3)*SK_ACC[1] + Kacc*P(15,1)*SK_ACC[4] + Kacc*P(15,5)*SK_ACC[6] - Kacc*P(15,6)*SK_ACC[5] - Kacc*P(15,23)*SK_ACC[6]);
        Kfusion(16) = -SK_ACC[0]*(Kacc*P(16,4)*SH_ACC[0] - Kacc*P(16,22)*SH_ACC[0] + Kacc*P(16,0)*SK_ACC[3] - Kacc*P(16,2)*SK_ACC[2] + Kacc*P(16,3)*SK_ACC[1] + Kacc*P(16,1)*SK_ACC[4] + Kacc*P(16,5)*SK_ACC[6] - Kacc*P(16,6)*SK_ACC[5] - Kacc*P(16,23)*SK_ACC[6]);
        Kfusion(17) = -SK_ACC[0]*(Kacc*P(17,4)*SH_ACC[0] - Kacc*P(17,22)*SH_ACC[0] + Kacc*P(17,0)*SK_ACC[3] - Kacc*P(17,2)*SK_ACC[2] + Kacc*P(17,3)*SK_ACC[1] + Kacc*P(17,1)*SK_ACC[4] + Kacc*P(17,5)*SK_ACC[6] - Kacc*P(17,6)*SK_ACC[5] - Kacc*P(17,23)*SK_ACC[6]);
        Kfusion(18) = -SK_ACC[0]*(Kacc*P(18,4)*SH_ACC[0] - Kacc*P(18,22)*SH_ACC[0] + Kacc*P(18,0)*SK_ACC[3] - Kacc*P(18,2)*SK_ACC[2] + Kacc*P(18,3)*SK_ACC[1] + Kacc*P(18,1)*SK_ACC[4] + Kacc*P(18,5)*SK_ACC[6] - Kacc*P(18,6)*SK_ACC[5] - Kacc*P(18,23)*SK_ACC[6]);
        Kfusion(19) = -SK_ACC[0]*(Kacc*P(19,4)*SH_ACC[0] - Kacc*P(19,22)*SH_ACC[0] + Kacc*P(19,0)*SK_ACC[3] - Kacc*P(19,2)*SK_ACC[2] + Kacc*P(19,3)*SK_ACC[1] + Kacc*P(19,1)*SK_ACC[4] + Kacc*P(19,5)*SK_ACC[6] - Kacc*P(19,6)*SK_ACC[5] - Kacc*P(19,23)*SK_ACC[6]);
        Kfusion(20) = -SK_ACC[0]*(Kacc*P(20,4)*SH_ACC[0] - Kacc*P(20,22)*SH_ACC[0] + Kacc*P(20,0)*SK_ACC[3] - Kacc*P(20,2)*SK_ACC[2] + Kacc*P(20,3)*SK_ACC[1] + Kacc*P(20,1)*SK_ACC[4] + Kacc*P(20,5)*SK_ACC[6] - Kacc*P(20,6)*SK_ACC[5] - Kacc*P(20,23)*SK_ACC[6]);
        Kfusion(21) = -SK_ACC[0]*(Kacc*P(21,4)*SH_ACC[0] - Kacc*P(21,22)*SH_ACC[0] + Kacc*P(21,0)*SK_ACC[3] - Kacc*P(21,2)*SK_ACC[2] + Kacc*P(21,3)*SK_ACC[1] + Kacc*P(21,1)*SK_ACC[4] + Kacc*P(21,5)*SK_ACC[6] - Kacc*P(21,6)*SK_ACC[5] - Kacc*P(21,23)*SK_ACC[6]);
        Kfusion(22) = -SK_ACC[0]*(Kacc*P(22,4)*SH_ACC[0] - Kacc*P(22,22)*SH_ACC[0] + Kacc*P(22,0)*SK_ACC[3] - Kacc*P(22,2)*SK_ACC[2] + Kacc*P(22,3)*SK_ACC[1] + Kacc*P(22,1)*SK_ACC[4] + Kacc*P(22,5)*SK_ACC[6] - Kacc*P(22,6)*SK_ACC[5] - Kacc*P(22,23)*SK_ACC[6]);
        Kfusion(23) = -SK_ACC[0]*(Kacc*P(23,4)*SH_ACC[0] - Kacc*P(23,22)*SH_ACC[0] + Kacc*P(23,0)*SK_ACC[3] - Kacc*P(23,2)*SK_ACC[2] + Kacc*P(23,3)*SK_ACC[1] + Kacc*P(23,1)*SK_ACC[4] + Kacc*P(23,5)*SK_ACC[6] - Kacc*P(23,6)*SK_ACC[5] - Kacc*P(23,23)*SK_ACC[6]);

        Hfusion_matlab = H_ACC;
        Kfusion_matlab = Kfusion;

        // find largest observation variance difference as a fraction of the matlab value
        float max_diff_fraction = 0.0f;
        int max_row;
        float max_old, max_new;
        for (int row=0; row<24; row++) {
            float diff_fraction;
            if (Hfusion_matlab(row) != 0.0f) {
                diff_fraction = fabsf(Hfusion_sympy(row) - Hfusion_matlab(row)) / fabsf(Hfusion_matlab(row));
            } else if (Hfusion_sympy(row) != 0.0f) {
                diff_fraction = fabsf(Hfusion_sympy(row) - Hfusion_matlab(row)) / fabsf(Hfusion_sympy(row));
            } else {
                diff_fraction = 0.0f;
            }
            if (diff_fraction > max_diff_fraction) {
                max_diff_fraction = diff_fraction;
                max_row = row;
                max_old = Hfusion_matlab(row);
                max_new = Hfusion_sympy(row);
            }
        }

        if (max_diff_fraction > 1e-5f) {
            printf("Fail: Specific Force X axis Hfusion max diff fraction = %e , old = %e , new = %e , location index = %i\n",max_diff_fraction, max_old, max_new, max_row);
        } else {
            printf("Pass: Specific Force X axis Hfusion max diff fraction = %e\n",max_diff_fraction);
        }

        // find largest Kalman gain difference as a fraction of the matlab value
        // find largest Kalman gain difference as a fraction of the matlab value
        max_diff_fraction = 0.0f;
        for (int row=0; row<24; row++) {
           float diff_fraction;
            if (Kfusion_matlab(row) != 0.0f) {
                diff_fraction = fabsf(Kfusion_sympy(row) - Kfusion_matlab(row)) / fabsf(Kfusion_matlab(row));
            } else if (Kfusion_sympy(row) != 0.0f) {
                diff_fraction = fabsf(Kfusion_sympy(row) - Kfusion_matlab(row)) / fabsf(Kfusion_sympy(row));
            } else {
                diff_fraction = 0.0f;
            }
            if (diff_fraction > max_diff_fraction) {
                max_diff_fraction = diff_fraction;
                max_row = row;
                max_old = Kfusion_matlab(row);
                max_new = Kfusion_sympy(row);
            }
        }

        if (max_diff_fraction > 1e-5f) {
            printf("Fail: Specific Force X axis Kfusion max diff fraction = %e , old = %e , new = %e , location index = %i\n",max_diff_fraction, max_old, max_new, max_row);
        } else {
            printf("Pass: Specific Force X axis Kfusion max diff fraction = %e\n",max_diff_fraction);
        }

    }

    // Compare Y axis equations
    {
        const float airSpd = 10.0f * (float)rand() + 5.0f;

        // Estimate the derivative of specific force wrt airspeed along the X axis
        // Limit lower value to prevent arithmetic exceptions
        const float Kaccy = fmaxf(1e-1f, rho * BC_inv_y * airSpd);

        // Sub Expressions
        const float HK0 = ve - vwe;
        const float HK1 = vn - vwn;
        const float HK2 = HK0*q0 - HK1*q3 + q1*vd;
        const float HK3 = 2*Kaccy;
        const float HK4 = -HK0*q1 + HK1*q2 + q0*vd;
        const float HK5 = HK0*q2 + HK1*q1 + q3*vd;
        const float HK6 = HK0*q3 + HK1*q0 - q2*vd;
        const float HK7 = q0*q3 - q1*q2;
        const float HK8 = HK3*HK7;
        const float HK9 = ecl::powf(q0, 2) - ecl::powf(q1, 2) + ecl::powf(q2, 2) - ecl::powf(q3, 2);
        const float HK10 = HK9*Kaccy;
        const float HK11 = q0*q1 + q2*q3;
        const float HK12 = 2*HK11;
        const float HK13 = 2*HK7;
        const float HK14 = 2*HK5;
        const float HK15 = 2*HK2;
        const float HK16 = 2*HK4;
        const float HK17 = 2*HK6;
        const float HK18 = HK12*P(0,6) + HK13*P(0,22) - HK13*P(0,4) + HK14*P(0,2) + HK15*P(0,0) + HK16*P(0,1) - HK17*P(0,3) - HK9*P(0,23) + HK9*P(0,5);
        const float HK19 = ecl::powf(Kaccy, 2);
        const float HK20 = HK12*P(6,6) - HK13*P(4,6) + HK13*P(6,22) + HK14*P(2,6) + HK15*P(0,6) + HK16*P(1,6) - HK17*P(3,6) + HK9*P(5,6) - HK9*P(6,23);
        const float HK21 = HK13*P(4,22);
        const float HK22 = HK12*P(6,22) + HK13*P(22,22) + HK14*P(2,22) + HK15*P(0,22) + HK16*P(1,22) - HK17*P(3,22) - HK21 - HK9*P(22,23) + HK9*P(5,22);
        const float HK23 = HK13*HK19;
        const float HK24 = HK12*P(4,6) - HK13*P(4,4) + HK14*P(2,4) + HK15*P(0,4) + HK16*P(1,4) - HK17*P(3,4) + HK21 - HK9*P(4,23) + HK9*P(4,5);
        const float HK25 = HK9*P(5,23);
        const float HK26 = HK12*P(5,6) - HK13*P(4,5) + HK13*P(5,22) + HK14*P(2,5) + HK15*P(0,5) + HK16*P(1,5) - HK17*P(3,5) - HK25 + HK9*P(5,5);
        const float HK27 = HK19*HK9;
        const float HK28 = HK12*P(6,23) + HK13*P(22,23) - HK13*P(4,23) + HK14*P(2,23) + HK15*P(0,23) + HK16*P(1,23) - HK17*P(3,23) + HK25 - HK9*P(23,23);
        const float HK29 = HK12*P(2,6) + HK13*P(2,22) - HK13*P(2,4) + HK14*P(2,2) + HK15*P(0,2) + HK16*P(1,2) - HK17*P(2,3) - HK9*P(2,23) + HK9*P(2,5);
        const float HK30 = HK12*P(1,6) + HK13*P(1,22) - HK13*P(1,4) + HK14*P(1,2) + HK15*P(0,1) + HK16*P(1,1) - HK17*P(1,3) - HK9*P(1,23) + HK9*P(1,5);
        const float HK31 = HK12*P(3,6) + HK13*P(3,22) - HK13*P(3,4) + HK14*P(2,3) + HK15*P(0,3) + HK16*P(1,3) - HK17*P(3,3) - HK9*P(3,23) + HK9*P(3,5);
        const float HK32 = Kaccy/(HK12*HK19*HK20 + HK14*HK19*HK29 + HK15*HK18*HK19 + HK16*HK19*HK30 - HK17*HK19*HK31 + HK22*HK23 - HK23*HK24 + HK26*HK27 - HK27*HK28 + R_ACC);

        // Observation Jacobians
        Hfusion.at<0>() = -HK2*HK3;
        Hfusion.at<1>() = -HK3*HK4;
        Hfusion.at<2>() = -HK3*HK5;
        Hfusion.at<3>() = HK3*HK6;
        Hfusion.at<4>() = HK8;
        Hfusion.at<5>() = -HK10;
        Hfusion.at<6>() = -HK11*HK3;
        Hfusion.at<22>() = -HK8;
        Hfusion.at<23>() = HK10;

        // Kalman gains
        Kfusion(0) = -HK18*HK32;
        Kfusion(1) = -HK30*HK32;
        Kfusion(2) = -HK29*HK32;
        Kfusion(3) = -HK31*HK32;
        Kfusion(4) = -HK24*HK32;
        Kfusion(5) = -HK26*HK32;
        Kfusion(6) = -HK20*HK32;
        Kfusion(7) = -HK32*(HK12*P(6,7) - HK13*P(4,7) + HK13*P(7,22) + HK14*P(2,7) + HK15*P(0,7) + HK16*P(1,7) - HK17*P(3,7) + HK9*P(5,7) - HK9*P(7,23));
        Kfusion(8) = -HK32*(HK12*P(6,8) - HK13*P(4,8) + HK13*P(8,22) + HK14*P(2,8) + HK15*P(0,8) + HK16*P(1,8) - HK17*P(3,8) + HK9*P(5,8) - HK9*P(8,23));
        Kfusion(9) = -HK32*(HK12*P(6,9) - HK13*P(4,9) + HK13*P(9,22) + HK14*P(2,9) + HK15*P(0,9) + HK16*P(1,9) - HK17*P(3,9) + HK9*P(5,9) - HK9*P(9,23));
        Kfusion(10) = -HK32*(HK12*P(6,10) + HK13*P(10,22) - HK13*P(4,10) + HK14*P(2,10) + HK15*P(0,10) + HK16*P(1,10) - HK17*P(3,10) - HK9*P(10,23) + HK9*P(5,10));
        Kfusion(11) = -HK32*(HK12*P(6,11) + HK13*P(11,22) - HK13*P(4,11) + HK14*P(2,11) + HK15*P(0,11) + HK16*P(1,11) - HK17*P(3,11) - HK9*P(11,23) + HK9*P(5,11));
        Kfusion(12) = -HK32*(HK12*P(6,12) + HK13*P(12,22) - HK13*P(4,12) + HK14*P(2,12) + HK15*P(0,12) + HK16*P(1,12) - HK17*P(3,12) - HK9*P(12,23) + HK9*P(5,12));
        Kfusion(13) = -HK32*(HK12*P(6,13) + HK13*P(13,22) - HK13*P(4,13) + HK14*P(2,13) + HK15*P(0,13) + HK16*P(1,13) - HK17*P(3,13) - HK9*P(13,23) + HK9*P(5,13));
        Kfusion(14) = -HK32*(HK12*P(6,14) + HK13*P(14,22) - HK13*P(4,14) + HK14*P(2,14) + HK15*P(0,14) + HK16*P(1,14) - HK17*P(3,14) - HK9*P(14,23) + HK9*P(5,14));
        Kfusion(15) = -HK32*(HK12*P(6,15) + HK13*P(15,22) - HK13*P(4,15) + HK14*P(2,15) + HK15*P(0,15) + HK16*P(1,15) - HK17*P(3,15) - HK9*P(15,23) + HK9*P(5,15));
        Kfusion(16) = -HK32*(HK12*P(6,16) + HK13*P(16,22) - HK13*P(4,16) + HK14*P(2,16) + HK15*P(0,16) + HK16*P(1,16) - HK17*P(3,16) - HK9*P(16,23) + HK9*P(5,16));
        Kfusion(17) = -HK32*(HK12*P(6,17) + HK13*P(17,22) - HK13*P(4,17) + HK14*P(2,17) + HK15*P(0,17) + HK16*P(1,17) - HK17*P(3,17) - HK9*P(17,23) + HK9*P(5,17));
        Kfusion(18) = -HK32*(HK12*P(6,18) + HK13*P(18,22) - HK13*P(4,18) + HK14*P(2,18) + HK15*P(0,18) + HK16*P(1,18) - HK17*P(3,18) - HK9*P(18,23) + HK9*P(5,18));
        Kfusion(19) = -HK32*(HK12*P(6,19) + HK13*P(19,22) - HK13*P(4,19) + HK14*P(2,19) + HK15*P(0,19) + HK16*P(1,19) - HK17*P(3,19) - HK9*P(19,23) + HK9*P(5,19));
        Kfusion(20) = -HK32*(HK12*P(6,20) + HK13*P(20,22) - HK13*P(4,20) + HK14*P(2,20) + HK15*P(0,20) + HK16*P(1,20) - HK17*P(3,20) - HK9*P(20,23) + HK9*P(5,20));
        Kfusion(21) = -HK32*(HK12*P(6,21) + HK13*P(21,22) - HK13*P(4,21) + HK14*P(2,21) + HK15*P(0,21) + HK16*P(1,21) - HK17*P(3,21) - HK9*P(21,23) + HK9*P(5,21));
        Kfusion(22) = -HK22*HK32;
        Kfusion(23) = -HK28*HK32;

        // save output
        Hfusion_sympy(0) = Hfusion.at<0>();
        Hfusion_sympy(1) = Hfusion.at<1>();
        Hfusion_sympy(2) = Hfusion.at<2>();
        Hfusion_sympy(3) = Hfusion.at<3>();
        Hfusion_sympy(4) = Hfusion.at<4>();
        Hfusion_sympy(5) = Hfusion.at<5>();
        Hfusion_sympy(6) = Hfusion.at<6>();
        Hfusion_sympy(22) = Hfusion.at<22>();
        Hfusion_sympy(23) = Hfusion.at<23>();
        Kfusion_sympy = Kfusion;

        // repeat calculation using matlab generated equations

        const float Kacc = Kaccy;

        SH_ACC[0] = sq(q0) - sq(q1) + sq(q2) - sq(q3);
        SH_ACC[1] = vn - vwn;
        SH_ACC[2] = ve - vwe;
        H_ACC(0) = -Kacc*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd);
        H_ACC(1) = -Kacc*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd);
        H_ACC(2) = -Kacc*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd);
        H_ACC(3) = Kacc*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd);
        H_ACC(4) = Kacc*(2.0f*q0*q3 - 2.0f*q1*q2);
        H_ACC(5) = -Kacc*SH_ACC[0];
        H_ACC(6) = -Kacc*(2.0f*q0*q1 + 2.0f*q2*q3);
        H_ACC(22) = -2.0f*Kacc*(q0*q3 - q1*q2);
        H_ACC(23) = Kacc*SH_ACC[0];
        drag_innov_var = (R_ACC + Kacc*SH_ACC[0]*(Kacc*P(5,5)*SH_ACC[0] - Kacc*P(23,5)*SH_ACC[0] - Kacc*P(4,5)*(2.0f*q0*q3 - 2.0f*q1*q2) + Kacc*P(6,5)*(2.0f*q0*q1 + 2.0f*q2*q3) + 2*Kacc*P(22,5)*(q0*q3 - q1*q2) + Kacc*P(0,5)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd) + Kacc*P(1,5)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(2,5)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(3,5)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd)) - Kacc*SH_ACC[0]*(Kacc*P(5,23)*SH_ACC[0] - Kacc*P(23,23)*SH_ACC[0] - Kacc*P(4,23)*(2.0f*q0*q3 - 2.0f*q1*q2) + Kacc*P(6,23)*(2.0f*q0*q1 + 2.0f*q2*q3) + 2*Kacc*P(22,23)*(q0*q3 - q1*q2) + Kacc*P(0,23)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd) + Kacc*P(1,23)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(2,23)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(3,23)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd)) - Kacc*(2.0f*q0*q3 - 2.0f*q1*q2)*(Kacc*P(5,4)*SH_ACC[0] - Kacc*P(23,4)*SH_ACC[0] - Kacc*P(4,4)*(2.0f*q0*q3 - 2.0f*q1*q2) + Kacc*P(6,4)*(2.0f*q0*q1 + 2.0f*q2*q3) + 2*Kacc*P(22,4)*(q0*q3 - q1*q2) + Kacc*P(0,4)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd) + Kacc*P(1,4)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(2,4)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(3,4)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd)) + Kacc*(2.0f*q0*q1 + 2.0f*q2*q3)*(Kacc*P(5,6)*SH_ACC[0] - Kacc*P(23,6)*SH_ACC[0] - Kacc*P(4,6)*(2.0f*q0*q3 - 2.0f*q1*q2) + Kacc*P(6,6)*(2.0f*q0*q1 + 2.0f*q2*q3) + 2*Kacc*P(22,6)*(q0*q3 - q1*q2) + Kacc*P(0,6)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd) + Kacc*P(1,6)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(2,6)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(3,6)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd)) + 2*Kacc*(q0*q3 - q1*q2)*(Kacc*P(5,22)*SH_ACC[0] - Kacc*P(23,22)*SH_ACC[0] - Kacc*P(4,22)*(2.0f*q0*q3 - 2.0f*q1*q2) + Kacc*P(6,22)*(2.0f*q0*q1 + 2.0f*q2*q3) + 2*Kacc*P(22,22)*(q0*q3 - q1*q2) + Kacc*P(0,22)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd) + Kacc*P(1,22)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(2,22)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(3,22)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd)) + Kacc*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd)*(Kacc*P(5,0)*SH_ACC[0] - Kacc*P(23,0)*SH_ACC[0] - Kacc*P(4,0)*(2.0f*q0*q3 - 2.0f*q1*q2) + Kacc*P(6,0)*(2.0f*q0*q1 + 2.0f*q2*q3) + 2*Kacc*P(22,0)*(q0*q3 - q1*q2) + Kacc*P(0,0)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd) + Kacc*P(1,0)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(2,0)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(3,0)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd)) + Kacc*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd)*(Kacc*P(5,1)*SH_ACC[0] - Kacc*P(23,1)*SH_ACC[0] - Kacc*P(4,1)*(2.0f*q0*q3 - 2.0f*q1*q2) + Kacc*P(6,1)*(2.0f*q0*q1 + 2.0f*q2*q3) + 2*Kacc*P(22,1)*(q0*q3 - q1*q2) + Kacc*P(0,1)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd) + Kacc*P(1,1)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(2,1)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(3,1)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd)) + Kacc*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd)*(Kacc*P(5,2)*SH_ACC[0] - Kacc*P(23,2)*SH_ACC[0] - Kacc*P(4,2)*(2.0f*q0*q3 - 2.0f*q1*q2) + Kacc*P(6,2)*(2.0f*q0*q1 + 2.0f*q2*q3) + 2*Kacc*P(22,2)*(q0*q3 - q1*q2) + Kacc*P(0,2)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd) + Kacc*P(1,2)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(2,2)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(3,2)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd)) - Kacc*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd)*(Kacc*P(5,3)*SH_ACC[0] - Kacc*P(23,3)*SH_ACC[0] - Kacc*P(4,3)*(2.0f*q0*q3 - 2.0f*q1*q2) + Kacc*P(6,3)*(2.0f*q0*q1 + 2.0f*q2*q3) + 2*Kacc*P(22,3)*(q0*q3 - q1*q2) + Kacc*P(0,3)*(2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd) + Kacc*P(1,3)*(2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd) + Kacc*P(2,3)*(2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd) - Kacc*P(3,3)*(2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd)));
        SK_ACC[0] = 1.0f/drag_innov_var;
        SK_ACC[1] = 2.0f*q0*SH_ACC[1] + 2.0f*q3*SH_ACC[2] - 2.0f*q2*vd;
        SK_ACC[2] = 2.0f*q2*SH_ACC[1] - 2.0f*q1*SH_ACC[2] + 2.0f*q0*vd;
        SK_ACC[3] = 2.0f*q0*SH_ACC[2] - 2.0f*q3*SH_ACC[1] + 2.0f*q1*vd;
        SK_ACC[4] = 2.0f*q1*SH_ACC[1] + 2.0f*q2*SH_ACC[2] + 2.0f*q3*vd;
        SK_ACC[5] = 2.0f*q0*q3 - 2.0f*q1*q2;
        SK_ACC[6] = q0*q3 - q1*q2;
        SK_ACC[7] = 2.0f*q0*q1 + 2.0f*q2*q3;
        SK_ACC[8] = SH_ACC[0];

        Kfusion(0) = -SK_ACC[0]*(Kacc*P(0,0)*SK_ACC[3] + Kacc*P(0,1)*SK_ACC[2] - Kacc*P(0,3)*SK_ACC[1] + Kacc*P(0,2)*SK_ACC[4] - Kacc*P(0,4)*SK_ACC[5] + Kacc*P(0,5)*SK_ACC[8] + Kacc*P(0,6)*SK_ACC[7] + 2*Kacc*P(0,22)*SK_ACC[6] - Kacc*P(0,23)*SK_ACC[8]);
        Kfusion(1) = -SK_ACC[0]*(Kacc*P(1,0)*SK_ACC[3] + Kacc*P(1,1)*SK_ACC[2] - Kacc*P(1,3)*SK_ACC[1] + Kacc*P(1,2)*SK_ACC[4] - Kacc*P(1,4)*SK_ACC[5] + Kacc*P(1,5)*SK_ACC[8] + Kacc*P(1,6)*SK_ACC[7] + 2*Kacc*P(1,22)*SK_ACC[6] - Kacc*P(1,23)*SK_ACC[8]);
        Kfusion(2) = -SK_ACC[0]*(Kacc*P(2,0)*SK_ACC[3] + Kacc*P(2,1)*SK_ACC[2] - Kacc*P(2,3)*SK_ACC[1] + Kacc*P(2,2)*SK_ACC[4] - Kacc*P(2,4)*SK_ACC[5] + Kacc*P(2,5)*SK_ACC[8] + Kacc*P(2,6)*SK_ACC[7] + 2*Kacc*P(2,22)*SK_ACC[6] - Kacc*P(2,23)*SK_ACC[8]);
        Kfusion(3) = -SK_ACC[0]*(Kacc*P(3,0)*SK_ACC[3] + Kacc*P(3,1)*SK_ACC[2] - Kacc*P(3,3)*SK_ACC[1] + Kacc*P(3,2)*SK_ACC[4] - Kacc*P(3,4)*SK_ACC[5] + Kacc*P(3,5)*SK_ACC[8] + Kacc*P(3,6)*SK_ACC[7] + 2*Kacc*P(3,22)*SK_ACC[6] - Kacc*P(3,23)*SK_ACC[8]);
        Kfusion(4) = -SK_ACC[0]*(Kacc*P(4,0)*SK_ACC[3] + Kacc*P(4,1)*SK_ACC[2] - Kacc*P(4,3)*SK_ACC[1] + Kacc*P(4,2)*SK_ACC[4] - Kacc*P(4,4)*SK_ACC[5] + Kacc*P(4,5)*SK_ACC[8] + Kacc*P(4,6)*SK_ACC[7] + 2*Kacc*P(4,22)*SK_ACC[6] - Kacc*P(4,23)*SK_ACC[8]);
        Kfusion(5) = -SK_ACC[0]*(Kacc*P(5,0)*SK_ACC[3] + Kacc*P(5,1)*SK_ACC[2] - Kacc*P(5,3)*SK_ACC[1] + Kacc*P(5,2)*SK_ACC[4] - Kacc*P(5,4)*SK_ACC[5] + Kacc*P(5,5)*SK_ACC[8] + Kacc*P(5,6)*SK_ACC[7] + 2*Kacc*P(5,22)*SK_ACC[6] - Kacc*P(5,23)*SK_ACC[8]);
        Kfusion(6) = -SK_ACC[0]*(Kacc*P(6,0)*SK_ACC[3] + Kacc*P(6,1)*SK_ACC[2] - Kacc*P(6,3)*SK_ACC[1] + Kacc*P(6,2)*SK_ACC[4] - Kacc*P(6,4)*SK_ACC[5] + Kacc*P(6,5)*SK_ACC[8] + Kacc*P(6,6)*SK_ACC[7] + 2*Kacc*P(6,22)*SK_ACC[6] - Kacc*P(6,23)*SK_ACC[8]);
        Kfusion(7) = -SK_ACC[0]*(Kacc*P(7,0)*SK_ACC[3] + Kacc*P(7,1)*SK_ACC[2] - Kacc*P(7,3)*SK_ACC[1] + Kacc*P(7,2)*SK_ACC[4] - Kacc*P(7,4)*SK_ACC[5] + Kacc*P(7,5)*SK_ACC[8] + Kacc*P(7,6)*SK_ACC[7] + 2*Kacc*P(7,22)*SK_ACC[6] - Kacc*P(7,23)*SK_ACC[8]);
        Kfusion(8) = -SK_ACC[0]*(Kacc*P(8,0)*SK_ACC[3] + Kacc*P(8,1)*SK_ACC[2] - Kacc*P(8,3)*SK_ACC[1] + Kacc*P(8,2)*SK_ACC[4] - Kacc*P(8,4)*SK_ACC[5] + Kacc*P(8,5)*SK_ACC[8] + Kacc*P(8,6)*SK_ACC[7] + 2*Kacc*P(8,22)*SK_ACC[6] - Kacc*P(8,23)*SK_ACC[8]);
        Kfusion(9) = -SK_ACC[0]*(Kacc*P(9,0)*SK_ACC[3] + Kacc*P(9,1)*SK_ACC[2] - Kacc*P(9,3)*SK_ACC[1] + Kacc*P(9,2)*SK_ACC[4] - Kacc*P(9,4)*SK_ACC[5] + Kacc*P(9,5)*SK_ACC[8] + Kacc*P(9,6)*SK_ACC[7] + 2*Kacc*P(9,22)*SK_ACC[6] - Kacc*P(9,23)*SK_ACC[8]);
        Kfusion(10) = -SK_ACC[0]*(Kacc*P(10,0)*SK_ACC[3] + Kacc*P(10,1)*SK_ACC[2] - Kacc*P(10,3)*SK_ACC[1] + Kacc*P(10,2)*SK_ACC[4] - Kacc*P(10,4)*SK_ACC[5] + Kacc*P(10,5)*SK_ACC[8] + Kacc*P(10,6)*SK_ACC[7] + 2*Kacc*P(10,22)*SK_ACC[6] - Kacc*P(10,23)*SK_ACC[8]);
        Kfusion(11) = -SK_ACC[0]*(Kacc*P(11,0)*SK_ACC[3] + Kacc*P(11,1)*SK_ACC[2] - Kacc*P(11,3)*SK_ACC[1] + Kacc*P(11,2)*SK_ACC[4] - Kacc*P(11,4)*SK_ACC[5] + Kacc*P(11,5)*SK_ACC[8] + Kacc*P(11,6)*SK_ACC[7] + 2*Kacc*P(11,22)*SK_ACC[6] - Kacc*P(11,23)*SK_ACC[8]);
        Kfusion(12) = -SK_ACC[0]*(Kacc*P(12,0)*SK_ACC[3] + Kacc*P(12,1)*SK_ACC[2] - Kacc*P(12,3)*SK_ACC[1] + Kacc*P(12,2)*SK_ACC[4] - Kacc*P(12,4)*SK_ACC[5] + Kacc*P(12,5)*SK_ACC[8] + Kacc*P(12,6)*SK_ACC[7] + 2*Kacc*P(12,22)*SK_ACC[6] - Kacc*P(12,23)*SK_ACC[8]);
        Kfusion(13) = -SK_ACC[0]*(Kacc*P(13,0)*SK_ACC[3] + Kacc*P(13,1)*SK_ACC[2] - Kacc*P(13,3)*SK_ACC[1] + Kacc*P(13,2)*SK_ACC[4] - Kacc*P(13,4)*SK_ACC[5] + Kacc*P(13,5)*SK_ACC[8] + Kacc*P(13,6)*SK_ACC[7] + 2*Kacc*P(13,22)*SK_ACC[6] - Kacc*P(13,23)*SK_ACC[8]);
        Kfusion(14) = -SK_ACC[0]*(Kacc*P(14,0)*SK_ACC[3] + Kacc*P(14,1)*SK_ACC[2] - Kacc*P(14,3)*SK_ACC[1] + Kacc*P(14,2)*SK_ACC[4] - Kacc*P(14,4)*SK_ACC[5] + Kacc*P(14,5)*SK_ACC[8] + Kacc*P(14,6)*SK_ACC[7] + 2*Kacc*P(14,22)*SK_ACC[6] - Kacc*P(14,23)*SK_ACC[8]);
        Kfusion(15) = -SK_ACC[0]*(Kacc*P(15,0)*SK_ACC[3] + Kacc*P(15,1)*SK_ACC[2] - Kacc*P(15,3)*SK_ACC[1] + Kacc*P(15,2)*SK_ACC[4] - Kacc*P(15,4)*SK_ACC[5] + Kacc*P(15,5)*SK_ACC[8] + Kacc*P(15,6)*SK_ACC[7] + 2*Kacc*P(15,22)*SK_ACC[6] - Kacc*P(15,23)*SK_ACC[8]);
        Kfusion(16) = -SK_ACC[0]*(Kacc*P(16,0)*SK_ACC[3] + Kacc*P(16,1)*SK_ACC[2] - Kacc*P(16,3)*SK_ACC[1] + Kacc*P(16,2)*SK_ACC[4] - Kacc*P(16,4)*SK_ACC[5] + Kacc*P(16,5)*SK_ACC[8] + Kacc*P(16,6)*SK_ACC[7] + 2*Kacc*P(16,22)*SK_ACC[6] - Kacc*P(16,23)*SK_ACC[8]);
        Kfusion(17) = -SK_ACC[0]*(Kacc*P(17,0)*SK_ACC[3] + Kacc*P(17,1)*SK_ACC[2] - Kacc*P(17,3)*SK_ACC[1] + Kacc*P(17,2)*SK_ACC[4] - Kacc*P(17,4)*SK_ACC[5] + Kacc*P(17,5)*SK_ACC[8] + Kacc*P(17,6)*SK_ACC[7] + 2*Kacc*P(17,22)*SK_ACC[6] - Kacc*P(17,23)*SK_ACC[8]);
        Kfusion(18) = -SK_ACC[0]*(Kacc*P(18,0)*SK_ACC[3] + Kacc*P(18,1)*SK_ACC[2] - Kacc*P(18,3)*SK_ACC[1] + Kacc*P(18,2)*SK_ACC[4] - Kacc*P(18,4)*SK_ACC[5] + Kacc*P(18,5)*SK_ACC[8] + Kacc*P(18,6)*SK_ACC[7] + 2*Kacc*P(18,22)*SK_ACC[6] - Kacc*P(18,23)*SK_ACC[8]);
        Kfusion(19) = -SK_ACC[0]*(Kacc*P(19,0)*SK_ACC[3] + Kacc*P(19,1)*SK_ACC[2] - Kacc*P(19,3)*SK_ACC[1] + Kacc*P(19,2)*SK_ACC[4] - Kacc*P(19,4)*SK_ACC[5] + Kacc*P(19,5)*SK_ACC[8] + Kacc*P(19,6)*SK_ACC[7] + 2*Kacc*P(19,22)*SK_ACC[6] - Kacc*P(19,23)*SK_ACC[8]);
        Kfusion(20) = -SK_ACC[0]*(Kacc*P(20,0)*SK_ACC[3] + Kacc*P(20,1)*SK_ACC[2] - Kacc*P(20,3)*SK_ACC[1] + Kacc*P(20,2)*SK_ACC[4] - Kacc*P(20,4)*SK_ACC[5] + Kacc*P(20,5)*SK_ACC[8] + Kacc*P(20,6)*SK_ACC[7] + 2*Kacc*P(20,22)*SK_ACC[6] - Kacc*P(20,23)*SK_ACC[8]);
        Kfusion(21) = -SK_ACC[0]*(Kacc*P(21,0)*SK_ACC[3] + Kacc*P(21,1)*SK_ACC[2] - Kacc*P(21,3)*SK_ACC[1] + Kacc*P(21,2)*SK_ACC[4] - Kacc*P(21,4)*SK_ACC[5] + Kacc*P(21,5)*SK_ACC[8] + Kacc*P(21,6)*SK_ACC[7] + 2*Kacc*P(21,22)*SK_ACC[6] - Kacc*P(21,23)*SK_ACC[8]);
        Kfusion(22) = -SK_ACC[0]*(Kacc*P(22,0)*SK_ACC[3] + Kacc*P(22,1)*SK_ACC[2] - Kacc*P(22,3)*SK_ACC[1] + Kacc*P(22,2)*SK_ACC[4] - Kacc*P(22,4)*SK_ACC[5] + Kacc*P(22,5)*SK_ACC[8] + Kacc*P(22,6)*SK_ACC[7] + 2*Kacc*P(22,22)*SK_ACC[6] - Kacc*P(22,23)*SK_ACC[8]);
        Kfusion(23) = -SK_ACC[0]*(Kacc*P(23,0)*SK_ACC[3] + Kacc*P(23,1)*SK_ACC[2] - Kacc*P(23,3)*SK_ACC[1] + Kacc*P(23,2)*SK_ACC[4] - Kacc*P(23,4)*SK_ACC[5] + Kacc*P(23,5)*SK_ACC[8] + Kacc*P(23,6)*SK_ACC[7] + 2*Kacc*P(23,22)*SK_ACC[6] - Kacc*P(23,23)*SK_ACC[8]);

        Hfusion_matlab = H_ACC;
        Kfusion_matlab = Kfusion;

        // find largest observation variance difference as a fraction of the matlab value
        float max_diff_fraction = 0.0f;
        int max_row;
        float max_old, max_new;
        for (int row=0; row<24; row++) {
            float diff_fraction;
            if (Hfusion_matlab(row) != 0.0f) {
                diff_fraction = fabsf(Hfusion_sympy(row) - Hfusion_matlab(row)) / fabsf(Hfusion_matlab(row));
            } else if (Hfusion_sympy(row) != 0.0f) {
                diff_fraction = fabsf(Hfusion_sympy(row) - H_ACC(row)) / fabsf(Hfusion_sympy(row));
            } else {
                diff_fraction = 0.0f;
            }
            if (diff_fraction > max_diff_fraction) {
                max_diff_fraction = diff_fraction;
                max_row = row;
                max_old = H_ACC(row);
                max_new = Hfusion_sympy(row);
            }
        }

        if (max_diff_fraction > 1e-5f) {
            printf("Fail: Specific Force Y axis Hfusion max diff fraction = %e , old = %e , new = %e , location index = %i\n",max_diff_fraction, max_old, max_new, max_row);
        } else {
            printf("Pass: Specific Force Y axis Hfusion max diff fraction = %e\n",max_diff_fraction);
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
            printf("Fail: Specific Force Y axis Kfusion max diff fraction = %e , old = %e , new = %e , location index = %i\n",max_diff_fraction, max_old, max_new, max_row);
        } else {
            printf("Pass: Specific Force Y axis Kfusion max diff fraction = %e\n",max_diff_fraction);
        }

    }

    return 0;
}
