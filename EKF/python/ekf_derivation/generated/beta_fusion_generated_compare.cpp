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
    Vector24f Hfusion_sympy;
    Vector24f Kfusion_sympy;

    Vector24f Hfusion_matlab;
    Vector24f Kfusion_matlab;

    float _beta_innov_var;

	const float R_BETA = sq(2.5f);

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
	const float vn = 10.0f * 2.0f * ((float)rand() - 0.5f);
	const float ve = 10.0f * 2.0f * ((float)rand() - 0.5f);
	const float vd = 2.0f * ((float)rand() - 0.5f);

	// get latest wind velocity in earth frame
	const float vwn = 5.0f * 2.0f * ((float)rand() - 0.5f);
	const float vwe = 5.0f * 2.0f * ((float)rand() - 0.5f);

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
    {
        // Intermediate Values
        const float HK0 = vn - vwn;
        const float HK1 = ve - vwe;
        const float HK2 = HK0*q0 + HK1*q3 - q2*vd;
        const float HK3 = q0*q2 - q1*q3;
        const float HK4 = 2*vd;
        const float HK5 = q0*q3;
        const float HK6 = q1*q2;
        const float HK7 = 2*HK5 + 2*HK6;
        const float HK8 = ecl::powf(q0, 2);
        const float HK9 = ecl::powf(q3, 2);
        const float HK10 = HK8 - HK9;
        const float HK11 = ecl::powf(q1, 2);
        const float HK12 = ecl::powf(q2, 2);
        const float HK13 = HK11 - HK12;
        const float HK14 = HK10 + HK13;
        const float HK15 = HK0*HK14 + HK1*HK7 - HK3*HK4;
        const float HK16 = 1.0F/HK15;
        const float HK17 = q0*q1 + q2*q3;
        const float HK18 = HK10 - HK11 + HK12;
        const float HK19 = HK16*(-2*HK0*(HK5 - HK6) + HK1*HK18 + HK17*HK4);
        const float HK20 = -HK0*q3 + HK1*q0 + q1*vd;
        const float HK21 = -HK19*HK2 + HK20;
        const float HK22 = 2*HK16;
        const float HK23 = HK0*q1 + HK1*q2 + q3*vd;
        const float HK24 = HK0*q2 - HK1*q1 + q0*vd;
        const float HK25 = -HK19*HK23 + HK24;
        const float HK26 = HK19*HK24 + HK23;
        const float HK27 = HK19*HK20 + HK2;
        const float HK28 = HK14*HK19 + 2*HK5 - 2*HK6;
        const float HK29 = HK16*HK28;
        const float HK30 = HK19*HK7;
        const float HK31 = HK17 + HK19*HK3;
        const float HK32 = HK13 + HK30 - HK8 + HK9;
        const float HK33 = 2*HK31;
        const float HK34 = 2*HK26;
        const float HK35 = 2*HK25;
        const float HK36 = 2*HK27;
        const float HK37 = 2*HK21;
        const float HK38 = HK28*P(0,22) - HK28*P(0,4) + HK32*P(0,23) - HK32*P(0,5) + HK33*P(0,6) + HK34*P(0,2) + HK35*P(0,1) - HK36*P(0,3) + HK37*P(0,0);
        const float HK39 = ecl::powf(HK15, -2);
        const float HK40 = -HK28*P(4,6) + HK28*P(6,22) - HK32*P(5,6) + HK32*P(6,23) + HK33*P(6,6) + HK34*P(2,6) + HK35*P(1,6) - HK36*P(3,6) + HK37*P(0,6);
        const float HK41 = HK32*P(5,23);
        const float HK42 = HK28*P(22,23) - HK28*P(4,23) + HK32*P(23,23) + HK33*P(6,23) + HK34*P(2,23) + HK35*P(1,23) - HK36*P(3,23) + HK37*P(0,23) - HK41;
        const float HK43 = HK32*HK39;
        const float HK44 = HK28*P(4,22);
        const float HK45 = HK28*P(22,22) + HK32*P(22,23) - HK32*P(5,22) + HK33*P(6,22) + HK34*P(2,22) + HK35*P(1,22) - HK36*P(3,22) + HK37*P(0,22) - HK44;
        const float HK46 = HK28*HK39;
        const float HK47 = -HK28*P(4,5) + HK28*P(5,22) - HK32*P(5,5) + HK33*P(5,6) + HK34*P(2,5) + HK35*P(1,5) - HK36*P(3,5) + HK37*P(0,5) + HK41;
        const float HK48 = -HK28*P(4,4) + HK32*P(4,23) - HK32*P(4,5) + HK33*P(4,6) + HK34*P(2,4) + HK35*P(1,4) - HK36*P(3,4) + HK37*P(0,4) + HK44;
        const float HK49 = HK28*P(2,22) - HK28*P(2,4) + HK32*P(2,23) - HK32*P(2,5) + HK33*P(2,6) + HK34*P(2,2) + HK35*P(1,2) - HK36*P(2,3) + HK37*P(0,2);
        const float HK50 = HK28*P(1,22) - HK28*P(1,4) + HK32*P(1,23) - HK32*P(1,5) + HK33*P(1,6) + HK34*P(1,2) + HK35*P(1,1) - HK36*P(1,3) + HK37*P(0,1);
        const float HK51 = HK28*P(3,22) - HK28*P(3,4) + HK32*P(3,23) - HK32*P(3,5) + HK33*P(3,6) + HK34*P(2,3) + HK35*P(1,3) - HK36*P(3,3) + HK37*P(0,3);
        //const float HK52 = HK16/(HK33*HK39*HK40 + HK34*HK39*HK49 + HK35*HK39*HK50 - HK36*HK39*HK51 + HK37*HK38*HK39 + HK42*HK43 - HK43*HK47 + HK45*HK46 - HK46*HK48 + R_BETA);

        // innovation variance
        _beta_innov_var = (HK33*HK39*HK40 + HK34*HK39*HK49 + HK35*HK39*HK50 - HK36*HK39*HK51 + HK37*HK38*HK39 + HK42*HK43 - HK43*HK47 + HK45*HK46 - HK46*HK48 + R_BETA);

        // // Reset covariance and states if the calculation is badly conditioned
        // if (_beta_innov_var < R_BETA) {
        //     _fault_status.flags.bad_sideslip = true;

        //     // if we are getting aiding from other sources, warn and reset the wind states and covariances only
        //     const char* action_string = nullptr;
        //     if (update_wind_only) {
        //         resetWindStates();
        //         resetWindCovariance();
        //         action_string = "wind";

        //     } else {
        //         initialiseCovariance();
        //         _state.wind_vel.setZero();
        //         action_string = "full";
        //     }
        //     ECL_ERR("sideslip badly conditioned - %s covariance reset", action_string);

        //     return;
        // }
        // _fault_status.flags.bad_sideslip = false;
        const float HK52 = HK16/_beta_innov_var;

        // determine if we need the sideslip fusion to correct states other than wind
        // bool update_wind_only = !_is_wind_dead_reckoning;
        bool update_wind_only = false;

        // // Calculate predicted sideslip angle and innovation using small angle approximation
        // _beta_innov = rel_wind_body(1) / rel_wind_body(0);

        // // Compute the ratio of innovation to gate size
        // _beta_test_ratio = sq(_beta_innov) / (sq(fmaxf(_params.beta_innov_gate, 1.0f)) * _beta_innov_var);

        // // if the innovation consistency check fails then don't fuse the sample and indicate bad beta health
        // if (_beta_test_ratio > 1.0f) {
        //     _innov_check_fail_status.flags.reject_sideslip = true;
        //     return;

        // } else {
        //     _innov_check_fail_status.flags.reject_sideslip = false;
        // }

        // Observation Jacobians
        SparseVector24f<0,1,2,3,4,5,6,22,23> Hfusion;
        Hfusion.at<0>() = HK21*HK22;
        Hfusion.at<1>() = HK22*HK25;
        Hfusion.at<2>() = HK22*HK26;
        Hfusion.at<3>() = -HK22*HK27;
        Hfusion.at<4>() = -HK29;
        Hfusion.at<5>() = HK16*(HK18 - HK30);
        Hfusion.at<6>() = HK22*HK31;
        Hfusion.at<22>() = HK29;
        Hfusion.at<23>() = HK16*HK32;

        // Calculate Kalman gains
        Vector24f Kfusion;
        if (!update_wind_only) {

            Kfusion(0) = HK38*HK52;
            Kfusion(1) = HK50*HK52;
            Kfusion(2) = HK49*HK52;
            Kfusion(3) = HK51*HK52;
            Kfusion(4) = HK48*HK52;
            Kfusion(5) = HK47*HK52;
            Kfusion(6) = HK40*HK52;

            for (unsigned row = 7; row <= 21; row++) {
                Kfusion(row) = HK52*(HK28*P(row,22) - HK28*P(4,row) + HK32*P(row,23) - HK32*P(5,row) + HK33*P(6,row) + HK34*P(2,row) + HK35*P(1,row) - HK36*P(3,row) + HK37*P(0,row));
            }

        }

        Kfusion(22) = HK45*HK52;
        Kfusion(23) = HK42*HK52;

        // save output and repeat calculation using legacy matlab generated code
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
    }
    // repeat calculation using matlab generated equations
    {
        // Calculate the observation jacobians

        const matrix::Vector3f rel_wind_earth(vn - vwn, ve - vwe, vd);

        float SH_BETA[13];  // intermediate variable for algebraic optimisation

        SH_BETA[0] = (vn - vwn)*(sq(q0) + sq(q1) - sq(q2) - sq(q3)) - vd*(2.0f*q0*q2 - 2.0f*q1*q3) + (ve - vwe)*(2.0f*q0*q3 + 2.0f*q1*q2);

        SH_BETA[1] = rel_wind_earth(1)*(sq(q0) - sq(q1) + sq(q2) - sq(q3)) + vd*(2.0f*q0*q1 + 2.0f*q2*q3) - rel_wind_earth(0)*(2.0f*q0*q3 - 2.0f*q1*q2);
        SH_BETA[2] = rel_wind_earth(0);
        SH_BETA[3] = rel_wind_earth(1);
        SH_BETA[4] = 1.0f/sq(SH_BETA[0]);
        SH_BETA[5] = 1.0f/SH_BETA[0];
        SH_BETA[6] = SH_BETA[5]*(sq(q0) - sq(q1) + sq(q2) - sq(q3));
        SH_BETA[7] = sq(q0) + sq(q1) - sq(q2) - sq(q3);
        SH_BETA[8] = 2.0f*q0*SH_BETA[3] - 2.0f*q3*SH_BETA[2] + 2.0f*q1*vd;
        SH_BETA[9] = 2.0f*q0*SH_BETA[2] + 2.0f*q3*SH_BETA[3] - 2.0f*q2*vd;
        SH_BETA[10] = 2.0f*q2*SH_BETA[2] - 2.0f*q1*SH_BETA[3] + 2.0f*q0*vd;
        SH_BETA[11] = 2.0f*q1*SH_BETA[2] + 2.0f*q2*SH_BETA[3] + 2.0f*q3*vd;
        SH_BETA[12] = 2.0f*q0*q3;

        Vector24f H_BETA;
        H_BETA(0) = SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9];
        H_BETA(1) = SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11];
        H_BETA(2) = SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10];
        H_BETA(3) = - SH_BETA[5]*SH_BETA[9] - SH_BETA[1]*SH_BETA[4]*SH_BETA[8];
        H_BETA(4) = - SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) - SH_BETA[1]*SH_BETA[4]*SH_BETA[7];
        H_BETA(5) = SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2);
        H_BETA(6) = SH_BETA[5]*(2.0f*q0*q1 + 2.0f*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2.0f*q0*q2 - 2.0f*q1*q3);
        H_BETA(22) = SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7];
        H_BETA(23) = SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2) - SH_BETA[6];

        _beta_innov_var = (R_BETA - (SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7])*(P(22,4)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P(4,4)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P(5,4)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) - P(23,4)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) + P(0,4)*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P(1,4)*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P(2,4)*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P(3,4)*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P(6,4)*(SH_BETA[5]*(2.0f*q0*q1 + 2.0f*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2.0f*q0*q2 - 2.0f*q1*q3))) + (SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7])*(P(22,22)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P(4,22)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P(5,22)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) - P(23,22)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) + P(0,22)*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P(1,22)*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P(2,22)*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P(3,22)*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P(6,22)*(SH_BETA[5]*(2.0f*q0*q1 + 2.0f*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2.0f*q0*q2 - 2.0f*q1*q3))) + (SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2))*(P(22,5)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P(4,5)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P(5,5)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) - P(23,5)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) + P(0,5)*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P(1,5)*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P(2,5)*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P(3,5)*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P(6,5)*(SH_BETA[5]*(2.0f*q0*q1 + 2.0f*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2.0f*q0*q2 - 2.0f*q1*q3))) - (SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2))*(P(22,23)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P(4,23)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P(5,23)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) - P(23,23)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) + P(0,23)*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P(1,23)*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P(2,23)*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P(3,23)*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P(6,23)*(SH_BETA[5]*(2.0f*q0*q1 + 2.0f*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2.0f*q0*q2 - 2.0f*q1*q3))) + (SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9])*(P(22,0)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P(4,0)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P(5,0)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) - P(23,0)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) + P(0,0)*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P(1,0)*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P(2,0)*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P(3,0)*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P(6,0)*(SH_BETA[5]*(2.0f*q0*q1 + 2.0f*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2.0f*q0*q2 - 2.0f*q1*q3))) + (SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11])*(P(22,1)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P(4,1)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P(5,1)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) - P(23,1)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) + P(0,1)*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P(1,1)*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P(2,1)*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P(3,1)*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P(6,1)*(SH_BETA[5]*(2.0f*q0*q1 + 2.0f*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2.0f*q0*q2 - 2.0f*q1*q3))) + (SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10])*(P(22,2)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P(4,2)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P(5,2)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) - P(23,2)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) + P(0,2)*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P(1,2)*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P(2,2)*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P(3,2)*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P(6,2)*(SH_BETA[5]*(2.0f*q0*q1 + 2.0f*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2.0f*q0*q2 - 2.0f*q1*q3))) - (SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8])*(P(22,3)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P(4,3)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P(5,3)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) - P(23,3)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) + P(0,3)*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P(1,3)*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P(2,3)*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P(3,3)*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P(6,3)*(SH_BETA[5]*(2.0f*q0*q1 + 2.0f*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2.0f*q0*q2 - 2.0f*q1*q3))) + (SH_BETA[5]*(2.0f*q0*q1 + 2.0f*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2.0f*q0*q2 - 2.0f*q1*q3))*(P(22,6)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P(4,6)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P(5,6)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) - P(23,6)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) + P(0,6)*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P(1,6)*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P(2,6)*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P(3,6)*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P(6,6)*(SH_BETA[5]*(2.0f*q0*q1 + 2.0f*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2.0f*q0*q2 - 2.0f*q1*q3))));

        // intermediate variables for optimising calculations of the Kalman gain
        float SK_BETA[8];
        SK_BETA[0] = 1.0f / _beta_innov_var;
        SK_BETA[1] = SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7];
        SK_BETA[2] = SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2);
        SK_BETA[3] = SH_BETA[5]*(2.0f*q0*q1 + 2.0f*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2.0f*q0*q2 - 2.0f*q1*q3);
        SK_BETA[4] = SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11];
        SK_BETA[5] = SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9];
        SK_BETA[6] = SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10];
        SK_BETA[7] = SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8];

        Vector24f Kfusion;
        Kfusion(0) = SK_BETA[0]*(P(0,0)*SK_BETA[5] + P(0,1)*SK_BETA[4] - P(0,4)*SK_BETA[1] + P(0,5)*SK_BETA[2] + P(0,2)*SK_BETA[6] + P(0,6)*SK_BETA[3] - P(0,3)*SK_BETA[7] + P(0,22)*SK_BETA[1] - P(0,23)*SK_BETA[2]);
        Kfusion(1) = SK_BETA[0]*(P(1,0)*SK_BETA[5] + P(1,1)*SK_BETA[4] - P(1,4)*SK_BETA[1] + P(1,5)*SK_BETA[2] + P(1,2)*SK_BETA[6] + P(1,6)*SK_BETA[3] - P(1,3)*SK_BETA[7] + P(1,22)*SK_BETA[1] - P(1,23)*SK_BETA[2]);
        Kfusion(2) = SK_BETA[0]*(P(2,0)*SK_BETA[5] + P(2,1)*SK_BETA[4] - P(2,4)*SK_BETA[1] + P(2,5)*SK_BETA[2] + P(2,2)*SK_BETA[6] + P(2,6)*SK_BETA[3] - P(2,3)*SK_BETA[7] + P(2,22)*SK_BETA[1] - P(2,23)*SK_BETA[2]);
        Kfusion(3) = SK_BETA[0]*(P(3,0)*SK_BETA[5] + P(3,1)*SK_BETA[4] - P(3,4)*SK_BETA[1] + P(3,5)*SK_BETA[2] + P(3,2)*SK_BETA[6] + P(3,6)*SK_BETA[3] - P(3,3)*SK_BETA[7] + P(3,22)*SK_BETA[1] - P(3,23)*SK_BETA[2]);
        Kfusion(4) = SK_BETA[0]*(P(4,0)*SK_BETA[5] + P(4,1)*SK_BETA[4] - P(4,4)*SK_BETA[1] + P(4,5)*SK_BETA[2] + P(4,2)*SK_BETA[6] + P(4,6)*SK_BETA[3] - P(4,3)*SK_BETA[7] + P(4,22)*SK_BETA[1] - P(4,23)*SK_BETA[2]);
        Kfusion(5) = SK_BETA[0]*(P(5,0)*SK_BETA[5] + P(5,1)*SK_BETA[4] - P(5,4)*SK_BETA[1] + P(5,5)*SK_BETA[2] + P(5,2)*SK_BETA[6] + P(5,6)*SK_BETA[3] - P(5,3)*SK_BETA[7] + P(5,22)*SK_BETA[1] - P(5,23)*SK_BETA[2]);
        Kfusion(6) = SK_BETA[0]*(P(6,0)*SK_BETA[5] + P(6,1)*SK_BETA[4] - P(6,4)*SK_BETA[1] + P(6,5)*SK_BETA[2] + P(6,2)*SK_BETA[6] + P(6,6)*SK_BETA[3] - P(6,3)*SK_BETA[7] + P(6,22)*SK_BETA[1] - P(6,23)*SK_BETA[2]);
        Kfusion(7) = SK_BETA[0]*(P(7,0)*SK_BETA[5] + P(7,1)*SK_BETA[4] - P(7,4)*SK_BETA[1] + P(7,5)*SK_BETA[2] + P(7,2)*SK_BETA[6] + P(7,6)*SK_BETA[3] - P(7,3)*SK_BETA[7] + P(7,22)*SK_BETA[1] - P(7,23)*SK_BETA[2]);
        Kfusion(8) = SK_BETA[0]*(P(8,0)*SK_BETA[5] + P(8,1)*SK_BETA[4] - P(8,4)*SK_BETA[1] + P(8,5)*SK_BETA[2] + P(8,2)*SK_BETA[6] + P(8,6)*SK_BETA[3] - P(8,3)*SK_BETA[7] + P(8,22)*SK_BETA[1] - P(8,23)*SK_BETA[2]);
        Kfusion(9) = SK_BETA[0]*(P(9,0)*SK_BETA[5] + P(9,1)*SK_BETA[4] - P(9,4)*SK_BETA[1] + P(9,5)*SK_BETA[2] + P(9,2)*SK_BETA[6] + P(9,6)*SK_BETA[3] - P(9,3)*SK_BETA[7] + P(9,22)*SK_BETA[1] - P(9,23)*SK_BETA[2]);
        Kfusion(10) = SK_BETA[0]*(P(10,0)*SK_BETA[5] + P(10,1)*SK_BETA[4] - P(10,4)*SK_BETA[1] + P(10,5)*SK_BETA[2] + P(10,2)*SK_BETA[6] + P(10,6)*SK_BETA[3] - P(10,3)*SK_BETA[7] + P(10,22)*SK_BETA[1] - P(10,23)*SK_BETA[2]);
        Kfusion(11) = SK_BETA[0]*(P(11,0)*SK_BETA[5] + P(11,1)*SK_BETA[4] - P(11,4)*SK_BETA[1] + P(11,5)*SK_BETA[2] + P(11,2)*SK_BETA[6] + P(11,6)*SK_BETA[3] - P(11,3)*SK_BETA[7] + P(11,22)*SK_BETA[1] - P(11,23)*SK_BETA[2]);
        Kfusion(12) = SK_BETA[0]*(P(12,0)*SK_BETA[5] + P(12,1)*SK_BETA[4] - P(12,4)*SK_BETA[1] + P(12,5)*SK_BETA[2] + P(12,2)*SK_BETA[6] + P(12,6)*SK_BETA[3] - P(12,3)*SK_BETA[7] + P(12,22)*SK_BETA[1] - P(12,23)*SK_BETA[2]);
        Kfusion(13) = SK_BETA[0]*(P(13,0)*SK_BETA[5] + P(13,1)*SK_BETA[4] - P(13,4)*SK_BETA[1] + P(13,5)*SK_BETA[2] + P(13,2)*SK_BETA[6] + P(13,6)*SK_BETA[3] - P(13,3)*SK_BETA[7] + P(13,22)*SK_BETA[1] - P(13,23)*SK_BETA[2]);
        Kfusion(14) = SK_BETA[0]*(P(14,0)*SK_BETA[5] + P(14,1)*SK_BETA[4] - P(14,4)*SK_BETA[1] + P(14,5)*SK_BETA[2] + P(14,2)*SK_BETA[6] + P(14,6)*SK_BETA[3] - P(14,3)*SK_BETA[7] + P(14,22)*SK_BETA[1] - P(14,23)*SK_BETA[2]);
        Kfusion(15) = SK_BETA[0]*(P(15,0)*SK_BETA[5] + P(15,1)*SK_BETA[4] - P(15,4)*SK_BETA[1] + P(15,5)*SK_BETA[2] + P(15,2)*SK_BETA[6] + P(15,6)*SK_BETA[3] - P(15,3)*SK_BETA[7] + P(15,22)*SK_BETA[1] - P(15,23)*SK_BETA[2]);
        Kfusion(16) = SK_BETA[0]*(P(16,0)*SK_BETA[5] + P(16,1)*SK_BETA[4] - P(16,4)*SK_BETA[1] + P(16,5)*SK_BETA[2] + P(16,2)*SK_BETA[6] + P(16,6)*SK_BETA[3] - P(16,3)*SK_BETA[7] + P(16,22)*SK_BETA[1] - P(16,23)*SK_BETA[2]);
        Kfusion(17) = SK_BETA[0]*(P(17,0)*SK_BETA[5] + P(17,1)*SK_BETA[4] - P(17,4)*SK_BETA[1] + P(17,5)*SK_BETA[2] + P(17,2)*SK_BETA[6] + P(17,6)*SK_BETA[3] - P(17,3)*SK_BETA[7] + P(17,22)*SK_BETA[1] - P(17,23)*SK_BETA[2]);
        Kfusion(18) = SK_BETA[0]*(P(18,0)*SK_BETA[5] + P(18,1)*SK_BETA[4] - P(18,4)*SK_BETA[1] + P(18,5)*SK_BETA[2] + P(18,2)*SK_BETA[6] + P(18,6)*SK_BETA[3] - P(18,3)*SK_BETA[7] + P(18,22)*SK_BETA[1] - P(18,23)*SK_BETA[2]);
        Kfusion(19) = SK_BETA[0]*(P(19,0)*SK_BETA[5] + P(19,1)*SK_BETA[4] - P(19,4)*SK_BETA[1] + P(19,5)*SK_BETA[2] + P(19,2)*SK_BETA[6] + P(19,6)*SK_BETA[3] - P(19,3)*SK_BETA[7] + P(19,22)*SK_BETA[1] - P(19,23)*SK_BETA[2]);
        Kfusion(20) = SK_BETA[0]*(P(20,0)*SK_BETA[5] + P(20,1)*SK_BETA[4] - P(20,4)*SK_BETA[1] + P(20,5)*SK_BETA[2] + P(20,2)*SK_BETA[6] + P(20,6)*SK_BETA[3] - P(20,3)*SK_BETA[7] + P(20,22)*SK_BETA[1] - P(20,23)*SK_BETA[2]);
        Kfusion(21) = SK_BETA[0]*(P(21,0)*SK_BETA[5] + P(21,1)*SK_BETA[4] - P(21,4)*SK_BETA[1] + P(21,5)*SK_BETA[2] + P(21,2)*SK_BETA[6] + P(21,6)*SK_BETA[3] - P(21,3)*SK_BETA[7] + P(21,22)*SK_BETA[1] - P(21,23)*SK_BETA[2]);

        // copy results
        Hfusion_matlab = H_BETA;
        Kfusion_matlab = Kfusion;
    }

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
        printf("Fail: Sideslip Hfusion max diff fraction = %e , old = %e , new = %e , location index = %i\n",max_diff_fraction, max_old, max_new, max_row);
    } else {
        printf("Pass: Sideslip Hfusion max diff fraction = %e\n",max_diff_fraction);
    }

    // find largest Kalman gain difference as a fraction of the matlab value
    max_diff_fraction = 0.0f;
    for (int row=0; row<4; row++) {
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
        printf("Fail: Sideslip Kfusion max diff fraction = %e , old = %e , new = %e , location index = %i\n",max_diff_fraction, max_old, max_new, max_row);
    } else {
        printf("Pass: Sideslip Kfusion max diff fraction = %e\n",max_diff_fraction);
    }

    return 0;
}
