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

    float airspeed_innov_var;

    Vector24f Kfusion; // Kalman gain vector

    Vector24f Hfusion_sympy;
    Vector24f Kfusion_sympy;

    Vector24f Hfusion_matlab;
    Vector24f Kfusion_matlab;

    const float R_TAS = sq(1.5f);

    const bool update_wind_only = false;

    // get latest velocity in earth frame
    const float vn = 9.0f;
    const float ve = 12.0f;
    const float vd = -1.5f;

    // get latest wind velocity in earth frame
    const float vwn = -4.0f;
    const float vwe = 3.0f;

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
        // Intermediate variables
        const float HK0 = vn - vwn;
        const float HK1 = ve - vwe;
        const float HK2 = ecl::powf(HK0, 2) + ecl::powf(HK1, 2) + ecl::powf(vd, 2);
        const float v_tas_pred = sqrtf(HK2); // predicted airspeed
        //const float HK3 = powf(HK2, -1.0F/2.0F);
        if (v_tas_pred < 1.0f) {
            // calculation can be badly conditioned for very low airspeed values so don't fuse this time
            return 0;
        }
        const float HK3 = 1.0f / v_tas_pred;
        const float HK4 = HK0*HK3;
        const float HK5 = HK1*HK3;
        const float HK6 = 1.0F/HK2;
        const float HK7 = HK0*P(4,6) - HK0*P(6,22) + HK1*P(5,6) - HK1*P(6,23) + P(6,6)*vd;
        const float HK8 = HK1*P(5,23);
        const float HK9 = HK0*P(4,5) - HK0*P(5,22) + HK1*P(5,5) - HK8 + P(5,6)*vd;
        const float HK10 = HK1*HK6;
        const float HK11 = HK0*P(4,22);
        const float HK12 = HK0*P(4,4) - HK1*P(4,23) + HK1*P(4,5) - HK11 + P(4,6)*vd;
        const float HK13 = HK0*HK6;
        const float HK14 = -HK0*P(22,23) + HK0*P(4,23) - HK1*P(23,23) + HK8 + P(6,23)*vd;
        const float HK15 = -HK0*P(22,22) - HK1*P(22,23) + HK1*P(5,22) + HK11 + P(6,22)*vd;
        const float HK16 = HK3/(-HK10*HK14 + HK10*HK9 + HK12*HK13 - HK13*HK15 + HK6*HK7*vd + R_TAS);

        // Observation Jacobians
        SparseVector24f<4,5,6,22,23> Hfusion;
        Hfusion.at<4>() = HK4;
        Hfusion.at<5>() = HK5;
        Hfusion.at<6>() = HK3*vd;
        Hfusion.at<22>() = -HK4;
        Hfusion.at<23>() = -HK5;

        if (true) {
            // we have no other source of aiding, so use airspeed measurements to correct states
            for (unsigned row = 0; row <= 3; row++) {
                Kfusion(row) = HK16*(HK0*P(4,row) - HK0*P(row,22) + HK1*P(5,row) - HK1*P(row,23) + P(6,row)*vd);
            }

            Kfusion(4) = HK12*HK16;
            Kfusion(5) = HK16*HK9;
            Kfusion(6) = HK16*HK7;

            for (unsigned row = 7; row <= 21; row++) {
                Kfusion(row) = HK16*(HK0*P(4,row) - HK0*P(row,22) + HK1*P(5,row) - HK1*P(row,23) + P(6,row)*vd);
            }
        }

        Kfusion(22) = HK15*HK16;
        Kfusion(23) = HK14*HK16;

        // save output
        Hfusion_sympy(4) = Hfusion.at<4>();
        Hfusion_sympy(5) = Hfusion.at<5>();
        Hfusion_sympy(6) = Hfusion.at<6>();
        Hfusion_sympy(22) = Hfusion.at<22>();
        Hfusion_sympy(23) = Hfusion.at<23>();
        Kfusion_sympy = Kfusion;
    }
    // repeat calculation using matlab generated equations
    {
        const float v_tas_pred = sqrtf((ve - vwe) * (ve - vwe) + (vn - vwn) * (vn - vwn) + vd * vd);

        // intermediate variable from algebraic optimisation
        float SH_TAS[3];
        SH_TAS[0] = 1.0f/v_tas_pred;
        SH_TAS[1] = (SH_TAS[0]*(2.0f*ve - 2.0f*vwe))*0.5f;
        SH_TAS[2] = (SH_TAS[0]*(2.0f*vn - 2.0f*vwn))*0.5f;

        // Observation Jacobian
        Vector24f H_TAS = {};
        H_TAS(4) = SH_TAS[2];
        H_TAS(5) = SH_TAS[1];
        H_TAS(6) = vd*SH_TAS[0];
        H_TAS(22) = -SH_TAS[2];
        H_TAS(23) = -SH_TAS[1];

        airspeed_innov_var = (R_TAS + SH_TAS[2]*(P(4,4)*SH_TAS[2] + P(5,4)*SH_TAS[1] - P(22,4)*SH_TAS[2] - P(23,4)*SH_TAS[1] + P(6,4)*vd*SH_TAS[0]) + SH_TAS[1]*(P(4,5)*SH_TAS[2] + P(5,5)*SH_TAS[1] - P(22,5)*SH_TAS[2] - P(23,5)*SH_TAS[1] + P(6,5)*vd*SH_TAS[0]) - SH_TAS[2]*(P(4,22)*SH_TAS[2] + P(5,22)*SH_TAS[1] - P(22,22)*SH_TAS[2] - P(23,22)*SH_TAS[1] + P(6,22)*vd*SH_TAS[0]) - SH_TAS[1]*(P(4,23)*SH_TAS[2] + P(5,23)*SH_TAS[1] - P(22,23)*SH_TAS[2] - P(23,23)*SH_TAS[1] + P(6,23)*vd*SH_TAS[0]) + vd*SH_TAS[0]*(P(4,6)*SH_TAS[2] + P(5,6)*SH_TAS[1] - P(22,6)*SH_TAS[2] - P(23,6)*SH_TAS[1] + P(6,6)*vd*SH_TAS[0]));

        float SK_TAS[2];
        SK_TAS[0] = 1.0f / airspeed_innov_var;
        SK_TAS[1] = SH_TAS[1];

        // Kalman gain
        Kfusion(0) = SK_TAS[0]*(P(0,4)*SH_TAS[2] - P(0,22)*SH_TAS[2] + P(0,5)*SK_TAS[1] - P(0,23)*SK_TAS[1] + P(0,6)*vd*SH_TAS[0]);
        Kfusion(1) = SK_TAS[0]*(P(1,4)*SH_TAS[2] - P(1,22)*SH_TAS[2] + P(1,5)*SK_TAS[1] - P(1,23)*SK_TAS[1] + P(1,6)*vd*SH_TAS[0]);
        Kfusion(2) = SK_TAS[0]*(P(2,4)*SH_TAS[2] - P(2,22)*SH_TAS[2] + P(2,5)*SK_TAS[1] - P(2,23)*SK_TAS[1] + P(2,6)*vd*SH_TAS[0]);
        Kfusion(3) = SK_TAS[0]*(P(3,4)*SH_TAS[2] - P(3,22)*SH_TAS[2] + P(3,5)*SK_TAS[1] - P(3,23)*SK_TAS[1] + P(3,6)*vd*SH_TAS[0]);
        Kfusion(4) = SK_TAS[0]*(P(4,4)*SH_TAS[2] - P(4,22)*SH_TAS[2] + P(4,5)*SK_TAS[1] - P(4,23)*SK_TAS[1] + P(4,6)*vd*SH_TAS[0]);
        Kfusion(5) = SK_TAS[0]*(P(5,4)*SH_TAS[2] - P(5,22)*SH_TAS[2] + P(5,5)*SK_TAS[1] - P(5,23)*SK_TAS[1] + P(5,6)*vd*SH_TAS[0]);
        Kfusion(6) = SK_TAS[0]*(P(6,4)*SH_TAS[2] - P(6,22)*SH_TAS[2] + P(6,5)*SK_TAS[1] - P(6,23)*SK_TAS[1] + P(6,6)*vd*SH_TAS[0]);
        Kfusion(7) = SK_TAS[0]*(P(7,4)*SH_TAS[2] - P(7,22)*SH_TAS[2] + P(7,5)*SK_TAS[1] - P(7,23)*SK_TAS[1] + P(7,6)*vd*SH_TAS[0]);
        Kfusion(8) = SK_TAS[0]*(P(8,4)*SH_TAS[2] - P(8,22)*SH_TAS[2] + P(8,5)*SK_TAS[1] - P(8,23)*SK_TAS[1] + P(8,6)*vd*SH_TAS[0]);
        Kfusion(9) = SK_TAS[0]*(P(9,4)*SH_TAS[2] - P(9,22)*SH_TAS[2] + P(9,5)*SK_TAS[1] - P(9,23)*SK_TAS[1] + P(9,6)*vd*SH_TAS[0]);
        Kfusion(10) = SK_TAS[0]*(P(10,4)*SH_TAS[2] - P(10,22)*SH_TAS[2] + P(10,5)*SK_TAS[1] - P(10,23)*SK_TAS[1] + P(10,6)*vd*SH_TAS[0]);
        Kfusion(11) = SK_TAS[0]*(P(11,4)*SH_TAS[2] - P(11,22)*SH_TAS[2] + P(11,5)*SK_TAS[1] - P(11,23)*SK_TAS[1] + P(11,6)*vd*SH_TAS[0]);
        Kfusion(12) = SK_TAS[0]*(P(12,4)*SH_TAS[2] - P(12,22)*SH_TAS[2] + P(12,5)*SK_TAS[1] - P(12,23)*SK_TAS[1] + P(12,6)*vd*SH_TAS[0]);
        Kfusion(13) = SK_TAS[0]*(P(13,4)*SH_TAS[2] - P(13,22)*SH_TAS[2] + P(13,5)*SK_TAS[1] - P(13,23)*SK_TAS[1] + P(13,6)*vd*SH_TAS[0]);
        Kfusion(14) = SK_TAS[0]*(P(14,4)*SH_TAS[2] - P(14,22)*SH_TAS[2] + P(14,5)*SK_TAS[1] - P(14,23)*SK_TAS[1] + P(14,6)*vd*SH_TAS[0]);
        Kfusion(15) = SK_TAS[0]*(P(15,4)*SH_TAS[2] - P(15,22)*SH_TAS[2] + P(15,5)*SK_TAS[1] - P(15,23)*SK_TAS[1] + P(15,6)*vd*SH_TAS[0]);
        Kfusion(16) = SK_TAS[0]*(P(16,4)*SH_TAS[2] - P(16,22)*SH_TAS[2] + P(16,5)*SK_TAS[1] - P(16,23)*SK_TAS[1] + P(16,6)*vd*SH_TAS[0]);
        Kfusion(17) = SK_TAS[0]*(P(17,4)*SH_TAS[2] - P(17,22)*SH_TAS[2] + P(17,5)*SK_TAS[1] - P(17,23)*SK_TAS[1] + P(17,6)*vd*SH_TAS[0]);
        Kfusion(18) = SK_TAS[0]*(P(18,4)*SH_TAS[2] - P(18,22)*SH_TAS[2] + P(18,5)*SK_TAS[1] - P(18,23)*SK_TAS[1] + P(18,6)*vd*SH_TAS[0]);
        Kfusion(19) = SK_TAS[0]*(P(19,4)*SH_TAS[2] - P(19,22)*SH_TAS[2] + P(19,5)*SK_TAS[1] - P(19,23)*SK_TAS[1] + P(19,6)*vd*SH_TAS[0]);
        Kfusion(20) = SK_TAS[0]*(P(20,4)*SH_TAS[2] - P(20,22)*SH_TAS[2] + P(20,5)*SK_TAS[1] - P(20,23)*SK_TAS[1] + P(20,6)*vd*SH_TAS[0]);
        Kfusion(21) = SK_TAS[0]*(P(21,4)*SH_TAS[2] - P(21,22)*SH_TAS[2] + P(21,5)*SK_TAS[1] - P(21,23)*SK_TAS[1] + P(21,6)*vd*SH_TAS[0]);
        Kfusion(22) = SK_TAS[0]*(P(22,4)*SH_TAS[2] - P(22,22)*SH_TAS[2] + P(22,5)*SK_TAS[1] - P(22,23)*SK_TAS[1] + P(22,6)*vd*SH_TAS[0]);
        Kfusion(23) = SK_TAS[0]*(P(23,4)*SH_TAS[2] - P(23,22)*SH_TAS[2] + P(23,5)*SK_TAS[1] - P(23,23)*SK_TAS[1] + P(23,6)*vd*SH_TAS[0]);

        // save output;
        Hfusion_matlab = H_TAS;
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
        if (Hfusion_sympy(row) - Hfusion_matlab(row) != 0.0f) {
            printf("new,old Hfusion(%i) = %e,%e\n",row,Hfusion_sympy(row),Hfusion_matlab(row));
        }
        if (diff_fraction > max_diff_fraction) {
            max_diff_fraction = diff_fraction;
            max_row = row;
            max_old = Hfusion_matlab(row);
            max_new = Hfusion_sympy(row);
        }
    }

    if (max_diff_fraction > 1e-5f) {
        printf("Fail: Airspeed Hfusion max diff fraction = %e , old = %e , new = %e , location index = %i\n",max_diff_fraction, max_old, max_new, max_row);
    } else {
        printf("Pass: Airspeed Hfusion max diff fraction = %e\n",max_diff_fraction);
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
        // if (Kfusion_sympy(row) - Kfusion(row) != 0.0f) {
        //     printf("new,old Kfusion(%i) = %e,%e\n",row,Kfusion_sympy(row),Kfusion(row));
        // }
        if (diff_fraction > max_diff_fraction) {
            max_diff_fraction = diff_fraction;
            max_row = row;
            max_old = Kfusion(row);
            max_new = Kfusion_sympy(row);
        }
    }

    if (max_diff_fraction > 1e-5f) {
        printf("Fail: Airspeed Kfusion max diff fraction = %e , old = %e , new = %e , location index = %i\n",max_diff_fraction, max_old, max_new, max_row);
    } else {
        printf("Pass: Airspeed Kfusion max diff fraction = %e\n",max_diff_fraction);
    }

    return 0;
}
