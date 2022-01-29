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

    float Hfusion[24];
    Vector24f H_MAG;
    Vector24f Kfusion;
    float mag_innov_var;

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

    const float magN = 2.0f * ((float)rand() - 0.5f);
    const float magE = 2.0f * ((float)rand() - 0.5f);
    const float magD = 2.0f * ((float)rand() - 0.5f);

    const float R_MAG = sq(0.05f);
    const bool update_all_states = true;

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

    // common expressions used by sympy generated equations
    // calculate intermediate variables used for X axis innovation variance, observation Jacobians and Kalman gainss
    const float HKX0 = -magD*q2 + magE*q3 + magN*q0;
    const float HKX1 = magD*q3 + magE*q2 + magN*q1;
    const float HKX2 = magE*q1;
    const float HKX3 = magD*q0;
    const float HKX4 = magN*q2;
    const float HKX5 = magD*q1 + magE*q0 - magN*q3;
    const float HKX6 = powf(q0, 2) + powf(q1, 2) - powf(q2, 2) - powf(q3, 2);
    const float HKX7 = q0*q3 + q1*q2;
    const float HKX8 = q1*q3;
    const float HKX9 = q0*q2;
    const float HKX10 = 2*HKX7;
    const float HKX11 = -2*HKX8 + 2*HKX9;
    const float HKX12 = 2*HKX1;
    const float HKX13 = 2*HKX0;
    const float HKX14 = -2*HKX2 + 2*HKX3 + 2*HKX4;
    const float HKX15 = 2*HKX5;
    const float HKX16 = HKX10*P(0,17) - HKX11*P(0,18) + HKX12*P(0,1) + HKX13*P(0,0) - HKX14*P(0,2) + HKX15*P(0,3) + HKX6*P(0,16) + P(0,19);
    const float HKX17 = HKX10*P(16,17) - HKX11*P(16,18) + HKX12*P(1,16) + HKX13*P(0,16) - HKX14*P(2,16) + HKX15*P(3,16) + HKX6*P(16,16) + P(16,19);
    const float HKX18 = HKX10*P(17,18) - HKX11*P(18,18) + HKX12*P(1,18) + HKX13*P(0,18) - HKX14*P(2,18) + HKX15*P(3,18) + HKX6*P(16,18) + P(18,19);
    const float HKX19 = HKX10*P(2,17) - HKX11*P(2,18) + HKX12*P(1,2) + HKX13*P(0,2) - HKX14*P(2,2) + HKX15*P(2,3) + HKX6*P(2,16) + P(2,19);
    const float HKX20 = HKX10*P(17,17) - HKX11*P(17,18) + HKX12*P(1,17) + HKX13*P(0,17) - HKX14*P(2,17) + HKX15*P(3,17) + HKX6*P(16,17) + P(17,19);
    const float HKX21 = HKX10*P(3,17) - HKX11*P(3,18) + HKX12*P(1,3) + HKX13*P(0,3) - HKX14*P(2,3) + HKX15*P(3,3) + HKX6*P(3,16) + P(3,19);
    const float HKX22 = HKX10*P(1,17) - HKX11*P(1,18) + HKX12*P(1,1) + HKX13*P(0,1) - HKX14*P(1,2) + HKX15*P(1,3) + HKX6*P(1,16) + P(1,19);
    const float HKX23 = HKX10*P(17,19) - HKX11*P(18,19) + HKX12*P(1,19) + HKX13*P(0,19) - HKX14*P(2,19) + HKX15*P(3,19) + HKX6*P(16,19) + P(19,19);
    const float HKX24 = 1.0F/(HKX10*HKX20 - HKX11*HKX18 + HKX12*HKX22 + HKX13*HKX16 - HKX14*HKX19 + HKX15*HKX21 + HKX17*HKX6 + HKX23 + R_MAG);

    // common expressions used by matlab generated equations
    float SH_MAG[9];
    SH_MAG[0] = 2.0f*magD*q3 + 2.0f*magE*q2 + 2.0f*magN*q1;
    SH_MAG[1] = 2.0f*magD*q0 - 2.0f*magE*q1 + 2.0f*magN*q2;
    SH_MAG[2] = 2.0f*magD*q1 + 2.0f*magE*q0 - 2.0f*magN*q3;
    SH_MAG[3] = sq(q3);
    SH_MAG[4] = sq(q2);
    SH_MAG[5] = sq(q1);
    SH_MAG[6] = sq(q0);
    SH_MAG[7] = 2.0f*magN*q0;
    SH_MAG[8] = 2.0f*magE*q3;

    // Compare X axis equations
    {
        mag_innov_var = (HKX10*HKX20 - HKX11*HKX18 + HKX12*HKX22 + HKX13*HKX16 - HKX14*HKX19 + HKX15*HKX21 + HKX17*HKX6 + HKX23 + R_MAG);
        float HK50 = 1.0F/mag_innov_var;

        // Calculate X axis observation jacobians
        memset(Hfusion, 0, sizeof(Hfusion));
        Hfusion[0] = 2*HKX0;
        Hfusion[1] = 2*HKX1;
        Hfusion[2] = 2*HKX2 - 2*HKX3 - 2*HKX4;
        Hfusion[3] = 2*HKX5;
        Hfusion[16] = HKX6;
        Hfusion[17] = 2*HKX7;
        Hfusion[18] = 2*HKX8 - 2*HKX9;
        Hfusion[19] = 1;

        // Calculate X axis Kalman gains
        if (update_all_states) {
            Kfusion(0) = HKX16*HKX24;
            Kfusion(1) = HKX22*HKX24;
            Kfusion(2) = HKX19*HKX24;
            Kfusion(3) = HKX21*HKX24;

            for (unsigned row = 4; row <= 15; row++) {
                Kfusion(row) = HKX24*(HKX10*P(row,17) - HKX11*P(row,18) + HKX12*P(1,row) + HKX13*P(0,row) - HKX14*P(2,row) + HKX15*P(3,row) + HKX6*P(row,16) + P(row,19));
            }

            for (unsigned row = 22; row <= 23; row++) {
                Kfusion(row) = HKX24*(HKX10*P(17,row) - HKX11*P(18,row) + HKX12*P(1,row) + HKX13*P(0,row) - HKX14*P(2,row) + HKX15*P(3,row) + HKX6*P(16,row) + P(19,row));
            }
        }

        Kfusion(16) = HKX17*HKX24;
        Kfusion(17) = HKX20*HKX24;
        Kfusion(18) = HKX18*HKX24;
        Kfusion(19) = HKX23*HKX24;

        for (unsigned row = 20; row <= 21; row++) {
            Kfusion(row) = HKX24*(HKX10*P(17,row) - HKX11*P(18,row) + HKX12*P(1,row) + HKX13*P(0,row) - HKX14*P(2,row) + HKX15*P(3,row) + HKX6*P(16,row) + P(19,row));
        }

        // save output and repeat calculation using legacy matlab generated code
        float Hfusion_sympy[24];
        Vector24f Kfusion_sympy;
        for (int row=0; row<24; row++) {
            Hfusion_sympy[row] = Hfusion[row];
            Kfusion_sympy(row) = Kfusion(row);
        }

        // repeat calculation using matlab generated equations
        // X axis innovation variance
        mag_innov_var = (P(19,19) + R_MAG + P(1,19)*SH_MAG[0] - P(2,19)*SH_MAG[1] + P(3,19)*SH_MAG[2] - P(16,19)*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + (2.0f*q0*q3 + 2.0f*q1*q2)*(P(19,17) + P(1,17)*SH_MAG[0] - P(2,17)*SH_MAG[1] + P(3,17)*SH_MAG[2] - P(16,17)*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P(17,17)*(2.0f*q0*q3 + 2.0f*q1*q2) - P(18,17)*(2.0f*q0*q2 - 2.0f*q1*q3) + P(0,17)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - (2.0f*q0*q2 - 2.0f*q1*q3)*(P(19,18) + P(1,18)*SH_MAG[0] - P(2,18)*SH_MAG[1] + P(3,18)*SH_MAG[2] - P(16,18)*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P(17,18)*(2.0f*q0*q3 + 2.0f*q1*q2) - P(18,18)*(2.0f*q0*q2 - 2.0f*q1*q3) + P(0,18)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + (SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)*(P(19,0) + P(1,0)*SH_MAG[0] - P(2,0)*SH_MAG[1] + P(3,0)*SH_MAG[2] - P(16,0)*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P(17,0)*(2.0f*q0*q3 + 2.0f*q1*q2) - P(18,0)*(2.0f*q0*q2 - 2.0f*q1*q3) + P(0,0)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + P(17,19)*(2.0f*q0*q3 + 2.0f*q1*q2) - P(18,19)*(2.0f*q0*q2 - 2.0f*q1*q3) + SH_MAG[0]*(P(19,1) + P(1,1)*SH_MAG[0] - P(2,1)*SH_MAG[1] + P(3,1)*SH_MAG[2] - P(16,1)*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P(17,1)*(2.0f*q0*q3 + 2.0f*q1*q2) - P(18,1)*(2.0f*q0*q2 - 2.0f*q1*q3) + P(0,1)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - SH_MAG[1]*(P(19,2) + P(1,2)*SH_MAG[0] - P(2,2)*SH_MAG[1] + P(3,2)*SH_MAG[2] - P(16,2)*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P(17,2)*(2.0f*q0*q3 + 2.0f*q1*q2) - P(18,2)*(2.0f*q0*q2 - 2.0f*q1*q3) + P(0,2)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + SH_MAG[2]*(P(19,3) + P(1,3)*SH_MAG[0] - P(2,3)*SH_MAG[1] + P(3,3)*SH_MAG[2] - P(16,3)*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P(17,3)*(2.0f*q0*q3 + 2.0f*q1*q2) - P(18,3)*(2.0f*q0*q2 - 2.0f*q1*q3) + P(0,3)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - (SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6])*(P(19,16) + P(1,16)*SH_MAG[0] - P(2,16)*SH_MAG[1] + P(3,16)*SH_MAG[2] - P(16,16)*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P(17,16)*(2.0f*q0*q3 + 2.0f*q1*q2) - P(18,16)*(2.0f*q0*q2 - 2.0f*q1*q3) + P(0,16)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + P(0,19)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2));

        // Calculate X axis observation jacobians
        H_MAG.setZero();
        H_MAG(0) = SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2;
        H_MAG(1) = SH_MAG[0];
        H_MAG(2) = -SH_MAG[1];
        H_MAG(3) = SH_MAG[2];
        H_MAG(16) = SH_MAG[5] - SH_MAG[4] - SH_MAG[3] + SH_MAG[6];
        H_MAG(17) = 2.0f*q0*q3 + 2.0f*q1*q2;
        H_MAG(18) = 2.0f*q1*q3 - 2.0f*q0*q2;
        H_MAG(19) = 1.0f;

        // Calculate X axis Kalman gains
        float SK_MX[5];
        SK_MX[0] = 1.0f / mag_innov_var;
        SK_MX[1] = SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6];
        SK_MX[2] = SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2;
        SK_MX[3] = 2.0f*q0*q2 - 2.0f*q1*q3;
        SK_MX[4] = 2.0f*q0*q3 + 2.0f*q1*q2;

        if (update_all_states) {
            Kfusion(0) = SK_MX[0]*(P(0,19) + P(0,1)*SH_MAG[0] - P(0,2)*SH_MAG[1] + P(0,3)*SH_MAG[2] + P(0,0)*SK_MX[2] - P(0,16)*SK_MX[1] + P(0,17)*SK_MX[4] - P(0,18)*SK_MX[3]);
            Kfusion(1) = SK_MX[0]*(P(1,19) + P(1,1)*SH_MAG[0] - P(1,2)*SH_MAG[1] + P(1,3)*SH_MAG[2] + P(1,0)*SK_MX[2] - P(1,16)*SK_MX[1] + P(1,17)*SK_MX[4] - P(1,18)*SK_MX[3]);
            Kfusion(2) = SK_MX[0]*(P(2,19) + P(2,1)*SH_MAG[0] - P(2,2)*SH_MAG[1] + P(2,3)*SH_MAG[2] + P(2,0)*SK_MX[2] - P(2,16)*SK_MX[1] + P(2,17)*SK_MX[4] - P(2,18)*SK_MX[3]);
            Kfusion(3) = SK_MX[0]*(P(3,19) + P(3,1)*SH_MAG[0] - P(3,2)*SH_MAG[1] + P(3,3)*SH_MAG[2] + P(3,0)*SK_MX[2] - P(3,16)*SK_MX[1] + P(3,17)*SK_MX[4] - P(3,18)*SK_MX[3]);
            Kfusion(4) = SK_MX[0]*(P(4,19) + P(4,1)*SH_MAG[0] - P(4,2)*SH_MAG[1] + P(4,3)*SH_MAG[2] + P(4,0)*SK_MX[2] - P(4,16)*SK_MX[1] + P(4,17)*SK_MX[4] - P(4,18)*SK_MX[3]);
            Kfusion(5) = SK_MX[0]*(P(5,19) + P(5,1)*SH_MAG[0] - P(5,2)*SH_MAG[1] + P(5,3)*SH_MAG[2] + P(5,0)*SK_MX[2] - P(5,16)*SK_MX[1] + P(5,17)*SK_MX[4] - P(5,18)*SK_MX[3]);
            Kfusion(6) = SK_MX[0]*(P(6,19) + P(6,1)*SH_MAG[0] - P(6,2)*SH_MAG[1] + P(6,3)*SH_MAG[2] + P(6,0)*SK_MX[2] - P(6,16)*SK_MX[1] + P(6,17)*SK_MX[4] - P(6,18)*SK_MX[3]);
            Kfusion(7) = SK_MX[0]*(P(7,19) + P(7,1)*SH_MAG[0] - P(7,2)*SH_MAG[1] + P(7,3)*SH_MAG[2] + P(7,0)*SK_MX[2] - P(7,16)*SK_MX[1] + P(7,17)*SK_MX[4] - P(7,18)*SK_MX[3]);
            Kfusion(8) = SK_MX[0]*(P(8,19) + P(8,1)*SH_MAG[0] - P(8,2)*SH_MAG[1] + P(8,3)*SH_MAG[2] + P(8,0)*SK_MX[2] - P(8,16)*SK_MX[1] + P(8,17)*SK_MX[4] - P(8,18)*SK_MX[3]);
            Kfusion(9) = SK_MX[0]*(P(9,19) + P(9,1)*SH_MAG[0] - P(9,2)*SH_MAG[1] + P(9,3)*SH_MAG[2] + P(9,0)*SK_MX[2] - P(9,16)*SK_MX[1] + P(9,17)*SK_MX[4] - P(9,18)*SK_MX[3]);
            Kfusion(10) = SK_MX[0]*(P(10,19) + P(10,1)*SH_MAG[0] - P(10,2)*SH_MAG[1] + P(10,3)*SH_MAG[2] + P(10,0)*SK_MX[2] - P(10,16)*SK_MX[1] + P(10,17)*SK_MX[4] - P(10,18)*SK_MX[3]);
            Kfusion(11) = SK_MX[0]*(P(11,19) + P(11,1)*SH_MAG[0] - P(11,2)*SH_MAG[1] + P(11,3)*SH_MAG[2] + P(11,0)*SK_MX[2] - P(11,16)*SK_MX[1] + P(11,17)*SK_MX[4] - P(11,18)*SK_MX[3]);
            Kfusion(12) = SK_MX[0]*(P(12,19) + P(12,1)*SH_MAG[0] - P(12,2)*SH_MAG[1] + P(12,3)*SH_MAG[2] + P(12,0)*SK_MX[2] - P(12,16)*SK_MX[1] + P(12,17)*SK_MX[4] - P(12,18)*SK_MX[3]);
            Kfusion(13) = SK_MX[0]*(P(13,19) + P(13,1)*SH_MAG[0] - P(13,2)*SH_MAG[1] + P(13,3)*SH_MAG[2] + P(13,0)*SK_MX[2] - P(13,16)*SK_MX[1] + P(13,17)*SK_MX[4] - P(13,18)*SK_MX[3]);
            Kfusion(14) = SK_MX[0]*(P(14,19) + P(14,1)*SH_MAG[0] - P(14,2)*SH_MAG[1] + P(14,3)*SH_MAG[2] + P(14,0)*SK_MX[2] - P(14,16)*SK_MX[1] + P(14,17)*SK_MX[4] - P(14,18)*SK_MX[3]);
            Kfusion(15) = SK_MX[0]*(P(15,19) + P(15,1)*SH_MAG[0] - P(15,2)*SH_MAG[1] + P(15,3)*SH_MAG[2] + P(15,0)*SK_MX[2] - P(15,16)*SK_MX[1] + P(15,17)*SK_MX[4] - P(15,18)*SK_MX[3]);
            Kfusion(22) = SK_MX[0]*(P(22,19) + P(22,1)*SH_MAG[0] - P(22,2)*SH_MAG[1] + P(22,3)*SH_MAG[2] + P(22,0)*SK_MX[2] - P(22,16)*SK_MX[1] + P(22,17)*SK_MX[4] - P(22,18)*SK_MX[3]);
            Kfusion(23) = SK_MX[0]*(P(23,19) + P(23,1)*SH_MAG[0] - P(23,2)*SH_MAG[1] + P(23,3)*SH_MAG[2] + P(23,0)*SK_MX[2] - P(23,16)*SK_MX[1] + P(23,17)*SK_MX[4] - P(23,18)*SK_MX[3]);
        }

        Kfusion(16) = SK_MX[0]*(P(16,19) + P(16,1)*SH_MAG[0] - P(16,2)*SH_MAG[1] + P(16,3)*SH_MAG[2] + P(16,0)*SK_MX[2] - P(16,16)*SK_MX[1] + P(16,17)*SK_MX[4] - P(16,18)*SK_MX[3]);
        Kfusion(17) = SK_MX[0]*(P(17,19) + P(17,1)*SH_MAG[0] - P(17,2)*SH_MAG[1] + P(17,3)*SH_MAG[2] + P(17,0)*SK_MX[2] - P(17,16)*SK_MX[1] + P(17,17)*SK_MX[4] - P(17,18)*SK_MX[3]);
        Kfusion(18) = SK_MX[0]*(P(18,19) + P(18,1)*SH_MAG[0] - P(18,2)*SH_MAG[1] + P(18,3)*SH_MAG[2] + P(18,0)*SK_MX[2] - P(18,16)*SK_MX[1] + P(18,17)*SK_MX[4] - P(18,18)*SK_MX[3]);
        Kfusion(19) = SK_MX[0]*(P(19,19) + P(19,1)*SH_MAG[0] - P(19,2)*SH_MAG[1] + P(19,3)*SH_MAG[2] + P(19,0)*SK_MX[2] - P(19,16)*SK_MX[1] + P(19,17)*SK_MX[4] - P(19,18)*SK_MX[3]);
        Kfusion(20) = SK_MX[0]*(P(20,19) + P(20,1)*SH_MAG[0] - P(20,2)*SH_MAG[1] + P(20,3)*SH_MAG[2] + P(20,0)*SK_MX[2] - P(20,16)*SK_MX[1] + P(20,17)*SK_MX[4] - P(20,18)*SK_MX[3]);
        Kfusion(21) = SK_MX[0]*(P(21,19) + P(21,1)*SH_MAG[0] - P(21,2)*SH_MAG[1] + P(21,3)*SH_MAG[2] + P(21,0)*SK_MX[2] - P(21,16)*SK_MX[1] + P(21,17)*SK_MX[4] - P(21,18)*SK_MX[3]);

        // find largest observation variance difference as a fraction of the matlab value
        float max_diff_fraction = 0.0f;
        int max_row;
        float max_old, max_new;
        for (int row=0; row<24; row++) {
            float diff_fraction;
            if (H_MAG(row) != 0.0f) {
                diff_fraction = fabsf(Hfusion_sympy[row] - H_MAG(row)) / fabsf(H_MAG(row));
            } else if (Hfusion_sympy[row] != 0.0f) {
                diff_fraction = fabsf(Hfusion_sympy[row] - H_MAG(row)) / fabsf(Hfusion_sympy[row]);
            } else {
                diff_fraction = 0.0f;
            }
            if (diff_fraction > max_diff_fraction) {
                max_diff_fraction = diff_fraction;
                max_row = row;
                max_old = H_MAG(row);
                max_new = Hfusion_sympy[row];
            }
        }

        if (max_diff_fraction > 1e-5f) {
            printf("Fail: Magnetomer X axis Hfusion max diff fraction = %e , old = %e , new = %e , location index = %i\n",max_diff_fraction, max_old, max_new, max_row);
        } else {
            printf("Pass: Magnetomer X axis Hfusion max diff fraction = %e\n",max_diff_fraction);
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
            printf("Fail: Magnetomer X axis Kfusion max diff fraction = %e , old = %e , new = %e , location index = %i\n",max_diff_fraction, max_old, max_new, max_row);
        } else {
            printf("Pass: Magnetomer X axis Kfusion max diff fraction = %e\n",max_diff_fraction);
        }

    }

    // Compare Y axis equations
    {
        // recalculate innovation variance becasue states and covariances have changed due to previous fusion
        const float HKY0 = magD*q1 + magE*q0 - magN*q3;
        const float HKY1 = magD*q0 - magE*q1 + magN*q2;
        const float HKY2 = magD*q3 + magE*q2 + magN*q1;
        const float HKY3 = magD*q2;
        const float HKY4 = magE*q3;
        const float HKY5 = magN*q0;
        const float HKY6 = q1*q2;
        const float HKY7 = q0*q3;
        const float HKY8 = powf(q0, 2) - powf(q1, 2) + powf(q2, 2) - powf(q3, 2);
        const float HKY9 = q0*q1 + q2*q3;
        const float HKY10 = 2*HKY9;
        const float HKY11 = -2*HKY6 + 2*HKY7;
        const float HKY12 = 2*HKY2;
        const float HKY13 = 2*HKY0;
        const float HKY14 = 2*HKY1;
        const float HKY15 = -2*HKY3 + 2*HKY4 + 2*HKY5;
        const float HKY16 = HKY10*P(0,18) - HKY11*P(0,16) + HKY12*P(0,2) + HKY13*P(0,0) + HKY14*P(0,1) - HKY15*P(0,3) + HKY8*P(0,17) + P(0,20);
        const float HKY17 = HKY10*P(17,18) - HKY11*P(16,17) + HKY12*P(2,17) + HKY13*P(0,17) + HKY14*P(1,17) - HKY15*P(3,17) + HKY8*P(17,17) + P(17,20);
        const float HKY18 = HKY10*P(16,18) - HKY11*P(16,16) + HKY12*P(2,16) + HKY13*P(0,16) + HKY14*P(1,16) - HKY15*P(3,16) + HKY8*P(16,17) + P(16,20);
        const float HKY19 = HKY10*P(3,18) - HKY11*P(3,16) + HKY12*P(2,3) + HKY13*P(0,3) + HKY14*P(1,3) - HKY15*P(3,3) + HKY8*P(3,17) + P(3,20);
        const float HKY20 = HKY10*P(18,18) - HKY11*P(16,18) + HKY12*P(2,18) + HKY13*P(0,18) + HKY14*P(1,18) - HKY15*P(3,18) + HKY8*P(17,18) + P(18,20);
        const float HKY21 = HKY10*P(1,18) - HKY11*P(1,16) + HKY12*P(1,2) + HKY13*P(0,1) + HKY14*P(1,1) - HKY15*P(1,3) + HKY8*P(1,17) + P(1,20);
        const float HKY22 = HKY10*P(2,18) - HKY11*P(2,16) + HKY12*P(2,2) + HKY13*P(0,2) + HKY14*P(1,2) - HKY15*P(2,3) + HKY8*P(2,17) + P(2,20);
        const float HKY23 = HKY10*P(18,20) - HKY11*P(16,20) + HKY12*P(2,20) + HKY13*P(0,20) + HKY14*P(1,20) - HKY15*P(3,20) + HKY8*P(17,20) + P(20,20);
        const float HKY24 = 1.0F/(HKY10*HKY20 - HKY11*HKY18 + HKY12*HKY22 + HKY13*HKY16 + HKY14*HKY21 - HKY15*HKY19 + HKY17*HKY8 + HKY23 + R_MAG);

        // Calculate Y axis observation jacobians
        memset(Hfusion, 0, sizeof(Hfusion));
        Hfusion[0] = 2*HKY0;
        Hfusion[1] = 2*HKY1;
        Hfusion[2] = 2*HKY2;
        Hfusion[3] = 2*HKY3 - 2*HKY4 - 2*HKY5;
        Hfusion[16] = 2*HKY6 - 2*HKY7;
        Hfusion[17] = HKY8;
        Hfusion[18] = 2*HKY9;
        Hfusion[20] = 1;

        // Calculate Y axis Kalman gains
        if (update_all_states) {
            Kfusion(0) = HKY16*HKY24;
            Kfusion(1) = HKY21*HKY24;
            Kfusion(2) = HKY22*HKY24;
            Kfusion(3) = HKY19*HKY24;

            for (unsigned row = 4; row <= 15; row++) {
                Kfusion(row) = HKY24*(HKY10*P(row,18) - HKY11*P(row,16) + HKY12*P(2,row) + HKY13*P(0,row) + HKY14*P(1,row) - HKY15*P(3,row) + HKY8*P(row,17) + P(row,20));
            }

            for (unsigned row = 22; row <= 23; row++) {
                Kfusion(row) = HKY24*(HKY10*P(18,row) - HKY11*P(16,row) + HKY12*P(2,row) + HKY13*P(0,row) + HKY14*P(1,row) - HKY15*P(3,row) + HKY8*P(17,row) + P(20,row));
            }
        }

        Kfusion(16) = HKY18*HKY24;
        Kfusion(17) = HKY17*HKY24;
        Kfusion(18) = HKY20*HKY24;
        Kfusion(19) = HKY24*(HKY10*P(18,19) - HKY11*P(16,19) + HKY12*P(2,19) + HKY13*P(0,19) + HKY14*P(1,19) - HKY15*P(3,19) + HKY8*P(17,19) + P(19,20));
        Kfusion(20) = HKY23*HKY24;
        Kfusion(21) = HKY24*(HKY10*P(18,21) - HKY11*P(16,21) + HKY12*P(2,21) + HKY13*P(0,21) + HKY14*P(1,21) - HKY15*P(3,21) + HKY8*P(17,21) + P(20,21));

        // save output and repeat calculation using legacy matlab generated code
        float Hfusion_sympy[24];
        Vector24f Kfusion_sympy;
        for (int row=0; row<24; row++) {
            Hfusion_sympy[row] = Hfusion[row];
            Kfusion_sympy(row) = Kfusion(row);
        }

        // repeat calculation using matlab generated equations

        // Y axis innovation variance
        mag_innov_var = (P(20,20) + R_MAG + P(0,20)*SH_MAG[2] + P(1,20)*SH_MAG[1] + P(2,20)*SH_MAG[0] - P(17,20)*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - (2.0f*q0*q3 - 2.0f*q1*q2)*(P(20,16) + P(0,16)*SH_MAG[2] + P(1,16)*SH_MAG[1] + P(2,16)*SH_MAG[0] - P(17,16)*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P(16,16)*(2.0f*q0*q3 - 2.0f*q1*q2) + P(18,16)*(2.0f*q0*q1 + 2.0f*q2*q3) - P(3,16)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + (2.0f*q0*q1 + 2.0f*q2*q3)*(P(20,18) + P(0,18)*SH_MAG[2] + P(1,18)*SH_MAG[1] + P(2,18)*SH_MAG[0] - P(17,18)*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P(16,18)*(2.0f*q0*q3 - 2.0f*q1*q2) + P(18,18)*(2.0f*q0*q1 + 2.0f*q2*q3) - P(3,18)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - (SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)*(P(20,3) + P(0,3)*SH_MAG[2] + P(1,3)*SH_MAG[1] + P(2,3)*SH_MAG[0] - P(17,3)*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P(16,3)*(2.0f*q0*q3 - 2.0f*q1*q2) + P(18,3)*(2.0f*q0*q1 + 2.0f*q2*q3) - P(3,3)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - P(16,20)*(2.0f*q0*q3 - 2.0f*q1*q2) + P(18,20)*(2.0f*q0*q1 + 2.0f*q2*q3) + SH_MAG[2]*(P(20,0) + P(0,0)*SH_MAG[2] + P(1,0)*SH_MAG[1] + P(2,0)*SH_MAG[0] - P(17,0)*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P(16,0)*(2.0f*q0*q3 - 2.0f*q1*q2) + P(18,0)*(2.0f*q0*q1 + 2.0f*q2*q3) - P(3,0)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + SH_MAG[1]*(P(20,1) + P(0,1)*SH_MAG[2] + P(1,1)*SH_MAG[1] + P(2,1)*SH_MAG[0] - P(17,1)*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P(16,1)*(2.0f*q0*q3 - 2.0f*q1*q2) + P(18,1)*(2.0f*q0*q1 + 2.0f*q2*q3) - P(3,1)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + SH_MAG[0]*(P(20,2) + P(0,2)*SH_MAG[2] + P(1,2)*SH_MAG[1] + P(2,2)*SH_MAG[0] - P(17,2)*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P(16,2)*(2.0f*q0*q3 - 2.0f*q1*q2) + P(18,2)*(2.0f*q0*q1 + 2.0f*q2*q3) - P(3,2)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - (SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6])*(P(20,17) + P(0,17)*SH_MAG[2] + P(1,17)*SH_MAG[1] + P(2,17)*SH_MAG[0] - P(17,17)*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P(16,17)*(2.0f*q0*q3 - 2.0f*q1*q2) + P(18,17)*(2.0f*q0*q1 + 2.0f*q2*q3) - P(3,17)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - P(3,20)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2));

        // Calculate Y axis observation jacobians
        H_MAG.setZero();
        H_MAG(0) = SH_MAG[2];
        H_MAG(1) = SH_MAG[1];
        H_MAG(2) = SH_MAG[0];
        H_MAG(3) = 2.0f*magD*q2 - SH_MAG[8] - SH_MAG[7];
        H_MAG(16) = 2.0f*q1*q2 - 2.0f*q0*q3;
        H_MAG(17) = SH_MAG[4] - SH_MAG[3] - SH_MAG[5] + SH_MAG[6];
        H_MAG(18) = 2.0f*q0*q1 + 2.0f*q2*q3;
        H_MAG(20) = 1.0f;

        // Calculate Y axis Kalman gains
        float SK_MY[5];
        SK_MY[0] = 1.0f / mag_innov_var;
        SK_MY[1] = SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6];
        SK_MY[2] = SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2;
        SK_MY[3] = 2.0f*q0*q3 - 2.0f*q1*q2;
        SK_MY[4] = 2.0f*q0*q1 + 2.0f*q2*q3;

        if (update_all_states) {
            Kfusion(0) = SK_MY[0]*(P(0,20) + P(0,0)*SH_MAG[2] + P(0,1)*SH_MAG[1] + P(0,2)*SH_MAG[0] - P(0,3)*SK_MY[2] - P(0,17)*SK_MY[1] - P(0,16)*SK_MY[3] + P(0,18)*SK_MY[4]);
            Kfusion(1) = SK_MY[0]*(P(1,20) + P(1,0)*SH_MAG[2] + P(1,1)*SH_MAG[1] + P(1,2)*SH_MAG[0] - P(1,3)*SK_MY[2] - P(1,17)*SK_MY[1] - P(1,16)*SK_MY[3] + P(1,18)*SK_MY[4]);
            Kfusion(2) = SK_MY[0]*(P(2,20) + P(2,0)*SH_MAG[2] + P(2,1)*SH_MAG[1] + P(2,2)*SH_MAG[0] - P(2,3)*SK_MY[2] - P(2,17)*SK_MY[1] - P(2,16)*SK_MY[3] + P(2,18)*SK_MY[4]);
            Kfusion(3) = SK_MY[0]*(P(3,20) + P(3,0)*SH_MAG[2] + P(3,1)*SH_MAG[1] + P(3,2)*SH_MAG[0] - P(3,3)*SK_MY[2] - P(3,17)*SK_MY[1] - P(3,16)*SK_MY[3] + P(3,18)*SK_MY[4]);
            Kfusion(4) = SK_MY[0]*(P(4,20) + P(4,0)*SH_MAG[2] + P(4,1)*SH_MAG[1] + P(4,2)*SH_MAG[0] - P(4,3)*SK_MY[2] - P(4,17)*SK_MY[1] - P(4,16)*SK_MY[3] + P(4,18)*SK_MY[4]);
            Kfusion(5) = SK_MY[0]*(P(5,20) + P(5,0)*SH_MAG[2] + P(5,1)*SH_MAG[1] + P(5,2)*SH_MAG[0] - P(5,3)*SK_MY[2] - P(5,17)*SK_MY[1] - P(5,16)*SK_MY[3] + P(5,18)*SK_MY[4]);
            Kfusion(6) = SK_MY[0]*(P(6,20) + P(6,0)*SH_MAG[2] + P(6,1)*SH_MAG[1] + P(6,2)*SH_MAG[0] - P(6,3)*SK_MY[2] - P(6,17)*SK_MY[1] - P(6,16)*SK_MY[3] + P(6,18)*SK_MY[4]);
            Kfusion(7) = SK_MY[0]*(P(7,20) + P(7,0)*SH_MAG[2] + P(7,1)*SH_MAG[1] + P(7,2)*SH_MAG[0] - P(7,3)*SK_MY[2] - P(7,17)*SK_MY[1] - P(7,16)*SK_MY[3] + P(7,18)*SK_MY[4]);
            Kfusion(8) = SK_MY[0]*(P(8,20) + P(8,0)*SH_MAG[2] + P(8,1)*SH_MAG[1] + P(8,2)*SH_MAG[0] - P(8,3)*SK_MY[2] - P(8,17)*SK_MY[1] - P(8,16)*SK_MY[3] + P(8,18)*SK_MY[4]);
            Kfusion(9) = SK_MY[0]*(P(9,20) + P(9,0)*SH_MAG[2] + P(9,1)*SH_MAG[1] + P(9,2)*SH_MAG[0] - P(9,3)*SK_MY[2] - P(9,17)*SK_MY[1] - P(9,16)*SK_MY[3] + P(9,18)*SK_MY[4]);
            Kfusion(10) = SK_MY[0]*(P(10,20) + P(10,0)*SH_MAG[2] + P(10,1)*SH_MAG[1] + P(10,2)*SH_MAG[0] - P(10,3)*SK_MY[2] - P(10,17)*SK_MY[1] - P(10,16)*SK_MY[3] + P(10,18)*SK_MY[4]);
            Kfusion(11) = SK_MY[0]*(P(11,20) + P(11,0)*SH_MAG[2] + P(11,1)*SH_MAG[1] + P(11,2)*SH_MAG[0] - P(11,3)*SK_MY[2] - P(11,17)*SK_MY[1] - P(11,16)*SK_MY[3] + P(11,18)*SK_MY[4]);
            Kfusion(12) = SK_MY[0]*(P(12,20) + P(12,0)*SH_MAG[2] + P(12,1)*SH_MAG[1] + P(12,2)*SH_MAG[0] - P(12,3)*SK_MY[2] - P(12,17)*SK_MY[1] - P(12,16)*SK_MY[3] + P(12,18)*SK_MY[4]);
            Kfusion(13) = SK_MY[0]*(P(13,20) + P(13,0)*SH_MAG[2] + P(13,1)*SH_MAG[1] + P(13,2)*SH_MAG[0] - P(13,3)*SK_MY[2] - P(13,17)*SK_MY[1] - P(13,16)*SK_MY[3] + P(13,18)*SK_MY[4]);
            Kfusion(14) = SK_MY[0]*(P(14,20) + P(14,0)*SH_MAG[2] + P(14,1)*SH_MAG[1] + P(14,2)*SH_MAG[0] - P(14,3)*SK_MY[2] - P(14,17)*SK_MY[1] - P(14,16)*SK_MY[3] + P(14,18)*SK_MY[4]);
            Kfusion(15) = SK_MY[0]*(P(15,20) + P(15,0)*SH_MAG[2] + P(15,1)*SH_MAG[1] + P(15,2)*SH_MAG[0] - P(15,3)*SK_MY[2] - P(15,17)*SK_MY[1] - P(15,16)*SK_MY[3] + P(15,18)*SK_MY[4]);
            Kfusion(22) = SK_MY[0]*(P(22,20) + P(22,0)*SH_MAG[2] + P(22,1)*SH_MAG[1] + P(22,2)*SH_MAG[0] - P(22,3)*SK_MY[2] - P(22,17)*SK_MY[1] - P(22,16)*SK_MY[3] + P(22,18)*SK_MY[4]);
            Kfusion(23) = SK_MY[0]*(P(23,20) + P(23,0)*SH_MAG[2] + P(23,1)*SH_MAG[1] + P(23,2)*SH_MAG[0] - P(23,3)*SK_MY[2] - P(23,17)*SK_MY[1] - P(23,16)*SK_MY[3] + P(23,18)*SK_MY[4]);
        }

        Kfusion(16) = SK_MY[0]*(P(16,20) + P(16,0)*SH_MAG[2] + P(16,1)*SH_MAG[1] + P(16,2)*SH_MAG[0] - P(16,3)*SK_MY[2] - P(16,17)*SK_MY[1] - P(16,16)*SK_MY[3] + P(16,18)*SK_MY[4]);
        Kfusion(17) = SK_MY[0]*(P(17,20) + P(17,0)*SH_MAG[2] + P(17,1)*SH_MAG[1] + P(17,2)*SH_MAG[0] - P(17,3)*SK_MY[2] - P(17,17)*SK_MY[1] - P(17,16)*SK_MY[3] + P(17,18)*SK_MY[4]);
        Kfusion(18) = SK_MY[0]*(P(18,20) + P(18,0)*SH_MAG[2] + P(18,1)*SH_MAG[1] + P(18,2)*SH_MAG[0] - P(18,3)*SK_MY[2] - P(18,17)*SK_MY[1] - P(18,16)*SK_MY[3] + P(18,18)*SK_MY[4]);
        Kfusion(19) = SK_MY[0]*(P(19,20) + P(19,0)*SH_MAG[2] + P(19,1)*SH_MAG[1] + P(19,2)*SH_MAG[0] - P(19,3)*SK_MY[2] - P(19,17)*SK_MY[1] - P(19,16)*SK_MY[3] + P(19,18)*SK_MY[4]);
        Kfusion(20) = SK_MY[0]*(P(20,20) + P(20,0)*SH_MAG[2] + P(20,1)*SH_MAG[1] + P(20,2)*SH_MAG[0] - P(20,3)*SK_MY[2] - P(20,17)*SK_MY[1] - P(20,16)*SK_MY[3] + P(20,18)*SK_MY[4]);
        Kfusion(21) = SK_MY[0]*(P(21,20) + P(21,0)*SH_MAG[2] + P(21,1)*SH_MAG[1] + P(21,2)*SH_MAG[0] - P(21,3)*SK_MY[2] - P(21,17)*SK_MY[1] - P(21,16)*SK_MY[3] + P(21,18)*SK_MY[4]);

        // find largest observation variance difference as a fraction of the matlab value
        float max_diff_fraction = 0.0f;
        int max_row;
        float max_old, max_new;
        for (int row=0; row<24; row++) {
            float diff_fraction;
            if (H_MAG(row) != 0.0f) {
                diff_fraction = fabsf(Hfusion_sympy[row] - H_MAG(row)) / fabsf(H_MAG(row));
            } else if (Hfusion_sympy[row] != 0.0f) {
                diff_fraction = fabsf(Hfusion_sympy[row] - H_MAG(row)) / fabsf(Hfusion_sympy[row]);
            } else {
                diff_fraction = 0.0f;
            }
            if (diff_fraction > max_diff_fraction) {
                max_diff_fraction = diff_fraction;
                max_row = row;
                max_old = H_MAG(row);
                max_new = Hfusion_sympy[row];
            }
        }

        if (max_diff_fraction > 1e-5f) {
            printf("Fail: Magnetomer Y axis Hfusion max diff fraction = %e , old = %e , new = %e , location index = %i\n",max_diff_fraction, max_old, max_new, max_row);
        } else {
            printf("Pass: Magnetomer Y axis Hfusion max diff fraction = %e\n",max_diff_fraction);
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
            printf("Fail: Magnetomer Y axis Kfusion max diff fraction = %e , old = %e , new = %e , location index = %i\n",max_diff_fraction, max_old, max_new, max_row);
        } else {
            printf("Pass: Magnetomer Y axis Kfusion max diff fraction = %e\n",max_diff_fraction);
        }

    }

    // Compare Z axis equations
    {
        // recalculate innovation variance becasue states and covariances have changed due to previous fusion
        const float HKZ0 = magD*q0 - magE*q1 + magN*q2;
        const float HKZ1 = magN*q3;
        const float HKZ2 = magD*q1;
        const float HKZ3 = magE*q0;
        const float HKZ4 = -magD*q2 + magE*q3 + magN*q0;
        const float HKZ5 = magD*q3 + magE*q2 + magN*q1;
        const float HKZ6 = q0*q2 + q1*q3;
        const float HKZ7 = q2*q3;
        const float HKZ8 = q0*q1;
        const float HKZ9 = powf(q0, 2) - powf(q1, 2) - powf(q2, 2) + powf(q3, 2);
        const float HKZ10 = 2*HKZ6;
        const float HKZ11 = -2*HKZ7 + 2*HKZ8;
        const float HKZ12 = 2*HKZ5;
        const float HKZ13 = 2*HKZ0;
        const float HKZ14 = -2*HKZ1 + 2*HKZ2 + 2*HKZ3;
        const float HKZ15 = 2*HKZ4;
        const float HKZ16 = HKZ10*P(0,16) - HKZ11*P(0,17) + HKZ12*P(0,3) + HKZ13*P(0,0) - HKZ14*P(0,1) + HKZ15*P(0,2) + HKZ9*P(0,18) + P(0,21);
        const float HKZ17 = HKZ10*P(16,18) - HKZ11*P(17,18) + HKZ12*P(3,18) + HKZ13*P(0,18) - HKZ14*P(1,18) + HKZ15*P(2,18) + HKZ9*P(18,18) + P(18,21);
        const float HKZ18 = HKZ10*P(16,17) - HKZ11*P(17,17) + HKZ12*P(3,17) + HKZ13*P(0,17) - HKZ14*P(1,17) + HKZ15*P(2,17) + HKZ9*P(17,18) + P(17,21);
        const float HKZ19 = HKZ10*P(1,16) - HKZ11*P(1,17) + HKZ12*P(1,3) + HKZ13*P(0,1) - HKZ14*P(1,1) + HKZ15*P(1,2) + HKZ9*P(1,18) + P(1,21);
        const float HKZ20 = HKZ10*P(16,16) - HKZ11*P(16,17) + HKZ12*P(3,16) + HKZ13*P(0,16) - HKZ14*P(1,16) + HKZ15*P(2,16) + HKZ9*P(16,18) + P(16,21);
        const float HKZ21 = HKZ10*P(3,16) - HKZ11*P(3,17) + HKZ12*P(3,3) + HKZ13*P(0,3) - HKZ14*P(1,3) + HKZ15*P(2,3) + HKZ9*P(3,18) + P(3,21);
        const float HKZ22 = HKZ10*P(2,16) - HKZ11*P(2,17) + HKZ12*P(2,3) + HKZ13*P(0,2) - HKZ14*P(1,2) + HKZ15*P(2,2) + HKZ9*P(2,18) + P(2,21);
        const float HKZ23 = HKZ10*P(16,21) - HKZ11*P(17,21) + HKZ12*P(3,21) + HKZ13*P(0,21) - HKZ14*P(1,21) + HKZ15*P(2,21) + HKZ9*P(18,21) + P(21,21);
        const float HKZ24 = 1.0F/(HKZ10*HKZ20 - HKZ11*HKZ18 + HKZ12*HKZ21 + HKZ13*HKZ16 - HKZ14*HKZ19 + HKZ15*HKZ22 + HKZ17*HKZ9 + HKZ23 + R_MAG);

        // calculate Z axis observation jacobians
        memset(Hfusion, 0, sizeof(Hfusion));
        Hfusion[0] = 2*HKZ0;
        Hfusion[1] = 2*HKZ1 - 2*HKZ2 - 2*HKZ3;
        Hfusion[2] = 2*HKZ4;
        Hfusion[3] = 2*HKZ5;
        Hfusion[16] = 2*HKZ6;
        Hfusion[17] = 2*HKZ7 - 2*HKZ8;
        Hfusion[18] = HKZ9;
        Hfusion[21] = 1;

        // Calculate Z axis Kalman gains
        if (update_all_states) {
            Kfusion(0) = HKZ16*HKZ24;
            Kfusion(1) = HKZ19*HKZ24;
            Kfusion(2) = HKZ22*HKZ24;
            Kfusion(3) = HKZ21*HKZ24;

            for (unsigned row = 4; row <= 15; row++) {
                Kfusion(row) = HKZ24*(HKZ10*P(row,16) - HKZ11*P(row,17) + HKZ12*P(3,row) + HKZ13*P(0,row) - HKZ14*P(1,row) + HKZ15*P(2,row) + HKZ9*P(row,18) + P(row,21));
            }

            for (unsigned row = 22; row <= 23; row++) {
                Kfusion(row) = HKZ24*(HKZ10*P(16,row) - HKZ11*P(17,row) + HKZ12*P(3,row) + HKZ13*P(0,row) - HKZ14*P(1,row) + HKZ15*P(2,row) + HKZ9*P(18,row) + P(21,row));
            }
        }

        Kfusion(16) = HKZ20*HKZ24;
        Kfusion(17) = HKZ18*HKZ24;
        Kfusion(18) = HKZ17*HKZ24;

        for (unsigned row = 19; row <= 20; row++) {
            Kfusion(row) = HKZ24*(HKZ10*P(16,row) - HKZ11*P(17,row) + HKZ12*P(3,row) + HKZ13*P(0,row) - HKZ14*P(1,row) + HKZ15*P(2,row) + HKZ9*P(18,row) + P(row,21));
        }

        Kfusion(21) = HKZ23*HKZ24;

        // save output and repeat calculation using legacy matlab generated code
        float Hfusion_sympy[24];
        Vector24f Kfusion_sympy;
        for (int row=0; row<24; row++) {
            Hfusion_sympy[row] = Hfusion[row];
            Kfusion_sympy(row) = Kfusion(row);
        }

        // repeat calculation using matlab generated equations

        // Z axis innovation variance
        mag_innov_var = (P(21,21) + R_MAG + P(0,21)*SH_MAG[1] - P(1,21)*SH_MAG[2] + P(3,21)*SH_MAG[0] + P(18,21)*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + (2.0f*q0*q2 + 2.0f*q1*q3)*(P(21,16) + P(0,16)*SH_MAG[1] - P(1,16)*SH_MAG[2] + P(3,16)*SH_MAG[0] + P(18,16)*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P(16,16)*(2.0f*q0*q2 + 2.0f*q1*q3) - P(17,16)*(2.0f*q0*q1 - 2.0f*q2*q3) + P(2,16)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - (2.0f*q0*q1 - 2.0f*q2*q3)*(P(21,17) + P(0,17)*SH_MAG[1] - P(1,17)*SH_MAG[2] + P(3,17)*SH_MAG[0] + P(18,17)*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P(16,17)*(2.0f*q0*q2 + 2.0f*q1*q3) - P(17,17)*(2.0f*q0*q1 - 2.0f*q2*q3) + P(2,17)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + (SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)*(P(21,2) + P(0,2)*SH_MAG[1] - P(1,2)*SH_MAG[2] + P(3,2)*SH_MAG[0] + P(18,2)*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P(16,2)*(2.0f*q0*q2 + 2.0f*q1*q3) - P(17,2)*(2.0f*q0*q1 - 2.0f*q2*q3) + P(2,2)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + P(16,21)*(2.0f*q0*q2 + 2.0f*q1*q3) - P(17,21)*(2.0f*q0*q1 - 2.0f*q2*q3) + SH_MAG[1]*(P(21,0) + P(0,0)*SH_MAG[1] - P(1,0)*SH_MAG[2] + P(3,0)*SH_MAG[0] + P(18,0)*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P(16,0)*(2.0f*q0*q2 + 2.0f*q1*q3) - P(17,0)*(2.0f*q0*q1 - 2.0f*q2*q3) + P(2,0)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) - SH_MAG[2]*(P(21,1) + P(0,1)*SH_MAG[1] - P(1,1)*SH_MAG[2] + P(3,1)*SH_MAG[0] + P(18,1)*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P(16,1)*(2.0f*q0*q2 + 2.0f*q1*q3) - P(17,1)*(2.0f*q0*q1 - 2.0f*q2*q3) + P(2,1)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + SH_MAG[0]*(P(21,3) + P(0,3)*SH_MAG[1] - P(1,3)*SH_MAG[2] + P(3,3)*SH_MAG[0] + P(18,3)*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P(16,3)*(2.0f*q0*q2 + 2.0f*q1*q3) - P(17,3)*(2.0f*q0*q1 - 2.0f*q2*q3) + P(2,3)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + (SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6])*(P(21,18) + P(0,18)*SH_MAG[1] - P(1,18)*SH_MAG[2] + P(3,18)*SH_MAG[0] + P(18,18)*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P(16,18)*(2.0f*q0*q2 + 2.0f*q1*q3) - P(17,18)*(2.0f*q0*q1 - 2.0f*q2*q3) + P(2,18)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2)) + P(2,21)*(SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2));

        // calculate Z axis observation jacobians
        H_MAG.setZero();
        H_MAG(0) = SH_MAG[1];
        H_MAG(1) = -SH_MAG[2];
        H_MAG(2) = SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2;
        H_MAG(3) = SH_MAG[0];
        H_MAG(16) = 2.0f*q0*q2 + 2.0f*q1*q3;
        H_MAG(17) = 2.0f*q2*q3 - 2.0f*q0*q1;
        H_MAG(18) = SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6];
        H_MAG(21) = 1.0f;

        // Calculate Z axis Kalman gains
        float SK_MZ[5];
        SK_MZ[0] = 1.0f / mag_innov_var;
        SK_MZ[1] = SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6];
        SK_MZ[2] = SH_MAG[7] + SH_MAG[8] - 2.0f*magD*q2;
        SK_MZ[3] = 2.0f*q0*q1 - 2.0f*q2*q3;
        SK_MZ[4] = 2.0f*q0*q2 + 2.0f*q1*q3;

        if (update_all_states) {
            Kfusion(0) = SK_MZ[0]*(P(0,21) + P(0,0)*SH_MAG[1] - P(0,1)*SH_MAG[2] + P(0,3)*SH_MAG[0] + P(0,2)*SK_MZ[2] + P(0,18)*SK_MZ[1] + P(0,16)*SK_MZ[4] - P(0,17)*SK_MZ[3]);
            Kfusion(1) = SK_MZ[0]*(P(1,21) + P(1,0)*SH_MAG[1] - P(1,1)*SH_MAG[2] + P(1,3)*SH_MAG[0] + P(1,2)*SK_MZ[2] + P(1,18)*SK_MZ[1] + P(1,16)*SK_MZ[4] - P(1,17)*SK_MZ[3]);
            Kfusion(2) = SK_MZ[0]*(P(2,21) + P(2,0)*SH_MAG[1] - P(2,1)*SH_MAG[2] + P(2,3)*SH_MAG[0] + P(2,2)*SK_MZ[2] + P(2,18)*SK_MZ[1] + P(2,16)*SK_MZ[4] - P(2,17)*SK_MZ[3]);
            Kfusion(3) = SK_MZ[0]*(P(3,21) + P(3,0)*SH_MAG[1] - P(3,1)*SH_MAG[2] + P(3,3)*SH_MAG[0] + P(3,2)*SK_MZ[2] + P(3,18)*SK_MZ[1] + P(3,16)*SK_MZ[4] - P(3,17)*SK_MZ[3]);
            Kfusion(4) = SK_MZ[0]*(P(4,21) + P(4,0)*SH_MAG[1] - P(4,1)*SH_MAG[2] + P(4,3)*SH_MAG[0] + P(4,2)*SK_MZ[2] + P(4,18)*SK_MZ[1] + P(4,16)*SK_MZ[4] - P(4,17)*SK_MZ[3]);
            Kfusion(5) = SK_MZ[0]*(P(5,21) + P(5,0)*SH_MAG[1] - P(5,1)*SH_MAG[2] + P(5,3)*SH_MAG[0] + P(5,2)*SK_MZ[2] + P(5,18)*SK_MZ[1] + P(5,16)*SK_MZ[4] - P(5,17)*SK_MZ[3]);
            Kfusion(6) = SK_MZ[0]*(P(6,21) + P(6,0)*SH_MAG[1] - P(6,1)*SH_MAG[2] + P(6,3)*SH_MAG[0] + P(6,2)*SK_MZ[2] + P(6,18)*SK_MZ[1] + P(6,16)*SK_MZ[4] - P(6,17)*SK_MZ[3]);
            Kfusion(7) = SK_MZ[0]*(P(7,21) + P(7,0)*SH_MAG[1] - P(7,1)*SH_MAG[2] + P(7,3)*SH_MAG[0] + P(7,2)*SK_MZ[2] + P(7,18)*SK_MZ[1] + P(7,16)*SK_MZ[4] - P(7,17)*SK_MZ[3]);
            Kfusion(8) = SK_MZ[0]*(P(8,21) + P(8,0)*SH_MAG[1] - P(8,1)*SH_MAG[2] + P(8,3)*SH_MAG[0] + P(8,2)*SK_MZ[2] + P(8,18)*SK_MZ[1] + P(8,16)*SK_MZ[4] - P(8,17)*SK_MZ[3]);
            Kfusion(9) = SK_MZ[0]*(P(9,21) + P(9,0)*SH_MAG[1] - P(9,1)*SH_MAG[2] + P(9,3)*SH_MAG[0] + P(9,2)*SK_MZ[2] + P(9,18)*SK_MZ[1] + P(9,16)*SK_MZ[4] - P(9,17)*SK_MZ[3]);
            Kfusion(10) = SK_MZ[0]*(P(10,21) + P(10,0)*SH_MAG[1] - P(10,1)*SH_MAG[2] + P(10,3)*SH_MAG[0] + P(10,2)*SK_MZ[2] + P(10,18)*SK_MZ[1] + P(10,16)*SK_MZ[4] - P(10,17)*SK_MZ[3]);
            Kfusion(11) = SK_MZ[0]*(P(11,21) + P(11,0)*SH_MAG[1] - P(11,1)*SH_MAG[2] + P(11,3)*SH_MAG[0] + P(11,2)*SK_MZ[2] + P(11,18)*SK_MZ[1] + P(11,16)*SK_MZ[4] - P(11,17)*SK_MZ[3]);
            Kfusion(12) = SK_MZ[0]*(P(12,21) + P(12,0)*SH_MAG[1] - P(12,1)*SH_MAG[2] + P(12,3)*SH_MAG[0] + P(12,2)*SK_MZ[2] + P(12,18)*SK_MZ[1] + P(12,16)*SK_MZ[4] - P(12,17)*SK_MZ[3]);
            Kfusion(13) = SK_MZ[0]*(P(13,21) + P(13,0)*SH_MAG[1] - P(13,1)*SH_MAG[2] + P(13,3)*SH_MAG[0] + P(13,2)*SK_MZ[2] + P(13,18)*SK_MZ[1] + P(13,16)*SK_MZ[4] - P(13,17)*SK_MZ[3]);
            Kfusion(14) = SK_MZ[0]*(P(14,21) + P(14,0)*SH_MAG[1] - P(14,1)*SH_MAG[2] + P(14,3)*SH_MAG[0] + P(14,2)*SK_MZ[2] + P(14,18)*SK_MZ[1] + P(14,16)*SK_MZ[4] - P(14,17)*SK_MZ[3]);
            Kfusion(15) = SK_MZ[0]*(P(15,21) + P(15,0)*SH_MAG[1] - P(15,1)*SH_MAG[2] + P(15,3)*SH_MAG[0] + P(15,2)*SK_MZ[2] + P(15,18)*SK_MZ[1] + P(15,16)*SK_MZ[4] - P(15,17)*SK_MZ[3]);
            Kfusion(22) = SK_MZ[0]*(P(22,21) + P(22,0)*SH_MAG[1] - P(22,1)*SH_MAG[2] + P(22,3)*SH_MAG[0] + P(22,2)*SK_MZ[2] + P(22,18)*SK_MZ[1] + P(22,16)*SK_MZ[4] - P(22,17)*SK_MZ[3]);
            Kfusion(23) = SK_MZ[0]*(P(23,21) + P(23,0)*SH_MAG[1] - P(23,1)*SH_MAG[2] + P(23,3)*SH_MAG[0] + P(23,2)*SK_MZ[2] + P(23,18)*SK_MZ[1] + P(23,16)*SK_MZ[4] - P(23,17)*SK_MZ[3]);
        }

        Kfusion(16) = SK_MZ[0]*(P(16,21) + P(16,0)*SH_MAG[1] - P(16,1)*SH_MAG[2] + P(16,3)*SH_MAG[0] + P(16,2)*SK_MZ[2] + P(16,18)*SK_MZ[1] + P(16,16)*SK_MZ[4] - P(16,17)*SK_MZ[3]);
        Kfusion(17) = SK_MZ[0]*(P(17,21) + P(17,0)*SH_MAG[1] - P(17,1)*SH_MAG[2] + P(17,3)*SH_MAG[0] + P(17,2)*SK_MZ[2] + P(17,18)*SK_MZ[1] + P(17,16)*SK_MZ[4] - P(17,17)*SK_MZ[3]);
        Kfusion(18) = SK_MZ[0]*(P(18,21) + P(18,0)*SH_MAG[1] - P(18,1)*SH_MAG[2] + P(18,3)*SH_MAG[0] + P(18,2)*SK_MZ[2] + P(18,18)*SK_MZ[1] + P(18,16)*SK_MZ[4] - P(18,17)*SK_MZ[3]);
        Kfusion(19) = SK_MZ[0]*(P(19,21) + P(19,0)*SH_MAG[1] - P(19,1)*SH_MAG[2] + P(19,3)*SH_MAG[0] + P(19,2)*SK_MZ[2] + P(19,18)*SK_MZ[1] + P(19,16)*SK_MZ[4] - P(19,17)*SK_MZ[3]);
        Kfusion(20) = SK_MZ[0]*(P(20,21) + P(20,0)*SH_MAG[1] - P(20,1)*SH_MAG[2] + P(20,3)*SH_MAG[0] + P(20,2)*SK_MZ[2] + P(20,18)*SK_MZ[1] + P(20,16)*SK_MZ[4] - P(20,17)*SK_MZ[3]);
        Kfusion(21) = SK_MZ[0]*(P(21,21) + P(21,0)*SH_MAG[1] - P(21,1)*SH_MAG[2] + P(21,3)*SH_MAG[0] + P(21,2)*SK_MZ[2] + P(21,18)*SK_MZ[1] + P(21,16)*SK_MZ[4] - P(21,17)*SK_MZ[3]);

        // find largest observation variance difference as a fraction of the matlab value
        float max_diff_fraction = 0.0f;
        int max_row;
        float max_old, max_new;
        for (int row=0; row<24; row++) {
            float diff_fraction;
            if (H_MAG(row) != 0.0f) {
                diff_fraction = fabsf(Hfusion_sympy[row] - H_MAG(row)) / fabsf(H_MAG(row));
            } else if (Hfusion_sympy[row] != 0.0f) {
                diff_fraction = fabsf(Hfusion_sympy[row] - H_MAG(row)) / fabsf(Hfusion_sympy[row]);
            } else {
                diff_fraction = 0.0f;
            }
            if (diff_fraction > max_diff_fraction) {
                max_diff_fraction = diff_fraction;
                max_row = row;
                max_old = H_MAG(row);
                max_new = Hfusion_sympy[row];
            }
        }

        if (max_diff_fraction > 1e-5f) {
            printf("Fail: Magnetomer Z axis Hfusion max diff fraction = %e , old = %e , new = %e , location index = %i\n",max_diff_fraction, max_old, max_new, max_row);
        } else {
            printf("Pass: Magnetomer Z axis Hfusion max diff fraction = %e\n",max_diff_fraction);
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
            printf("Fail: Magnetomer Z axis Kfusion max diff fraction = %e , old = %e , new = %e , location index = %i\n",max_diff_fraction, max_old, max_new, max_row);
        } else {
            printf("Pass: Magnetomer Z axis Kfusion max diff fraction = %e\n",max_diff_fraction);
        }

    }

    return 0;
}
