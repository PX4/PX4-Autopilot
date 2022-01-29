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

	SparseVector24f<0,1,2,3,4,5,6> Hfusion; // Optical flow observation Jacobians
    Vector24f Kfusion; // Optical flow observation Kalman gains

    Vector24f Hfusion_sympy;
    Vector24f Kfusion_sympy;

    Vector24f Hfusion_matlab;
    Vector24f Kfusion_matlab;

	const float R_LOS = sq(0.15f);

    float flow_innov_var;

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

    const float range = 5.0f;

    matrix::Dcmf Tbs;
	Tbs.identity();

    // create a symmetrical positive definite matrix with off diagonals between -1 and 1 and diagonals between 0 and 1
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

    // evaluate sub expressions used by sympy code
    const float HK0 = -Tbs(1,0)*q2 + Tbs(1,1)*q1 + Tbs(1,2)*q0;
    const float HK1 = Tbs(1,0)*q3 + Tbs(1,1)*q0 - Tbs(1,2)*q1;
    const float HK2 = Tbs(1,0)*q0 - Tbs(1,1)*q3 + Tbs(1,2)*q2;
    const float HK3 = HK0*vd + HK1*ve + HK2*vn;
    const float HK4 = 1.0F/range;
    const float HK5 = 2*HK4;
    const float HK6 = Tbs(1,0)*q1 + Tbs(1,1)*q2 + Tbs(1,2)*q3;
    const float HK7 = -HK0*ve + HK1*vd + HK6*vn;
    const float HK8 = HK0*vn - HK2*vd + HK6*ve;
    const float HK9 = -HK1*vn + HK2*ve + HK6*vd;
    const float HK10 = q0*q2;
    const float HK11 = q1*q3;
    const float HK12 = HK10 + HK11;
    const float HK13 = 2*Tbs(1,2);
    const float HK14 = q0*q3;
    const float HK15 = q1*q2;
    const float HK16 = HK14 - HK15;
    const float HK17 = 2*Tbs(1,1);
    const float HK18 = ecl::powf(q1, 2);
    const float HK19 = ecl::powf(q2, 2);
    const float HK20 = -HK19;
    const float HK21 = ecl::powf(q0, 2);
    const float HK22 = ecl::powf(q3, 2);
    const float HK23 = HK21 - HK22;
    const float HK24 = HK18 + HK20 + HK23;
    const float HK25 = HK12*HK13 - HK16*HK17 + HK24*Tbs(1,0);
    const float HK26 = HK14 + HK15;
    const float HK27 = 2*Tbs(1,0);
    const float HK28 = q0*q1;
    const float HK29 = q2*q3;
    const float HK30 = HK28 - HK29;
    const float HK31 = -HK18;
    const float HK32 = HK19 + HK23 + HK31;
    const float HK33 = -HK13*HK30 + HK26*HK27 + HK32*Tbs(1,1);
    const float HK34 = HK28 + HK29;
    const float HK35 = HK10 - HK11;
    const float HK36 = HK20 + HK21 + HK22 + HK31;
    const float HK37 = HK17*HK34 - HK27*HK35 + HK36*Tbs(1,2);
    const float HK38 = 2*HK3;
    const float HK39 = 2*HK7;
    const float HK40 = 2*HK8;
    const float HK41 = 2*HK9;
    const float HK42 = HK25*P(0,4) + HK33*P(0,5) + HK37*P(0,6) + HK38*P(0,0) + HK39*P(0,1) + HK40*P(0,2) + HK41*P(0,3);
    const float HK43 = ecl::powf(range, -2);
    const float HK44 = HK25*P(4,6) + HK33*P(5,6) + HK37*P(6,6) + HK38*P(0,6) + HK39*P(1,6) + HK40*P(2,6) + HK41*P(3,6);
    const float HK45 = HK25*P(4,5) + HK33*P(5,5) + HK37*P(5,6) + HK38*P(0,5) + HK39*P(1,5) + HK40*P(2,5) + HK41*P(3,5);
    const float HK46 = HK25*P(4,4) + HK33*P(4,5) + HK37*P(4,6) + HK38*P(0,4) + HK39*P(1,4) + HK40*P(2,4) + HK41*P(3,4);
    const float HK47 = HK25*P(2,4) + HK33*P(2,5) + HK37*P(2,6) + HK38*P(0,2) + HK39*P(1,2) + HK40*P(2,2) + HK41*P(2,3);
    const float HK48 = HK25*P(3,4) + HK33*P(3,5) + HK37*P(3,6) + HK38*P(0,3) + HK39*P(1,3) + HK40*P(2,3) + HK41*P(3,3);
    const float HK49 = HK25*P(1,4) + HK33*P(1,5) + HK37*P(1,6) + HK38*P(0,1) + HK39*P(1,1) + HK40*P(1,2) + HK41*P(1,3);
    const float HK50 = HK4/(HK25*HK43*HK46 + HK33*HK43*HK45 + HK37*HK43*HK44 + HK38*HK42*HK43 + HK39*HK43*HK49 + HK40*HK43*HK47 + HK41*HK43*HK48 + R_LOS);

    const float HK51 = Tbs(0,1)*q1;
    const float HK52 = Tbs(0,2)*q0;
    const float HK53 = Tbs(0,0)*q2;
    const float HK54 = HK51 + HK52 - HK53;
    const float HK55 = Tbs(0,0)*q3;
    const float HK56 = Tbs(0,1)*q0;
    const float HK57 = Tbs(0,2)*q1;
    const float HK58 = HK55 + HK56 - HK57;
    const float HK59 = Tbs(0,0)*q0;
    const float HK60 = Tbs(0,2)*q2;
    const float HK61 = Tbs(0,1)*q3;
    const float HK62 = HK59 + HK60 - HK61;
    const float HK63 = HK54*vd + HK58*ve + HK62*vn;
    const float HK64 = Tbs(0,0)*q1 + Tbs(0,1)*q2 + Tbs(0,2)*q3;
    const float HK65 = HK58*vd + HK64*vn;
    const float HK66 = -HK54*ve + HK65;
    const float HK67 = HK54*vn + HK64*ve;
    const float HK68 = -HK62*vd + HK67;
    const float HK69 = HK62*ve + HK64*vd;
    const float HK70 = -HK58*vn + HK69;
    const float HK71 = 2*Tbs(0,1);
    const float HK72 = 2*Tbs(0,2);
    const float HK73 = HK12*HK72 + HK24*Tbs(0,0);
    const float HK74 = -HK16*HK71 + HK73;
    const float HK75 = 2*Tbs(0,0);
    const float HK76 = HK26*HK75 + HK32*Tbs(0,1);
    const float HK77 = -HK30*HK72 + HK76;
    const float HK78 = HK34*HK71 + HK36*Tbs(0,2);
    const float HK79 = -HK35*HK75 + HK78;
    const float HK80 = 2*HK63;
    const float HK81 = 2*HK65 + 2*ve*(-HK51 - HK52 + HK53);
    const float HK82 = 2*HK67 + 2*vd*(-HK59 - HK60 + HK61);
    const float HK83 = 2*HK69 + 2*vn*(-HK55 - HK56 + HK57);
    const float HK84 = HK71*(-HK14 + HK15) + HK73;
    const float HK85 = HK72*(-HK28 + HK29) + HK76;
    const float HK86 = HK75*(-HK10 + HK11) + HK78;
    const float HK87 = HK80*P(0,0) + HK81*P(0,1) + HK82*P(0,2) + HK83*P(0,3) + HK84*P(0,4) + HK85*P(0,5) + HK86*P(0,6);
    const float HK88 = HK80*P(0,6) + HK81*P(1,6) + HK82*P(2,6) + HK83*P(3,6) + HK84*P(4,6) + HK85*P(5,6) + HK86*P(6,6);
    const float HK89 = HK80*P(0,5) + HK81*P(1,5) + HK82*P(2,5) + HK83*P(3,5) + HK84*P(4,5) + HK85*P(5,5) + HK86*P(5,6);
    const float HK90 = HK80*P(0,4) + HK81*P(1,4) + HK82*P(2,4) + HK83*P(3,4) + HK84*P(4,4) + HK85*P(4,5) + HK86*P(4,6);
    const float HK91 = HK80*P(0,2) + HK81*P(1,2) + HK82*P(2,2) + HK83*P(2,3) + HK84*P(2,4) + HK85*P(2,5) + HK86*P(2,6);
    const float HK92 = 2*HK43;
    const float HK93 = HK80*P(0,3) + HK81*P(1,3) + HK82*P(2,3) + HK83*P(3,3) + HK84*P(3,4) + HK85*P(3,5) + HK86*P(3,6);
    const float HK94 = HK80*P(0,1) + HK81*P(1,1) + HK82*P(1,2) + HK83*P(1,3) + HK84*P(1,4) + HK85*P(1,5) + HK86*P(1,6);
    const float HK95 = HK4/(HK43*HK74*HK90 + HK43*HK77*HK89 + HK43*HK79*HK88 + HK43*HK80*HK87 + HK66*HK92*HK94 + HK68*HK91*HK92 + HK70*HK92*HK93 + R_LOS);


    // Compare X axis equations
    {
        // evaluate sympy genrated equations for observatio Jacobians and Kalman gains
        {
            // calculate innovation variance for X axis observation and protect against a badly conditioned calculation
            flow_innov_var = (HK25*HK43*HK46 + HK33*HK43*HK45 + HK37*HK43*HK44 + HK38*HK42*HK43 + HK39*HK43*HK49 + HK40*HK43*HK47 + HK41*HK43*HK48 + R_LOS);

            const float HK50 = HK4/flow_innov_var;

            // Observation Jacobians - axis 0
            Hfusion.at<0>() = HK3*HK5;
            Hfusion.at<1>() = HK5*HK7;
            Hfusion.at<2>() = HK5*HK8;
            Hfusion.at<3>() = HK5*HK9;
            Hfusion.at<4>() = HK25*HK4;
            Hfusion.at<5>() = HK33*HK4;
            Hfusion.at<6>() = HK37*HK4;

			// Kalman gains - axis 0
			Kfusion(0) = HK42*HK50;
			Kfusion(1) = HK49*HK50;
			Kfusion(2) = HK47*HK50;
			Kfusion(3) = HK48*HK50;
			Kfusion(4) = HK46*HK50;
			Kfusion(5) = HK45*HK50;
			Kfusion(6) = HK44*HK50;

			for (unsigned row = 7; row <= 23; row++) {
				Kfusion(row) = HK50*(HK25*P(4,row) + HK33*P(5,row) + HK37*P(6,row) + HK38*P(0,row) + HK39*P(1,row) + HK40*P(2,row) + HK41*P(3,row));
			}

            // copy to arrays used for comparison
            for (int row=0; row<7; row++) {
                Hfusion_sympy(row) = Hfusion.atCompressedIndex(row);
            }
            for (int row=0; row<24; row++) {
                Kfusion_sympy(row) = Kfusion(row);
            }

        }

        // repeat calculation using matlab generated equations

        {
            // calculate X axis observation Jacobian
            float t2 = 1.0f / range;
            float H_LOS[24] = {};
            H_LOS[0] = t2*(q1*vd*2.0f+q0*ve*2.0f-q3*vn*2.0f);
            H_LOS[1] = t2*(q0*vd*2.0f-q1*ve*2.0f+q2*vn*2.0f);
            H_LOS[2] = t2*(q3*vd*2.0f+q2*ve*2.0f+q1*vn*2.0f);
            H_LOS[3] = -t2*(q2*vd*-2.0f+q3*ve*2.0f+q0*vn*2.0f);
            H_LOS[4] = -t2*(q0*q3*2.0f-q1*q2*2.0f);
            H_LOS[5] = t2*(q0*q0-q1*q1+q2*q2-q3*q3);
            H_LOS[6] = t2*(q0*q1*2.0f+q2*q3*2.0f);

            // calculate intermediate variables for the X observation innovatoin variance and Kalman gains
            float t3 = q1*vd*2.0f;
            float t4 = q0*ve*2.0f;
            float t11 = q3*vn*2.0f;
            float t5 = t3+t4-t11;
            float t6 = q0*q3*2.0f;
            float t29 = q1*q2*2.0f;
            float t7 = t6-t29;
            float t8 = q0*q1*2.0f;
            float t9 = q2*q3*2.0f;
            float t10 = t8+t9;
            float t12 = P(0,0)*t2*t5;
            float t13 = q0*vd*2.0f;
            float t14 = q2*vn*2.0f;
            float t28 = q1*ve*2.0f;
            float t15 = t13+t14-t28;
            float t16 = q3*vd*2.0f;
            float t17 = q2*ve*2.0f;
            float t18 = q1*vn*2.0f;
            float t19 = t16+t17+t18;
            float t20 = q3*ve*2.0f;
            float t21 = q0*vn*2.0f;
            float t30 = q2*vd*2.0f;
            float t22 = t20+t21-t30;
            float t23 = q0*q0;
            float t24 = q1*q1;
            float t25 = q2*q2;
            float t26 = q3*q3;
            float t27 = t23-t24+t25-t26;
            float t31 = P(1,1)*t2*t15;
            float t32 = P(6,0)*t2*t10;
            float t33 = P(1,0)*t2*t15;
            float t34 = P(2,0)*t2*t19;
            float t35 = P(5,0)*t2*t27;
            float t79 = P(4,0)*t2*t7;
            float t80 = P(3,0)*t2*t22;
            float t36 = t12+t32+t33+t34+t35-t79-t80;
            float t37 = t2*t5*t36;
            float t38 = P(6,1)*t2*t10;
            float t39 = P(0,1)*t2*t5;
            float t40 = P(2,1)*t2*t19;
            float t41 = P(5,1)*t2*t27;
            float t81 = P(4,1)*t2*t7;
            float t82 = P(3,1)*t2*t22;
            float t42 = t31+t38+t39+t40+t41-t81-t82;
            float t43 = t2*t15*t42;
            float t44 = P(6,2)*t2*t10;
            float t45 = P(0,2)*t2*t5;
            float t46 = P(1,2)*t2*t15;
            float t47 = P(2,2)*t2*t19;
            float t48 = P(5,2)*t2*t27;
            float t83 = P(4,2)*t2*t7;
            float t84 = P(3,2)*t2*t22;
            float t49 = t44+t45+t46+t47+t48-t83-t84;
            float t50 = t2*t19*t49;
            float t51 = P(6,3)*t2*t10;
            float t52 = P(0,3)*t2*t5;
            float t53 = P(1,3)*t2*t15;
            float t54 = P(2,3)*t2*t19;
            float t55 = P(5,3)*t2*t27;
            float t85 = P(4,3)*t2*t7;
            float t86 = P(3,3)*t2*t22;
            float t56 = t51+t52+t53+t54+t55-t85-t86;
            float t57 = P(6,5)*t2*t10;
            float t58 = P(0,5)*t2*t5;
            float t59 = P(1,5)*t2*t15;
            float t60 = P(2,5)*t2*t19;
            float t61 = P(5,5)*t2*t27;
            float t88 = P(4,5)*t2*t7;
            float t89 = P(3,5)*t2*t22;
            float t62 = t57+t58+t59+t60+t61-t88-t89;
            float t63 = t2*t27*t62;
            float t64 = P(6,4)*t2*t10;
            float t65 = P(0,4)*t2*t5;
            float t66 = P(1,4)*t2*t15;
            float t67 = P(2,4)*t2*t19;
            float t68 = P(5,4)*t2*t27;
            float t90 = P(4,4)*t2*t7;
            float t91 = P(3,4)*t2*t22;
            float t69 = t64+t65+t66+t67+t68-t90-t91;
            float t70 = P(6,6)*t2*t10;
            float t71 = P(0,6)*t2*t5;
            float t72 = P(1,6)*t2*t15;
            float t73 = P(2,6)*t2*t19;
            float t74 = P(5,6)*t2*t27;
            float t93 = P(4,6)*t2*t7;
            float t94 = P(3,6)*t2*t22;
            float t75 = t70+t71+t72+t73+t74-t93-t94;
            float t76 = t2*t10*t75;
            float t87 = t2*t22*t56;
            float t92 = t2*t7*t69;
            float t77 = R_LOS+t37+t43+t50+t63+t76-t87-t92;
            float t78 = 1.0f / t77;
            flow_innov_var = t77;

            // calculate Kalman gains for X-axis observation
            float Kfusion[24];
            Kfusion[0] = t78*(t12-P(0,4)*t2*t7+P(0,1)*t2*t15+P(0,6)*t2*t10+P(0,2)*t2*t19-P(0,3)*t2*t22+P(0,5)*t2*t27);
            Kfusion[1] = t78*(t31+P(1,0)*t2*t5-P(1,4)*t2*t7+P(1,6)*t2*t10+P(1,2)*t2*t19-P(1,3)*t2*t22+P(1,5)*t2*t27);
            Kfusion[2] = t78*(t47+P(2,0)*t2*t5-P(2,4)*t2*t7+P(2,1)*t2*t15+P(2,6)*t2*t10-P(2,3)*t2*t22+P(2,5)*t2*t27);
            Kfusion[3] = t78*(-t86+P(3,0)*t2*t5-P(3,4)*t2*t7+P(3,1)*t2*t15+P(3,6)*t2*t10+P(3,2)*t2*t19+P(3,5)*t2*t27);
            Kfusion[4] = t78*(-t90+P(4,0)*t2*t5+P(4,1)*t2*t15+P(4,6)*t2*t10+P(4,2)*t2*t19-P(4,3)*t2*t22+P(4,5)*t2*t27);
            Kfusion[5] = t78*(t61+P(5,0)*t2*t5-P(5,4)*t2*t7+P(5,1)*t2*t15+P(5,6)*t2*t10+P(5,2)*t2*t19-P(5,3)*t2*t22);
            Kfusion[6] = t78*(t70+P(6,0)*t2*t5-P(6,4)*t2*t7+P(6,1)*t2*t15+P(6,2)*t2*t19-P(6,3)*t2*t22+P(6,5)*t2*t27);
            Kfusion[7] = t78*(P(7,0)*t2*t5-P(7,4)*t2*t7+P(7,1)*t2*t15+P(7,6)*t2*t10+P(7,2)*t2*t19-P(7,3)*t2*t22+P(7,5)*t2*t27);
            Kfusion[8] = t78*(P(8,0)*t2*t5-P(8,4)*t2*t7+P(8,1)*t2*t15+P(8,6)*t2*t10+P(8,2)*t2*t19-P(8,3)*t2*t22+P(8,5)*t2*t27);
            Kfusion[9] = t78*(P(9,0)*t2*t5-P(9,4)*t2*t7+P(9,1)*t2*t15+P(9,6)*t2*t10+P(9,2)*t2*t19-P(9,3)*t2*t22+P(9,5)*t2*t27);
            Kfusion[10] = t78*(P(10,0)*t2*t5-P(10,4)*t2*t7+P(10,1)*t2*t15+P(10,6)*t2*t10+P(10,2)*t2*t19-P(10,3)*t2*t22+P(10,5)*t2*t27);
            Kfusion[11] = t78*(P(11,0)*t2*t5-P(11,4)*t2*t7+P(11,1)*t2*t15+P(11,6)*t2*t10+P(11,2)*t2*t19-P(11,3)*t2*t22+P(11,5)*t2*t27);
            Kfusion[12] = t78*(P(12,0)*t2*t5-P(12,4)*t2*t7+P(12,1)*t2*t15+P(12,6)*t2*t10+P(12,2)*t2*t19-P(12,3)*t2*t22+P(12,5)*t2*t27);
            Kfusion[13] = t78*(P(13,0)*t2*t5-P(13,4)*t2*t7+P(13,1)*t2*t15+P(13,6)*t2*t10+P(13,2)*t2*t19-P(13,3)*t2*t22+P(13,5)*t2*t27);
            Kfusion[14] = t78*(P(14,0)*t2*t5-P(14,4)*t2*t7+P(14,1)*t2*t15+P(14,6)*t2*t10+P(14,2)*t2*t19-P(14,3)*t2*t22+P(14,5)*t2*t27);
            Kfusion[15] = t78*(P(15,0)*t2*t5-P(15,4)*t2*t7+P(15,1)*t2*t15+P(15,6)*t2*t10+P(15,2)*t2*t19-P(15,3)*t2*t22+P(15,5)*t2*t27);
            Kfusion[16] = t78*(P(16,0)*t2*t5-P(16,4)*t2*t7+P(16,1)*t2*t15+P(16,6)*t2*t10+P(16,2)*t2*t19-P(16,3)*t2*t22+P(16,5)*t2*t27);
            Kfusion[17] = t78*(P(17,0)*t2*t5-P(17,4)*t2*t7+P(17,1)*t2*t15+P(17,6)*t2*t10+P(17,2)*t2*t19-P(17,3)*t2*t22+P(17,5)*t2*t27);
            Kfusion[18] = t78*(P(18,0)*t2*t5-P(18,4)*t2*t7+P(18,1)*t2*t15+P(18,6)*t2*t10+P(18,2)*t2*t19-P(18,3)*t2*t22+P(18,5)*t2*t27);
            Kfusion[19] = t78*(P(19,0)*t2*t5-P(19,4)*t2*t7+P(19,1)*t2*t15+P(19,6)*t2*t10+P(19,2)*t2*t19-P(19,3)*t2*t22+P(19,5)*t2*t27);
            Kfusion[20] = t78*(P(20,0)*t2*t5-P(20,4)*t2*t7+P(20,1)*t2*t15+P(20,6)*t2*t10+P(20,2)*t2*t19-P(20,3)*t2*t22+P(20,5)*t2*t27);
            Kfusion[21] = t78*(P(21,0)*t2*t5-P(21,4)*t2*t7+P(21,1)*t2*t15+P(21,6)*t2*t10+P(21,2)*t2*t19-P(21,3)*t2*t22+P(21,5)*t2*t27);
            Kfusion[22] = t78*(P(22,0)*t2*t5-P(22,4)*t2*t7+P(22,1)*t2*t15+P(22,6)*t2*t10+P(22,2)*t2*t19-P(22,3)*t2*t22+P(22,5)*t2*t27);
            Kfusion[23] = t78*(P(23,0)*t2*t5-P(23,4)*t2*t7+P(23,1)*t2*t15+P(23,6)*t2*t10+P(23,2)*t2*t19-P(23,3)*t2*t22+P(23,5)*t2*t27);

            for (int row=0; row<24; row++) {
                Hfusion_matlab(row) = H_LOS[row];
                Kfusion_matlab(row) = Kfusion[row];
            }
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
            printf("Fail: Optical Flow X axis Hfusion max diff fraction = %e , old = %e , new = %e , location index = %i\n",max_diff_fraction, max_old, max_new, max_row);
        } else {
            printf("Pass: Optical Flow X axis Hfusion max diff fraction = %e\n",max_diff_fraction);
        }

        // find largest Kalman gain difference as a fraction of the matlab value
        max_diff_fraction = 0.0f;
        for (int row=0; row<24; row++) {
            float diff_fraction;
            if (Kfusion_matlab(row) != 0.0f) {
                diff_fraction = fabsf(Kfusion_sympy(row) - Kfusion_matlab(row)) / fabsf(Kfusion_matlab(row));
            } else if (Hfusion_sympy(row) != 0.0f) {
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
            printf("Fail: Optical Flow X axis Kfusion max diff fraction = %e , old = %e , new = %e , location index = %i\n",max_diff_fraction, max_old, max_new, max_row);
        } else {
            printf("Pass: Optical Flow X axis Kfusion max diff fraction = %e\n",max_diff_fraction);
        }

    }

    // Compare Y axis equations

    {
        // evaluate sympy genrated equations for observatio Jacobians and Kalman gains
        {
            // calculate innovation variance for Y axis observation and protect against a badly conditioned calculation
            flow_innov_var = (HK43*HK74*HK90 + HK43*HK77*HK89 + HK43*HK79*HK88 + HK43*HK80*HK87 + HK66*HK92*HK94 + HK68*HK91*HK92 + HK70*HK92*HK93 + R_LOS);

            const float HK95 = HK4/flow_innov_var;

			// Observation Jacobians - axis 1
			Hfusion.at<0>() = -HK5*HK63;
			Hfusion.at<1>() = -HK5*HK66;
			Hfusion.at<2>() = -HK5*HK68;
			Hfusion.at<3>() = -HK5*HK70;
			Hfusion.at<4>() = -HK4*HK74;
			Hfusion.at<5>() = -HK4*HK77;
			Hfusion.at<6>() = -HK4*HK79;

			// Kalman gains - axis 1
			Kfusion(0) = -HK87*HK95;
			Kfusion(1) = -HK94*HK95;
			Kfusion(2) = -HK91*HK95;
			Kfusion(3) = -HK93*HK95;
			Kfusion(4) = -HK90*HK95;
			Kfusion(5) = -HK89*HK95;
			Kfusion(6) = -HK88*HK95;

			for (unsigned row = 7; row <= 23; row++) {
				Kfusion(row) = -HK95*(HK80*P(0,row) + HK81*P(1,row) + HK82*P(2,row) + HK83*P(3,row) + HK84*P(4,row) + HK85*P(5,row) + HK86*P(6,row));
			}

            // copy to arrays used for comparison
            for (int row=0; row<7; row++) {
                Hfusion_sympy(row) = Hfusion.atCompressedIndex(row);
            }
            for (int row=0; row<24; row++) {
                Kfusion_sympy(row) = Kfusion(row);
            }

        }

        // repeat calculation using matlab generated equations

        {
			// calculate Y axis observation Jacobian
			float t2 = 1.0f / range;
            float H_LOS[24] = {};
			H_LOS[0] = -t2*(q2*vd*-2.0f+q3*ve*2.0f+q0*vn*2.0f);
			H_LOS[1] = -t2*(q3*vd*2.0f+q2*ve*2.0f+q1*vn*2.0f);
			H_LOS[2] = t2*(q0*vd*2.0f-q1*ve*2.0f+q2*vn*2.0f);
			H_LOS[3] = -t2*(q1*vd*2.0f+q0*ve*2.0f-q3*vn*2.0f);
			H_LOS[4] = -t2*(q0*q0+q1*q1-q2*q2-q3*q3);
			H_LOS[5] = -t2*(q0*q3*2.0f+q1*q2*2.0f);
			H_LOS[6] = t2*(q0*q2*2.0f-q1*q3*2.0f);

			// calculate intermediate variables for the Y observation innovatoin variance and Kalman gains
			float t3 = q3*ve*2.0f;
			float t4 = q0*vn*2.0f;
			float t11 = q2*vd*2.0f;
			float t5 = t3+t4-t11;
			float t6 = q0*q3*2.0f;
			float t7 = q1*q2*2.0f;
			float t8 = t6+t7;
			float t9 = q0*q2*2.0f;
			float t28 = q1*q3*2.0f;
			float t10 = t9-t28;
			float t12 = P(0,0)*t2*t5;
			float t13 = q3*vd*2.0f;
			float t14 = q2*ve*2.0f;
			float t15 = q1*vn*2.0f;
			float t16 = t13+t14+t15;
			float t17 = q0*vd*2.0f;
			float t18 = q2*vn*2.0f;
			float t29 = q1*ve*2.0f;
			float t19 = t17+t18-t29;
			float t20 = q1*vd*2.0f;
			float t21 = q0*ve*2.0f;
			float t30 = q3*vn*2.0f;
			float t22 = t20+t21-t30;
			float t23 = q0*q0;
			float t24 = q1*q1;
			float t25 = q2*q2;
			float t26 = q3*q3;
			float t27 = t23+t24-t25-t26;
			float t31 = P(1,1)*t2*t16;
			float t32 = P(5,0)*t2*t8;
			float t33 = P(1,0)*t2*t16;
			float t34 = P(3,0)*t2*t22;
			float t35 = P(4,0)*t2*t27;
			float t80 = P(6,0)*t2*t10;
			float t81 = P(2,0)*t2*t19;
			float t36 = t12+t32+t33+t34+t35-t80-t81;
			float t37 = t2*t5*t36;
			float t38 = P(5,1)*t2*t8;
			float t39 = P(0,1)*t2*t5;
			float t40 = P(3,1)*t2*t22;
			float t41 = P(4,1)*t2*t27;
			float t82 = P(6,1)*t2*t10;
			float t83 = P(2,1)*t2*t19;
			float t42 = t31+t38+t39+t40+t41-t82-t83;
			float t43 = t2*t16*t42;
			float t44 = P(5,2)*t2*t8;
			float t45 = P(0,2)*t2*t5;
			float t46 = P(1,2)*t2*t16;
			float t47 = P(3,2)*t2*t22;
			float t48 = P(4,2)*t2*t27;
			float t79 = P(2,2)*t2*t19;
			float t84 = P(6,2)*t2*t10;
			float t49 = t44+t45+t46+t47+t48-t79-t84;
			float t50 = P(5,3)*t2*t8;
			float t51 = P(0,3)*t2*t5;
			float t52 = P(1,3)*t2*t16;
			float t53 = P(3,3)*t2*t22;
			float t54 = P(4,3)*t2*t27;
			float t86 = P(6,3)*t2*t10;
			float t87 = P(2,3)*t2*t19;
			float t55 = t50+t51+t52+t53+t54-t86-t87;
			float t56 = t2*t22*t55;
			float t57 = P(5,4)*t2*t8;
			float t58 = P(0,4)*t2*t5;
			float t59 = P(1,4)*t2*t16;
			float t60 = P(3,4)*t2*t22;
			float t61 = P(4,4)*t2*t27;
			float t88 = P(6,4)*t2*t10;
			float t89 = P(2,4)*t2*t19;
			float t62 = t57+t58+t59+t60+t61-t88-t89;
			float t63 = t2*t27*t62;
			float t64 = P(5,5)*t2*t8;
			float t65 = P(0,5)*t2*t5;
			float t66 = P(1,5)*t2*t16;
			float t67 = P(3,5)*t2*t22;
			float t68 = P(4,5)*t2*t27;
			float t90 = P(6,5)*t2*t10;
			float t91 = P(2,5)*t2*t19;
			float t69 = t64+t65+t66+t67+t68-t90-t91;
			float t70 = t2*t8*t69;
			float t71 = P(5,6)*t2*t8;
			float t72 = P(0,6)*t2*t5;
			float t73 = P(1,6)*t2*t16;
			float t74 = P(3,6)*t2*t22;
			float t75 = P(4,6)*t2*t27;
			float t92 = P(6,6)*t2*t10;
			float t93 = P(2,6)*t2*t19;
			float t76 = t71+t72+t73+t74+t75-t92-t93;
			float t85 = t2*t19*t49;
			float t94 = t2*t10*t76;
			float t77 = R_LOS+t37+t43+t56+t63+t70-t85-t94;

			float t78 = 1.0f / t77;
			flow_innov_var = t77;

			// calculate Kalman gains for Y-axis observation
            float Kfusion[24];
			Kfusion[0] = -t78*(t12+P(0,5)*t2*t8-P(0,6)*t2*t10+P(0,1)*t2*t16-P(0,2)*t2*t19+P(0,3)*t2*t22+P(0,4)*t2*t27);
			Kfusion[1] = -t78*(t31+P(1,0)*t2*t5+P(1,5)*t2*t8-P(1,6)*t2*t10-P(1,2)*t2*t19+P(1,3)*t2*t22+P(1,4)*t2*t27);
			Kfusion[2] = -t78*(-t79+P(2,0)*t2*t5+P(2,5)*t2*t8-P(2,6)*t2*t10+P(2,1)*t2*t16+P(2,3)*t2*t22+P(2,4)*t2*t27);
			Kfusion[3] = -t78*(t53+P(3,0)*t2*t5+P(3,5)*t2*t8-P(3,6)*t2*t10+P(3,1)*t2*t16-P(3,2)*t2*t19+P(3,4)*t2*t27);
			Kfusion[4] = -t78*(t61+P(4,0)*t2*t5+P(4,5)*t2*t8-P(4,6)*t2*t10+P(4,1)*t2*t16-P(4,2)*t2*t19+P(4,3)*t2*t22);
			Kfusion[5] = -t78*(t64+P(5,0)*t2*t5-P(5,6)*t2*t10+P(5,1)*t2*t16-P(5,2)*t2*t19+P(5,3)*t2*t22+P(5,4)*t2*t27);
			Kfusion[6] = -t78*(-t92+P(6,0)*t2*t5+P(6,5)*t2*t8+P(6,1)*t2*t16-P(6,2)*t2*t19+P(6,3)*t2*t22+P(6,4)*t2*t27);
			Kfusion[7] = -t78*(P(7,0)*t2*t5+P(7,5)*t2*t8-P(7,6)*t2*t10+P(7,1)*t2*t16-P(7,2)*t2*t19+P(7,3)*t2*t22+P(7,4)*t2*t27);
			Kfusion[8] = -t78*(P(8,0)*t2*t5+P(8,5)*t2*t8-P(8,6)*t2*t10+P(8,1)*t2*t16-P(8,2)*t2*t19+P(8,3)*t2*t22+P(8,4)*t2*t27);
			Kfusion[9] = -t78*(P(9,0)*t2*t5+P(9,5)*t2*t8-P(9,6)*t2*t10+P(9,1)*t2*t16-P(9,2)*t2*t19+P(9,3)*t2*t22+P(9,4)*t2*t27);
			Kfusion[10] = -t78*(P(10,0)*t2*t5+P(10,5)*t2*t8-P(10,6)*t2*t10+P(10,1)*t2*t16-P(10,2)*t2*t19+P(10,3)*t2*t22+P(10,4)*t2*t27);
			Kfusion[11] = -t78*(P(11,0)*t2*t5+P(11,5)*t2*t8-P(11,6)*t2*t10+P(11,1)*t2*t16-P(11,2)*t2*t19+P(11,3)*t2*t22+P(11,4)*t2*t27);
			Kfusion[12] = -t78*(P(12,0)*t2*t5+P(12,5)*t2*t8-P(12,6)*t2*t10+P(12,1)*t2*t16-P(12,2)*t2*t19+P(12,3)*t2*t22+P(12,4)*t2*t27);
			Kfusion[13] = -t78*(P(13,0)*t2*t5+P(13,5)*t2*t8-P(13,6)*t2*t10+P(13,1)*t2*t16-P(13,2)*t2*t19+P(13,3)*t2*t22+P(13,4)*t2*t27);
			Kfusion[14] = -t78*(P(14,0)*t2*t5+P(14,5)*t2*t8-P(14,6)*t2*t10+P(14,1)*t2*t16-P(14,2)*t2*t19+P(14,3)*t2*t22+P(14,4)*t2*t27);
			Kfusion[15] = -t78*(P(15,0)*t2*t5+P(15,5)*t2*t8-P(15,6)*t2*t10+P(15,1)*t2*t16-P(15,2)*t2*t19+P(15,3)*t2*t22+P(15,4)*t2*t27);
			Kfusion[16] = -t78*(P(16,0)*t2*t5+P(16,5)*t2*t8-P(16,6)*t2*t10+P(16,1)*t2*t16-P(16,2)*t2*t19+P(16,3)*t2*t22+P(16,4)*t2*t27);
			Kfusion[17] = -t78*(P(17,0)*t2*t5+P(17,5)*t2*t8-P(17,6)*t2*t10+P(17,1)*t2*t16-P(17,2)*t2*t19+P(17,3)*t2*t22+P(17,4)*t2*t27);
			Kfusion[18] = -t78*(P(18,0)*t2*t5+P(18,5)*t2*t8-P(18,6)*t2*t10+P(18,1)*t2*t16-P(18,2)*t2*t19+P(18,3)*t2*t22+P(18,4)*t2*t27);
			Kfusion[19] = -t78*(P(19,0)*t2*t5+P(19,5)*t2*t8-P(19,6)*t2*t10+P(19,1)*t2*t16-P(19,2)*t2*t19+P(19,3)*t2*t22+P(19,4)*t2*t27);
			Kfusion[20] = -t78*(P(20,0)*t2*t5+P(20,5)*t2*t8-P(20,6)*t2*t10+P(20,1)*t2*t16-P(20,2)*t2*t19+P(20,3)*t2*t22+P(20,4)*t2*t27);
			Kfusion[21] = -t78*(P(21,0)*t2*t5+P(21,5)*t2*t8-P(21,6)*t2*t10+P(21,1)*t2*t16-P(21,2)*t2*t19+P(21,3)*t2*t22+P(21,4)*t2*t27);
			Kfusion[22] = -t78*(P(22,0)*t2*t5+P(22,5)*t2*t8-P(22,6)*t2*t10+P(22,1)*t2*t16-P(22,2)*t2*t19+P(22,3)*t2*t22+P(22,4)*t2*t27);
			Kfusion[23] = -t78*(P(23,0)*t2*t5+P(23,5)*t2*t8-P(23,6)*t2*t10+P(23,1)*t2*t16-P(23,2)*t2*t19+P(23,3)*t2*t22+P(23,4)*t2*t27);

            for (int row=0; row<24; row++) {
                Hfusion_matlab(row) = H_LOS[row];
                Kfusion_matlab(row) = Kfusion[row];
            }
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
            printf("Fail: Optical Flow Y axis Hfusion max diff fraction = %e , old = %e , new = %e , location index = %i\n",max_diff_fraction, max_old, max_new, max_row);
        } else {
            printf("Pass: Optical Flow Y axis Hfusion max diff fraction = %e\n",max_diff_fraction);
        }

        // find largest Kalman gain difference as a fraction of the matlab value
        max_diff_fraction = 0.0f;
        for (int row=0; row<24; row++) {
            float diff_fraction;
            if (Kfusion_matlab(row) != 0.0f) {
                diff_fraction = fabsf(Kfusion_sympy(row) - Kfusion_matlab(row)) / fabsf(Kfusion_matlab(row));
            } else if (Hfusion_sympy(row) != 0.0f) {
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
            printf("Fail: Optical Flow Y axis Kfusion max diff fraction = %e , old = %e , new = %e , location index = %i\n",max_diff_fraction, max_old, max_new, max_row);
        } else {
            printf("Pass: Optical Flow Y axis Kfusion max diff fraction = %e\n",max_diff_fraction);
        }

    }

    return 0;
}
