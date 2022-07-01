// Sub Expressions
const float HK0 = q1*vd;
const float HK1 = vn - vwn;
const float HK2 = HK1*q3;
const float HK3 = q2*vd;
const float HK4 = ve - vwe;
const float HK5 = HK4*q3;
const float HK6 = q0*q2 - q1*q3;
const float HK7 = 2*vd;
const float HK8 = q0*q3;
const float HK9 = q1*q2;
const float HK10 = 2*HK8 + 2*HK9;
const float HK11 = 2*(q3)*(q3) - 1;
const float HK12 = HK11 + 2*(q2)*(q2);
const float HK13 = HK1*HK12 - HK10*HK4 + HK6*HK7;
const float HK14 = 1.0F/(HK13);
const float HK15 = q0*q1 + q2*q3;
const float HK16 = HK11 + 2*(q1)*(q1);
const float HK17 = 2*HK1*(HK8 - HK9) - HK15*HK7 + HK16*HK4;
const float HK18 = HK14*HK17;
const float HK19 = 2*HK14;
const float HK20 = HK19*(HK0 + HK18*(HK3 - HK5) - HK2);
const float HK21 = q0*vd;
const float HK22 = HK1*q2;
const float HK23 = HK4*q1;
const float HK24 = q3*vd;
const float HK25 = -1/HK13;
const float HK26 = -HK17*HK25;
const float HK27 = HK21 + HK22 - 2*HK23 - HK26*(HK24 + HK4*q2);
const float HK28 = 2*HK25;
const float HK29 = HK19*(HK1*q1 + HK18*(HK21 + 2*HK22 - HK23) + HK24);
const float HK30 = HK28*(HK1*q0 + HK26*(HK0 - 2*HK2 + HK4*q0) - HK3 + 2*HK5);
const float HK31 = -2*HK8 + 2*HK9;
const float HK32 = HK12*HK26 + HK31;
const float HK33 = HK25*(HK10*HK26 + HK16);
const float HK34 = HK19*(HK15 + HK18*HK6);
const float HK35 = HK12*HK18 + HK31;
const float HK36 = HK14*(HK10*HK18 + HK16);
const float HK37 = -HK25*HK32;
const float HK38 = -HK27*HK28;
const float HK39 = HK14*HK35*P(0,22) - HK20*P(0,0) - HK29*P(0,2) - HK30*P(0,3) - HK33*P(0,5) - HK34*P(0,6) - HK36*P(0,23) - HK37*P(0,4) - HK38*P(0,1);
const float HK40 = HK14*HK35*P(6,22) - HK20*P(0,6) - HK29*P(2,6) - HK30*P(3,6) - HK33*P(5,6) - HK34*P(6,6) - HK36*P(6,23) - HK37*P(4,6) - HK38*P(1,6);
const float HK41 = HK14*HK35*P(22,23) - HK20*P(0,23) - HK29*P(2,23) - HK30*P(3,23) - HK33*P(5,23) - HK34*P(6,23) - HK36*P(23,23) - HK37*P(4,23) - HK38*P(1,23);
const float HK42 = HK14*HK35*P(22,22) - HK20*P(0,22) - HK29*P(2,22) - HK30*P(3,22) - HK33*P(5,22) - HK34*P(6,22) - HK36*P(22,23) - HK37*P(4,22) - HK38*P(1,22);
const float HK43 = HK14*HK35*P(5,22) - HK20*P(0,5) - HK29*P(2,5) - HK30*P(3,5) - HK33*P(5,5) - HK34*P(5,6) - HK36*P(5,23) - HK37*P(4,5) - HK38*P(1,5);
const float HK44 = HK14*HK35*P(4,22) - HK20*P(0,4) - HK29*P(2,4) - HK30*P(3,4) - HK33*P(4,5) - HK34*P(4,6) - HK36*P(4,23) - HK37*P(4,4) - HK38*P(1,4);
const float HK45 = HK14*HK35*P(2,22) - HK20*P(0,2) - HK29*P(2,2) - HK30*P(2,3) - HK33*P(2,5) - HK34*P(2,6) - HK36*P(2,23) - HK37*P(2,4) - HK38*P(1,2);
const float HK46 = HK14*HK35*P(1,22) - HK20*P(0,1) - HK29*P(1,2) - HK30*P(1,3) - HK33*P(1,5) - HK34*P(1,6) - HK36*P(1,23) - HK37*P(1,4) - HK38*P(1,1);
const float HK47 = HK14*HK35*P(3,22) - HK20*P(0,3) - HK29*P(2,3) - HK30*P(3,3) - HK33*P(3,5) - HK34*P(3,6) - HK36*P(3,23) - HK37*P(3,4) - HK38*P(1,3);
const float HK48 = 1.0F/(HK14*HK35*HK42 - HK20*HK39 - HK29*HK45 - HK30*HK47 - HK33*HK43 - HK34*HK40 - HK36*HK41 - HK37*HK44 - HK38*HK46 + R_BETA);


// Observation Jacobians
Hfusion.at<0>() = -HK20;
Hfusion.at<1>() = HK27*HK28;
Hfusion.at<2>() = -HK29;
Hfusion.at<3>() = -HK30;
Hfusion.at<4>() = HK25*HK32;
Hfusion.at<5>() = -HK33;
Hfusion.at<6>() = -HK34;
Hfusion.at<22>() = HK14*HK35;
Hfusion.at<23>() = -HK36;


// Kalman gains
Kfusion(0) = HK39*HK48;
Kfusion(1) = HK46*HK48;
Kfusion(2) = HK45*HK48;
Kfusion(3) = HK47*HK48;
Kfusion(4) = HK44*HK48;
Kfusion(5) = HK43*HK48;
Kfusion(6) = HK40*HK48;
Kfusion(7) = HK48*(HK14*HK35*P(7,22) - HK20*P(0,7) - HK29*P(2,7) - HK30*P(3,7) - HK33*P(5,7) - HK34*P(6,7) - HK36*P(7,23) - HK37*P(4,7) - HK38*P(1,7));
Kfusion(8) = HK48*(HK14*HK35*P(8,22) - HK20*P(0,8) - HK29*P(2,8) - HK30*P(3,8) - HK33*P(5,8) - HK34*P(6,8) - HK36*P(8,23) - HK37*P(4,8) - HK38*P(1,8));
Kfusion(9) = HK48*(HK14*HK35*P(9,22) - HK20*P(0,9) - HK29*P(2,9) - HK30*P(3,9) - HK33*P(5,9) - HK34*P(6,9) - HK36*P(9,23) - HK37*P(4,9) - HK38*P(1,9));
Kfusion(10) = HK48*(HK14*HK35*P(10,22) - HK20*P(0,10) - HK29*P(2,10) - HK30*P(3,10) - HK33*P(5,10) - HK34*P(6,10) - HK36*P(10,23) - HK37*P(4,10) - HK38*P(1,10));
Kfusion(11) = HK48*(HK14*HK35*P(11,22) - HK20*P(0,11) - HK29*P(2,11) - HK30*P(3,11) - HK33*P(5,11) - HK34*P(6,11) - HK36*P(11,23) - HK37*P(4,11) - HK38*P(1,11));
Kfusion(12) = HK48*(HK14*HK35*P(12,22) - HK20*P(0,12) - HK29*P(2,12) - HK30*P(3,12) - HK33*P(5,12) - HK34*P(6,12) - HK36*P(12,23) - HK37*P(4,12) - HK38*P(1,12));
Kfusion(13) = HK48*(HK14*HK35*P(13,22) - HK20*P(0,13) - HK29*P(2,13) - HK30*P(3,13) - HK33*P(5,13) - HK34*P(6,13) - HK36*P(13,23) - HK37*P(4,13) - HK38*P(1,13));
Kfusion(14) = HK48*(HK14*HK35*P(14,22) - HK20*P(0,14) - HK29*P(2,14) - HK30*P(3,14) - HK33*P(5,14) - HK34*P(6,14) - HK36*P(14,23) - HK37*P(4,14) - HK38*P(1,14));
Kfusion(15) = HK48*(HK14*HK35*P(15,22) - HK20*P(0,15) - HK29*P(2,15) - HK30*P(3,15) - HK33*P(5,15) - HK34*P(6,15) - HK36*P(15,23) - HK37*P(4,15) - HK38*P(1,15));
Kfusion(16) = HK48*(HK14*HK35*P(16,22) - HK20*P(0,16) - HK29*P(2,16) - HK30*P(3,16) - HK33*P(5,16) - HK34*P(6,16) - HK36*P(16,23) - HK37*P(4,16) - HK38*P(1,16));
Kfusion(17) = HK48*(HK14*HK35*P(17,22) - HK20*P(0,17) - HK29*P(2,17) - HK30*P(3,17) - HK33*P(5,17) - HK34*P(6,17) - HK36*P(17,23) - HK37*P(4,17) - HK38*P(1,17));
Kfusion(18) = HK48*(HK14*HK35*P(18,22) - HK20*P(0,18) - HK29*P(2,18) - HK30*P(3,18) - HK33*P(5,18) - HK34*P(6,18) - HK36*P(18,23) - HK37*P(4,18) - HK38*P(1,18));
Kfusion(19) = HK48*(HK14*HK35*P(19,22) - HK20*P(0,19) - HK29*P(2,19) - HK30*P(3,19) - HK33*P(5,19) - HK34*P(6,19) - HK36*P(19,23) - HK37*P(4,19) - HK38*P(1,19));
Kfusion(20) = HK48*(HK14*HK35*P(20,22) - HK20*P(0,20) - HK29*P(2,20) - HK30*P(3,20) - HK33*P(5,20) - HK34*P(6,20) - HK36*P(20,23) - HK37*P(4,20) - HK38*P(1,20));
Kfusion(21) = HK48*(HK14*HK35*P(21,22) - HK20*P(0,21) - HK29*P(2,21) - HK30*P(3,21) - HK33*P(5,21) - HK34*P(6,21) - HK36*P(21,23) - HK37*P(4,21) - HK38*P(1,21));
Kfusion(22) = HK42*HK48;
Kfusion(23) = HK41*HK48;


