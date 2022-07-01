// Sub Expressions
const float HK0 = vn - vwn;
const float HK1 = ve - vwe;
const float HK2 = HK0*q0 + HK1*q3 - q2*vd;
const float HK3 = q0*q2 - q1*q3;
const float HK4 = 2*vd;
const float HK5 = q0*q3;
const float HK6 = q1*q2;
const float HK7 = 2*HK5 + 2*HK6;
const float HK8 = (q0)*(q0);
const float HK9 = (q3)*(q3);
const float HK10 = HK8 - HK9;
const float HK11 = (q1)*(q1);
const float HK12 = (q2)*(q2);
const float HK13 = HK11 - HK12;
const float HK14 = HK10 + HK13;
const float HK15 = HK0*HK14 + HK1*HK7 - HK3*HK4;
const float HK16 = 1.0F/(HK15);
const float HK17 = q0*q1 + q2*q3;
const float HK18 = HK16*(-2*HK0*(HK5 - HK6) + HK1*(HK10 - HK11 + HK12) + HK17*HK4);
const float HK19 = -HK0*q3 + HK1*q0 + q1*vd;
const float HK20 = -HK18*HK2 + HK19;
const float HK21 = 2*HK16;
const float HK22 = HK0*q1 + HK1*q2 + q3*vd;
const float HK23 = HK0*q2 - HK1*q1 + q0*vd;
const float HK24 = -HK18*HK22 + HK23;
const float HK25 = HK18*HK23 + HK22;
const float HK26 = HK18*HK19 + HK2;
const float HK27 = HK14*HK18 + 2*HK5 - 2*HK6;
const float HK28 = HK16*HK27;
const float HK29 = HK13 + HK18*HK7 - HK8 + HK9;
const float HK30 = HK17 + HK18*HK3;
const float HK31 = 2*HK30;
const float HK32 = 2*HK25;
const float HK33 = 2*HK24;
const float HK34 = 2*HK26;
const float HK35 = 2*HK20;
const float HK36 = HK27*P(0,22) - HK27*P(0,4) + HK29*P(0,23) - HK29*P(0,5) + HK31*P(0,6) + HK32*P(0,2) + HK33*P(0,1) - HK34*P(0,3) + HK35*P(0,0);
const float HK37 = 1.0F/((HK15)*(HK15));
const float HK38 = -HK27*P(4,6) + HK27*P(6,22) - HK29*P(5,6) + HK29*P(6,23) + HK31*P(6,6) + HK32*P(2,6) + HK33*P(1,6) - HK34*P(3,6) + HK35*P(0,6);
const float HK39 = HK29*P(5,23);
const float HK40 = HK27*P(22,23) - HK27*P(4,23) + HK29*P(23,23) + HK31*P(6,23) + HK32*P(2,23) + HK33*P(1,23) - HK34*P(3,23) + HK35*P(0,23) - HK39;
const float HK41 = HK29*HK37;
const float HK42 = HK27*P(4,22);
const float HK43 = HK27*P(22,22) + HK29*P(22,23) - HK29*P(5,22) + HK31*P(6,22) + HK32*P(2,22) + HK33*P(1,22) - HK34*P(3,22) + HK35*P(0,22) - HK42;
const float HK44 = HK27*HK37;
const float HK45 = -HK27*P(4,5) + HK27*P(5,22) - HK29*P(5,5) + HK31*P(5,6) + HK32*P(2,5) + HK33*P(1,5) - HK34*P(3,5) + HK35*P(0,5) + HK39;
const float HK46 = -HK27*P(4,4) + HK29*P(4,23) - HK29*P(4,5) + HK31*P(4,6) + HK32*P(2,4) + HK33*P(1,4) - HK34*P(3,4) + HK35*P(0,4) + HK42;
const float HK47 = HK27*P(2,22) - HK27*P(2,4) + HK29*P(2,23) - HK29*P(2,5) + HK31*P(2,6) + HK32*P(2,2) + HK33*P(1,2) - HK34*P(2,3) + HK35*P(0,2);
const float HK48 = HK27*P(1,22) - HK27*P(1,4) + HK29*P(1,23) - HK29*P(1,5) + HK31*P(1,6) + HK32*P(1,2) + HK33*P(1,1) - HK34*P(1,3) + HK35*P(0,1);
const float HK49 = HK27*P(3,22) - HK27*P(3,4) + HK29*P(3,23) - HK29*P(3,5) + HK31*P(3,6) + HK32*P(2,3) + HK33*P(1,3) - HK34*P(3,3) + HK35*P(0,3);
const float HK50 = HK16/(HK31*HK37*HK38 + HK32*HK37*HK47 + HK33*HK37*HK48 - HK34*HK37*HK49 + HK35*HK36*HK37 + HK40*HK41 - HK41*HK45 + HK43*HK44 - HK44*HK46 + R_BETA);


// Observation Jacobians
Hfusion.at<0>() = HK20*HK21;
Hfusion.at<1>() = HK21*HK24;
Hfusion.at<2>() = HK21*HK25;
Hfusion.at<3>() = -HK21*HK26;
Hfusion.at<4>() = -HK28;
Hfusion.at<5>() = -HK16*HK29;
Hfusion.at<6>() = HK21*HK30;
Hfusion.at<22>() = HK28;
Hfusion.at<23>() = HK16*HK29;


// Kalman gains
Kfusion(0) = HK36*HK50;
Kfusion(1) = HK48*HK50;
Kfusion(2) = HK47*HK50;
Kfusion(3) = HK49*HK50;
Kfusion(4) = HK46*HK50;
Kfusion(5) = HK45*HK50;
Kfusion(6) = HK38*HK50;
Kfusion(7) = HK50*(-HK27*P(4,7) + HK27*P(7,22) - HK29*P(5,7) + HK29*P(7,23) + HK31*P(6,7) + HK32*P(2,7) + HK33*P(1,7) - HK34*P(3,7) + HK35*P(0,7));
Kfusion(8) = HK50*(-HK27*P(4,8) + HK27*P(8,22) - HK29*P(5,8) + HK29*P(8,23) + HK31*P(6,8) + HK32*P(2,8) + HK33*P(1,8) - HK34*P(3,8) + HK35*P(0,8));
Kfusion(9) = HK50*(-HK27*P(4,9) + HK27*P(9,22) - HK29*P(5,9) + HK29*P(9,23) + HK31*P(6,9) + HK32*P(2,9) + HK33*P(1,9) - HK34*P(3,9) + HK35*P(0,9));
Kfusion(10) = HK50*(HK27*P(10,22) - HK27*P(4,10) + HK29*P(10,23) - HK29*P(5,10) + HK31*P(6,10) + HK32*P(2,10) + HK33*P(1,10) - HK34*P(3,10) + HK35*P(0,10));
Kfusion(11) = HK50*(HK27*P(11,22) - HK27*P(4,11) + HK29*P(11,23) - HK29*P(5,11) + HK31*P(6,11) + HK32*P(2,11) + HK33*P(1,11) - HK34*P(3,11) + HK35*P(0,11));
Kfusion(12) = HK50*(HK27*P(12,22) - HK27*P(4,12) + HK29*P(12,23) - HK29*P(5,12) + HK31*P(6,12) + HK32*P(2,12) + HK33*P(1,12) - HK34*P(3,12) + HK35*P(0,12));
Kfusion(13) = HK50*(HK27*P(13,22) - HK27*P(4,13) + HK29*P(13,23) - HK29*P(5,13) + HK31*P(6,13) + HK32*P(2,13) + HK33*P(1,13) - HK34*P(3,13) + HK35*P(0,13));
Kfusion(14) = HK50*(HK27*P(14,22) - HK27*P(4,14) + HK29*P(14,23) - HK29*P(5,14) + HK31*P(6,14) + HK32*P(2,14) + HK33*P(1,14) - HK34*P(3,14) + HK35*P(0,14));
Kfusion(15) = HK50*(HK27*P(15,22) - HK27*P(4,15) + HK29*P(15,23) - HK29*P(5,15) + HK31*P(6,15) + HK32*P(2,15) + HK33*P(1,15) - HK34*P(3,15) + HK35*P(0,15));
Kfusion(16) = HK50*(HK27*P(16,22) - HK27*P(4,16) + HK29*P(16,23) - HK29*P(5,16) + HK31*P(6,16) + HK32*P(2,16) + HK33*P(1,16) - HK34*P(3,16) + HK35*P(0,16));
Kfusion(17) = HK50*(HK27*P(17,22) - HK27*P(4,17) + HK29*P(17,23) - HK29*P(5,17) + HK31*P(6,17) + HK32*P(2,17) + HK33*P(1,17) - HK34*P(3,17) + HK35*P(0,17));
Kfusion(18) = HK50*(HK27*P(18,22) - HK27*P(4,18) + HK29*P(18,23) - HK29*P(5,18) + HK31*P(6,18) + HK32*P(2,18) + HK33*P(1,18) - HK34*P(3,18) + HK35*P(0,18));
Kfusion(19) = HK50*(HK27*P(19,22) - HK27*P(4,19) + HK29*P(19,23) - HK29*P(5,19) + HK31*P(6,19) + HK32*P(2,19) + HK33*P(1,19) - HK34*P(3,19) + HK35*P(0,19));
Kfusion(20) = HK50*(HK27*P(20,22) - HK27*P(4,20) + HK29*P(20,23) - HK29*P(5,20) + HK31*P(6,20) + HK32*P(2,20) + HK33*P(1,20) - HK34*P(3,20) + HK35*P(0,20));
Kfusion(21) = HK50*(HK27*P(21,22) - HK27*P(4,21) + HK29*P(21,23) - HK29*P(5,21) + HK31*P(6,21) + HK32*P(2,21) + HK33*P(1,21) - HK34*P(3,21) + HK35*P(0,21));
Kfusion(22) = HK43*HK50;
Kfusion(23) = HK40*HK50;


// Predicted observation


// Innovation variance


