// X Axis Equations
// Sub Expressions
const float HK0 = -Tbs(1,0)*q2 + Tbs(1,1)*q1 + Tbs(1,2)*q0;
const float HK1 = Tbs(1,0)*q3 + Tbs(1,1)*q0 - Tbs(1,2)*q1;
const float HK2 = Tbs(1,0)*q0 - Tbs(1,1)*q3 + Tbs(1,2)*q2;
const float HK3 = HK0*vd + HK1*ve + HK2*vn;
const float HK4 = 1.0F/(range);
const float HK5 = 2*HK4;
const float HK6 = Tbs(1,0)*q1 + Tbs(1,1)*q2 + Tbs(1,2)*q3;
const float HK7 = -HK0*ve + HK1*vd + HK6*vn;
const float HK8 = HK0*vn - HK2*vd + HK6*ve;
const float HK9 = -HK1*vn + HK2*ve + HK6*vd;
const float HK10 = q0*q2;
const float HK11 = q1*q3;
const float HK12 = 2*Tbs(1,2);
const float HK13 = q0*q3;
const float HK14 = q1*q2;
const float HK15 = 2*Tbs(1,1);
const float HK16 = (q1)*(q1);
const float HK17 = (q2)*(q2);
const float HK18 = -HK17;
const float HK19 = (q0)*(q0);
const float HK20 = (q3)*(q3);
const float HK21 = HK19 - HK20;
const float HK22 = HK12*(HK10 + HK11) - HK15*(HK13 - HK14) + Tbs(1,0)*(HK16 + HK18 + HK21);
const float HK23 = 2*Tbs(1,0);
const float HK24 = q0*q1;
const float HK25 = q2*q3;
const float HK26 = -HK16;
const float HK27 = -HK12*(HK24 - HK25) + HK23*(HK13 + HK14) + Tbs(1,1)*(HK17 + HK21 + HK26);
const float HK28 = HK15*(HK24 + HK25) - HK23*(HK10 - HK11) + Tbs(1,2)*(HK18 + HK19 + HK20 + HK26);
const float HK29 = 2*HK3;
const float HK30 = 2*HK7;
const float HK31 = 2*HK8;
const float HK32 = 2*HK9;
const float HK33 = HK22*P(0,4) + HK27*P(0,5) + HK28*P(0,6) + HK29*P(0,0) + HK30*P(0,1) + HK31*P(0,2) + HK32*P(0,3);
const float HK34 = 1.0F/((range)*(range));
const float HK35 = HK22*P(4,6) + HK27*P(5,6) + HK28*P(6,6) + HK29*P(0,6) + HK30*P(1,6) + HK31*P(2,6) + HK32*P(3,6);
const float HK36 = HK22*P(4,5) + HK27*P(5,5) + HK28*P(5,6) + HK29*P(0,5) + HK30*P(1,5) + HK31*P(2,5) + HK32*P(3,5);
const float HK37 = HK22*P(4,4) + HK27*P(4,5) + HK28*P(4,6) + HK29*P(0,4) + HK30*P(1,4) + HK31*P(2,4) + HK32*P(3,4);
const float HK38 = HK22*P(2,4) + HK27*P(2,5) + HK28*P(2,6) + HK29*P(0,2) + HK30*P(1,2) + HK31*P(2,2) + HK32*P(2,3);
const float HK39 = HK22*P(3,4) + HK27*P(3,5) + HK28*P(3,6) + HK29*P(0,3) + HK30*P(1,3) + HK31*P(2,3) + HK32*P(3,3);
const float HK40 = HK22*P(1,4) + HK27*P(1,5) + HK28*P(1,6) + HK29*P(0,1) + HK30*P(1,1) + HK31*P(1,2) + HK32*P(1,3);
const float HK41 = HK4/(HK22*HK34*HK37 + HK27*HK34*HK36 + HK28*HK34*HK35 + HK29*HK33*HK34 + HK30*HK34*HK40 + HK31*HK34*HK38 + HK32*HK34*HK39 + R_LOS);


// Observation Jacobians
Hfusion.at<0>() = HK3*HK5;
Hfusion.at<1>() = HK5*HK7;
Hfusion.at<2>() = HK5*HK8;
Hfusion.at<3>() = HK5*HK9;
Hfusion.at<4>() = HK22*HK4;
Hfusion.at<5>() = HK27*HK4;
Hfusion.at<6>() = HK28*HK4;


// Kalman gains
Kfusion(0) = HK33*HK41;
Kfusion(1) = HK40*HK41;
Kfusion(2) = HK38*HK41;
Kfusion(3) = HK39*HK41;
Kfusion(4) = HK37*HK41;
Kfusion(5) = HK36*HK41;
Kfusion(6) = HK35*HK41;
Kfusion(7) = HK41*(HK22*P(4,7) + HK27*P(5,7) + HK28*P(6,7) + HK29*P(0,7) + HK30*P(1,7) + HK31*P(2,7) + HK32*P(3,7));
Kfusion(8) = HK41*(HK22*P(4,8) + HK27*P(5,8) + HK28*P(6,8) + HK29*P(0,8) + HK30*P(1,8) + HK31*P(2,8) + HK32*P(3,8));
Kfusion(9) = HK41*(HK22*P(4,9) + HK27*P(5,9) + HK28*P(6,9) + HK29*P(0,9) + HK30*P(1,9) + HK31*P(2,9) + HK32*P(3,9));
Kfusion(10) = HK41*(HK22*P(4,10) + HK27*P(5,10) + HK28*P(6,10) + HK29*P(0,10) + HK30*P(1,10) + HK31*P(2,10) + HK32*P(3,10));
Kfusion(11) = HK41*(HK22*P(4,11) + HK27*P(5,11) + HK28*P(6,11) + HK29*P(0,11) + HK30*P(1,11) + HK31*P(2,11) + HK32*P(3,11));
Kfusion(12) = HK41*(HK22*P(4,12) + HK27*P(5,12) + HK28*P(6,12) + HK29*P(0,12) + HK30*P(1,12) + HK31*P(2,12) + HK32*P(3,12));
Kfusion(13) = HK41*(HK22*P(4,13) + HK27*P(5,13) + HK28*P(6,13) + HK29*P(0,13) + HK30*P(1,13) + HK31*P(2,13) + HK32*P(3,13));
Kfusion(14) = HK41*(HK22*P(4,14) + HK27*P(5,14) + HK28*P(6,14) + HK29*P(0,14) + HK30*P(1,14) + HK31*P(2,14) + HK32*P(3,14));
Kfusion(15) = HK41*(HK22*P(4,15) + HK27*P(5,15) + HK28*P(6,15) + HK29*P(0,15) + HK30*P(1,15) + HK31*P(2,15) + HK32*P(3,15));
Kfusion(16) = HK41*(HK22*P(4,16) + HK27*P(5,16) + HK28*P(6,16) + HK29*P(0,16) + HK30*P(1,16) + HK31*P(2,16) + HK32*P(3,16));
Kfusion(17) = HK41*(HK22*P(4,17) + HK27*P(5,17) + HK28*P(6,17) + HK29*P(0,17) + HK30*P(1,17) + HK31*P(2,17) + HK32*P(3,17));
Kfusion(18) = HK41*(HK22*P(4,18) + HK27*P(5,18) + HK28*P(6,18) + HK29*P(0,18) + HK30*P(1,18) + HK31*P(2,18) + HK32*P(3,18));
Kfusion(19) = HK41*(HK22*P(4,19) + HK27*P(5,19) + HK28*P(6,19) + HK29*P(0,19) + HK30*P(1,19) + HK31*P(2,19) + HK32*P(3,19));
Kfusion(20) = HK41*(HK22*P(4,20) + HK27*P(5,20) + HK28*P(6,20) + HK29*P(0,20) + HK30*P(1,20) + HK31*P(2,20) + HK32*P(3,20));
Kfusion(21) = HK41*(HK22*P(4,21) + HK27*P(5,21) + HK28*P(6,21) + HK29*P(0,21) + HK30*P(1,21) + HK31*P(2,21) + HK32*P(3,21));
Kfusion(22) = HK41*(HK22*P(4,22) + HK27*P(5,22) + HK28*P(6,22) + HK29*P(0,22) + HK30*P(1,22) + HK31*P(2,22) + HK32*P(3,22));
Kfusion(23) = HK41*(HK22*P(4,23) + HK27*P(5,23) + HK28*P(6,23) + HK29*P(0,23) + HK30*P(1,23) + HK31*P(2,23) + HK32*P(3,23));


// Predicted observation


// Innovation variance


// Y Axis Equations
// Sub Expressions
const float HK0 = -Tbs(0,0)*q2 + Tbs(0,1)*q1 + Tbs(0,2)*q0;
const float HK1 = Tbs(0,0)*q3 + Tbs(0,1)*q0 - Tbs(0,2)*q1;
const float HK2 = Tbs(0,0)*q0 - Tbs(0,1)*q3 + Tbs(0,2)*q2;
const float HK3 = HK0*vd + HK1*ve + HK2*vn;
const float HK4 = 1.0F/(range);
const float HK5 = 2*HK4;
const float HK6 = Tbs(0,0)*q1 + Tbs(0,1)*q2 + Tbs(0,2)*q3;
const float HK7 = HK1*vd + HK6*vn;
const float HK8 = HK0*vn + HK6*ve;
const float HK9 = HK2*ve + HK6*vd;
const float HK10 = q0*q3;
const float HK11 = q1*q2;
const float HK12 = HK10 - HK11;
const float HK13 = 2*Tbs(0,1);
const float HK14 = (q1)*(q1);
const float HK15 = (q2)*(q2);
const float HK16 = -HK15;
const float HK17 = (q0)*(q0);
const float HK18 = (q3)*(q3);
const float HK19 = HK17 - HK18;
const float HK20 = q0*q2;
const float HK21 = q1*q3;
const float HK22 = 2*Tbs(0,2);
const float HK23 = HK22*(HK20 + HK21) + Tbs(0,0)*(HK14 + HK16 + HK19);
const float HK24 = q0*q1;
const float HK25 = q2*q3;
const float HK26 = HK24 - HK25;
const float HK27 = -HK14;
const float HK28 = 2*Tbs(0,0);
const float HK29 = HK28*(HK10 + HK11) + Tbs(0,1)*(HK15 + HK19 + HK27);
const float HK30 = HK20 - HK21;
const float HK31 = HK13*(HK24 + HK25) + Tbs(0,2)*(HK16 + HK17 + HK18 + HK27);
const float HK32 = 2*HK3;
const float HK33 = -2*HK0*ve + 2*HK7;
const float HK34 = -2*HK2*vd + 2*HK8;
const float HK35 = -2*HK1*vn + 2*HK9;
const float HK36 = -HK12*HK13 + HK23;
const float HK37 = -HK22*HK26 + HK29;
const float HK38 = -HK28*HK30 + HK31;
const float HK39 = HK32*P(0,0) + HK33*P(0,1) + HK34*P(0,2) + HK35*P(0,3) + HK36*P(0,4) + HK37*P(0,5) + HK38*P(0,6);
const float HK40 = 1.0F/((range)*(range));
const float HK41 = HK32*P(0,6) + HK33*P(1,6) + HK34*P(2,6) + HK35*P(3,6) + HK36*P(4,6) + HK37*P(5,6) + HK38*P(6,6);
const float HK42 = HK32*P(0,5) + HK33*P(1,5) + HK34*P(2,5) + HK35*P(3,5) + HK36*P(4,5) + HK37*P(5,5) + HK38*P(5,6);
const float HK43 = HK32*P(0,4) + HK33*P(1,4) + HK34*P(2,4) + HK35*P(3,4) + HK36*P(4,4) + HK37*P(4,5) + HK38*P(4,6);
const float HK44 = HK32*P(0,2) + HK33*P(1,2) + HK34*P(2,2) + HK35*P(2,3) + HK36*P(2,4) + HK37*P(2,5) + HK38*P(2,6);
const float HK45 = HK32*P(0,3) + HK33*P(1,3) + HK34*P(2,3) + HK35*P(3,3) + HK36*P(3,4) + HK37*P(3,5) + HK38*P(3,6);
const float HK46 = HK32*P(0,1) + HK33*P(1,1) + HK34*P(1,2) + HK35*P(1,3) + HK36*P(1,4) + HK37*P(1,5) + HK38*P(1,6);
const float HK47 = HK4/(HK32*HK39*HK40 + HK33*HK40*HK46 + HK34*HK40*HK44 + HK35*HK40*HK45 + HK36*HK40*HK43 + HK37*HK40*HK42 + HK38*HK40*HK41 + R_LOS);


// Observation Jacobians
Hfusion.at<0>() = -HK3*HK5;
Hfusion.at<1>() = -HK5*(-HK0*ve + HK7);
Hfusion.at<2>() = -HK5*(-HK2*vd + HK8);
Hfusion.at<3>() = -HK5*(-HK1*vn + HK9);
Hfusion.at<4>() = -HK4*(-HK12*HK13 + HK23);
Hfusion.at<5>() = -HK4*(-HK22*HK26 + HK29);
Hfusion.at<6>() = -HK4*(-HK28*HK30 + HK31);


// Kalman gains
Kfusion(0) = -HK39*HK47;
Kfusion(1) = -HK46*HK47;
Kfusion(2) = -HK44*HK47;
Kfusion(3) = -HK45*HK47;
Kfusion(4) = -HK43*HK47;
Kfusion(5) = -HK42*HK47;
Kfusion(6) = -HK41*HK47;
Kfusion(7) = -HK47*(HK32*P(0,7) + HK33*P(1,7) + HK34*P(2,7) + HK35*P(3,7) + HK36*P(4,7) + HK37*P(5,7) + HK38*P(6,7));
Kfusion(8) = -HK47*(HK32*P(0,8) + HK33*P(1,8) + HK34*P(2,8) + HK35*P(3,8) + HK36*P(4,8) + HK37*P(5,8) + HK38*P(6,8));
Kfusion(9) = -HK47*(HK32*P(0,9) + HK33*P(1,9) + HK34*P(2,9) + HK35*P(3,9) + HK36*P(4,9) + HK37*P(5,9) + HK38*P(6,9));
Kfusion(10) = -HK47*(HK32*P(0,10) + HK33*P(1,10) + HK34*P(2,10) + HK35*P(3,10) + HK36*P(4,10) + HK37*P(5,10) + HK38*P(6,10));
Kfusion(11) = -HK47*(HK32*P(0,11) + HK33*P(1,11) + HK34*P(2,11) + HK35*P(3,11) + HK36*P(4,11) + HK37*P(5,11) + HK38*P(6,11));
Kfusion(12) = -HK47*(HK32*P(0,12) + HK33*P(1,12) + HK34*P(2,12) + HK35*P(3,12) + HK36*P(4,12) + HK37*P(5,12) + HK38*P(6,12));
Kfusion(13) = -HK47*(HK32*P(0,13) + HK33*P(1,13) + HK34*P(2,13) + HK35*P(3,13) + HK36*P(4,13) + HK37*P(5,13) + HK38*P(6,13));
Kfusion(14) = -HK47*(HK32*P(0,14) + HK33*P(1,14) + HK34*P(2,14) + HK35*P(3,14) + HK36*P(4,14) + HK37*P(5,14) + HK38*P(6,14));
Kfusion(15) = -HK47*(HK32*P(0,15) + HK33*P(1,15) + HK34*P(2,15) + HK35*P(3,15) + HK36*P(4,15) + HK37*P(5,15) + HK38*P(6,15));
Kfusion(16) = -HK47*(HK32*P(0,16) + HK33*P(1,16) + HK34*P(2,16) + HK35*P(3,16) + HK36*P(4,16) + HK37*P(5,16) + HK38*P(6,16));
Kfusion(17) = -HK47*(HK32*P(0,17) + HK33*P(1,17) + HK34*P(2,17) + HK35*P(3,17) + HK36*P(4,17) + HK37*P(5,17) + HK38*P(6,17));
Kfusion(18) = -HK47*(HK32*P(0,18) + HK33*P(1,18) + HK34*P(2,18) + HK35*P(3,18) + HK36*P(4,18) + HK37*P(5,18) + HK38*P(6,18));
Kfusion(19) = -HK47*(HK32*P(0,19) + HK33*P(1,19) + HK34*P(2,19) + HK35*P(3,19) + HK36*P(4,19) + HK37*P(5,19) + HK38*P(6,19));
Kfusion(20) = -HK47*(HK32*P(0,20) + HK33*P(1,20) + HK34*P(2,20) + HK35*P(3,20) + HK36*P(4,20) + HK37*P(5,20) + HK38*P(6,20));
Kfusion(21) = -HK47*(HK32*P(0,21) + HK33*P(1,21) + HK34*P(2,21) + HK35*P(3,21) + HK36*P(4,21) + HK37*P(5,21) + HK38*P(6,21));
Kfusion(22) = -HK47*(HK32*P(0,22) + HK33*P(1,22) + HK34*P(2,22) + HK35*P(3,22) + HK36*P(4,22) + HK37*P(5,22) + HK38*P(6,22));
Kfusion(23) = -HK47*(HK32*P(0,23) + HK33*P(1,23) + HK34*P(2,23) + HK35*P(3,23) + HK36*P(4,23) + HK37*P(5,23) + HK38*P(6,23));


// Predicted observation


// Innovation variance


