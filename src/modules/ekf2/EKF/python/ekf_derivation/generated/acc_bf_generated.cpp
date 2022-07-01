// Axis 0 equations
// Sub Expressions
const float HK0 = vn - vwn;
const float HK1 = ve - vwe;
const float HK2 = HK0*q0 + HK1*q3 - q2*vd;
const float HK3 = 2*Kaccx;
const float HK4 = HK0*q1 + HK1*q2 + q3*vd;
const float HK5 = HK0*q2 - HK1*q1 + q0*vd;
const float HK6 = -HK0*q3 + HK1*q0 + q1*vd;
const float HK7 = (q0)*(q0) + (q1)*(q1) - (q2)*(q2) - (q3)*(q3);
const float HK8 = HK7*Kaccx;
const float HK9 = q0*q3 + q1*q2;
const float HK10 = HK3*HK9;
const float HK11 = q0*q2 - q1*q3;
const float HK12 = 2*HK5;
const float HK13 = 2*HK11;
const float HK14 = 2*HK9;
const float HK15 = 2*HK2;
const float HK16 = 2*HK4;
const float HK17 = 2*HK6;
const float HK18 = -HK14*P(0,23) + HK14*P(0,5) + HK15*P(0,0) + HK16*P(0,1) + HK17*P(0,3) + HK7*P(0,4);
const float HK19 = (Kaccx)*(Kaccx);
const float HK20 = -HK7;
const float HK21 = -2*HK5;
const float HK22 = -2*HK11;
const float HK23 = HK14*P(5,23);
const float HK24 = -HK14*P(23,23) + HK15*P(0,23) + HK16*P(1,23) + HK17*P(3,23) + HK23 + HK7*P(4,23);
const float HK25 = HK14*P(5,5) + HK15*P(0,5) + HK16*P(1,5) + HK17*P(3,5) - HK23 + HK7*P(4,5);
const float HK26 = HK14*P(5,6) - HK14*P(6,23) + HK15*P(0,6) + HK16*P(1,6) + HK17*P(3,6) + HK7*P(4,6);
const float HK27 = -HK14*P(4,23) + HK14*P(4,5) + HK15*P(0,4) + HK16*P(1,4) + HK17*P(3,4) + HK7*P(4,4);
const float HK28 = HK7*P(4,22);
const float HK29 = -HK14*P(22,23) + HK14*P(5,22) + HK15*P(0,22) + HK16*P(1,22) + HK17*P(3,22) + HK28;
const float HK30 = -HK14*P(1,23) + HK14*P(1,5) + HK15*P(0,1) + HK16*P(1,1) + HK17*P(1,3) + HK7*P(1,4);
const float HK31 = -HK14*P(2,23) + HK14*P(2,5) + HK15*P(0,2) + HK16*P(1,2) + HK17*P(2,3) + HK7*P(2,4);
const float HK32 = -HK14*P(3,23) + HK14*P(3,5) + HK15*P(0,3) + HK16*P(1,3) + HK17*P(3,3) + HK7*P(3,4);
const float HK33 = Kaccx/(2*HK11*HK19*(HK20*P(6,22) + HK21*P(2,6) + HK22*P(6,6) + HK26) - HK14*HK19*(HK20*P(5,22) + HK21*P(2,5) + HK22*P(5,6) + HK25) - HK15*HK19*(HK18 + HK20*P(0,22) + HK21*P(0,2) + HK22*P(0,6)) - HK16*HK19*(HK20*P(1,22) + HK21*P(1,2) + HK22*P(1,6) + HK30) - HK17*HK19*(HK20*P(3,22) + HK21*P(2,3) + HK22*P(3,6) + HK32) + 2*HK19*HK5*(HK20*P(2,22) + HK21*P(2,2) + HK22*P(2,6) + HK31) + HK19*HK7*(HK20*P(22,22) + HK21*P(2,22) + HK22*P(6,22) + HK29) - HK19*HK7*(HK20*P(4,22) + HK21*P(2,4) + HK22*P(4,6) + HK27) + 2*HK19*HK9*(HK20*P(22,23) + HK21*P(2,23) + HK22*P(6,23) + HK24) - R_ACC);


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
Kfusion(0) = HK33*(-HK12*P(0,2) - HK13*P(0,6) + HK18 - HK7*P(0,22));
Kfusion(1) = HK33*(-HK12*P(1,2) - HK13*P(1,6) + HK30 - HK7*P(1,22));
Kfusion(2) = HK33*(-HK12*P(2,2) - HK13*P(2,6) + HK31 - HK7*P(2,22));
Kfusion(3) = HK33*(-HK12*P(2,3) - HK13*P(3,6) + HK32 - HK7*P(3,22));
Kfusion(4) = HK33*(-HK12*P(2,4) - HK13*P(4,6) + HK27 - HK28);
Kfusion(5) = HK33*(-HK12*P(2,5) - HK13*P(5,6) + HK25 - HK7*P(5,22));
Kfusion(6) = HK33*(-HK12*P(2,6) - HK13*P(6,6) + HK26 - HK7*P(6,22));
Kfusion(7) = HK33*(-HK12*P(2,7) - HK13*P(6,7) + HK14*P(5,7) - HK14*P(7,23) + HK15*P(0,7) + HK16*P(1,7) + HK17*P(3,7) + HK7*P(4,7) - HK7*P(7,22));
Kfusion(8) = HK33*(-HK12*P(2,8) - HK13*P(6,8) + HK14*P(5,8) - HK14*P(8,23) + HK15*P(0,8) + HK16*P(1,8) + HK17*P(3,8) + HK7*P(4,8) - HK7*P(8,22));
Kfusion(9) = HK33*(-HK12*P(2,9) - HK13*P(6,9) + HK14*P(5,9) - HK14*P(9,23) + HK15*P(0,9) + HK16*P(1,9) + HK17*P(3,9) + HK7*P(4,9) - HK7*P(9,22));
Kfusion(10) = HK33*(-HK12*P(2,10) - HK13*P(6,10) - HK14*P(10,23) + HK14*P(5,10) + HK15*P(0,10) + HK16*P(1,10) + HK17*P(3,10) - HK7*P(10,22) + HK7*P(4,10));
Kfusion(11) = HK33*(-HK12*P(2,11) - HK13*P(6,11) - HK14*P(11,23) + HK14*P(5,11) + HK15*P(0,11) + HK16*P(1,11) + HK17*P(3,11) - HK7*P(11,22) + HK7*P(4,11));
Kfusion(12) = HK33*(-HK12*P(2,12) - HK13*P(6,12) - HK14*P(12,23) + HK14*P(5,12) + HK15*P(0,12) + HK16*P(1,12) + HK17*P(3,12) - HK7*P(12,22) + HK7*P(4,12));
Kfusion(13) = HK33*(-HK12*P(2,13) - HK13*P(6,13) - HK14*P(13,23) + HK14*P(5,13) + HK15*P(0,13) + HK16*P(1,13) + HK17*P(3,13) - HK7*P(13,22) + HK7*P(4,13));
Kfusion(14) = HK33*(-HK12*P(2,14) - HK13*P(6,14) - HK14*P(14,23) + HK14*P(5,14) + HK15*P(0,14) + HK16*P(1,14) + HK17*P(3,14) - HK7*P(14,22) + HK7*P(4,14));
Kfusion(15) = HK33*(-HK12*P(2,15) - HK13*P(6,15) - HK14*P(15,23) + HK14*P(5,15) + HK15*P(0,15) + HK16*P(1,15) + HK17*P(3,15) - HK7*P(15,22) + HK7*P(4,15));
Kfusion(16) = HK33*(-HK12*P(2,16) - HK13*P(6,16) - HK14*P(16,23) + HK14*P(5,16) + HK15*P(0,16) + HK16*P(1,16) + HK17*P(3,16) - HK7*P(16,22) + HK7*P(4,16));
Kfusion(17) = HK33*(-HK12*P(2,17) - HK13*P(6,17) - HK14*P(17,23) + HK14*P(5,17) + HK15*P(0,17) + HK16*P(1,17) + HK17*P(3,17) - HK7*P(17,22) + HK7*P(4,17));
Kfusion(18) = HK33*(-HK12*P(2,18) - HK13*P(6,18) - HK14*P(18,23) + HK14*P(5,18) + HK15*P(0,18) + HK16*P(1,18) + HK17*P(3,18) - HK7*P(18,22) + HK7*P(4,18));
Kfusion(19) = HK33*(-HK12*P(2,19) - HK13*P(6,19) - HK14*P(19,23) + HK14*P(5,19) + HK15*P(0,19) + HK16*P(1,19) + HK17*P(3,19) - HK7*P(19,22) + HK7*P(4,19));
Kfusion(20) = HK33*(-HK12*P(2,20) - HK13*P(6,20) - HK14*P(20,23) + HK14*P(5,20) + HK15*P(0,20) + HK16*P(1,20) + HK17*P(3,20) - HK7*P(20,22) + HK7*P(4,20));
Kfusion(21) = HK33*(-HK12*P(2,21) - HK13*P(6,21) - HK14*P(21,23) + HK14*P(5,21) + HK15*P(0,21) + HK16*P(1,21) + HK17*P(3,21) - HK7*P(21,22) + HK7*P(4,21));
Kfusion(22) = HK33*(-HK12*P(2,22) - HK13*P(6,22) + HK29 - HK7*P(22,22));
Kfusion(23) = HK33*(-HK12*P(2,23) - HK13*P(6,23) + HK24 - HK7*P(22,23));


// Predicted observation


// Innovation variance


// Axis 1 equations
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
const float HK9 = (q0)*(q0) - (q1)*(q1) + (q2)*(q2) - (q3)*(q3);
const float HK10 = HK9*Kaccy;
const float HK11 = q0*q1 + q2*q3;
const float HK12 = 2*HK6;
const float HK13 = 2*HK7;
const float HK14 = 2*HK2;
const float HK15 = 2*HK4;
const float HK16 = 2*HK5;
const float HK17 = 2*HK11;
const float HK18 = HK13*P(0,22) + HK14*P(0,0) + HK15*P(0,1) + HK16*P(0,2) + HK17*P(0,6) + HK9*P(0,5);
const float HK19 = (Kaccy)*(Kaccy);
const float HK20 = -HK9;
const float HK21 = -2*HK6;
const float HK22 = -2*HK7;
const float HK23 = HK13*P(6,22) + HK14*P(0,6) + HK15*P(1,6) + HK16*P(2,6) + HK17*P(6,6) + HK9*P(5,6);
const float HK24 = HK13*P(22,22) + HK14*P(0,22) + HK15*P(1,22) + HK16*P(2,22) + HK17*P(6,22) + HK9*P(5,22);
const float HK25 = HK13*P(4,22);
const float HK26 = HK14*P(0,4) + HK15*P(1,4) + HK16*P(2,4) + HK17*P(4,6) + HK25 + HK9*P(4,5);
const float HK27 = HK13*P(5,22) + HK14*P(0,5) + HK15*P(1,5) + HK16*P(2,5) + HK17*P(5,6) + HK9*P(5,5);
const float HK28 = HK9*P(5,23);
const float HK29 = HK13*P(22,23) + HK14*P(0,23) + HK15*P(1,23) + HK16*P(2,23) + HK17*P(6,23) + HK28;
const float HK30 = HK13*P(2,22) + HK14*P(0,2) + HK15*P(1,2) + HK16*P(2,2) + HK17*P(2,6) + HK9*P(2,5);
const float HK31 = HK13*P(1,22) + HK14*P(0,1) + HK15*P(1,1) + HK16*P(1,2) + HK17*P(1,6) + HK9*P(1,5);
const float HK32 = HK13*P(3,22) + HK14*P(0,3) + HK15*P(1,3) + HK16*P(2,3) + HK17*P(3,6) + HK9*P(3,5);
const float HK33 = Kaccy/(-HK13*HK19*(HK20*P(22,23) + HK21*P(3,22) + HK22*P(4,22) + HK24) - HK14*HK19*(HK18 + HK20*P(0,23) + HK21*P(0,3) + HK22*P(0,4)) - HK15*HK19*(HK20*P(1,23) + HK21*P(1,3) + HK22*P(1,4) + HK31) - HK16*HK19*(HK20*P(2,23) + HK21*P(2,3) + HK22*P(2,4) + HK30) - HK17*HK19*(HK20*P(6,23) + HK21*P(3,6) + HK22*P(4,6) + HK23) + 2*HK19*HK6*(HK20*P(3,23) + HK21*P(3,3) + HK22*P(3,4) + HK32) + 2*HK19*HK7*(HK20*P(4,23) + HK21*P(3,4) + HK22*P(4,4) + HK26) + HK19*HK9*(HK20*P(23,23) + HK21*P(3,23) + HK22*P(4,23) + HK29) - HK19*HK9*(HK20*P(5,23) + HK21*P(3,5) + HK22*P(4,5) + HK27) - R_ACC);


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
Kfusion(0) = HK33*(-HK12*P(0,3) - HK13*P(0,4) + HK18 - HK9*P(0,23));
Kfusion(1) = HK33*(-HK12*P(1,3) - HK13*P(1,4) + HK31 - HK9*P(1,23));
Kfusion(2) = HK33*(-HK12*P(2,3) - HK13*P(2,4) + HK30 - HK9*P(2,23));
Kfusion(3) = HK33*(-HK12*P(3,3) - HK13*P(3,4) + HK32 - HK9*P(3,23));
Kfusion(4) = HK33*(-HK12*P(3,4) - HK13*P(4,4) + HK26 - HK9*P(4,23));
Kfusion(5) = HK33*(-HK12*P(3,5) - HK13*P(4,5) + HK27 - HK28);
Kfusion(6) = HK33*(-HK12*P(3,6) - HK13*P(4,6) + HK23 - HK9*P(6,23));
Kfusion(7) = HK33*(-HK12*P(3,7) - HK13*P(4,7) + HK13*P(7,22) + HK14*P(0,7) + HK15*P(1,7) + HK16*P(2,7) + HK17*P(6,7) + HK9*P(5,7) - HK9*P(7,23));
Kfusion(8) = HK33*(-HK12*P(3,8) - HK13*P(4,8) + HK13*P(8,22) + HK14*P(0,8) + HK15*P(1,8) + HK16*P(2,8) + HK17*P(6,8) + HK9*P(5,8) - HK9*P(8,23));
Kfusion(9) = HK33*(-HK12*P(3,9) - HK13*P(4,9) + HK13*P(9,22) + HK14*P(0,9) + HK15*P(1,9) + HK16*P(2,9) + HK17*P(6,9) + HK9*P(5,9) - HK9*P(9,23));
Kfusion(10) = HK33*(-HK12*P(3,10) + HK13*P(10,22) - HK13*P(4,10) + HK14*P(0,10) + HK15*P(1,10) + HK16*P(2,10) + HK17*P(6,10) - HK9*P(10,23) + HK9*P(5,10));
Kfusion(11) = HK33*(-HK12*P(3,11) + HK13*P(11,22) - HK13*P(4,11) + HK14*P(0,11) + HK15*P(1,11) + HK16*P(2,11) + HK17*P(6,11) - HK9*P(11,23) + HK9*P(5,11));
Kfusion(12) = HK33*(-HK12*P(3,12) + HK13*P(12,22) - HK13*P(4,12) + HK14*P(0,12) + HK15*P(1,12) + HK16*P(2,12) + HK17*P(6,12) - HK9*P(12,23) + HK9*P(5,12));
Kfusion(13) = HK33*(-HK12*P(3,13) + HK13*P(13,22) - HK13*P(4,13) + HK14*P(0,13) + HK15*P(1,13) + HK16*P(2,13) + HK17*P(6,13) - HK9*P(13,23) + HK9*P(5,13));
Kfusion(14) = HK33*(-HK12*P(3,14) + HK13*P(14,22) - HK13*P(4,14) + HK14*P(0,14) + HK15*P(1,14) + HK16*P(2,14) + HK17*P(6,14) - HK9*P(14,23) + HK9*P(5,14));
Kfusion(15) = HK33*(-HK12*P(3,15) + HK13*P(15,22) - HK13*P(4,15) + HK14*P(0,15) + HK15*P(1,15) + HK16*P(2,15) + HK17*P(6,15) - HK9*P(15,23) + HK9*P(5,15));
Kfusion(16) = HK33*(-HK12*P(3,16) + HK13*P(16,22) - HK13*P(4,16) + HK14*P(0,16) + HK15*P(1,16) + HK16*P(2,16) + HK17*P(6,16) - HK9*P(16,23) + HK9*P(5,16));
Kfusion(17) = HK33*(-HK12*P(3,17) + HK13*P(17,22) - HK13*P(4,17) + HK14*P(0,17) + HK15*P(1,17) + HK16*P(2,17) + HK17*P(6,17) - HK9*P(17,23) + HK9*P(5,17));
Kfusion(18) = HK33*(-HK12*P(3,18) + HK13*P(18,22) - HK13*P(4,18) + HK14*P(0,18) + HK15*P(1,18) + HK16*P(2,18) + HK17*P(6,18) - HK9*P(18,23) + HK9*P(5,18));
Kfusion(19) = HK33*(-HK12*P(3,19) + HK13*P(19,22) - HK13*P(4,19) + HK14*P(0,19) + HK15*P(1,19) + HK16*P(2,19) + HK17*P(6,19) - HK9*P(19,23) + HK9*P(5,19));
Kfusion(20) = HK33*(-HK12*P(3,20) + HK13*P(20,22) - HK13*P(4,20) + HK14*P(0,20) + HK15*P(1,20) + HK16*P(2,20) + HK17*P(6,20) - HK9*P(20,23) + HK9*P(5,20));
Kfusion(21) = HK33*(-HK12*P(3,21) + HK13*P(21,22) - HK13*P(4,21) + HK14*P(0,21) + HK15*P(1,21) + HK16*P(2,21) + HK17*P(6,21) - HK9*P(21,23) + HK9*P(5,21));
Kfusion(22) = HK33*(-HK12*P(3,22) + HK24 - HK25 - HK9*P(22,23));
Kfusion(23) = HK33*(-HK12*P(3,23) - HK13*P(4,23) + HK29 - HK9*P(23,23));


// Predicted observation


// Innovation variance


