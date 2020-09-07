// Sub Expressions
const float HK0 = vn - vwn;
const float HK1 = ve - vwe;
const float HK2 = HK0*q0 + HK1*q3 - q2*vd;
const float HK3 = 2*Kaccx;
const float HK4 = HK0*q1 + HK1*q2 + q3*vd;
const float HK5 = HK0*q2 - HK1*q1 + q0*vd;
const float HK6 = -HK0*q3 + HK1*q0 + q1*vd;
const float HK7 = powf(q1, 2);
const float HK8 = powf(q2, 2);
const float HK9 = powf(q0, 2) - powf(q3, 2);
const float HK10 = HK7 - HK8 + HK9;
const float HK11 = HK10*Kaccx;
const float HK12 = q0*q3;
const float HK13 = q1*q2;
const float HK14 = HK12 + HK13;
const float HK15 = HK14*HK3;
const float HK16 = q0*q2 - q1*q3;
const float HK17 = 2*HK14;
const float HK18 = 2*HK16;
const float HK19 = 2*HK4;
const float HK20 = 2*HK2;
const float HK21 = 2*HK5;
const float HK22 = 2*HK6;
const float HK23 = HK22*P(0,3);
const float HK24 = -HK10*P(0,22) + HK10*P(0,4) - HK17*P(0,23) + HK17*P(0,5) - HK18*P(0,6) + HK19*P(0,1) + HK20*P(0,0) - HK21*P(0,2) + HK23;
const float HK25 = HK17*P(5,23);
const float HK26 = -HK10*P(22,23) + HK10*P(4,23) - HK17*P(23,23) - HK18*P(6,23) + HK19*P(1,23) + HK20*P(0,23) - HK21*P(2,23) + HK22*P(3,23) + HK25;
const float HK27 = powf(Kaccx, 2);
const float HK28 = HK17*HK27;
const float HK29 = HK10*P(4,5) - HK10*P(5,22) + HK17*P(5,5) - HK18*P(5,6) + HK19*P(1,5) + HK20*P(0,5) - HK21*P(2,5) + HK22*P(3,5) - HK25;
const float HK30 = HK10*P(4,6) - HK10*P(6,22) + HK17*P(5,6) - HK17*P(6,23) - HK18*P(6,6) + HK19*P(1,6) + HK20*P(0,6) - HK21*P(2,6) + HK22*P(3,6);
const float HK31 = HK10*P(4,22);
const float HK32 = HK10*P(4,4) - HK17*P(4,23) + HK17*P(4,5) - HK18*P(4,6) + HK19*P(1,4) + HK20*P(0,4) - HK21*P(2,4) + HK22*P(3,4) - HK31;
const float HK33 = HK10*HK27;
const float HK34 = -HK10*P(22,22) - HK17*P(22,23) + HK17*P(5,22) - HK18*P(6,22) + HK19*P(1,22) + HK20*P(0,22) - HK21*P(2,22) + HK22*P(3,22) + HK31;
const float HK35 = HK21*P(1,2);
const float HK36 = -HK10*P(1,22) + HK10*P(1,4) - HK17*P(1,23) + HK17*P(1,5) - HK18*P(1,6) + HK19*P(1,1) + HK20*P(0,1) + HK22*P(1,3) - HK35;
const float HK37 = HK19*P(1,2);
const float HK38 = -HK10*P(2,22) + HK10*P(2,4) - HK17*P(2,23) + HK17*P(2,5) - HK18*P(2,6) + HK20*P(0,2) - HK21*P(2,2) + HK22*P(2,3) + HK37;
const float HK39 = HK20*P(0,3);
const float HK40 = -HK10*P(3,22) + HK10*P(3,4) - HK17*P(3,23) + HK17*P(3,5) - HK18*P(3,6) + HK19*P(1,3) - HK21*P(2,3) + HK22*P(3,3) + HK39;
const float HK41 = Kaccx/(-HK18*HK27*HK30 + HK19*HK27*HK36 + HK20*HK24*HK27 - HK21*HK27*HK38 + HK22*HK27*HK40 - HK26*HK28 + HK28*HK29 + HK32*HK33 - HK33*HK34 + R_ACC);
const float HK42 = HK12 - HK13;
const float HK43 = 2*Kaccy;
const float HK44 = HK42*HK43;
const float HK45 = -HK7 + HK8 + HK9;
const float HK46 = HK45*Kaccy;
const float HK47 = q0*q1 + q2*q3;
const float HK48 = 2*HK47;
const float HK49 = 2*HK42;
const float HK50 = HK19*P(0,2) + HK21*P(0,1) + HK22*P(0,0) - HK39 - HK45*P(0,23) + HK45*P(0,5) + HK48*P(0,6) + HK49*P(0,22) - HK49*P(0,4);
const float HK51 = powf(Kaccy, 2);
const float HK52 = HK19*P(2,6) - HK20*P(3,6) + HK21*P(1,6) + HK22*P(0,6) + HK45*P(5,6) - HK45*P(6,23) + HK48*P(6,6) - HK49*P(4,6) + HK49*P(6,22);
const float HK53 = HK49*P(4,22);
const float HK54 = HK19*P(2,22) - HK20*P(3,22) + HK21*P(1,22) + HK22*P(0,22) - HK45*P(22,23) + HK45*P(5,22) + HK48*P(6,22) + HK49*P(22,22) - HK53;
const float HK55 = HK49*HK51;
const float HK56 = HK19*P(2,4) - HK20*P(3,4) + HK21*P(1,4) + HK22*P(0,4) - HK45*P(4,23) + HK45*P(4,5) + HK48*P(4,6) - HK49*P(4,4) + HK53;
const float HK57 = HK45*P(5,23);
const float HK58 = HK19*P(2,5) - HK20*P(3,5) + HK21*P(1,5) + HK22*P(0,5) + HK45*P(5,5) + HK48*P(5,6) - HK49*P(4,5) + HK49*P(5,22) - HK57;
const float HK59 = HK45*HK51;
const float HK60 = HK19*P(2,23) - HK20*P(3,23) + HK21*P(1,23) + HK22*P(0,23) - HK45*P(23,23) + HK48*P(6,23) + HK49*P(22,23) - HK49*P(4,23) + HK57;
const float HK61 = HK19*P(2,2) - HK20*P(2,3) + HK22*P(0,2) + HK35 - HK45*P(2,23) + HK45*P(2,5) + HK48*P(2,6) + HK49*P(2,22) - HK49*P(2,4);
const float HK62 = -HK20*P(1,3) + HK21*P(1,1) + HK22*P(0,1) + HK37 - HK45*P(1,23) + HK45*P(1,5) + HK48*P(1,6) + HK49*P(1,22) - HK49*P(1,4);
const float HK63 = HK19*P(2,3) - HK20*P(3,3) + HK21*P(1,3) + HK23 - HK45*P(3,23) + HK45*P(3,5) + HK48*P(3,6) + HK49*P(3,22) - HK49*P(3,4);
const float HK64 = Kaccy/(HK19*HK51*HK61 - HK20*HK51*HK63 + HK21*HK51*HK62 + HK22*HK50*HK51 + HK48*HK51*HK52 + HK54*HK55 - HK55*HK56 + HK58*HK59 - HK59*HK60 + R_ACC);


// Observation Jacobians - axis 0
Hfusion.at<0>() = -HK2*HK3;
Hfusion.at<1>() = -HK3*HK4;
Hfusion.at<2>() = HK3*HK5;
Hfusion.at<3>() = -HK3*HK6;
Hfusion.at<4>() = -HK11;
Hfusion.at<5>() = -HK15;
Hfusion.at<6>() = HK16*HK3;
Hfusion.at<7>() = 0;
Hfusion.at<8>() = 0;
Hfusion.at<9>() = 0;
Hfusion.at<10>() = 0;
Hfusion.at<11>() = 0;
Hfusion.at<12>() = 0;
Hfusion.at<13>() = 0;
Hfusion.at<14>() = 0;
Hfusion.at<15>() = 0;
Hfusion.at<16>() = 0;
Hfusion.at<17>() = 0;
Hfusion.at<18>() = 0;
Hfusion.at<19>() = 0;
Hfusion.at<20>() = 0;
Hfusion.at<21>() = 0;
Hfusion.at<22>() = HK11;
Hfusion.at<23>() = HK15;


// Kalman gains - axis 0
Kfusion(0) = -HK24*HK41;
Kfusion(1) = -HK36*HK41;
Kfusion(2) = -HK38*HK41;
Kfusion(3) = -HK40*HK41;
Kfusion(4) = -HK32*HK41;
Kfusion(5) = -HK29*HK41;
Kfusion(6) = -HK30*HK41;
Kfusion(7) = -HK41*(HK10*P(4,7) - HK10*P(7,22) + HK17*P(5,7) - HK17*P(7,23) - HK18*P(6,7) + HK19*P(1,7) + HK20*P(0,7) - HK21*P(2,7) + HK22*P(3,7));
Kfusion(8) = -HK41*(HK10*P(4,8) - HK10*P(8,22) + HK17*P(5,8) - HK17*P(8,23) - HK18*P(6,8) + HK19*P(1,8) + HK20*P(0,8) - HK21*P(2,8) + HK22*P(3,8));
Kfusion(9) = -HK41*(HK10*P(4,9) - HK10*P(9,22) + HK17*P(5,9) - HK17*P(9,23) - HK18*P(6,9) + HK19*P(1,9) + HK20*P(0,9) - HK21*P(2,9) + HK22*P(3,9));
Kfusion(10) = -HK41*(-HK10*P(10,22) + HK10*P(4,10) - HK17*P(10,23) + HK17*P(5,10) - HK18*P(6,10) + HK19*P(1,10) + HK20*P(0,10) - HK21*P(2,10) + HK22*P(3,10));
Kfusion(11) = -HK41*(-HK10*P(11,22) + HK10*P(4,11) - HK17*P(11,23) + HK17*P(5,11) - HK18*P(6,11) + HK19*P(1,11) + HK20*P(0,11) - HK21*P(2,11) + HK22*P(3,11));
Kfusion(12) = -HK41*(-HK10*P(12,22) + HK10*P(4,12) - HK17*P(12,23) + HK17*P(5,12) - HK18*P(6,12) + HK19*P(1,12) + HK20*P(0,12) - HK21*P(2,12) + HK22*P(3,12));
Kfusion(13) = -HK41*(-HK10*P(13,22) + HK10*P(4,13) - HK17*P(13,23) + HK17*P(5,13) - HK18*P(6,13) + HK19*P(1,13) + HK20*P(0,13) - HK21*P(2,13) + HK22*P(3,13));
Kfusion(14) = -HK41*(-HK10*P(14,22) + HK10*P(4,14) - HK17*P(14,23) + HK17*P(5,14) - HK18*P(6,14) + HK19*P(1,14) + HK20*P(0,14) - HK21*P(2,14) + HK22*P(3,14));
Kfusion(15) = -HK41*(-HK10*P(15,22) + HK10*P(4,15) - HK17*P(15,23) + HK17*P(5,15) - HK18*P(6,15) + HK19*P(1,15) + HK20*P(0,15) - HK21*P(2,15) + HK22*P(3,15));
Kfusion(16) = -HK41*(-HK10*P(16,22) + HK10*P(4,16) - HK17*P(16,23) + HK17*P(5,16) - HK18*P(6,16) + HK19*P(1,16) + HK20*P(0,16) - HK21*P(2,16) + HK22*P(3,16));
Kfusion(17) = -HK41*(-HK10*P(17,22) + HK10*P(4,17) - HK17*P(17,23) + HK17*P(5,17) - HK18*P(6,17) + HK19*P(1,17) + HK20*P(0,17) - HK21*P(2,17) + HK22*P(3,17));
Kfusion(18) = -HK41*(-HK10*P(18,22) + HK10*P(4,18) - HK17*P(18,23) + HK17*P(5,18) - HK18*P(6,18) + HK19*P(1,18) + HK20*P(0,18) - HK21*P(2,18) + HK22*P(3,18));
Kfusion(19) = -HK41*(-HK10*P(19,22) + HK10*P(4,19) - HK17*P(19,23) + HK17*P(5,19) - HK18*P(6,19) + HK19*P(1,19) + HK20*P(0,19) - HK21*P(2,19) + HK22*P(3,19));
Kfusion(20) = -HK41*(-HK10*P(20,22) + HK10*P(4,20) - HK17*P(20,23) + HK17*P(5,20) - HK18*P(6,20) + HK19*P(1,20) + HK20*P(0,20) - HK21*P(2,20) + HK22*P(3,20));
Kfusion(21) = -HK41*(-HK10*P(21,22) + HK10*P(4,21) - HK17*P(21,23) + HK17*P(5,21) - HK18*P(6,21) + HK19*P(1,21) + HK20*P(0,21) - HK21*P(2,21) + HK22*P(3,21));
Kfusion(22) = -HK34*HK41;
Kfusion(23) = -HK26*HK41;


// Observation Jacobians - axis 1
Hfusion.at<0>() = -HK22*Kaccy;
Hfusion.at<1>() = -HK21*Kaccy;
Hfusion.at<2>() = -HK19*Kaccy;
Hfusion.at<3>() = HK20*Kaccy;
Hfusion.at<4>() = HK44;
Hfusion.at<5>() = -HK46;
Hfusion.at<6>() = -HK43*HK47;
Hfusion.at<7>() = 0;
Hfusion.at<8>() = 0;
Hfusion.at<9>() = 0;
Hfusion.at<10>() = 0;
Hfusion.at<11>() = 0;
Hfusion.at<12>() = 0;
Hfusion.at<13>() = 0;
Hfusion.at<14>() = 0;
Hfusion.at<15>() = 0;
Hfusion.at<16>() = 0;
Hfusion.at<17>() = 0;
Hfusion.at<18>() = 0;
Hfusion.at<19>() = 0;
Hfusion.at<20>() = 0;
Hfusion.at<21>() = 0;
Hfusion.at<22>() = -HK44;
Hfusion.at<23>() = HK46;


// Kalman gains - axis 1
Kfusion(0) = -HK50*HK64;
Kfusion(1) = -HK62*HK64;
Kfusion(2) = -HK61*HK64;
Kfusion(3) = -HK63*HK64;
Kfusion(4) = -HK56*HK64;
Kfusion(5) = -HK58*HK64;
Kfusion(6) = -HK52*HK64;
Kfusion(7) = -HK64*(HK19*P(2,7) - HK20*P(3,7) + HK21*P(1,7) + HK22*P(0,7) + HK45*P(5,7) - HK45*P(7,23) + HK48*P(6,7) - HK49*P(4,7) + HK49*P(7,22));
Kfusion(8) = -HK64*(HK19*P(2,8) - HK20*P(3,8) + HK21*P(1,8) + HK22*P(0,8) + HK45*P(5,8) - HK45*P(8,23) + HK48*P(6,8) - HK49*P(4,8) + HK49*P(8,22));
Kfusion(9) = -HK64*(HK19*P(2,9) - HK20*P(3,9) + HK21*P(1,9) + HK22*P(0,9) + HK45*P(5,9) - HK45*P(9,23) + HK48*P(6,9) - HK49*P(4,9) + HK49*P(9,22));
Kfusion(10) = -HK64*(HK19*P(2,10) - HK20*P(3,10) + HK21*P(1,10) + HK22*P(0,10) - HK45*P(10,23) + HK45*P(5,10) + HK48*P(6,10) + HK49*P(10,22) - HK49*P(4,10));
Kfusion(11) = -HK64*(HK19*P(2,11) - HK20*P(3,11) + HK21*P(1,11) + HK22*P(0,11) - HK45*P(11,23) + HK45*P(5,11) + HK48*P(6,11) + HK49*P(11,22) - HK49*P(4,11));
Kfusion(12) = -HK64*(HK19*P(2,12) - HK20*P(3,12) + HK21*P(1,12) + HK22*P(0,12) - HK45*P(12,23) + HK45*P(5,12) + HK48*P(6,12) + HK49*P(12,22) - HK49*P(4,12));
Kfusion(13) = -HK64*(HK19*P(2,13) - HK20*P(3,13) + HK21*P(1,13) + HK22*P(0,13) - HK45*P(13,23) + HK45*P(5,13) + HK48*P(6,13) + HK49*P(13,22) - HK49*P(4,13));
Kfusion(14) = -HK64*(HK19*P(2,14) - HK20*P(3,14) + HK21*P(1,14) + HK22*P(0,14) - HK45*P(14,23) + HK45*P(5,14) + HK48*P(6,14) + HK49*P(14,22) - HK49*P(4,14));
Kfusion(15) = -HK64*(HK19*P(2,15) - HK20*P(3,15) + HK21*P(1,15) + HK22*P(0,15) - HK45*P(15,23) + HK45*P(5,15) + HK48*P(6,15) + HK49*P(15,22) - HK49*P(4,15));
Kfusion(16) = -HK64*(HK19*P(2,16) - HK20*P(3,16) + HK21*P(1,16) + HK22*P(0,16) - HK45*P(16,23) + HK45*P(5,16) + HK48*P(6,16) + HK49*P(16,22) - HK49*P(4,16));
Kfusion(17) = -HK64*(HK19*P(2,17) - HK20*P(3,17) + HK21*P(1,17) + HK22*P(0,17) - HK45*P(17,23) + HK45*P(5,17) + HK48*P(6,17) + HK49*P(17,22) - HK49*P(4,17));
Kfusion(18) = -HK64*(HK19*P(2,18) - HK20*P(3,18) + HK21*P(1,18) + HK22*P(0,18) - HK45*P(18,23) + HK45*P(5,18) + HK48*P(6,18) + HK49*P(18,22) - HK49*P(4,18));
Kfusion(19) = -HK64*(HK19*P(2,19) - HK20*P(3,19) + HK21*P(1,19) + HK22*P(0,19) - HK45*P(19,23) + HK45*P(5,19) + HK48*P(6,19) + HK49*P(19,22) - HK49*P(4,19));
Kfusion(20) = -HK64*(HK19*P(2,20) - HK20*P(3,20) + HK21*P(1,20) + HK22*P(0,20) - HK45*P(20,23) + HK45*P(5,20) + HK48*P(6,20) + HK49*P(20,22) - HK49*P(4,20));
Kfusion(21) = -HK64*(HK19*P(2,21) - HK20*P(3,21) + HK21*P(1,21) + HK22*P(0,21) - HK45*P(21,23) + HK45*P(5,21) + HK48*P(6,21) + HK49*P(21,22) - HK49*P(4,21));
Kfusion(22) = -HK54*HK64;
Kfusion(23) = -HK60*HK64;


// Observation Jacobians - axis 2


// Kalman gains - axis 2


