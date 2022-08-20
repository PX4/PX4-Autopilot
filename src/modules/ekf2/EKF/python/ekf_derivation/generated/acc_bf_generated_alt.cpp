// Sub Expressions
const float HK0 = vn - vwn;
const float HK1 = ve - vwe;
const float HK2 = HK0*q0 + HK1*q3 - q2*vd;
const float HK3 = 2*Kaccx;
const float HK4 = HK0*q1 + HK1*q2 + q3*vd;
const float HK5 = HK0*q2 - HK1*q1 + q0*vd;
const float HK6 = -HK0*q3 + HK1*q0 + q1*vd;
const float HK7 = (q1)*(q1);
const float HK8 = (q2)*(q2);
const float HK9 = (q0)*(q0) - (q3)*(q3);
const float HK10 = HK7 - HK8 + HK9;
const float HK11 = HK10*Kaccx;
const float HK12 = q0*q3;
const float HK13 = q1*q2;
const float HK14 = HK12 + HK13;
const float HK15 = HK14*HK3;
const float HK16 = q0*q2 - q1*q3;
const float HK17 = 2*HK5;
const float HK18 = 2*HK16;
const float HK19 = 2*HK14;
const float HK20 = 2*HK2;
const float HK21 = 2*HK4;
const float HK22 = 2*HK6;
const float HK23 = HK22*P(0,3);
const float HK24 = HK10*P(0,4) - HK19*P(0,23) + HK19*P(0,5) + HK20*P(0,0) + HK21*P(0,1) + HK23;
const float HK25 = (Kaccx)*(Kaccx);
const float HK26 = -HK10;
const float HK27 = -2*HK5;
const float HK28 = -2*HK16;
const float HK29 = HK19*P(5,23);
const float HK30 = HK10*P(4,23) - HK19*P(23,23) + HK20*P(0,23) + HK21*P(1,23) + HK22*P(3,23) + HK29;
const float HK31 = HK10*P(4,5) + HK19*P(5,5) + HK20*P(0,5) + HK21*P(1,5) + HK22*P(3,5) - HK29;
const float HK32 = HK10*P(4,6) + HK19*P(5,6) - HK19*P(6,23) + HK20*P(0,6) + HK21*P(1,6) + HK22*P(3,6);
const float HK33 = HK10*P(4,4) - HK19*P(4,23) + HK19*P(4,5) + HK20*P(0,4) + HK21*P(1,4) + HK22*P(3,4);
const float HK34 = HK10*P(4,22);
const float HK35 = -HK19*P(22,23) + HK19*P(5,22) + HK20*P(0,22) + HK21*P(1,22) + HK22*P(3,22) + HK34;
const float HK36 = HK10*P(1,4) - HK19*P(1,23) + HK19*P(1,5) + HK20*P(0,1) + HK21*P(1,1) + HK22*P(1,3);
const float HK37 = HK21*P(1,2);
const float HK38 = HK10*P(2,4) - HK19*P(2,23) + HK19*P(2,5) + HK20*P(0,2) + HK22*P(2,3) + HK37;
const float HK39 = HK20*P(0,3);
const float HK40 = HK10*P(3,4) - HK19*P(3,23) + HK19*P(3,5) + HK21*P(1,3) + HK22*P(3,3) + HK39;
const float HK41 = Kaccx/(HK10*HK25*(HK26*P(22,22) + HK27*P(2,22) + HK28*P(6,22) + HK35) - HK10*HK25*(HK26*P(4,22) + HK27*P(2,4) + HK28*P(4,6) + HK33) + 2*HK14*HK25*(HK26*P(22,23) + HK27*P(2,23) + HK28*P(6,23) + HK30) + 2*HK16*HK25*(HK26*P(6,22) + HK27*P(2,6) + HK28*P(6,6) + HK32) - HK19*HK25*(HK26*P(5,22) + HK27*P(2,5) + HK28*P(5,6) + HK31) - HK20*HK25*(HK24 + HK26*P(0,22) + HK27*P(0,2) + HK28*P(0,6)) - HK21*HK25*(HK26*P(1,22) + HK27*P(1,2) + HK28*P(1,6) + HK36) - HK22*HK25*(HK26*P(3,22) + HK27*P(2,3) + HK28*P(3,6) + HK40) + 2*HK25*HK5*(HK26*P(2,22) + HK27*P(2,2) + HK28*P(2,6) + HK38) - R_ACC);
const float HK42 = HK17*P(1,2);
const float HK43 = HK12 - HK13;
const float HK44 = 2*Kaccy;
const float HK45 = HK43*HK44;
const float HK46 = -HK7 + HK8 + HK9;
const float HK47 = HK46*Kaccy;
const float HK48 = q0*q1 + q2*q3;
const float HK49 = 2*HK43;
const float HK50 = 2*HK48;
const float HK51 = HK17*P(0,1) + HK21*P(0,2) + HK22*P(0,0) + HK46*P(0,5) + HK49*P(0,22) + HK50*P(0,6);
const float HK52 = (Kaccy)*(Kaccy);
const float HK53 = -HK46;
const float HK54 = -2*HK2;
const float HK55 = -2*HK43;
const float HK56 = HK17*P(1,6) + HK21*P(2,6) + HK22*P(0,6) + HK46*P(5,6) + HK49*P(6,22) + HK50*P(6,6);
const float HK57 = HK17*P(1,22) + HK21*P(2,22) + HK22*P(0,22) + HK46*P(5,22) + HK49*P(22,22) + HK50*P(6,22);
const float HK58 = HK49*P(4,22);
const float HK59 = HK17*P(1,4) + HK21*P(2,4) + HK22*P(0,4) + HK46*P(4,5) + HK50*P(4,6) + HK58;
const float HK60 = HK17*P(1,5) + HK21*P(2,5) + HK22*P(0,5) + HK46*P(5,5) + HK49*P(5,22) + HK50*P(5,6);
const float HK61 = HK46*P(5,23);
const float HK62 = HK17*P(1,23) + HK21*P(2,23) + HK22*P(0,23) + HK49*P(22,23) + HK50*P(6,23) + HK61;
const float HK63 = HK21*P(2,2) + HK22*P(0,2) + HK42 + HK46*P(2,5) + HK49*P(2,22) + HK50*P(2,6);
const float HK64 = HK17*P(1,1) + HK22*P(0,1) + HK37 + HK46*P(1,5) + HK49*P(1,22) + HK50*P(1,6);
const float HK65 = HK17*P(1,3) + HK21*P(2,3) + HK23 + HK46*P(3,5) + HK49*P(3,22) + HK50*P(3,6);
const float HK66 = Kaccy/(-HK17*HK52*(HK53*P(1,23) + HK54*P(1,3) + HK55*P(1,4) + HK64) + 2*HK2*HK52*(HK53*P(3,23) + HK54*P(3,3) + HK55*P(3,4) + HK65) - HK21*HK52*(HK53*P(2,23) + HK54*P(2,3) + HK55*P(2,4) + HK63) - HK22*HK52*(HK51 + HK53*P(0,23) + HK54*P(0,3) + HK55*P(0,4)) + 2*HK43*HK52*(HK53*P(4,23) + HK54*P(3,4) + HK55*P(4,4) + HK59) + HK46*HK52*(HK53*P(23,23) + HK54*P(3,23) + HK55*P(4,23) + HK62) - HK46*HK52*(HK53*P(5,23) + HK54*P(3,5) + HK55*P(4,5) + HK60) - HK49*HK52*(HK53*P(22,23) + HK54*P(3,22) + HK55*P(4,22) + HK57) - HK50*HK52*(HK53*P(6,23) + HK54*P(3,6) + HK55*P(4,6) + HK56) - R_ACC);


// Observation Jacobians - axis 0
Hfusion.at<0>() = -HK2*HK3;
Hfusion.at<1>() = -HK3*HK4;
Hfusion.at<2>() = HK3*HK5;
Hfusion.at<3>() = -HK3*HK6;
Hfusion.at<4>() = -HK11;
Hfusion.at<5>() = -HK15;
Hfusion.at<6>() = HK16*HK3;
Hfusion.at<22>() = HK11;
Hfusion.at<23>() = HK15;


// Kalman gains - axis 0
Kfusion(0) = HK41*(-HK10*P(0,22) - HK17*P(0,2) - HK18*P(0,6) + HK24);
Kfusion(1) = HK41*(-HK10*P(1,22) - HK18*P(1,6) + HK36 - HK42);
Kfusion(2) = HK41*(-HK10*P(2,22) - HK17*P(2,2) - HK18*P(2,6) + HK38);
Kfusion(3) = HK41*(-HK10*P(3,22) - HK17*P(2,3) - HK18*P(3,6) + HK40);
Kfusion(4) = HK41*(-HK17*P(2,4) - HK18*P(4,6) + HK33 - HK34);
Kfusion(5) = HK41*(-HK10*P(5,22) - HK17*P(2,5) - HK18*P(5,6) + HK31);
Kfusion(6) = HK41*(-HK10*P(6,22) - HK17*P(2,6) - HK18*P(6,6) + HK32);
Kfusion(7) = HK41*(HK10*P(4,7) - HK10*P(7,22) - HK17*P(2,7) - HK18*P(6,7) + HK19*P(5,7) - HK19*P(7,23) + HK20*P(0,7) + HK21*P(1,7) + HK22*P(3,7));
Kfusion(8) = HK41*(HK10*P(4,8) - HK10*P(8,22) - HK17*P(2,8) - HK18*P(6,8) + HK19*P(5,8) - HK19*P(8,23) + HK20*P(0,8) + HK21*P(1,8) + HK22*P(3,8));
Kfusion(9) = HK41*(HK10*P(4,9) - HK10*P(9,22) - HK17*P(2,9) - HK18*P(6,9) + HK19*P(5,9) - HK19*P(9,23) + HK20*P(0,9) + HK21*P(1,9) + HK22*P(3,9));
Kfusion(10) = HK41*(-HK10*P(10,22) + HK10*P(4,10) - HK17*P(2,10) - HK18*P(6,10) - HK19*P(10,23) + HK19*P(5,10) + HK20*P(0,10) + HK21*P(1,10) + HK22*P(3,10));
Kfusion(11) = HK41*(-HK10*P(11,22) + HK10*P(4,11) - HK17*P(2,11) - HK18*P(6,11) - HK19*P(11,23) + HK19*P(5,11) + HK20*P(0,11) + HK21*P(1,11) + HK22*P(3,11));
Kfusion(12) = HK41*(-HK10*P(12,22) + HK10*P(4,12) - HK17*P(2,12) - HK18*P(6,12) - HK19*P(12,23) + HK19*P(5,12) + HK20*P(0,12) + HK21*P(1,12) + HK22*P(3,12));
Kfusion(13) = HK41*(-HK10*P(13,22) + HK10*P(4,13) - HK17*P(2,13) - HK18*P(6,13) - HK19*P(13,23) + HK19*P(5,13) + HK20*P(0,13) + HK21*P(1,13) + HK22*P(3,13));
Kfusion(14) = HK41*(-HK10*P(14,22) + HK10*P(4,14) - HK17*P(2,14) - HK18*P(6,14) - HK19*P(14,23) + HK19*P(5,14) + HK20*P(0,14) + HK21*P(1,14) + HK22*P(3,14));
Kfusion(15) = HK41*(-HK10*P(15,22) + HK10*P(4,15) - HK17*P(2,15) - HK18*P(6,15) - HK19*P(15,23) + HK19*P(5,15) + HK20*P(0,15) + HK21*P(1,15) + HK22*P(3,15));
Kfusion(16) = HK41*(-HK10*P(16,22) + HK10*P(4,16) - HK17*P(2,16) - HK18*P(6,16) - HK19*P(16,23) + HK19*P(5,16) + HK20*P(0,16) + HK21*P(1,16) + HK22*P(3,16));
Kfusion(17) = HK41*(-HK10*P(17,22) + HK10*P(4,17) - HK17*P(2,17) - HK18*P(6,17) - HK19*P(17,23) + HK19*P(5,17) + HK20*P(0,17) + HK21*P(1,17) + HK22*P(3,17));
Kfusion(18) = HK41*(-HK10*P(18,22) + HK10*P(4,18) - HK17*P(2,18) - HK18*P(6,18) - HK19*P(18,23) + HK19*P(5,18) + HK20*P(0,18) + HK21*P(1,18) + HK22*P(3,18));
Kfusion(19) = HK41*(-HK10*P(19,22) + HK10*P(4,19) - HK17*P(2,19) - HK18*P(6,19) - HK19*P(19,23) + HK19*P(5,19) + HK20*P(0,19) + HK21*P(1,19) + HK22*P(3,19));
Kfusion(20) = HK41*(-HK10*P(20,22) + HK10*P(4,20) - HK17*P(2,20) - HK18*P(6,20) - HK19*P(20,23) + HK19*P(5,20) + HK20*P(0,20) + HK21*P(1,20) + HK22*P(3,20));
Kfusion(21) = HK41*(-HK10*P(21,22) + HK10*P(4,21) - HK17*P(2,21) - HK18*P(6,21) - HK19*P(21,23) + HK19*P(5,21) + HK20*P(0,21) + HK21*P(1,21) + HK22*P(3,21));
Kfusion(22) = HK41*(-HK10*P(22,22) - HK17*P(2,22) - HK18*P(6,22) + HK35);
Kfusion(23) = HK41*(-HK10*P(22,23) - HK17*P(2,23) - HK18*P(6,23) + HK30);


// Observation Jacobians - axis 1
Hfusion.at<0>() = -HK22*Kaccy;
Hfusion.at<1>() = -HK17*Kaccy;
Hfusion.at<2>() = -HK21*Kaccy;
Hfusion.at<3>() = HK20*Kaccy;
Hfusion.at<4>() = HK45;
Hfusion.at<5>() = -HK47;
Hfusion.at<6>() = -HK44*HK48;
Hfusion.at<22>() = -HK45;
Hfusion.at<23>() = HK47;


// Kalman gains - axis 1
Kfusion(0) = HK66*(-HK39 - HK46*P(0,23) - HK49*P(0,4) + HK51);
Kfusion(1) = HK66*(-HK20*P(1,3) - HK46*P(1,23) - HK49*P(1,4) + HK64);
Kfusion(2) = HK66*(-HK20*P(2,3) - HK46*P(2,23) - HK49*P(2,4) + HK63);
Kfusion(3) = HK66*(-HK20*P(3,3) - HK46*P(3,23) - HK49*P(3,4) + HK65);
Kfusion(4) = HK66*(-HK20*P(3,4) - HK46*P(4,23) - HK49*P(4,4) + HK59);
Kfusion(5) = HK66*(-HK20*P(3,5) - HK49*P(4,5) + HK60 - HK61);
Kfusion(6) = HK66*(-HK20*P(3,6) - HK46*P(6,23) - HK49*P(4,6) + HK56);
Kfusion(7) = HK66*(HK17*P(1,7) - HK20*P(3,7) + HK21*P(2,7) + HK22*P(0,7) + HK46*P(5,7) - HK46*P(7,23) - HK49*P(4,7) + HK49*P(7,22) + HK50*P(6,7));
Kfusion(8) = HK66*(HK17*P(1,8) - HK20*P(3,8) + HK21*P(2,8) + HK22*P(0,8) + HK46*P(5,8) - HK46*P(8,23) - HK49*P(4,8) + HK49*P(8,22) + HK50*P(6,8));
Kfusion(9) = HK66*(HK17*P(1,9) - HK20*P(3,9) + HK21*P(2,9) + HK22*P(0,9) + HK46*P(5,9) - HK46*P(9,23) - HK49*P(4,9) + HK49*P(9,22) + HK50*P(6,9));
Kfusion(10) = HK66*(HK17*P(1,10) - HK20*P(3,10) + HK21*P(2,10) + HK22*P(0,10) - HK46*P(10,23) + HK46*P(5,10) + HK49*P(10,22) - HK49*P(4,10) + HK50*P(6,10));
Kfusion(11) = HK66*(HK17*P(1,11) - HK20*P(3,11) + HK21*P(2,11) + HK22*P(0,11) - HK46*P(11,23) + HK46*P(5,11) + HK49*P(11,22) - HK49*P(4,11) + HK50*P(6,11));
Kfusion(12) = HK66*(HK17*P(1,12) - HK20*P(3,12) + HK21*P(2,12) + HK22*P(0,12) - HK46*P(12,23) + HK46*P(5,12) + HK49*P(12,22) - HK49*P(4,12) + HK50*P(6,12));
Kfusion(13) = HK66*(HK17*P(1,13) - HK20*P(3,13) + HK21*P(2,13) + HK22*P(0,13) - HK46*P(13,23) + HK46*P(5,13) + HK49*P(13,22) - HK49*P(4,13) + HK50*P(6,13));
Kfusion(14) = HK66*(HK17*P(1,14) - HK20*P(3,14) + HK21*P(2,14) + HK22*P(0,14) - HK46*P(14,23) + HK46*P(5,14) + HK49*P(14,22) - HK49*P(4,14) + HK50*P(6,14));
Kfusion(15) = HK66*(HK17*P(1,15) - HK20*P(3,15) + HK21*P(2,15) + HK22*P(0,15) - HK46*P(15,23) + HK46*P(5,15) + HK49*P(15,22) - HK49*P(4,15) + HK50*P(6,15));
Kfusion(16) = HK66*(HK17*P(1,16) - HK20*P(3,16) + HK21*P(2,16) + HK22*P(0,16) - HK46*P(16,23) + HK46*P(5,16) + HK49*P(16,22) - HK49*P(4,16) + HK50*P(6,16));
Kfusion(17) = HK66*(HK17*P(1,17) - HK20*P(3,17) + HK21*P(2,17) + HK22*P(0,17) - HK46*P(17,23) + HK46*P(5,17) + HK49*P(17,22) - HK49*P(4,17) + HK50*P(6,17));
Kfusion(18) = HK66*(HK17*P(1,18) - HK20*P(3,18) + HK21*P(2,18) + HK22*P(0,18) - HK46*P(18,23) + HK46*P(5,18) + HK49*P(18,22) - HK49*P(4,18) + HK50*P(6,18));
Kfusion(19) = HK66*(HK17*P(1,19) - HK20*P(3,19) + HK21*P(2,19) + HK22*P(0,19) - HK46*P(19,23) + HK46*P(5,19) + HK49*P(19,22) - HK49*P(4,19) + HK50*P(6,19));
Kfusion(20) = HK66*(HK17*P(1,20) - HK20*P(3,20) + HK21*P(2,20) + HK22*P(0,20) - HK46*P(20,23) + HK46*P(5,20) + HK49*P(20,22) - HK49*P(4,20) + HK50*P(6,20));
Kfusion(21) = HK66*(HK17*P(1,21) - HK20*P(3,21) + HK21*P(2,21) + HK22*P(0,21) - HK46*P(21,23) + HK46*P(5,21) + HK49*P(21,22) - HK49*P(4,21) + HK50*P(6,21));
Kfusion(22) = HK66*(-HK20*P(3,22) - HK46*P(22,23) + HK57 - HK58);
Kfusion(23) = HK66*(-HK20*P(3,23) - HK46*P(23,23) - HK49*P(4,23) + HK62);


// Observation Jacobians - axis 2


// Kalman gains - axis 2


