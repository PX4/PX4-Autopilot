// Sub Expressions
const float HK0 = sinf(ant_yaw);
const float HK1 = q0*q3;
const float HK2 = q1*q2;
const float HK3 = 2*HK0*(HK1 - HK2);
const float HK4 = cosf(ant_yaw);
const float HK5 = powf(q1, 2);
const float HK6 = powf(q2, 2);
const float HK7 = powf(q0, 2) - powf(q3, 2);
const float HK8 = HK4*(HK5 - HK6 + HK7);
const float HK9 = HK3 - HK8;
const float HK10 = 1.0F/HK9;
const float HK11 = HK4*q0;
const float HK12 = HK0*q3;
const float HK13 = HK0*(-HK5 + HK6 + HK7) + 2*HK4*(HK1 + HK2);
const float HK14 = HK10*HK13;
const float HK15 = HK0*q0 + HK4*q3;
const float HK16 = HK10*(HK14*(HK11 - HK12) + HK15);
const float HK17 = powf(HK13, 2)/powf(HK9, 2) + 1;
const float HK18 = 2/HK17;
const float HK19 = 1.0F/(-HK3 + HK8);
const float HK20 = HK4*q1;
const float HK21 = HK0*q2;
const float HK22 = HK13*HK19;
const float HK23 = HK0*q1 - HK4*q2;
const float HK24 = HK19*(HK22*(HK20 + HK21) + HK23);
const float HK25 = HK19*(-HK20 - HK21 + HK22*HK23);
const float HK26 = HK10*(-HK11 + HK12 + HK14*HK15);
const float HK27 = -HK16*P(0,0) - HK24*P(0,1) - HK25*P(0,2) + HK26*P(0,3);
const float HK28 = -HK16*P(0,1) - HK24*P(1,1) - HK25*P(1,2) + HK26*P(1,3);
const float HK29 = 4/powf(HK17, 2);
const float HK30 = -HK16*P(0,2) - HK24*P(1,2) - HK25*P(2,2) + HK26*P(2,3);
const float HK31 = -HK16*P(0,3) - HK24*P(1,3) - HK25*P(2,3) + HK26*P(3,3);
const float HK32 = HK18/(-HK16*HK27*HK29 - HK24*HK28*HK29 - HK25*HK29*HK30 + HK26*HK29*HK31 + R_YAW);


// Observation Jacobians
Hfusion.at<0>() = -HK16*HK18;
Hfusion.at<1>() = -HK18*HK24;
Hfusion.at<2>() = -HK18*HK25;
Hfusion.at<3>() = HK18*HK26;
Hfusion.at<4>() = 0;
Hfusion.at<5>() = 0;
Hfusion.at<6>() = 0;
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
Hfusion.at<22>() = 0;
Hfusion.at<23>() = 0;


// Kalman gains
Kfusion(0) = HK27*HK32;
Kfusion(1) = HK28*HK32;
Kfusion(2) = HK30*HK32;
Kfusion(3) = HK31*HK32;
Kfusion(4) = HK32*(-HK16*P(0,4) - HK24*P(1,4) - HK25*P(2,4) + HK26*P(3,4));
Kfusion(5) = HK32*(-HK16*P(0,5) - HK24*P(1,5) - HK25*P(2,5) + HK26*P(3,5));
Kfusion(6) = HK32*(-HK16*P(0,6) - HK24*P(1,6) - HK25*P(2,6) + HK26*P(3,6));
Kfusion(7) = HK32*(-HK16*P(0,7) - HK24*P(1,7) - HK25*P(2,7) + HK26*P(3,7));
Kfusion(8) = HK32*(-HK16*P(0,8) - HK24*P(1,8) - HK25*P(2,8) + HK26*P(3,8));
Kfusion(9) = HK32*(-HK16*P(0,9) - HK24*P(1,9) - HK25*P(2,9) + HK26*P(3,9));
Kfusion(10) = HK32*(-HK16*P(0,10) - HK24*P(1,10) - HK25*P(2,10) + HK26*P(3,10));
Kfusion(11) = HK32*(-HK16*P(0,11) - HK24*P(1,11) - HK25*P(2,11) + HK26*P(3,11));
Kfusion(12) = HK32*(-HK16*P(0,12) - HK24*P(1,12) - HK25*P(2,12) + HK26*P(3,12));
Kfusion(13) = HK32*(-HK16*P(0,13) - HK24*P(1,13) - HK25*P(2,13) + HK26*P(3,13));
Kfusion(14) = HK32*(-HK16*P(0,14) - HK24*P(1,14) - HK25*P(2,14) + HK26*P(3,14));
Kfusion(15) = HK32*(-HK16*P(0,15) - HK24*P(1,15) - HK25*P(2,15) + HK26*P(3,15));
Kfusion(16) = HK32*(-HK16*P(0,16) - HK24*P(1,16) - HK25*P(2,16) + HK26*P(3,16));
Kfusion(17) = HK32*(-HK16*P(0,17) - HK24*P(1,17) - HK25*P(2,17) + HK26*P(3,17));
Kfusion(18) = HK32*(-HK16*P(0,18) - HK24*P(1,18) - HK25*P(2,18) + HK26*P(3,18));
Kfusion(19) = HK32*(-HK16*P(0,19) - HK24*P(1,19) - HK25*P(2,19) + HK26*P(3,19));
Kfusion(20) = HK32*(-HK16*P(0,20) - HK24*P(1,20) - HK25*P(2,20) + HK26*P(3,20));
Kfusion(21) = HK32*(-HK16*P(0,21) - HK24*P(1,21) - HK25*P(2,21) + HK26*P(3,21));
Kfusion(22) = HK32*(-HK16*P(0,22) - HK24*P(1,22) - HK25*P(2,22) + HK26*P(3,22));
Kfusion(23) = HK32*(-HK16*P(0,23) - HK24*P(1,23) - HK25*P(2,23) + HK26*P(3,23));


