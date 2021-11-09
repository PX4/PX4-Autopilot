// Sub Expressions
const float HK0 = vn - vwn;
const float HK1 = ve - vwe;
const float HK2 = powf(HK0, 2) + powf(HK1, 2) + powf(vd, 2);
const float HK3 = powf(HK2, -1.0F/2.0F);
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
Hfusion.at<0>() = 0;
Hfusion.at<1>() = 0;
Hfusion.at<2>() = 0;
Hfusion.at<3>() = 0;
Hfusion.at<4>() = HK4;
Hfusion.at<5>() = HK5;
Hfusion.at<6>() = HK3*vd;
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
Hfusion.at<22>() = -HK4;
Hfusion.at<23>() = -HK5;


// Kalman gains
Kfusion(0) = HK16*(-HK0*P(0,22) + HK0*P(0,4) - HK1*P(0,23) + HK1*P(0,5) + P(0,6)*vd);
Kfusion(1) = HK16*(-HK0*P(1,22) + HK0*P(1,4) - HK1*P(1,23) + HK1*P(1,5) + P(1,6)*vd);
Kfusion(2) = HK16*(-HK0*P(2,22) + HK0*P(2,4) - HK1*P(2,23) + HK1*P(2,5) + P(2,6)*vd);
Kfusion(3) = HK16*(-HK0*P(3,22) + HK0*P(3,4) - HK1*P(3,23) + HK1*P(3,5) + P(3,6)*vd);
Kfusion(4) = HK12*HK16;
Kfusion(5) = HK16*HK9;
Kfusion(6) = HK16*HK7;
Kfusion(7) = HK16*(HK0*P(4,7) - HK0*P(7,22) + HK1*P(5,7) - HK1*P(7,23) + P(6,7)*vd);
Kfusion(8) = HK16*(HK0*P(4,8) - HK0*P(8,22) + HK1*P(5,8) - HK1*P(8,23) + P(6,8)*vd);
Kfusion(9) = HK16*(HK0*P(4,9) - HK0*P(9,22) + HK1*P(5,9) - HK1*P(9,23) + P(6,9)*vd);
Kfusion(10) = HK16*(-HK0*P(10,22) + HK0*P(4,10) - HK1*P(10,23) + HK1*P(5,10) + P(6,10)*vd);
Kfusion(11) = HK16*(-HK0*P(11,22) + HK0*P(4,11) - HK1*P(11,23) + HK1*P(5,11) + P(6,11)*vd);
Kfusion(12) = HK16*(-HK0*P(12,22) + HK0*P(4,12) - HK1*P(12,23) + HK1*P(5,12) + P(6,12)*vd);
Kfusion(13) = HK16*(-HK0*P(13,22) + HK0*P(4,13) - HK1*P(13,23) + HK1*P(5,13) + P(6,13)*vd);
Kfusion(14) = HK16*(-HK0*P(14,22) + HK0*P(4,14) - HK1*P(14,23) + HK1*P(5,14) + P(6,14)*vd);
Kfusion(15) = HK16*(-HK0*P(15,22) + HK0*P(4,15) - HK1*P(15,23) + HK1*P(5,15) + P(6,15)*vd);
Kfusion(16) = HK16*(-HK0*P(16,22) + HK0*P(4,16) - HK1*P(16,23) + HK1*P(5,16) + P(6,16)*vd);
Kfusion(17) = HK16*(-HK0*P(17,22) + HK0*P(4,17) - HK1*P(17,23) + HK1*P(5,17) + P(6,17)*vd);
Kfusion(18) = HK16*(-HK0*P(18,22) + HK0*P(4,18) - HK1*P(18,23) + HK1*P(5,18) + P(6,18)*vd);
Kfusion(19) = HK16*(-HK0*P(19,22) + HK0*P(4,19) - HK1*P(19,23) + HK1*P(5,19) + P(6,19)*vd);
Kfusion(20) = HK16*(-HK0*P(20,22) + HK0*P(4,20) - HK1*P(20,23) + HK1*P(5,20) + P(6,20)*vd);
Kfusion(21) = HK16*(-HK0*P(21,22) + HK0*P(4,21) - HK1*P(21,23) + HK1*P(5,21) + P(6,21)*vd);
Kfusion(22) = HK15*HK16;
Kfusion(23) = HK14*HK16;
