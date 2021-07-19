// Sub Expressions
const float HK0 = powf(magN, -2);
const float HK1 = HK0*powf(magE, 2) + 1;
const float HK2 = 1.0F/HK1;
const float HK3 = 1.0F/magN;
const float HK4 = HK2*HK3;
const float HK5 = HK3*magE;
const float HK6 = HK5*P(16,17) - P(17,17);
const float HK7 = powf(HK1, -2);
const float HK8 = HK5*P(16,16) - P(16,17);
const float HK9 = HK4/(-HK0*HK6*HK7 + HK7*HK8*magE/powf(magN, 3) + R_DECL);


// Observation Jacobians
Hfusion.at<0>() = 0;
Hfusion.at<1>() = 0;
Hfusion.at<2>() = 0;
Hfusion.at<3>() = 0;
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
Hfusion.at<16>() = -HK0*HK2*magE;
Hfusion.at<17>() = HK4;
Hfusion.at<18>() = 0;
Hfusion.at<19>() = 0;
Hfusion.at<20>() = 0;
Hfusion.at<21>() = 0;
Hfusion.at<22>() = 0;
Hfusion.at<23>() = 0;


// Kalman gains
Kfusion(0) = -HK9*(HK5*P(0,16) - P(0,17));
Kfusion(1) = -HK9*(HK5*P(1,16) - P(1,17));
Kfusion(2) = -HK9*(HK5*P(2,16) - P(2,17));
Kfusion(3) = -HK9*(HK5*P(3,16) - P(3,17));
Kfusion(4) = -HK9*(HK5*P(4,16) - P(4,17));
Kfusion(5) = -HK9*(HK5*P(5,16) - P(5,17));
Kfusion(6) = -HK9*(HK5*P(6,16) - P(6,17));
Kfusion(7) = -HK9*(HK5*P(7,16) - P(7,17));
Kfusion(8) = -HK9*(HK5*P(8,16) - P(8,17));
Kfusion(9) = -HK9*(HK5*P(9,16) - P(9,17));
Kfusion(10) = -HK9*(HK5*P(10,16) - P(10,17));
Kfusion(11) = -HK9*(HK5*P(11,16) - P(11,17));
Kfusion(12) = -HK9*(HK5*P(12,16) - P(12,17));
Kfusion(13) = -HK9*(HK5*P(13,16) - P(13,17));
Kfusion(14) = -HK9*(HK5*P(14,16) - P(14,17));
Kfusion(15) = -HK9*(HK5*P(15,16) - P(15,17));
Kfusion(16) = -HK8*HK9;
Kfusion(17) = -HK6*HK9;
Kfusion(18) = -HK9*(HK5*P(16,18) - P(17,18));
Kfusion(19) = -HK9*(HK5*P(16,19) - P(17,19));
Kfusion(20) = -HK9*(HK5*P(16,20) - P(17,20));
Kfusion(21) = -HK9*(HK5*P(16,21) - P(17,21));
Kfusion(22) = -HK9*(HK5*P(16,22) - P(17,22));
Kfusion(23) = -HK9*(HK5*P(16,23) - P(17,23));


