// Sub Expressions
const float HK0 = 1.0F/(P(24,24) - 2*P(9,24) + P(9,9) + R_HAGL);


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
Hfusion.at<9>() = -1;
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
Hfusion.at<24>() = 1;


// Kalman gains
Kfusion(0) = HK0*(P(0,24) - P(0,9));
Kfusion(1) = HK0*(P(1,24) - P(1,9));
Kfusion(2) = HK0*(P(2,24) - P(2,9));
Kfusion(3) = HK0*(P(3,24) - P(3,9));
Kfusion(4) = HK0*(P(4,24) - P(4,9));
Kfusion(5) = HK0*(P(5,24) - P(5,9));
Kfusion(6) = HK0*(P(6,24) - P(6,9));
Kfusion(7) = HK0*(P(7,24) - P(7,9));
Kfusion(8) = HK0*(P(8,24) - P(8,9));
Kfusion(9) = HK0*(P(9,24) - P(9,9));
Kfusion(10) = HK0*(P(10,24) - P(9,10));
Kfusion(11) = HK0*(P(11,24) - P(9,11));
Kfusion(12) = HK0*(P(12,24) - P(9,12));
Kfusion(13) = HK0*(P(13,24) - P(9,13));
Kfusion(14) = HK0*(P(14,24) - P(9,14));
Kfusion(15) = HK0*(P(15,24) - P(9,15));
Kfusion(16) = HK0*(P(16,24) - P(9,16));
Kfusion(17) = HK0*(P(17,24) - P(9,17));
Kfusion(18) = HK0*(P(18,24) - P(9,18));
Kfusion(19) = HK0*(P(19,24) - P(9,19));
Kfusion(20) = HK0*(P(20,24) - P(9,20));
Kfusion(21) = HK0*(P(21,24) - P(9,21));
Kfusion(22) = HK0*(P(22,24) - P(9,22));
Kfusion(23) = HK0*(P(23,24) - P(9,23));
Kfusion(24) = HK0*(P(24,24) - P(9,24));
