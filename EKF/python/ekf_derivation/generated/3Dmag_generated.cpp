// Axis 0 equations
// Sub Expressions
const float HKX0 = -magD*q2 + magE*q3 + magN*q0;
const float HKX1 = magD*q3 + magE*q2 + magN*q1;
const float HKX2 = magE*q1;
const float HKX3 = magD*q0;
const float HKX4 = magN*q2;
const float HKX5 = magD*q1 + magE*q0 - magN*q3;
const float HKX6 = powf(q0, 2) + powf(q1, 2) - powf(q2, 2) - powf(q3, 2);
const float HKX7 = q0*q3 + q1*q2;
const float HKX8 = q1*q3;
const float HKX9 = q0*q2;
const float HKX10 = 2*HKX7;
const float HKX11 = -2*HKX8 + 2*HKX9;
const float HKX12 = 2*HKX1;
const float HKX13 = 2*HKX0;
const float HKX14 = -2*HKX2 + 2*HKX3 + 2*HKX4;
const float HKX15 = 2*HKX5;
const float HKX16 = HKX10*P(0,17) - HKX11*P(0,18) + HKX12*P(0,1) + HKX13*P(0,0) - HKX14*P(0,2) + HKX15*P(0,3) + HKX6*P(0,16) + P(0,19);
const float HKX17 = HKX10*P(16,17) - HKX11*P(16,18) + HKX12*P(1,16) + HKX13*P(0,16) - HKX14*P(2,16) + HKX15*P(3,16) + HKX6*P(16,16) + P(16,19);
const float HKX18 = HKX10*P(17,18) - HKX11*P(18,18) + HKX12*P(1,18) + HKX13*P(0,18) - HKX14*P(2,18) + HKX15*P(3,18) + HKX6*P(16,18) + P(18,19);
const float HKX19 = HKX10*P(2,17) - HKX11*P(2,18) + HKX12*P(1,2) + HKX13*P(0,2) - HKX14*P(2,2) + HKX15*P(2,3) + HKX6*P(2,16) + P(2,19);
const float HKX20 = HKX10*P(17,17) - HKX11*P(17,18) + HKX12*P(1,17) + HKX13*P(0,17) - HKX14*P(2,17) + HKX15*P(3,17) + HKX6*P(16,17) + P(17,19);
const float HKX21 = HKX10*P(3,17) - HKX11*P(3,18) + HKX12*P(1,3) + HKX13*P(0,3) - HKX14*P(2,3) + HKX15*P(3,3) + HKX6*P(3,16) + P(3,19);
const float HKX22 = HKX10*P(1,17) - HKX11*P(1,18) + HKX12*P(1,1) + HKX13*P(0,1) - HKX14*P(1,2) + HKX15*P(1,3) + HKX6*P(1,16) + P(1,19);
const float HKX23 = HKX10*P(17,19) - HKX11*P(18,19) + HKX12*P(1,19) + HKX13*P(0,19) - HKX14*P(2,19) + HKX15*P(3,19) + HKX6*P(16,19) + P(19,19);
const float HKX24 = 1.0F/(HKX10*HKX20 - HKX11*HKX18 + HKX12*HKX22 + HKX13*HKX16 - HKX14*HKX19 + HKX15*HKX21 + HKX17*HKX6 + HKX23 + R_MAG);


// Observation Jacobians
Hfusion.at<0>() = 2*HKX0;
Hfusion.at<1>() = 2*HKX1;
Hfusion.at<2>() = 2*HKX2 - 2*HKX3 - 2*HKX4;
Hfusion.at<3>() = 2*HKX5;
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
Hfusion.at<16>() = HKX6;
Hfusion.at<17>() = 2*HKX7;
Hfusion.at<18>() = 2*HKX8 - 2*HKX9;
Hfusion.at<19>() = 1;
Hfusion.at<20>() = 0;
Hfusion.at<21>() = 0;
Hfusion.at<22>() = 0;
Hfusion.at<23>() = 0;


// Kalman gains
Kfusion(0) = HKX16*HKX24;
Kfusion(1) = HKX22*HKX24;
Kfusion(2) = HKX19*HKX24;
Kfusion(3) = HKX21*HKX24;
Kfusion(4) = HKX24*(HKX10*P(4,17) - HKX11*P(4,18) + HKX12*P(1,4) + HKX13*P(0,4) - HKX14*P(2,4) + HKX15*P(3,4) + HKX6*P(4,16) + P(4,19));
Kfusion(5) = HKX24*(HKX10*P(5,17) - HKX11*P(5,18) + HKX12*P(1,5) + HKX13*P(0,5) - HKX14*P(2,5) + HKX15*P(3,5) + HKX6*P(5,16) + P(5,19));
Kfusion(6) = HKX24*(HKX10*P(6,17) - HKX11*P(6,18) + HKX12*P(1,6) + HKX13*P(0,6) - HKX14*P(2,6) + HKX15*P(3,6) + HKX6*P(6,16) + P(6,19));
Kfusion(7) = HKX24*(HKX10*P(7,17) - HKX11*P(7,18) + HKX12*P(1,7) + HKX13*P(0,7) - HKX14*P(2,7) + HKX15*P(3,7) + HKX6*P(7,16) + P(7,19));
Kfusion(8) = HKX24*(HKX10*P(8,17) - HKX11*P(8,18) + HKX12*P(1,8) + HKX13*P(0,8) - HKX14*P(2,8) + HKX15*P(3,8) + HKX6*P(8,16) + P(8,19));
Kfusion(9) = HKX24*(HKX10*P(9,17) - HKX11*P(9,18) + HKX12*P(1,9) + HKX13*P(0,9) - HKX14*P(2,9) + HKX15*P(3,9) + HKX6*P(9,16) + P(9,19));
Kfusion(10) = HKX24*(HKX10*P(10,17) - HKX11*P(10,18) + HKX12*P(1,10) + HKX13*P(0,10) - HKX14*P(2,10) + HKX15*P(3,10) + HKX6*P(10,16) + P(10,19));
Kfusion(11) = HKX24*(HKX10*P(11,17) - HKX11*P(11,18) + HKX12*P(1,11) + HKX13*P(0,11) - HKX14*P(2,11) + HKX15*P(3,11) + HKX6*P(11,16) + P(11,19));
Kfusion(12) = HKX24*(HKX10*P(12,17) - HKX11*P(12,18) + HKX12*P(1,12) + HKX13*P(0,12) - HKX14*P(2,12) + HKX15*P(3,12) + HKX6*P(12,16) + P(12,19));
Kfusion(13) = HKX24*(HKX10*P(13,17) - HKX11*P(13,18) + HKX12*P(1,13) + HKX13*P(0,13) - HKX14*P(2,13) + HKX15*P(3,13) + HKX6*P(13,16) + P(13,19));
Kfusion(14) = HKX24*(HKX10*P(14,17) - HKX11*P(14,18) + HKX12*P(1,14) + HKX13*P(0,14) - HKX14*P(2,14) + HKX15*P(3,14) + HKX6*P(14,16) + P(14,19));
Kfusion(15) = HKX24*(HKX10*P(15,17) - HKX11*P(15,18) + HKX12*P(1,15) + HKX13*P(0,15) - HKX14*P(2,15) + HKX15*P(3,15) + HKX6*P(15,16) + P(15,19));
Kfusion(16) = HKX17*HKX24;
Kfusion(17) = HKX20*HKX24;
Kfusion(18) = HKX18*HKX24;
Kfusion(19) = HKX23*HKX24;
Kfusion(20) = HKX24*(HKX10*P(17,20) - HKX11*P(18,20) + HKX12*P(1,20) + HKX13*P(0,20) - HKX14*P(2,20) + HKX15*P(3,20) + HKX6*P(16,20) + P(19,20));
Kfusion(21) = HKX24*(HKX10*P(17,21) - HKX11*P(18,21) + HKX12*P(1,21) + HKX13*P(0,21) - HKX14*P(2,21) + HKX15*P(3,21) + HKX6*P(16,21) + P(19,21));
Kfusion(22) = HKX24*(HKX10*P(17,22) - HKX11*P(18,22) + HKX12*P(1,22) + HKX13*P(0,22) - HKX14*P(2,22) + HKX15*P(3,22) + HKX6*P(16,22) + P(19,22));
Kfusion(23) = HKX24*(HKX10*P(17,23) - HKX11*P(18,23) + HKX12*P(1,23) + HKX13*P(0,23) - HKX14*P(2,23) + HKX15*P(3,23) + HKX6*P(16,23) + P(19,23));


// Axis 1 equations
// Sub Expressions
const float HKY0 = magD*q1 + magE*q0 - magN*q3;
const float HKY1 = magD*q0 - magE*q1 + magN*q2;
const float HKY2 = magD*q3 + magE*q2 + magN*q1;
const float HKY3 = magD*q2;
const float HKY4 = magE*q3;
const float HKY5 = magN*q0;
const float HKY6 = q1*q2;
const float HKY7 = q0*q3;
const float HKY8 = powf(q0, 2) - powf(q1, 2) + powf(q2, 2) - powf(q3, 2);
const float HKY9 = q0*q1 + q2*q3;
const float HKY10 = 2*HKY9;
const float HKY11 = -2*HKY6 + 2*HKY7;
const float HKY12 = 2*HKY2;
const float HKY13 = 2*HKY0;
const float HKY14 = 2*HKY1;
const float HKY15 = -2*HKY3 + 2*HKY4 + 2*HKY5;
const float HKY16 = HKY10*P(0,18) - HKY11*P(0,16) + HKY12*P(0,2) + HKY13*P(0,0) + HKY14*P(0,1) - HKY15*P(0,3) + HKY8*P(0,17) + P(0,20);
const float HKY17 = HKY10*P(17,18) - HKY11*P(16,17) + HKY12*P(2,17) + HKY13*P(0,17) + HKY14*P(1,17) - HKY15*P(3,17) + HKY8*P(17,17) + P(17,20);
const float HKY18 = HKY10*P(16,18) - HKY11*P(16,16) + HKY12*P(2,16) + HKY13*P(0,16) + HKY14*P(1,16) - HKY15*P(3,16) + HKY8*P(16,17) + P(16,20);
const float HKY19 = HKY10*P(3,18) - HKY11*P(3,16) + HKY12*P(2,3) + HKY13*P(0,3) + HKY14*P(1,3) - HKY15*P(3,3) + HKY8*P(3,17) + P(3,20);
const float HKY20 = HKY10*P(18,18) - HKY11*P(16,18) + HKY12*P(2,18) + HKY13*P(0,18) + HKY14*P(1,18) - HKY15*P(3,18) + HKY8*P(17,18) + P(18,20);
const float HKY21 = HKY10*P(1,18) - HKY11*P(1,16) + HKY12*P(1,2) + HKY13*P(0,1) + HKY14*P(1,1) - HKY15*P(1,3) + HKY8*P(1,17) + P(1,20);
const float HKY22 = HKY10*P(2,18) - HKY11*P(2,16) + HKY12*P(2,2) + HKY13*P(0,2) + HKY14*P(1,2) - HKY15*P(2,3) + HKY8*P(2,17) + P(2,20);
const float HKY23 = HKY10*P(18,20) - HKY11*P(16,20) + HKY12*P(2,20) + HKY13*P(0,20) + HKY14*P(1,20) - HKY15*P(3,20) + HKY8*P(17,20) + P(20,20);
const float HKY24 = 1.0F/(HKY10*HKY20 - HKY11*HKY18 + HKY12*HKY22 + HKY13*HKY16 + HKY14*HKY21 - HKY15*HKY19 + HKY17*HKY8 + HKY23 + R_MAG);


// Observation Jacobians
Hfusion.at<0>() = 2*HKY0;
Hfusion.at<1>() = 2*HKY1;
Hfusion.at<2>() = 2*HKY2;
Hfusion.at<3>() = 2*HKY3 - 2*HKY4 - 2*HKY5;
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
Hfusion.at<16>() = 2*HKY6 - 2*HKY7;
Hfusion.at<17>() = HKY8;
Hfusion.at<18>() = 2*HKY9;
Hfusion.at<19>() = 0;
Hfusion.at<20>() = 1;
Hfusion.at<21>() = 0;
Hfusion.at<22>() = 0;
Hfusion.at<23>() = 0;


// Kalman gains
Kfusion(0) = HKY16*HKY24;
Kfusion(1) = HKY21*HKY24;
Kfusion(2) = HKY22*HKY24;
Kfusion(3) = HKY19*HKY24;
Kfusion(4) = HKY24*(HKY10*P(4,18) - HKY11*P(4,16) + HKY12*P(2,4) + HKY13*P(0,4) + HKY14*P(1,4) - HKY15*P(3,4) + HKY8*P(4,17) + P(4,20));
Kfusion(5) = HKY24*(HKY10*P(5,18) - HKY11*P(5,16) + HKY12*P(2,5) + HKY13*P(0,5) + HKY14*P(1,5) - HKY15*P(3,5) + HKY8*P(5,17) + P(5,20));
Kfusion(6) = HKY24*(HKY10*P(6,18) - HKY11*P(6,16) + HKY12*P(2,6) + HKY13*P(0,6) + HKY14*P(1,6) - HKY15*P(3,6) + HKY8*P(6,17) + P(6,20));
Kfusion(7) = HKY24*(HKY10*P(7,18) - HKY11*P(7,16) + HKY12*P(2,7) + HKY13*P(0,7) + HKY14*P(1,7) - HKY15*P(3,7) + HKY8*P(7,17) + P(7,20));
Kfusion(8) = HKY24*(HKY10*P(8,18) - HKY11*P(8,16) + HKY12*P(2,8) + HKY13*P(0,8) + HKY14*P(1,8) - HKY15*P(3,8) + HKY8*P(8,17) + P(8,20));
Kfusion(9) = HKY24*(HKY10*P(9,18) - HKY11*P(9,16) + HKY12*P(2,9) + HKY13*P(0,9) + HKY14*P(1,9) - HKY15*P(3,9) + HKY8*P(9,17) + P(9,20));
Kfusion(10) = HKY24*(HKY10*P(10,18) - HKY11*P(10,16) + HKY12*P(2,10) + HKY13*P(0,10) + HKY14*P(1,10) - HKY15*P(3,10) + HKY8*P(10,17) + P(10,20));
Kfusion(11) = HKY24*(HKY10*P(11,18) - HKY11*P(11,16) + HKY12*P(2,11) + HKY13*P(0,11) + HKY14*P(1,11) - HKY15*P(3,11) + HKY8*P(11,17) + P(11,20));
Kfusion(12) = HKY24*(HKY10*P(12,18) - HKY11*P(12,16) + HKY12*P(2,12) + HKY13*P(0,12) + HKY14*P(1,12) - HKY15*P(3,12) + HKY8*P(12,17) + P(12,20));
Kfusion(13) = HKY24*(HKY10*P(13,18) - HKY11*P(13,16) + HKY12*P(2,13) + HKY13*P(0,13) + HKY14*P(1,13) - HKY15*P(3,13) + HKY8*P(13,17) + P(13,20));
Kfusion(14) = HKY24*(HKY10*P(14,18) - HKY11*P(14,16) + HKY12*P(2,14) + HKY13*P(0,14) + HKY14*P(1,14) - HKY15*P(3,14) + HKY8*P(14,17) + P(14,20));
Kfusion(15) = HKY24*(HKY10*P(15,18) - HKY11*P(15,16) + HKY12*P(2,15) + HKY13*P(0,15) + HKY14*P(1,15) - HKY15*P(3,15) + HKY8*P(15,17) + P(15,20));
Kfusion(16) = HKY18*HKY24;
Kfusion(17) = HKY17*HKY24;
Kfusion(18) = HKY20*HKY24;
Kfusion(19) = HKY24*(HKY10*P(18,19) - HKY11*P(16,19) + HKY12*P(2,19) + HKY13*P(0,19) + HKY14*P(1,19) - HKY15*P(3,19) + HKY8*P(17,19) + P(19,20));
Kfusion(20) = HKY23*HKY24;
Kfusion(21) = HKY24*(HKY10*P(18,21) - HKY11*P(16,21) + HKY12*P(2,21) + HKY13*P(0,21) + HKY14*P(1,21) - HKY15*P(3,21) + HKY8*P(17,21) + P(20,21));
Kfusion(22) = HKY24*(HKY10*P(18,22) - HKY11*P(16,22) + HKY12*P(2,22) + HKY13*P(0,22) + HKY14*P(1,22) - HKY15*P(3,22) + HKY8*P(17,22) + P(20,22));
Kfusion(23) = HKY24*(HKY10*P(18,23) - HKY11*P(16,23) + HKY12*P(2,23) + HKY13*P(0,23) + HKY14*P(1,23) - HKY15*P(3,23) + HKY8*P(17,23) + P(20,23));


// Axis 2 equations
// Sub Expressions
const float HKZ0 = magD*q0 - magE*q1 + magN*q2;
const float HKZ1 = magN*q3;
const float HKZ2 = magD*q1;
const float HKZ3 = magE*q0;
const float HKZ4 = -magD*q2 + magE*q3 + magN*q0;
const float HKZ5 = magD*q3 + magE*q2 + magN*q1;
const float HKZ6 = q0*q2 + q1*q3;
const float HKZ7 = q2*q3;
const float HKZ8 = q0*q1;
const float HKZ9 = powf(q0, 2) - powf(q1, 2) - powf(q2, 2) + powf(q3, 2);
const float HKZ10 = 2*HKZ6;
const float HKZ11 = -2*HKZ7 + 2*HKZ8;
const float HKZ12 = 2*HKZ5;
const float HKZ13 = 2*HKZ0;
const float HKZ14 = -2*HKZ1 + 2*HKZ2 + 2*HKZ3;
const float HKZ15 = 2*HKZ4;
const float HKZ16 = HKZ10*P(0,16) - HKZ11*P(0,17) + HKZ12*P(0,3) + HKZ13*P(0,0) - HKZ14*P(0,1) + HKZ15*P(0,2) + HKZ9*P(0,18) + P(0,21);
const float HKZ17 = HKZ10*P(16,18) - HKZ11*P(17,18) + HKZ12*P(3,18) + HKZ13*P(0,18) - HKZ14*P(1,18) + HKZ15*P(2,18) + HKZ9*P(18,18) + P(18,21);
const float HKZ18 = HKZ10*P(16,17) - HKZ11*P(17,17) + HKZ12*P(3,17) + HKZ13*P(0,17) - HKZ14*P(1,17) + HKZ15*P(2,17) + HKZ9*P(17,18) + P(17,21);
const float HKZ19 = HKZ10*P(1,16) - HKZ11*P(1,17) + HKZ12*P(1,3) + HKZ13*P(0,1) - HKZ14*P(1,1) + HKZ15*P(1,2) + HKZ9*P(1,18) + P(1,21);
const float HKZ20 = HKZ10*P(16,16) - HKZ11*P(16,17) + HKZ12*P(3,16) + HKZ13*P(0,16) - HKZ14*P(1,16) + HKZ15*P(2,16) + HKZ9*P(16,18) + P(16,21);
const float HKZ21 = HKZ10*P(3,16) - HKZ11*P(3,17) + HKZ12*P(3,3) + HKZ13*P(0,3) - HKZ14*P(1,3) + HKZ15*P(2,3) + HKZ9*P(3,18) + P(3,21);
const float HKZ22 = HKZ10*P(2,16) - HKZ11*P(2,17) + HKZ12*P(2,3) + HKZ13*P(0,2) - HKZ14*P(1,2) + HKZ15*P(2,2) + HKZ9*P(2,18) + P(2,21);
const float HKZ23 = HKZ10*P(16,21) - HKZ11*P(17,21) + HKZ12*P(3,21) + HKZ13*P(0,21) - HKZ14*P(1,21) + HKZ15*P(2,21) + HKZ9*P(18,21) + P(21,21);
const float HKZ24 = 1.0F/(HKZ10*HKZ20 - HKZ11*HKZ18 + HKZ12*HKZ21 + HKZ13*HKZ16 - HKZ14*HKZ19 + HKZ15*HKZ22 + HKZ17*HKZ9 + HKZ23 + R_MAG);


// Observation Jacobians
Hfusion.at<0>() = 2*HKZ0;
Hfusion.at<1>() = 2*HKZ1 - 2*HKZ2 - 2*HKZ3;
Hfusion.at<2>() = 2*HKZ4;
Hfusion.at<3>() = 2*HKZ5;
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
Hfusion.at<16>() = 2*HKZ6;
Hfusion.at<17>() = 2*HKZ7 - 2*HKZ8;
Hfusion.at<18>() = HKZ9;
Hfusion.at<19>() = 0;
Hfusion.at<20>() = 0;
Hfusion.at<21>() = 1;
Hfusion.at<22>() = 0;
Hfusion.at<23>() = 0;


// Kalman gains
Kfusion(0) = HKZ16*HKZ24;
Kfusion(1) = HKZ19*HKZ24;
Kfusion(2) = HKZ22*HKZ24;
Kfusion(3) = HKZ21*HKZ24;
Kfusion(4) = HKZ24*(HKZ10*P(4,16) - HKZ11*P(4,17) + HKZ12*P(3,4) + HKZ13*P(0,4) - HKZ14*P(1,4) + HKZ15*P(2,4) + HKZ9*P(4,18) + P(4,21));
Kfusion(5) = HKZ24*(HKZ10*P(5,16) - HKZ11*P(5,17) + HKZ12*P(3,5) + HKZ13*P(0,5) - HKZ14*P(1,5) + HKZ15*P(2,5) + HKZ9*P(5,18) + P(5,21));
Kfusion(6) = HKZ24*(HKZ10*P(6,16) - HKZ11*P(6,17) + HKZ12*P(3,6) + HKZ13*P(0,6) - HKZ14*P(1,6) + HKZ15*P(2,6) + HKZ9*P(6,18) + P(6,21));
Kfusion(7) = HKZ24*(HKZ10*P(7,16) - HKZ11*P(7,17) + HKZ12*P(3,7) + HKZ13*P(0,7) - HKZ14*P(1,7) + HKZ15*P(2,7) + HKZ9*P(7,18) + P(7,21));
Kfusion(8) = HKZ24*(HKZ10*P(8,16) - HKZ11*P(8,17) + HKZ12*P(3,8) + HKZ13*P(0,8) - HKZ14*P(1,8) + HKZ15*P(2,8) + HKZ9*P(8,18) + P(8,21));
Kfusion(9) = HKZ24*(HKZ10*P(9,16) - HKZ11*P(9,17) + HKZ12*P(3,9) + HKZ13*P(0,9) - HKZ14*P(1,9) + HKZ15*P(2,9) + HKZ9*P(9,18) + P(9,21));
Kfusion(10) = HKZ24*(HKZ10*P(10,16) - HKZ11*P(10,17) + HKZ12*P(3,10) + HKZ13*P(0,10) - HKZ14*P(1,10) + HKZ15*P(2,10) + HKZ9*P(10,18) + P(10,21));
Kfusion(11) = HKZ24*(HKZ10*P(11,16) - HKZ11*P(11,17) + HKZ12*P(3,11) + HKZ13*P(0,11) - HKZ14*P(1,11) + HKZ15*P(2,11) + HKZ9*P(11,18) + P(11,21));
Kfusion(12) = HKZ24*(HKZ10*P(12,16) - HKZ11*P(12,17) + HKZ12*P(3,12) + HKZ13*P(0,12) - HKZ14*P(1,12) + HKZ15*P(2,12) + HKZ9*P(12,18) + P(12,21));
Kfusion(13) = HKZ24*(HKZ10*P(13,16) - HKZ11*P(13,17) + HKZ12*P(3,13) + HKZ13*P(0,13) - HKZ14*P(1,13) + HKZ15*P(2,13) + HKZ9*P(13,18) + P(13,21));
Kfusion(14) = HKZ24*(HKZ10*P(14,16) - HKZ11*P(14,17) + HKZ12*P(3,14) + HKZ13*P(0,14) - HKZ14*P(1,14) + HKZ15*P(2,14) + HKZ9*P(14,18) + P(14,21));
Kfusion(15) = HKZ24*(HKZ10*P(15,16) - HKZ11*P(15,17) + HKZ12*P(3,15) + HKZ13*P(0,15) - HKZ14*P(1,15) + HKZ15*P(2,15) + HKZ9*P(15,18) + P(15,21));
Kfusion(16) = HKZ20*HKZ24;
Kfusion(17) = HKZ18*HKZ24;
Kfusion(18) = HKZ17*HKZ24;
Kfusion(19) = HKZ24*(HKZ10*P(16,19) - HKZ11*P(17,19) + HKZ12*P(3,19) + HKZ13*P(0,19) - HKZ14*P(1,19) + HKZ15*P(2,19) + HKZ9*P(18,19) + P(19,21));
Kfusion(20) = HKZ24*(HKZ10*P(16,20) - HKZ11*P(17,20) + HKZ12*P(3,20) + HKZ13*P(0,20) - HKZ14*P(1,20) + HKZ15*P(2,20) + HKZ9*P(18,20) + P(20,21));
Kfusion(21) = HKZ23*HKZ24;
Kfusion(22) = HKZ24*(HKZ10*P(16,22) - HKZ11*P(17,22) + HKZ12*P(3,22) + HKZ13*P(0,22) - HKZ14*P(1,22) + HKZ15*P(2,22) + HKZ9*P(18,22) + P(21,22));
Kfusion(23) = HKZ24*(HKZ10*P(16,23) - HKZ11*P(17,23) + HKZ12*P(3,23) + HKZ13*P(0,23) - HKZ14*P(1,23) + HKZ15*P(2,23) + HKZ9*P(18,23) + P(21,23));


