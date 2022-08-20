// Axis 0 equations
// Sub Expressions
const float HKX0 = -magD*q2 + magE*q3 + magN*q0;
const float HKX1 = magD*q3 + magE*q2 + magN*q1;
const float HKX2 = magD*q0 - magE*q1 + magN*q2;
const float HKX3 = magD*q1 + magE*q0 - magN*q3;
const float HKX4 = (q0)*(q0) + (q1)*(q1) - (q2)*(q2) - (q3)*(q3);
const float HKX5 = q0*q3 + q1*q2;
const float HKX6 = q0*q2 - q1*q3;
const float HKX7 = 2*HKX5;
const float HKX8 = 2*HKX6;
const float HKX9 = 2*HKX1;
const float HKX10 = 2*HKX0;
const float HKX11 = 2*HKX2;
const float HKX12 = 2*HKX3;
const float HKX13 = HKX10*P(0,0) - HKX11*P(0,2) + HKX12*P(0,3) + HKX4*P(0,16) + HKX7*P(0,17) - HKX8*P(0,18) + HKX9*P(0,1) + P(0,19);
const float HKX14 = HKX10*P(0,16) - HKX11*P(2,16) + HKX12*P(3,16) + HKX4*P(16,16) + HKX7*P(16,17) - HKX8*P(16,18) + HKX9*P(1,16) + P(16,19);
const float HKX15 = HKX10*P(0,18) - HKX11*P(2,18) + HKX12*P(3,18) + HKX4*P(16,18) + HKX7*P(17,18) - HKX8*P(18,18) + HKX9*P(1,18) + P(18,19);
const float HKX16 = HKX10*P(0,2) - HKX11*P(2,2) + HKX12*P(2,3) + HKX4*P(2,16) + HKX7*P(2,17) - HKX8*P(2,18) + HKX9*P(1,2) + P(2,19);
const float HKX17 = HKX10*P(0,17) - HKX11*P(2,17) + HKX12*P(3,17) + HKX4*P(16,17) + HKX7*P(17,17) - HKX8*P(17,18) + HKX9*P(1,17) + P(17,19);
const float HKX18 = HKX10*P(0,3) - HKX11*P(2,3) + HKX12*P(3,3) + HKX4*P(3,16) + HKX7*P(3,17) - HKX8*P(3,18) + HKX9*P(1,3) + P(3,19);
const float HKX19 = HKX10*P(0,1) - HKX11*P(1,2) + HKX12*P(1,3) + HKX4*P(1,16) + HKX7*P(1,17) - HKX8*P(1,18) + HKX9*P(1,1) + P(1,19);
const float HKX20 = HKX10*P(0,19) - HKX11*P(2,19) + HKX12*P(3,19) + HKX4*P(16,19) + HKX7*P(17,19) - HKX8*P(18,19) + HKX9*P(1,19) + P(19,19);
const float HKX21 = 1.0F/(HKX10*HKX13 - HKX11*HKX16 + HKX12*HKX18 + HKX14*HKX4 - HKX15*HKX8 + HKX17*HKX7 + HKX19*HKX9 + HKX20 + R_MAG);


// Observation Jacobians
Hfusion.at<0>() = 2*HKX0;
Hfusion.at<1>() = 2*HKX1;
Hfusion.at<2>() = -2*HKX2;
Hfusion.at<3>() = 2*HKX3;
Hfusion.at<16>() = HKX4;
Hfusion.at<17>() = 2*HKX5;
Hfusion.at<18>() = -2*HKX6;
Hfusion.at<19>() = 1;


// Kalman gains
Kfusion(0) = HKX13*HKX21;
Kfusion(1) = HKX19*HKX21;
Kfusion(2) = HKX16*HKX21;
Kfusion(3) = HKX18*HKX21;
Kfusion(4) = HKX21*(HKX10*P(0,4) - HKX11*P(2,4) + HKX12*P(3,4) + HKX4*P(4,16) + HKX7*P(4,17) - HKX8*P(4,18) + HKX9*P(1,4) + P(4,19));
Kfusion(5) = HKX21*(HKX10*P(0,5) - HKX11*P(2,5) + HKX12*P(3,5) + HKX4*P(5,16) + HKX7*P(5,17) - HKX8*P(5,18) + HKX9*P(1,5) + P(5,19));
Kfusion(6) = HKX21*(HKX10*P(0,6) - HKX11*P(2,6) + HKX12*P(3,6) + HKX4*P(6,16) + HKX7*P(6,17) - HKX8*P(6,18) + HKX9*P(1,6) + P(6,19));
Kfusion(7) = HKX21*(HKX10*P(0,7) - HKX11*P(2,7) + HKX12*P(3,7) + HKX4*P(7,16) + HKX7*P(7,17) - HKX8*P(7,18) + HKX9*P(1,7) + P(7,19));
Kfusion(8) = HKX21*(HKX10*P(0,8) - HKX11*P(2,8) + HKX12*P(3,8) + HKX4*P(8,16) + HKX7*P(8,17) - HKX8*P(8,18) + HKX9*P(1,8) + P(8,19));
Kfusion(9) = HKX21*(HKX10*P(0,9) - HKX11*P(2,9) + HKX12*P(3,9) + HKX4*P(9,16) + HKX7*P(9,17) - HKX8*P(9,18) + HKX9*P(1,9) + P(9,19));
Kfusion(10) = HKX21*(HKX10*P(0,10) - HKX11*P(2,10) + HKX12*P(3,10) + HKX4*P(10,16) + HKX7*P(10,17) - HKX8*P(10,18) + HKX9*P(1,10) + P(10,19));
Kfusion(11) = HKX21*(HKX10*P(0,11) - HKX11*P(2,11) + HKX12*P(3,11) + HKX4*P(11,16) + HKX7*P(11,17) - HKX8*P(11,18) + HKX9*P(1,11) + P(11,19));
Kfusion(12) = HKX21*(HKX10*P(0,12) - HKX11*P(2,12) + HKX12*P(3,12) + HKX4*P(12,16) + HKX7*P(12,17) - HKX8*P(12,18) + HKX9*P(1,12) + P(12,19));
Kfusion(13) = HKX21*(HKX10*P(0,13) - HKX11*P(2,13) + HKX12*P(3,13) + HKX4*P(13,16) + HKX7*P(13,17) - HKX8*P(13,18) + HKX9*P(1,13) + P(13,19));
Kfusion(14) = HKX21*(HKX10*P(0,14) - HKX11*P(2,14) + HKX12*P(3,14) + HKX4*P(14,16) + HKX7*P(14,17) - HKX8*P(14,18) + HKX9*P(1,14) + P(14,19));
Kfusion(15) = HKX21*(HKX10*P(0,15) - HKX11*P(2,15) + HKX12*P(3,15) + HKX4*P(15,16) + HKX7*P(15,17) - HKX8*P(15,18) + HKX9*P(1,15) + P(15,19));
Kfusion(16) = HKX14*HKX21;
Kfusion(17) = HKX17*HKX21;
Kfusion(18) = HKX15*HKX21;
Kfusion(19) = HKX20*HKX21;
Kfusion(20) = HKX21*(HKX10*P(0,20) - HKX11*P(2,20) + HKX12*P(3,20) + HKX4*P(16,20) + HKX7*P(17,20) - HKX8*P(18,20) + HKX9*P(1,20) + P(19,20));
Kfusion(21) = HKX21*(HKX10*P(0,21) - HKX11*P(2,21) + HKX12*P(3,21) + HKX4*P(16,21) + HKX7*P(17,21) - HKX8*P(18,21) + HKX9*P(1,21) + P(19,21));
Kfusion(22) = HKX21*(HKX10*P(0,22) - HKX11*P(2,22) + HKX12*P(3,22) + HKX4*P(16,22) + HKX7*P(17,22) - HKX8*P(18,22) + HKX9*P(1,22) + P(19,22));
Kfusion(23) = HKX21*(HKX10*P(0,23) - HKX11*P(2,23) + HKX12*P(3,23) + HKX4*P(16,23) + HKX7*P(17,23) - HKX8*P(18,23) + HKX9*P(1,23) + P(19,23));


// Predicted observation


// Innovation variance


// Axis 1 equations
// Sub Expressions
const float HKY0 = magD*q1 + magE*q0 - magN*q3;
const float HKY1 = magD*q0 - magE*q1 + magN*q2;
const float HKY2 = magD*q3 + magE*q2 + magN*q1;
const float HKY3 = -magD*q2 + magE*q3 + magN*q0;
const float HKY4 = q0*q3 - q1*q2;
const float HKY5 = (q0)*(q0) - (q1)*(q1) + (q2)*(q2) - (q3)*(q3);
const float HKY6 = q0*q1 + q2*q3;
const float HKY7 = 2*HKY6;
const float HKY8 = 2*HKY4;
const float HKY9 = 2*HKY2;
const float HKY10 = 2*HKY0;
const float HKY11 = 2*HKY1;
const float HKY12 = 2*HKY3;
const float HKY13 = HKY10*P(0,0) + HKY11*P(0,1) - HKY12*P(0,3) + HKY5*P(0,17) + HKY7*P(0,18) - HKY8*P(0,16) + HKY9*P(0,2) + P(0,20);
const float HKY14 = HKY10*P(0,17) + HKY11*P(1,17) - HKY12*P(3,17) + HKY5*P(17,17) + HKY7*P(17,18) - HKY8*P(16,17) + HKY9*P(2,17) + P(17,20);
const float HKY15 = HKY10*P(0,16) + HKY11*P(1,16) - HKY12*P(3,16) + HKY5*P(16,17) + HKY7*P(16,18) - HKY8*P(16,16) + HKY9*P(2,16) + P(16,20);
const float HKY16 = HKY10*P(0,3) + HKY11*P(1,3) - HKY12*P(3,3) + HKY5*P(3,17) + HKY7*P(3,18) - HKY8*P(3,16) + HKY9*P(2,3) + P(3,20);
const float HKY17 = HKY10*P(0,18) + HKY11*P(1,18) - HKY12*P(3,18) + HKY5*P(17,18) + HKY7*P(18,18) - HKY8*P(16,18) + HKY9*P(2,18) + P(18,20);
const float HKY18 = HKY10*P(0,1) + HKY11*P(1,1) - HKY12*P(1,3) + HKY5*P(1,17) + HKY7*P(1,18) - HKY8*P(1,16) + HKY9*P(1,2) + P(1,20);
const float HKY19 = HKY10*P(0,2) + HKY11*P(1,2) - HKY12*P(2,3) + HKY5*P(2,17) + HKY7*P(2,18) - HKY8*P(2,16) + HKY9*P(2,2) + P(2,20);
const float HKY20 = HKY10*P(0,20) + HKY11*P(1,20) - HKY12*P(3,20) + HKY5*P(17,20) + HKY7*P(18,20) - HKY8*P(16,20) + HKY9*P(2,20) + P(20,20);
const float HKY21 = 1.0F/(HKY10*HKY13 + HKY11*HKY18 - HKY12*HKY16 + HKY14*HKY5 - HKY15*HKY8 + HKY17*HKY7 + HKY19*HKY9 + HKY20 + R_MAG);


// Observation Jacobians
Hfusion.at<0>() = 2*HKY0;
Hfusion.at<1>() = 2*HKY1;
Hfusion.at<2>() = 2*HKY2;
Hfusion.at<3>() = -2*HKY3;
Hfusion.at<16>() = -2*HKY4;
Hfusion.at<17>() = HKY5;
Hfusion.at<18>() = 2*HKY6;
Hfusion.at<20>() = 1;


// Kalman gains
Kfusion(0) = HKY13*HKY21;
Kfusion(1) = HKY18*HKY21;
Kfusion(2) = HKY19*HKY21;
Kfusion(3) = HKY16*HKY21;
Kfusion(4) = HKY21*(HKY10*P(0,4) + HKY11*P(1,4) - HKY12*P(3,4) + HKY5*P(4,17) + HKY7*P(4,18) - HKY8*P(4,16) + HKY9*P(2,4) + P(4,20));
Kfusion(5) = HKY21*(HKY10*P(0,5) + HKY11*P(1,5) - HKY12*P(3,5) + HKY5*P(5,17) + HKY7*P(5,18) - HKY8*P(5,16) + HKY9*P(2,5) + P(5,20));
Kfusion(6) = HKY21*(HKY10*P(0,6) + HKY11*P(1,6) - HKY12*P(3,6) + HKY5*P(6,17) + HKY7*P(6,18) - HKY8*P(6,16) + HKY9*P(2,6) + P(6,20));
Kfusion(7) = HKY21*(HKY10*P(0,7) + HKY11*P(1,7) - HKY12*P(3,7) + HKY5*P(7,17) + HKY7*P(7,18) - HKY8*P(7,16) + HKY9*P(2,7) + P(7,20));
Kfusion(8) = HKY21*(HKY10*P(0,8) + HKY11*P(1,8) - HKY12*P(3,8) + HKY5*P(8,17) + HKY7*P(8,18) - HKY8*P(8,16) + HKY9*P(2,8) + P(8,20));
Kfusion(9) = HKY21*(HKY10*P(0,9) + HKY11*P(1,9) - HKY12*P(3,9) + HKY5*P(9,17) + HKY7*P(9,18) - HKY8*P(9,16) + HKY9*P(2,9) + P(9,20));
Kfusion(10) = HKY21*(HKY10*P(0,10) + HKY11*P(1,10) - HKY12*P(3,10) + HKY5*P(10,17) + HKY7*P(10,18) - HKY8*P(10,16) + HKY9*P(2,10) + P(10,20));
Kfusion(11) = HKY21*(HKY10*P(0,11) + HKY11*P(1,11) - HKY12*P(3,11) + HKY5*P(11,17) + HKY7*P(11,18) - HKY8*P(11,16) + HKY9*P(2,11) + P(11,20));
Kfusion(12) = HKY21*(HKY10*P(0,12) + HKY11*P(1,12) - HKY12*P(3,12) + HKY5*P(12,17) + HKY7*P(12,18) - HKY8*P(12,16) + HKY9*P(2,12) + P(12,20));
Kfusion(13) = HKY21*(HKY10*P(0,13) + HKY11*P(1,13) - HKY12*P(3,13) + HKY5*P(13,17) + HKY7*P(13,18) - HKY8*P(13,16) + HKY9*P(2,13) + P(13,20));
Kfusion(14) = HKY21*(HKY10*P(0,14) + HKY11*P(1,14) - HKY12*P(3,14) + HKY5*P(14,17) + HKY7*P(14,18) - HKY8*P(14,16) + HKY9*P(2,14) + P(14,20));
Kfusion(15) = HKY21*(HKY10*P(0,15) + HKY11*P(1,15) - HKY12*P(3,15) + HKY5*P(15,17) + HKY7*P(15,18) - HKY8*P(15,16) + HKY9*P(2,15) + P(15,20));
Kfusion(16) = HKY15*HKY21;
Kfusion(17) = HKY14*HKY21;
Kfusion(18) = HKY17*HKY21;
Kfusion(19) = HKY21*(HKY10*P(0,19) + HKY11*P(1,19) - HKY12*P(3,19) + HKY5*P(17,19) + HKY7*P(18,19) - HKY8*P(16,19) + HKY9*P(2,19) + P(19,20));
Kfusion(20) = HKY20*HKY21;
Kfusion(21) = HKY21*(HKY10*P(0,21) + HKY11*P(1,21) - HKY12*P(3,21) + HKY5*P(17,21) + HKY7*P(18,21) - HKY8*P(16,21) + HKY9*P(2,21) + P(20,21));
Kfusion(22) = HKY21*(HKY10*P(0,22) + HKY11*P(1,22) - HKY12*P(3,22) + HKY5*P(17,22) + HKY7*P(18,22) - HKY8*P(16,22) + HKY9*P(2,22) + P(20,22));
Kfusion(23) = HKY21*(HKY10*P(0,23) + HKY11*P(1,23) - HKY12*P(3,23) + HKY5*P(17,23) + HKY7*P(18,23) - HKY8*P(16,23) + HKY9*P(2,23) + P(20,23));


// Predicted observation


// Innovation variance


// Axis 2 equations
// Sub Expressions
const float HKZ0 = magD*q0 - magE*q1 + magN*q2;
const float HKZ1 = magD*q1 + magE*q0 - magN*q3;
const float HKZ2 = -magD*q2 + magE*q3 + magN*q0;
const float HKZ3 = magD*q3 + magE*q2 + magN*q1;
const float HKZ4 = q0*q2 + q1*q3;
const float HKZ5 = q0*q1 - q2*q3;
const float HKZ6 = (q0)*(q0) - (q1)*(q1) - (q2)*(q2) + (q3)*(q3);
const float HKZ7 = 2*HKZ4;
const float HKZ8 = 2*HKZ5;
const float HKZ9 = 2*HKZ3;
const float HKZ10 = 2*HKZ0;
const float HKZ11 = 2*HKZ1;
const float HKZ12 = 2*HKZ2;
const float HKZ13 = HKZ10*P(0,0) - HKZ11*P(0,1) + HKZ12*P(0,2) + HKZ6*P(0,18) + HKZ7*P(0,16) - HKZ8*P(0,17) + HKZ9*P(0,3) + P(0,21);
const float HKZ14 = HKZ10*P(0,18) - HKZ11*P(1,18) + HKZ12*P(2,18) + HKZ6*P(18,18) + HKZ7*P(16,18) - HKZ8*P(17,18) + HKZ9*P(3,18) + P(18,21);
const float HKZ15 = HKZ10*P(0,17) - HKZ11*P(1,17) + HKZ12*P(2,17) + HKZ6*P(17,18) + HKZ7*P(16,17) - HKZ8*P(17,17) + HKZ9*P(3,17) + P(17,21);
const float HKZ16 = HKZ10*P(0,1) - HKZ11*P(1,1) + HKZ12*P(1,2) + HKZ6*P(1,18) + HKZ7*P(1,16) - HKZ8*P(1,17) + HKZ9*P(1,3) + P(1,21);
const float HKZ17 = HKZ10*P(0,16) - HKZ11*P(1,16) + HKZ12*P(2,16) + HKZ6*P(16,18) + HKZ7*P(16,16) - HKZ8*P(16,17) + HKZ9*P(3,16) + P(16,21);
const float HKZ18 = HKZ10*P(0,3) - HKZ11*P(1,3) + HKZ12*P(2,3) + HKZ6*P(3,18) + HKZ7*P(3,16) - HKZ8*P(3,17) + HKZ9*P(3,3) + P(3,21);
const float HKZ19 = HKZ10*P(0,2) - HKZ11*P(1,2) + HKZ12*P(2,2) + HKZ6*P(2,18) + HKZ7*P(2,16) - HKZ8*P(2,17) + HKZ9*P(2,3) + P(2,21);
const float HKZ20 = HKZ10*P(0,21) - HKZ11*P(1,21) + HKZ12*P(2,21) + HKZ6*P(18,21) + HKZ7*P(16,21) - HKZ8*P(17,21) + HKZ9*P(3,21) + P(21,21);
const float HKZ21 = 1.0F/(HKZ10*HKZ13 - HKZ11*HKZ16 + HKZ12*HKZ19 + HKZ14*HKZ6 - HKZ15*HKZ8 + HKZ17*HKZ7 + HKZ18*HKZ9 + HKZ20 + R_MAG);


// Observation Jacobians
Hfusion.at<0>() = 2*HKZ0;
Hfusion.at<1>() = -2*HKZ1;
Hfusion.at<2>() = 2*HKZ2;
Hfusion.at<3>() = 2*HKZ3;
Hfusion.at<16>() = 2*HKZ4;
Hfusion.at<17>() = -2*HKZ5;
Hfusion.at<18>() = HKZ6;
Hfusion.at<21>() = 1;


// Kalman gains
Kfusion(0) = HKZ13*HKZ21;
Kfusion(1) = HKZ16*HKZ21;
Kfusion(2) = HKZ19*HKZ21;
Kfusion(3) = HKZ18*HKZ21;
Kfusion(4) = HKZ21*(HKZ10*P(0,4) - HKZ11*P(1,4) + HKZ12*P(2,4) + HKZ6*P(4,18) + HKZ7*P(4,16) - HKZ8*P(4,17) + HKZ9*P(3,4) + P(4,21));
Kfusion(5) = HKZ21*(HKZ10*P(0,5) - HKZ11*P(1,5) + HKZ12*P(2,5) + HKZ6*P(5,18) + HKZ7*P(5,16) - HKZ8*P(5,17) + HKZ9*P(3,5) + P(5,21));
Kfusion(6) = HKZ21*(HKZ10*P(0,6) - HKZ11*P(1,6) + HKZ12*P(2,6) + HKZ6*P(6,18) + HKZ7*P(6,16) - HKZ8*P(6,17) + HKZ9*P(3,6) + P(6,21));
Kfusion(7) = HKZ21*(HKZ10*P(0,7) - HKZ11*P(1,7) + HKZ12*P(2,7) + HKZ6*P(7,18) + HKZ7*P(7,16) - HKZ8*P(7,17) + HKZ9*P(3,7) + P(7,21));
Kfusion(8) = HKZ21*(HKZ10*P(0,8) - HKZ11*P(1,8) + HKZ12*P(2,8) + HKZ6*P(8,18) + HKZ7*P(8,16) - HKZ8*P(8,17) + HKZ9*P(3,8) + P(8,21));
Kfusion(9) = HKZ21*(HKZ10*P(0,9) - HKZ11*P(1,9) + HKZ12*P(2,9) + HKZ6*P(9,18) + HKZ7*P(9,16) - HKZ8*P(9,17) + HKZ9*P(3,9) + P(9,21));
Kfusion(10) = HKZ21*(HKZ10*P(0,10) - HKZ11*P(1,10) + HKZ12*P(2,10) + HKZ6*P(10,18) + HKZ7*P(10,16) - HKZ8*P(10,17) + HKZ9*P(3,10) + P(10,21));
Kfusion(11) = HKZ21*(HKZ10*P(0,11) - HKZ11*P(1,11) + HKZ12*P(2,11) + HKZ6*P(11,18) + HKZ7*P(11,16) - HKZ8*P(11,17) + HKZ9*P(3,11) + P(11,21));
Kfusion(12) = HKZ21*(HKZ10*P(0,12) - HKZ11*P(1,12) + HKZ12*P(2,12) + HKZ6*P(12,18) + HKZ7*P(12,16) - HKZ8*P(12,17) + HKZ9*P(3,12) + P(12,21));
Kfusion(13) = HKZ21*(HKZ10*P(0,13) - HKZ11*P(1,13) + HKZ12*P(2,13) + HKZ6*P(13,18) + HKZ7*P(13,16) - HKZ8*P(13,17) + HKZ9*P(3,13) + P(13,21));
Kfusion(14) = HKZ21*(HKZ10*P(0,14) - HKZ11*P(1,14) + HKZ12*P(2,14) + HKZ6*P(14,18) + HKZ7*P(14,16) - HKZ8*P(14,17) + HKZ9*P(3,14) + P(14,21));
Kfusion(15) = HKZ21*(HKZ10*P(0,15) - HKZ11*P(1,15) + HKZ12*P(2,15) + HKZ6*P(15,18) + HKZ7*P(15,16) - HKZ8*P(15,17) + HKZ9*P(3,15) + P(15,21));
Kfusion(16) = HKZ17*HKZ21;
Kfusion(17) = HKZ15*HKZ21;
Kfusion(18) = HKZ14*HKZ21;
Kfusion(19) = HKZ21*(HKZ10*P(0,19) - HKZ11*P(1,19) + HKZ12*P(2,19) + HKZ6*P(18,19) + HKZ7*P(16,19) - HKZ8*P(17,19) + HKZ9*P(3,19) + P(19,21));
Kfusion(20) = HKZ21*(HKZ10*P(0,20) - HKZ11*P(1,20) + HKZ12*P(2,20) + HKZ6*P(18,20) + HKZ7*P(16,20) - HKZ8*P(17,20) + HKZ9*P(3,20) + P(20,21));
Kfusion(21) = HKZ20*HKZ21;
Kfusion(22) = HKZ21*(HKZ10*P(0,22) - HKZ11*P(1,22) + HKZ12*P(2,22) + HKZ6*P(18,22) + HKZ7*P(16,22) - HKZ8*P(17,22) + HKZ9*P(3,22) + P(21,22));
Kfusion(23) = HKZ21*(HKZ10*P(0,23) - HKZ11*P(1,23) + HKZ12*P(2,23) + HKZ6*P(18,23) + HKZ7*P(16,23) - HKZ8*P(17,23) + HKZ9*P(3,23) + P(21,23));


// Predicted observation


// Innovation variance


