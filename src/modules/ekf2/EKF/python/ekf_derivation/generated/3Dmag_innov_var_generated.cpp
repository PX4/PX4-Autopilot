// Sub Expressions
const float IV0 = q0*q1;
const float IV1 = q2*q3;
const float IV2 = 2*IV0 + 2*IV1;
const float IV3 = magN*q1;
const float IV4 = 2*IV3 + 2*magD*q3;
const float IV5 = magD*q1;
const float IV6 = -magN*q3;
const float IV7 = 2*IV5 + 2*IV6;
const float IV8 = 2*q0*q3 - 2*q1*q2;
const float IV9 = magN*q2;
const float IV10 = magE*q1;
const float IV11 = -4*IV10 + 2*IV9 + 2*magD*q0;
const float IV12 = 2*(q1)*(q1) - 1;
const float IV13 = IV12 + 2*(q3)*(q3);
const float IV14 = magN*q0;
const float IV15 = magD*q2;
const float IV16 = magE*q3;
const float IV17 = 2*IV14 - 2*IV15 + 4*IV16;
const float IV18 = q0*q2 + q1*q3;
const float IV19 = IV3 + magE*q2;
const float IV20 = 2*IV10 - 2*IV9;
const float IV21 = 2*IV0 - 2*IV1;
const float IV22 = IV12 + 2*(q2)*(q2);
const float IV23 = IV14 - 2*IV15 + IV16;
const float IV24 = 4*IV5 + 2*IV6 + 2*magE*q0;


// Observation Jacobians - axis 0
Hfusion.at<0>() = R_MAG;
Hfusion.at<1>() = IV11*P(1,20) + IV11*(IV11*P(1,1) - IV13*P(1,17) - IV17*P(1,3) + IV2*P(1,18) + IV4*P(1,2) + IV7*P(0,1) - IV8*P(1,16) + P(1,20)) - IV13*P(17,20) - IV13*(IV11*P(1,17) - IV13*P(17,17) - IV17*P(3,17) + IV2*P(17,18) + IV4*P(2,17) + IV7*P(0,17) - IV8*P(16,17) + P(17,20)) - IV17*P(3,20) - IV17*(IV11*P(1,3) - IV13*P(3,17) - IV17*P(3,3) + IV2*P(3,18) + IV4*P(2,3) + IV7*P(0,3) - IV8*P(3,16) + P(3,20)) + IV2*P(18,20) + IV2*(IV11*P(1,18) - IV13*P(17,18) - IV17*P(3,18) + IV2*P(18,18) + IV4*P(2,18) + IV7*P(0,18) - IV8*P(16,18) + P(18,20)) + IV4*P(2,20) + IV4*(IV11*P(1,2) - IV13*P(2,17) - IV17*P(2,3) + IV2*P(2,18) + IV4*P(2,2) + IV7*P(0,2) - IV8*P(2,16) + P(2,20)) + IV7*P(0,20) + IV7*(IV11*P(0,1) - IV13*P(0,17) - IV17*P(0,3) + IV2*P(0,18) + IV4*P(0,2) + IV7*P(0,0) - IV8*P(0,16) + P(0,20)) - IV8*P(16,20) - IV8*(IV11*P(1,16) - IV13*P(16,17) - IV17*P(3,16) + IV2*P(16,18) + IV4*P(2,16) + IV7*P(0,16) - IV8*P(16,16) + P(16,20)) + P(20,20) + R_MAG;
Hfusion.at<2>() = 2*IV18*P(16,21) + 2*IV18*(2*IV18*P(16,16) + 2*IV19*P(3,16) - IV20*P(0,16) - IV21*P(16,17) - IV22*P(16,18) + 2*IV23*P(2,16) - IV24*P(1,16) + P(16,21)) + 2*IV19*P(3,21) + 2*IV19*(2*IV18*P(3,16) + 2*IV19*P(3,3) - IV20*P(0,3) - IV21*P(3,17) - IV22*P(3,18) + 2*IV23*P(2,3) - IV24*P(1,3) + P(3,21)) - IV20*P(0,21) - IV20*(2*IV18*P(0,16) + 2*IV19*P(0,3) - IV20*P(0,0) - IV21*P(0,17) - IV22*P(0,18) + 2*IV23*P(0,2) - IV24*P(0,1) + P(0,21)) - IV21*P(17,21) - IV21*(2*IV18*P(16,17) + 2*IV19*P(3,17) - IV20*P(0,17) - IV21*P(17,17) - IV22*P(17,18) + 2*IV23*P(2,17) - IV24*P(1,17) + P(17,21)) - IV22*P(18,21) - IV22*(2*IV18*P(16,18) + 2*IV19*P(3,18) - IV20*P(0,18) - IV21*P(17,18) - IV22*P(18,18) + 2*IV23*P(2,18) - IV24*P(1,18) + P(18,21)) + 2*IV23*P(2,21) + 2*IV23*(2*IV18*P(2,16) + 2*IV19*P(2,3) - IV20*P(0,2) - IV21*P(2,17) - IV22*P(2,18) + 2*IV23*P(2,2) - IV24*P(1,2) + P(2,21)) - IV24*P(1,21) - IV24*(2*IV18*P(1,16) + 2*IV19*P(1,3) - IV20*P(0,1) - IV21*P(1,17) - IV22*P(1,18) + 2*IV23*P(1,2) - IV24*P(1,1) + P(1,21)) + P(21,21) + R_MAG;


// Kalman gains - axis 0


// Observation Jacobians - axis 1


// Kalman gains - axis 1


// Observation Jacobians - axis 2


// Kalman gains - axis 2


