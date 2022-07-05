// Sub Expressions
const float IV0 = q0*q1;
const float IV1 = q2*q3;
const float IV2 = 2*IV0 + 2*IV1;
const float IV3 = 2*q0*q3 - 2*q1*q2;
const float IV4 = 2*magD*q3 + 2*magE*q2 + 2*magN*q1;
const float IV5 = 2*magD*q1 + 2*magE*q0 - 2*magN*q3;
const float IV6 = 2*magD*q0 - 2*magE*q1 + 2*magN*q2;
const float IV7 = -2*magD*q2 + 2*magE*q3 + 2*magN*q0;
const float IV8 = (q2)*(q2);
const float IV9 = (q3)*(q3);
const float IV10 = (q0)*(q0) - (q1)*(q1);
const float IV11 = IV10 + IV8 - IV9;
const float IV12 = IV7*P(2,3);
const float IV13 = IV5*P(0,1);
const float IV14 = IV6*P(0,1);
const float IV15 = IV4*P(2,3);
const float IV16 = 2*q0*q2 + 2*q1*q3;
const float IV17 = 2*IV0 - 2*IV1;
const float IV18 = IV10 - IV8 + IV9;


// Observation Jacobians - axis 0
Hfusion.at<0>() = R_MAG;
Hfusion.at<1>() = IV11*P(17,20) + IV11*(IV11*P(17,17) + IV2*P(17,18) - IV3*P(16,17) + IV4*P(2,17) + IV5*P(0,17) + IV6*P(1,17) - IV7*P(3,17) + P(17,20)) + IV2*P(18,20) + IV2*(IV11*P(17,18) + IV2*P(18,18) - IV3*P(16,18) + IV4*P(2,18) + IV5*P(0,18) + IV6*P(1,18) - IV7*P(3,18) + P(18,20)) - IV3*P(16,20) - IV3*(IV11*P(16,17) + IV2*P(16,18) - IV3*P(16,16) + IV4*P(2,16) + IV5*P(0,16) + IV6*P(1,16) - IV7*P(3,16) + P(16,20)) + IV4*P(2,20) + IV4*(IV11*P(2,17) - IV12 + IV2*P(2,18) - IV3*P(2,16) + IV4*P(2,2) + IV5*P(0,2) + IV6*P(1,2) + P(2,20)) + IV5*P(0,20) + IV5*(IV11*P(0,17) + IV14 + IV2*P(0,18) - IV3*P(0,16) + IV4*P(0,2) + IV5*P(0,0) - IV7*P(0,3) + P(0,20)) + IV6*P(1,20) + IV6*(IV11*P(1,17) + IV13 + IV2*P(1,18) - IV3*P(1,16) + IV4*P(1,2) + IV6*P(1,1) - IV7*P(1,3) + P(1,20)) - IV7*P(3,20) - IV7*(IV11*P(3,17) + IV15 + IV2*P(3,18) - IV3*P(3,16) + IV5*P(0,3) + IV6*P(1,3) - IV7*P(3,3) + P(3,20)) + P(20,20) + R_MAG;
Hfusion.at<2>() = IV16*P(16,21) + IV16*(IV16*P(16,16) - IV17*P(16,17) + IV18*P(16,18) + IV4*P(3,16) - IV5*P(1,16) + IV6*P(0,16) + IV7*P(2,16) + P(16,21)) - IV17*P(17,21) - IV17*(IV16*P(16,17) - IV17*P(17,17) + IV18*P(17,18) + IV4*P(3,17) - IV5*P(1,17) + IV6*P(0,17) + IV7*P(2,17) + P(17,21)) + IV18*P(18,21) + IV18*(IV16*P(16,18) - IV17*P(17,18) + IV18*P(18,18) + IV4*P(3,18) - IV5*P(1,18) + IV6*P(0,18) + IV7*P(2,18) + P(18,21)) + IV4*P(3,21) + IV4*(IV12 + IV16*P(3,16) - IV17*P(3,17) + IV18*P(3,18) + IV4*P(3,3) - IV5*P(1,3) + IV6*P(0,3) + P(3,21)) - IV5*P(1,21) - IV5*(IV14 + IV16*P(1,16) - IV17*P(1,17) + IV18*P(1,18) + IV4*P(1,3) - IV5*P(1,1) + IV7*P(1,2) + P(1,21)) + IV6*P(0,21) + IV6*(-IV13 + IV16*P(0,16) - IV17*P(0,17) + IV18*P(0,18) + IV4*P(0,3) + IV6*P(0,0) + IV7*P(0,2) + P(0,21)) + IV7*P(2,21) + IV7*(IV15 + IV16*P(2,16) - IV17*P(2,17) + IV18*P(2,18) - IV5*P(1,2) + IV6*P(0,2) + IV7*P(2,2) + P(2,21)) + P(21,21) + R_MAG;


// Kalman gains - axis 0


// Observation Jacobians - axis 1


// Kalman gains - axis 1


// Observation Jacobians - axis 2


// Kalman gains - axis 2


