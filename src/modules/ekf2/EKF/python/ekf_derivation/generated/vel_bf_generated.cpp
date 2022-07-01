// axis 0
const float HK0 = q0*vn - q2*vd + q3*ve;
const float HK1 = q1*vn + q2*ve + q3*vd;
const float HK2 = q0*vd - q1*ve + q2*vn;
const float HK3 = q0*ve + q1*vd - q3*vn;
const float HK4 = (q0)*(q0) + (q1)*(q1) - (q2)*(q2) - (q3)*(q3);
const float HK5 = q0*q3 + q1*q2;
const float HK6 = q0*q2 - q1*q3;
const float HK7 = 2*HK5;
const float HK8 = 2*HK6;
const float HK9 = 2*HK1;
const float HK10 = 2*HK0;
const float HK11 = 2*HK2;
const float HK12 = 2*HK3;
const float HK13 = HK10*P(0,0) - HK11*P(0,2) + HK12*P(0,3) + HK4*P(0,4) + HK7*P(0,5) - HK8*P(0,6) + HK9*P(0,1);
const float HK14 = HK10*P(0,5) - HK11*P(2,5) + HK12*P(3,5) + HK4*P(4,5) + HK7*P(5,5) - HK8*P(5,6) + HK9*P(1,5);
const float HK15 = HK10*P(0,6) - HK11*P(2,6) + HK12*P(3,6) + HK4*P(4,6) + HK7*P(5,6) - HK8*P(6,6) + HK9*P(1,6);
const float HK16 = HK10*P(0,1) - HK11*P(1,2) + HK12*P(1,3) + HK4*P(1,4) + HK7*P(1,5) - HK8*P(1,6) + HK9*P(1,1);
const float HK17 = HK10*P(0,2) - HK11*P(2,2) + HK12*P(2,3) + HK4*P(2,4) + HK7*P(2,5) - HK8*P(2,6) + HK9*P(1,2);
const float HK18 = HK10*P(0,3) - HK11*P(2,3) + HK12*P(3,3) + HK4*P(3,4) + HK7*P(3,5) - HK8*P(3,6) + HK9*P(1,3);
const float HK19 = HK10*P(0,4) - HK11*P(2,4) + HK12*P(3,4) + HK4*P(4,4) + HK7*P(4,5) - HK8*P(4,6) + HK9*P(1,4);
const float HK20 = 1.0F/(HK10*HK13 - HK11*HK17 + HK12*HK18 + HK14*HK7 - HK15*HK8 + HK16*HK9 + HK19*HK4 + R_VEL);


H_VEL(0) = 2*HK0;
H_VEL(1) = 2*HK1;
H_VEL(2) = -2*HK2;
H_VEL(3) = 2*HK3;
H_VEL(4) = HK4;
H_VEL(5) = 2*HK5;
H_VEL(6) = -2*HK6;


Kfusion(0) = HK13*HK20;
Kfusion(1) = HK16*HK20;
Kfusion(2) = HK17*HK20;
Kfusion(3) = HK18*HK20;
Kfusion(4) = HK19*HK20;
Kfusion(5) = HK14*HK20;
Kfusion(6) = HK15*HK20;
Kfusion(7) = HK20*(HK10*P(0,7) - HK11*P(2,7) + HK12*P(3,7) + HK4*P(4,7) + HK7*P(5,7) - HK8*P(6,7) + HK9*P(1,7));
Kfusion(8) = HK20*(HK10*P(0,8) - HK11*P(2,8) + HK12*P(3,8) + HK4*P(4,8) + HK7*P(5,8) - HK8*P(6,8) + HK9*P(1,8));
Kfusion(9) = HK20*(HK10*P(0,9) - HK11*P(2,9) + HK12*P(3,9) + HK4*P(4,9) + HK7*P(5,9) - HK8*P(6,9) + HK9*P(1,9));
Kfusion(10) = HK20*(HK10*P(0,10) - HK11*P(2,10) + HK12*P(3,10) + HK4*P(4,10) + HK7*P(5,10) - HK8*P(6,10) + HK9*P(1,10));
Kfusion(11) = HK20*(HK10*P(0,11) - HK11*P(2,11) + HK12*P(3,11) + HK4*P(4,11) + HK7*P(5,11) - HK8*P(6,11) + HK9*P(1,11));
Kfusion(12) = HK20*(HK10*P(0,12) - HK11*P(2,12) + HK12*P(3,12) + HK4*P(4,12) + HK7*P(5,12) - HK8*P(6,12) + HK9*P(1,12));
Kfusion(13) = HK20*(HK10*P(0,13) - HK11*P(2,13) + HK12*P(3,13) + HK4*P(4,13) + HK7*P(5,13) - HK8*P(6,13) + HK9*P(1,13));
Kfusion(14) = HK20*(HK10*P(0,14) - HK11*P(2,14) + HK12*P(3,14) + HK4*P(4,14) + HK7*P(5,14) - HK8*P(6,14) + HK9*P(1,14));
Kfusion(15) = HK20*(HK10*P(0,15) - HK11*P(2,15) + HK12*P(3,15) + HK4*P(4,15) + HK7*P(5,15) - HK8*P(6,15) + HK9*P(1,15));
Kfusion(16) = HK20*(HK10*P(0,16) - HK11*P(2,16) + HK12*P(3,16) + HK4*P(4,16) + HK7*P(5,16) - HK8*P(6,16) + HK9*P(1,16));
Kfusion(17) = HK20*(HK10*P(0,17) - HK11*P(2,17) + HK12*P(3,17) + HK4*P(4,17) + HK7*P(5,17) - HK8*P(6,17) + HK9*P(1,17));
Kfusion(18) = HK20*(HK10*P(0,18) - HK11*P(2,18) + HK12*P(3,18) + HK4*P(4,18) + HK7*P(5,18) - HK8*P(6,18) + HK9*P(1,18));
Kfusion(19) = HK20*(HK10*P(0,19) - HK11*P(2,19) + HK12*P(3,19) + HK4*P(4,19) + HK7*P(5,19) - HK8*P(6,19) + HK9*P(1,19));
Kfusion(20) = HK20*(HK10*P(0,20) - HK11*P(2,20) + HK12*P(3,20) + HK4*P(4,20) + HK7*P(5,20) - HK8*P(6,20) + HK9*P(1,20));
Kfusion(21) = HK20*(HK10*P(0,21) - HK11*P(2,21) + HK12*P(3,21) + HK4*P(4,21) + HK7*P(5,21) - HK8*P(6,21) + HK9*P(1,21));
Kfusion(22) = HK20*(HK10*P(0,22) - HK11*P(2,22) + HK12*P(3,22) + HK4*P(4,22) + HK7*P(5,22) - HK8*P(6,22) + HK9*P(1,22));
Kfusion(23) = HK20*(HK10*P(0,23) - HK11*P(2,23) + HK12*P(3,23) + HK4*P(4,23) + HK7*P(5,23) - HK8*P(6,23) + HK9*P(1,23));


// axis 1
const float HK0 = q0*ve + q1*vd - q3*vn;
const float HK1 = q0*vd - q1*ve + q2*vn;
const float HK2 = q1*vn + q2*ve + q3*vd;
const float HK3 = q0*vn - q2*vd + q3*ve;
const float HK4 = q0*q3 - q1*q2;
const float HK5 = (q0)*(q0) - (q1)*(q1) + (q2)*(q2) - (q3)*(q3);
const float HK6 = q0*q1 + q2*q3;
const float HK7 = 2*HK6;
const float HK8 = 2*HK4;
const float HK9 = 2*HK2;
const float HK10 = 2*HK0;
const float HK11 = 2*HK1;
const float HK12 = 2*HK3;
const float HK13 = HK10*P(0,0) + HK11*P(0,1) - HK12*P(0,3) + HK5*P(0,5) + HK7*P(0,6) - HK8*P(0,4) + HK9*P(0,2);
const float HK14 = HK10*P(0,6) + HK11*P(1,6) - HK12*P(3,6) + HK5*P(5,6) + HK7*P(6,6) - HK8*P(4,6) + HK9*P(2,6);
const float HK15 = HK10*P(0,4) + HK11*P(1,4) - HK12*P(3,4) + HK5*P(4,5) + HK7*P(4,6) - HK8*P(4,4) + HK9*P(2,4);
const float HK16 = HK10*P(0,2) + HK11*P(1,2) - HK12*P(2,3) + HK5*P(2,5) + HK7*P(2,6) - HK8*P(2,4) + HK9*P(2,2);
const float HK17 = HK10*P(0,1) + HK11*P(1,1) - HK12*P(1,3) + HK5*P(1,5) + HK7*P(1,6) - HK8*P(1,4) + HK9*P(1,2);
const float HK18 = HK10*P(0,3) + HK11*P(1,3) - HK12*P(3,3) + HK5*P(3,5) + HK7*P(3,6) - HK8*P(3,4) + HK9*P(2,3);
const float HK19 = HK10*P(0,5) + HK11*P(1,5) - HK12*P(3,5) + HK5*P(5,5) + HK7*P(5,6) - HK8*P(4,5) + HK9*P(2,5);
const float HK20 = 1.0F/(HK10*HK13 + HK11*HK17 - HK12*HK18 + HK14*HK7 - HK15*HK8 + HK16*HK9 + HK19*HK5 + R_VEL);


H_VEL(0) = 2*HK0;
H_VEL(1) = 2*HK1;
H_VEL(2) = 2*HK2;
H_VEL(3) = -2*HK3;
H_VEL(4) = -2*HK4;
H_VEL(5) = HK5;
H_VEL(6) = 2*HK6;


Kfusion(0) = HK13*HK20;
Kfusion(1) = HK17*HK20;
Kfusion(2) = HK16*HK20;
Kfusion(3) = HK18*HK20;
Kfusion(4) = HK15*HK20;
Kfusion(5) = HK19*HK20;
Kfusion(6) = HK14*HK20;
Kfusion(7) = HK20*(HK10*P(0,7) + HK11*P(1,7) - HK12*P(3,7) + HK5*P(5,7) + HK7*P(6,7) - HK8*P(4,7) + HK9*P(2,7));
Kfusion(8) = HK20*(HK10*P(0,8) + HK11*P(1,8) - HK12*P(3,8) + HK5*P(5,8) + HK7*P(6,8) - HK8*P(4,8) + HK9*P(2,8));
Kfusion(9) = HK20*(HK10*P(0,9) + HK11*P(1,9) - HK12*P(3,9) + HK5*P(5,9) + HK7*P(6,9) - HK8*P(4,9) + HK9*P(2,9));
Kfusion(10) = HK20*(HK10*P(0,10) + HK11*P(1,10) - HK12*P(3,10) + HK5*P(5,10) + HK7*P(6,10) - HK8*P(4,10) + HK9*P(2,10));
Kfusion(11) = HK20*(HK10*P(0,11) + HK11*P(1,11) - HK12*P(3,11) + HK5*P(5,11) + HK7*P(6,11) - HK8*P(4,11) + HK9*P(2,11));
Kfusion(12) = HK20*(HK10*P(0,12) + HK11*P(1,12) - HK12*P(3,12) + HK5*P(5,12) + HK7*P(6,12) - HK8*P(4,12) + HK9*P(2,12));
Kfusion(13) = HK20*(HK10*P(0,13) + HK11*P(1,13) - HK12*P(3,13) + HK5*P(5,13) + HK7*P(6,13) - HK8*P(4,13) + HK9*P(2,13));
Kfusion(14) = HK20*(HK10*P(0,14) + HK11*P(1,14) - HK12*P(3,14) + HK5*P(5,14) + HK7*P(6,14) - HK8*P(4,14) + HK9*P(2,14));
Kfusion(15) = HK20*(HK10*P(0,15) + HK11*P(1,15) - HK12*P(3,15) + HK5*P(5,15) + HK7*P(6,15) - HK8*P(4,15) + HK9*P(2,15));
Kfusion(16) = HK20*(HK10*P(0,16) + HK11*P(1,16) - HK12*P(3,16) + HK5*P(5,16) + HK7*P(6,16) - HK8*P(4,16) + HK9*P(2,16));
Kfusion(17) = HK20*(HK10*P(0,17) + HK11*P(1,17) - HK12*P(3,17) + HK5*P(5,17) + HK7*P(6,17) - HK8*P(4,17) + HK9*P(2,17));
Kfusion(18) = HK20*(HK10*P(0,18) + HK11*P(1,18) - HK12*P(3,18) + HK5*P(5,18) + HK7*P(6,18) - HK8*P(4,18) + HK9*P(2,18));
Kfusion(19) = HK20*(HK10*P(0,19) + HK11*P(1,19) - HK12*P(3,19) + HK5*P(5,19) + HK7*P(6,19) - HK8*P(4,19) + HK9*P(2,19));
Kfusion(20) = HK20*(HK10*P(0,20) + HK11*P(1,20) - HK12*P(3,20) + HK5*P(5,20) + HK7*P(6,20) - HK8*P(4,20) + HK9*P(2,20));
Kfusion(21) = HK20*(HK10*P(0,21) + HK11*P(1,21) - HK12*P(3,21) + HK5*P(5,21) + HK7*P(6,21) - HK8*P(4,21) + HK9*P(2,21));
Kfusion(22) = HK20*(HK10*P(0,22) + HK11*P(1,22) - HK12*P(3,22) + HK5*P(5,22) + HK7*P(6,22) - HK8*P(4,22) + HK9*P(2,22));
Kfusion(23) = HK20*(HK10*P(0,23) + HK11*P(1,23) - HK12*P(3,23) + HK5*P(5,23) + HK7*P(6,23) - HK8*P(4,23) + HK9*P(2,23));


// axis 2
const float HK0 = q0*vd - q1*ve + q2*vn;
const float HK1 = q0*ve + q1*vd - q3*vn;
const float HK2 = q0*vn - q2*vd + q3*ve;
const float HK3 = q1*vn + q2*ve + q3*vd;
const float HK4 = q0*q2 + q1*q3;
const float HK5 = q0*q1 - q2*q3;
const float HK6 = (q0)*(q0) - (q1)*(q1) - (q2)*(q2) + (q3)*(q3);
const float HK7 = 2*HK4;
const float HK8 = 2*HK5;
const float HK9 = 2*HK3;
const float HK10 = 2*HK0;
const float HK11 = 2*HK1;
const float HK12 = 2*HK2;
const float HK13 = HK10*P(0,0) - HK11*P(0,1) + HK12*P(0,2) + HK6*P(0,6) + HK7*P(0,4) - HK8*P(0,5) + HK9*P(0,3);
const float HK14 = HK10*P(0,4) - HK11*P(1,4) + HK12*P(2,4) + HK6*P(4,6) + HK7*P(4,4) - HK8*P(4,5) + HK9*P(3,4);
const float HK15 = HK10*P(0,5) - HK11*P(1,5) + HK12*P(2,5) + HK6*P(5,6) + HK7*P(4,5) - HK8*P(5,5) + HK9*P(3,5);
const float HK16 = HK10*P(0,3) - HK11*P(1,3) + HK12*P(2,3) + HK6*P(3,6) + HK7*P(3,4) - HK8*P(3,5) + HK9*P(3,3);
const float HK17 = HK10*P(0,1) - HK11*P(1,1) + HK12*P(1,2) + HK6*P(1,6) + HK7*P(1,4) - HK8*P(1,5) + HK9*P(1,3);
const float HK18 = HK10*P(0,2) - HK11*P(1,2) + HK12*P(2,2) + HK6*P(2,6) + HK7*P(2,4) - HK8*P(2,5) + HK9*P(2,3);
const float HK19 = HK10*P(0,6) - HK11*P(1,6) + HK12*P(2,6) + HK6*P(6,6) + HK7*P(4,6) - HK8*P(5,6) + HK9*P(3,6);
const float HK20 = 1.0F/(HK10*HK13 - HK11*HK17 + HK12*HK18 + HK14*HK7 - HK15*HK8 + HK16*HK9 + HK19*HK6 + R_VEL);


H_VEL(0) = 2*HK0;
H_VEL(1) = -2*HK1;
H_VEL(2) = 2*HK2;
H_VEL(3) = 2*HK3;
H_VEL(4) = 2*HK4;
H_VEL(5) = -2*HK5;
H_VEL(6) = HK6;


Kfusion(0) = HK13*HK20;
Kfusion(1) = HK17*HK20;
Kfusion(2) = HK18*HK20;
Kfusion(3) = HK16*HK20;
Kfusion(4) = HK14*HK20;
Kfusion(5) = HK15*HK20;
Kfusion(6) = HK19*HK20;
Kfusion(7) = HK20*(HK10*P(0,7) - HK11*P(1,7) + HK12*P(2,7) + HK6*P(6,7) + HK7*P(4,7) - HK8*P(5,7) + HK9*P(3,7));
Kfusion(8) = HK20*(HK10*P(0,8) - HK11*P(1,8) + HK12*P(2,8) + HK6*P(6,8) + HK7*P(4,8) - HK8*P(5,8) + HK9*P(3,8));
Kfusion(9) = HK20*(HK10*P(0,9) - HK11*P(1,9) + HK12*P(2,9) + HK6*P(6,9) + HK7*P(4,9) - HK8*P(5,9) + HK9*P(3,9));
Kfusion(10) = HK20*(HK10*P(0,10) - HK11*P(1,10) + HK12*P(2,10) + HK6*P(6,10) + HK7*P(4,10) - HK8*P(5,10) + HK9*P(3,10));
Kfusion(11) = HK20*(HK10*P(0,11) - HK11*P(1,11) + HK12*P(2,11) + HK6*P(6,11) + HK7*P(4,11) - HK8*P(5,11) + HK9*P(3,11));
Kfusion(12) = HK20*(HK10*P(0,12) - HK11*P(1,12) + HK12*P(2,12) + HK6*P(6,12) + HK7*P(4,12) - HK8*P(5,12) + HK9*P(3,12));
Kfusion(13) = HK20*(HK10*P(0,13) - HK11*P(1,13) + HK12*P(2,13) + HK6*P(6,13) + HK7*P(4,13) - HK8*P(5,13) + HK9*P(3,13));
Kfusion(14) = HK20*(HK10*P(0,14) - HK11*P(1,14) + HK12*P(2,14) + HK6*P(6,14) + HK7*P(4,14) - HK8*P(5,14) + HK9*P(3,14));
Kfusion(15) = HK20*(HK10*P(0,15) - HK11*P(1,15) + HK12*P(2,15) + HK6*P(6,15) + HK7*P(4,15) - HK8*P(5,15) + HK9*P(3,15));
Kfusion(16) = HK20*(HK10*P(0,16) - HK11*P(1,16) + HK12*P(2,16) + HK6*P(6,16) + HK7*P(4,16) - HK8*P(5,16) + HK9*P(3,16));
Kfusion(17) = HK20*(HK10*P(0,17) - HK11*P(1,17) + HK12*P(2,17) + HK6*P(6,17) + HK7*P(4,17) - HK8*P(5,17) + HK9*P(3,17));
Kfusion(18) = HK20*(HK10*P(0,18) - HK11*P(1,18) + HK12*P(2,18) + HK6*P(6,18) + HK7*P(4,18) - HK8*P(5,18) + HK9*P(3,18));
Kfusion(19) = HK20*(HK10*P(0,19) - HK11*P(1,19) + HK12*P(2,19) + HK6*P(6,19) + HK7*P(4,19) - HK8*P(5,19) + HK9*P(3,19));
Kfusion(20) = HK20*(HK10*P(0,20) - HK11*P(1,20) + HK12*P(2,20) + HK6*P(6,20) + HK7*P(4,20) - HK8*P(5,20) + HK9*P(3,20));
Kfusion(21) = HK20*(HK10*P(0,21) - HK11*P(1,21) + HK12*P(2,21) + HK6*P(6,21) + HK7*P(4,21) - HK8*P(5,21) + HK9*P(3,21));
Kfusion(22) = HK20*(HK10*P(0,22) - HK11*P(1,22) + HK12*P(2,22) + HK6*P(6,22) + HK7*P(4,22) - HK8*P(5,22) + HK9*P(3,22));
Kfusion(23) = HK20*(HK10*P(0,23) - HK11*P(1,23) + HK12*P(2,23) + HK6*P(6,23) + HK7*P(4,23) - HK8*P(5,23) + HK9*P(3,23));


