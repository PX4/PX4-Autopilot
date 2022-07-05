// Sub Expressions
const float HK0 = q0*vn - q2*vd + q3*ve;
const float HK1 = 2*HK0;
const float HK2 = q1*vn + q2*ve + q3*vd;
const float HK3 = 2*HK2;
const float HK4 = q0*vd - q1*ve + q2*vn;
const float HK5 = q0*ve + q1*vd - q3*vn;
const float HK6 = 2*HK5;
const float HK7 = (q1)*(q1);
const float HK8 = (q2)*(q2);
const float HK9 = -HK8;
const float HK10 = (q0)*(q0);
const float HK11 = (q3)*(q3);
const float HK12 = HK10 - HK11;
const float HK13 = HK12 + HK7 + HK9;
const float HK14 = q0*q3;
const float HK15 = HK14 + q1*q2;
const float HK16 = q0*q2;
const float HK17 = HK16 - q1*q3;
const float HK18 = 2*HK15;
const float HK19 = 2*HK17;
const float HK20 = 2*HK2;
const float HK21 = 2*HK0;
const float HK22 = 2*HK4;
const float HK23 = HK22*P(0,2);
const float HK24 = 2*HK5;
const float HK25 = HK24*P(0,3);
const float HK26 = HK13*P(0,4) + HK18*P(0,5) - HK19*P(0,6) + HK20*P(0,1) + HK21*P(0,0) - HK23 + HK25;
const float HK27 = HK13*P(4,5) + HK18*P(5,5) - HK19*P(5,6) + HK20*P(1,5) + HK21*P(0,5) - HK22*P(2,5) + HK24*P(3,5);
const float HK28 = HK13*P(4,6) + HK18*P(5,6) - HK19*P(6,6) + HK20*P(1,6) + HK21*P(0,6) - HK22*P(2,6) + HK24*P(3,6);
const float HK29 = HK22*P(1,2);
const float HK30 = HK24*P(1,3);
const float HK31 = HK13*P(1,4) + HK18*P(1,5) - HK19*P(1,6) + HK20*P(1,1) + HK21*P(0,1) - HK29 + HK30;
const float HK32 = HK20*P(1,2);
const float HK33 = HK21*P(0,2);
const float HK34 = HK13*P(2,4) + HK18*P(2,5) - HK19*P(2,6) - HK22*P(2,2) + HK24*P(2,3) + HK32 + HK33;
const float HK35 = HK20*P(1,3);
const float HK36 = HK21*P(0,3);
const float HK37 = HK13*P(3,4) + HK18*P(3,5) - HK19*P(3,6) - HK22*P(2,3) + HK24*P(3,3) + HK35 + HK36;
const float HK38 = HK13*P(4,4) + HK18*P(4,5) - HK19*P(4,6) + HK20*P(1,4) + HK21*P(0,4) - HK22*P(2,4) + HK24*P(3,4);
const float HK39 = 1.0F/(HK13*HK38 + HK18*HK27 - HK19*HK28 + HK20*HK31 + HK21*HK26 - HK22*HK34 + HK24*HK37 + R_VEL);
const float HK40 = 2*HK4;
const float HK41 = HK14 - q1*q2;
const float HK42 = -HK7;
const float HK43 = HK12 + HK42 + HK8;
const float HK44 = q0*q1;
const float HK45 = HK44 + q2*q3;
const float HK46 = 2*HK45;
const float HK47 = 2*HK41;
const float HK48 = HK22*P(0,1);
const float HK49 = HK20*P(0,2) + HK24*P(0,0) - HK36 + HK43*P(0,5) + HK46*P(0,6) - HK47*P(0,4) + HK48;
const float HK50 = HK20*P(2,6) - HK21*P(3,6) + HK22*P(1,6) + HK24*P(0,6) + HK43*P(5,6) + HK46*P(6,6) - HK47*P(4,6);
const float HK51 = HK20*P(2,4) - HK21*P(3,4) + HK22*P(1,4) + HK24*P(0,4) + HK43*P(4,5) + HK46*P(4,6) - HK47*P(4,4);
const float HK52 = HK21*P(2,3);
const float HK53 = HK20*P(2,2) + HK24*P(0,2) + HK29 + HK43*P(2,5) + HK46*P(2,6) - HK47*P(2,4) - HK52;
const float HK54 = HK24*P(0,1);
const float HK55 = -HK21*P(1,3) + HK22*P(1,1) + HK32 + HK43*P(1,5) + HK46*P(1,6) - HK47*P(1,4) + HK54;
const float HK56 = HK20*P(2,3);
const float HK57 = -HK21*P(3,3) + HK22*P(1,3) + HK25 + HK43*P(3,5) + HK46*P(3,6) - HK47*P(3,4) + HK56;
const float HK58 = HK20*P(2,5) - HK21*P(3,5) + HK22*P(1,5) + HK24*P(0,5) + HK43*P(5,5) + HK46*P(5,6) - HK47*P(4,5);
const float HK59 = 1.0F/(HK20*HK53 - HK21*HK57 + HK22*HK55 + HK24*HK49 + HK43*HK58 + HK46*HK50 - HK47*HK51 + R_VEL);
const float HK60 = HK16 + q1*q3;
const float HK61 = HK44 - q2*q3;
const float HK62 = HK10 + HK11 + HK42 + HK9;
const float HK63 = 2*HK60;
const float HK64 = 2*HK61;
const float HK65 = HK20*P(0,3) + HK22*P(0,0) + HK33 - HK54 + HK62*P(0,6) + HK63*P(0,4) - HK64*P(0,5);
const float HK66 = HK20*P(3,4) + HK21*P(2,4) + HK22*P(0,4) - HK24*P(1,4) + HK62*P(4,6) + HK63*P(4,4) - HK64*P(4,5);
const float HK67 = HK20*P(3,5) + HK21*P(2,5) + HK22*P(0,5) - HK24*P(1,5) + HK62*P(5,6) + HK63*P(4,5) - HK64*P(5,5);
const float HK68 = HK20*P(3,3) + HK22*P(0,3) - HK30 + HK52 + HK62*P(3,6) + HK63*P(3,4) - HK64*P(3,5);
const float HK69 = HK21*P(1,2) - HK24*P(1,1) + HK35 + HK48 + HK62*P(1,6) + HK63*P(1,4) - HK64*P(1,5);
const float HK70 = HK21*P(2,2) + HK23 - HK24*P(1,2) + HK56 + HK62*P(2,6) + HK63*P(2,4) - HK64*P(2,5);
const float HK71 = HK20*P(3,6) + HK21*P(2,6) + HK22*P(0,6) - HK24*P(1,6) + HK62*P(6,6) + HK63*P(4,6) - HK64*P(5,6);
const float HK72 = 1.0F/(HK20*HK68 + HK21*HK70 + HK22*HK65 - HK24*HK69 + HK62*HK71 + HK63*HK66 - HK64*HK67 + R_VEL);


// Observation Jacobians - axis 0
Hfusion.at<0>() = HK1;
Hfusion.at<1>() = HK3;
Hfusion.at<2>() = -2*HK4;
Hfusion.at<3>() = HK6;
Hfusion.at<4>() = HK13;
Hfusion.at<5>() = 2*HK15;
Hfusion.at<6>() = -2*HK17;


// Kalman gains - axis 0
Kfusion(0) = HK26*HK39;
Kfusion(1) = HK31*HK39;
Kfusion(2) = HK34*HK39;
Kfusion(3) = HK37*HK39;
Kfusion(4) = HK38*HK39;
Kfusion(5) = HK27*HK39;
Kfusion(6) = HK28*HK39;
Kfusion(7) = HK39*(HK13*P(4,7) + HK18*P(5,7) - HK19*P(6,7) + HK20*P(1,7) + HK21*P(0,7) - HK22*P(2,7) + HK24*P(3,7));
Kfusion(8) = HK39*(HK13*P(4,8) + HK18*P(5,8) - HK19*P(6,8) + HK20*P(1,8) + HK21*P(0,8) - HK22*P(2,8) + HK24*P(3,8));
Kfusion(9) = HK39*(HK13*P(4,9) + HK18*P(5,9) - HK19*P(6,9) + HK20*P(1,9) + HK21*P(0,9) - HK22*P(2,9) + HK24*P(3,9));
Kfusion(10) = HK39*(HK13*P(4,10) + HK18*P(5,10) - HK19*P(6,10) + HK20*P(1,10) + HK21*P(0,10) - HK22*P(2,10) + HK24*P(3,10));
Kfusion(11) = HK39*(HK13*P(4,11) + HK18*P(5,11) - HK19*P(6,11) + HK20*P(1,11) + HK21*P(0,11) - HK22*P(2,11) + HK24*P(3,11));
Kfusion(12) = HK39*(HK13*P(4,12) + HK18*P(5,12) - HK19*P(6,12) + HK20*P(1,12) + HK21*P(0,12) - HK22*P(2,12) + HK24*P(3,12));
Kfusion(13) = HK39*(HK13*P(4,13) + HK18*P(5,13) - HK19*P(6,13) + HK20*P(1,13) + HK21*P(0,13) - HK22*P(2,13) + HK24*P(3,13));
Kfusion(14) = HK39*(HK13*P(4,14) + HK18*P(5,14) - HK19*P(6,14) + HK20*P(1,14) + HK21*P(0,14) - HK22*P(2,14) + HK24*P(3,14));
Kfusion(15) = HK39*(HK13*P(4,15) + HK18*P(5,15) - HK19*P(6,15) + HK20*P(1,15) + HK21*P(0,15) - HK22*P(2,15) + HK24*P(3,15));
Kfusion(16) = HK39*(HK13*P(4,16) + HK18*P(5,16) - HK19*P(6,16) + HK20*P(1,16) + HK21*P(0,16) - HK22*P(2,16) + HK24*P(3,16));
Kfusion(17) = HK39*(HK13*P(4,17) + HK18*P(5,17) - HK19*P(6,17) + HK20*P(1,17) + HK21*P(0,17) - HK22*P(2,17) + HK24*P(3,17));
Kfusion(18) = HK39*(HK13*P(4,18) + HK18*P(5,18) - HK19*P(6,18) + HK20*P(1,18) + HK21*P(0,18) - HK22*P(2,18) + HK24*P(3,18));
Kfusion(19) = HK39*(HK13*P(4,19) + HK18*P(5,19) - HK19*P(6,19) + HK20*P(1,19) + HK21*P(0,19) - HK22*P(2,19) + HK24*P(3,19));
Kfusion(20) = HK39*(HK13*P(4,20) + HK18*P(5,20) - HK19*P(6,20) + HK20*P(1,20) + HK21*P(0,20) - HK22*P(2,20) + HK24*P(3,20));
Kfusion(21) = HK39*(HK13*P(4,21) + HK18*P(5,21) - HK19*P(6,21) + HK20*P(1,21) + HK21*P(0,21) - HK22*P(2,21) + HK24*P(3,21));
Kfusion(22) = HK39*(HK13*P(4,22) + HK18*P(5,22) - HK19*P(6,22) + HK20*P(1,22) + HK21*P(0,22) - HK22*P(2,22) + HK24*P(3,22));
Kfusion(23) = HK39*(HK13*P(4,23) + HK18*P(5,23) - HK19*P(6,23) + HK20*P(1,23) + HK21*P(0,23) - HK22*P(2,23) + HK24*P(3,23));


// Observation Jacobians - axis 1
Hfusion.at<0>() = HK6;
Hfusion.at<1>() = HK40;
Hfusion.at<2>() = HK3;
Hfusion.at<3>() = -2*HK0;
Hfusion.at<4>() = -2*HK41;
Hfusion.at<5>() = HK43;
Hfusion.at<6>() = 2*HK45;


// Kalman gains - axis 1
Kfusion(0) = HK49*HK59;
Kfusion(1) = HK55*HK59;
Kfusion(2) = HK53*HK59;
Kfusion(3) = HK57*HK59;
Kfusion(4) = HK51*HK59;
Kfusion(5) = HK58*HK59;
Kfusion(6) = HK50*HK59;
Kfusion(7) = HK59*(HK20*P(2,7) - HK21*P(3,7) + HK22*P(1,7) + HK24*P(0,7) + HK43*P(5,7) + HK46*P(6,7) - HK47*P(4,7));
Kfusion(8) = HK59*(HK20*P(2,8) - HK21*P(3,8) + HK22*P(1,8) + HK24*P(0,8) + HK43*P(5,8) + HK46*P(6,8) - HK47*P(4,8));
Kfusion(9) = HK59*(HK20*P(2,9) - HK21*P(3,9) + HK22*P(1,9) + HK24*P(0,9) + HK43*P(5,9) + HK46*P(6,9) - HK47*P(4,9));
Kfusion(10) = HK59*(HK20*P(2,10) - HK21*P(3,10) + HK22*P(1,10) + HK24*P(0,10) + HK43*P(5,10) + HK46*P(6,10) - HK47*P(4,10));
Kfusion(11) = HK59*(HK20*P(2,11) - HK21*P(3,11) + HK22*P(1,11) + HK24*P(0,11) + HK43*P(5,11) + HK46*P(6,11) - HK47*P(4,11));
Kfusion(12) = HK59*(HK20*P(2,12) - HK21*P(3,12) + HK22*P(1,12) + HK24*P(0,12) + HK43*P(5,12) + HK46*P(6,12) - HK47*P(4,12));
Kfusion(13) = HK59*(HK20*P(2,13) - HK21*P(3,13) + HK22*P(1,13) + HK24*P(0,13) + HK43*P(5,13) + HK46*P(6,13) - HK47*P(4,13));
Kfusion(14) = HK59*(HK20*P(2,14) - HK21*P(3,14) + HK22*P(1,14) + HK24*P(0,14) + HK43*P(5,14) + HK46*P(6,14) - HK47*P(4,14));
Kfusion(15) = HK59*(HK20*P(2,15) - HK21*P(3,15) + HK22*P(1,15) + HK24*P(0,15) + HK43*P(5,15) + HK46*P(6,15) - HK47*P(4,15));
Kfusion(16) = HK59*(HK20*P(2,16) - HK21*P(3,16) + HK22*P(1,16) + HK24*P(0,16) + HK43*P(5,16) + HK46*P(6,16) - HK47*P(4,16));
Kfusion(17) = HK59*(HK20*P(2,17) - HK21*P(3,17) + HK22*P(1,17) + HK24*P(0,17) + HK43*P(5,17) + HK46*P(6,17) - HK47*P(4,17));
Kfusion(18) = HK59*(HK20*P(2,18) - HK21*P(3,18) + HK22*P(1,18) + HK24*P(0,18) + HK43*P(5,18) + HK46*P(6,18) - HK47*P(4,18));
Kfusion(19) = HK59*(HK20*P(2,19) - HK21*P(3,19) + HK22*P(1,19) + HK24*P(0,19) + HK43*P(5,19) + HK46*P(6,19) - HK47*P(4,19));
Kfusion(20) = HK59*(HK20*P(2,20) - HK21*P(3,20) + HK22*P(1,20) + HK24*P(0,20) + HK43*P(5,20) + HK46*P(6,20) - HK47*P(4,20));
Kfusion(21) = HK59*(HK20*P(2,21) - HK21*P(3,21) + HK22*P(1,21) + HK24*P(0,21) + HK43*P(5,21) + HK46*P(6,21) - HK47*P(4,21));
Kfusion(22) = HK59*(HK20*P(2,22) - HK21*P(3,22) + HK22*P(1,22) + HK24*P(0,22) + HK43*P(5,22) + HK46*P(6,22) - HK47*P(4,22));
Kfusion(23) = HK59*(HK20*P(2,23) - HK21*P(3,23) + HK22*P(1,23) + HK24*P(0,23) + HK43*P(5,23) + HK46*P(6,23) - HK47*P(4,23));


// Observation Jacobians - axis 2
Hfusion.at<0>() = HK40;
Hfusion.at<1>() = -2*HK5;
Hfusion.at<2>() = HK1;
Hfusion.at<3>() = HK3;
Hfusion.at<4>() = 2*HK60;
Hfusion.at<5>() = -2*HK61;
Hfusion.at<6>() = HK62;


// Kalman gains - axis 2
Kfusion(0) = HK65*HK72;
Kfusion(1) = HK69*HK72;
Kfusion(2) = HK70*HK72;
Kfusion(3) = HK68*HK72;
Kfusion(4) = HK66*HK72;
Kfusion(5) = HK67*HK72;
Kfusion(6) = HK71*HK72;
Kfusion(7) = HK72*(HK20*P(3,7) + HK21*P(2,7) + HK22*P(0,7) - HK24*P(1,7) + HK62*P(6,7) + HK63*P(4,7) - HK64*P(5,7));
Kfusion(8) = HK72*(HK20*P(3,8) + HK21*P(2,8) + HK22*P(0,8) - HK24*P(1,8) + HK62*P(6,8) + HK63*P(4,8) - HK64*P(5,8));
Kfusion(9) = HK72*(HK20*P(3,9) + HK21*P(2,9) + HK22*P(0,9) - HK24*P(1,9) + HK62*P(6,9) + HK63*P(4,9) - HK64*P(5,9));
Kfusion(10) = HK72*(HK20*P(3,10) + HK21*P(2,10) + HK22*P(0,10) - HK24*P(1,10) + HK62*P(6,10) + HK63*P(4,10) - HK64*P(5,10));
Kfusion(11) = HK72*(HK20*P(3,11) + HK21*P(2,11) + HK22*P(0,11) - HK24*P(1,11) + HK62*P(6,11) + HK63*P(4,11) - HK64*P(5,11));
Kfusion(12) = HK72*(HK20*P(3,12) + HK21*P(2,12) + HK22*P(0,12) - HK24*P(1,12) + HK62*P(6,12) + HK63*P(4,12) - HK64*P(5,12));
Kfusion(13) = HK72*(HK20*P(3,13) + HK21*P(2,13) + HK22*P(0,13) - HK24*P(1,13) + HK62*P(6,13) + HK63*P(4,13) - HK64*P(5,13));
Kfusion(14) = HK72*(HK20*P(3,14) + HK21*P(2,14) + HK22*P(0,14) - HK24*P(1,14) + HK62*P(6,14) + HK63*P(4,14) - HK64*P(5,14));
Kfusion(15) = HK72*(HK20*P(3,15) + HK21*P(2,15) + HK22*P(0,15) - HK24*P(1,15) + HK62*P(6,15) + HK63*P(4,15) - HK64*P(5,15));
Kfusion(16) = HK72*(HK20*P(3,16) + HK21*P(2,16) + HK22*P(0,16) - HK24*P(1,16) + HK62*P(6,16) + HK63*P(4,16) - HK64*P(5,16));
Kfusion(17) = HK72*(HK20*P(3,17) + HK21*P(2,17) + HK22*P(0,17) - HK24*P(1,17) + HK62*P(6,17) + HK63*P(4,17) - HK64*P(5,17));
Kfusion(18) = HK72*(HK20*P(3,18) + HK21*P(2,18) + HK22*P(0,18) - HK24*P(1,18) + HK62*P(6,18) + HK63*P(4,18) - HK64*P(5,18));
Kfusion(19) = HK72*(HK20*P(3,19) + HK21*P(2,19) + HK22*P(0,19) - HK24*P(1,19) + HK62*P(6,19) + HK63*P(4,19) - HK64*P(5,19));
Kfusion(20) = HK72*(HK20*P(3,20) + HK21*P(2,20) + HK22*P(0,20) - HK24*P(1,20) + HK62*P(6,20) + HK63*P(4,20) - HK64*P(5,20));
Kfusion(21) = HK72*(HK20*P(3,21) + HK21*P(2,21) + HK22*P(0,21) - HK24*P(1,21) + HK62*P(6,21) + HK63*P(4,21) - HK64*P(5,21));
Kfusion(22) = HK72*(HK20*P(3,22) + HK21*P(2,22) + HK22*P(0,22) - HK24*P(1,22) + HK62*P(6,22) + HK63*P(4,22) - HK64*P(5,22));
Kfusion(23) = HK72*(HK20*P(3,23) + HK21*P(2,23) + HK22*P(0,23) - HK24*P(1,23) + HK62*P(6,23) + HK63*P(4,23) - HK64*P(5,23));


