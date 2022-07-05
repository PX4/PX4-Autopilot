// Sub Expressions
const float HK0 = -magD*q2 + magE*q3 + magN*q0;
const float HK1 = 2*HK0;
const float HK2 = magD*q3 + magE*q2 + magN*q1;
const float HK3 = 2*HK2;
const float HK4 = magD*q0 - magE*q1 + magN*q2;
const float HK5 = magD*q1 + magE*q0 - magN*q3;
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
const float HK26 = HK13*P(0,16) + HK18*P(0,17) - HK19*P(0,18) + HK20*P(0,1) + HK21*P(0,0) - HK23 + HK25 + P(0,19);
const float HK27 = HK13*P(16,16) + HK18*P(16,17) - HK19*P(16,18) + HK20*P(1,16) + HK21*P(0,16) - HK22*P(2,16) + HK24*P(3,16) + P(16,19);
const float HK28 = HK13*P(16,18) + HK18*P(17,18) - HK19*P(18,18) + HK20*P(1,18) + HK21*P(0,18) - HK22*P(2,18) + HK24*P(3,18) + P(18,19);
const float HK29 = HK20*P(1,2);
const float HK30 = HK21*P(0,2);
const float HK31 = HK13*P(2,16) + HK18*P(2,17) - HK19*P(2,18) - HK22*P(2,2) + HK24*P(2,3) + HK29 + HK30 + P(2,19);
const float HK32 = HK13*P(16,17) + HK18*P(17,17) - HK19*P(17,18) + HK20*P(1,17) + HK21*P(0,17) - HK22*P(2,17) + HK24*P(3,17) + P(17,19);
const float HK33 = HK20*P(1,3);
const float HK34 = HK21*P(0,3);
const float HK35 = HK13*P(3,16) + HK18*P(3,17) - HK19*P(3,18) - HK22*P(2,3) + HK24*P(3,3) + HK33 + HK34 + P(3,19);
const float HK36 = HK22*P(1,2);
const float HK37 = HK24*P(1,3);
const float HK38 = HK13*P(1,16) + HK18*P(1,17) - HK19*P(1,18) + HK20*P(1,1) + HK21*P(0,1) - HK36 + HK37 + P(1,19);
const float HK39 = HK13*P(16,19) + HK18*P(17,19) - HK19*P(18,19) + HK20*P(1,19) + HK21*P(0,19) - HK22*P(2,19) + HK24*P(3,19) + P(19,19);
const float HK40 = 1.0F/(HK13*HK27 + HK18*HK32 - HK19*HK28 + HK20*HK38 + HK21*HK26 - HK22*HK31 + HK24*HK35 + HK39 + R_MAG);
const float HK41 = 2*HK4;
const float HK42 = HK14 - q1*q2;
const float HK43 = -HK7;
const float HK44 = HK12 + HK43 + HK8;
const float HK45 = q0*q1;
const float HK46 = HK45 + q2*q3;
const float HK47 = 2*HK46;
const float HK48 = 2*HK42;
const float HK49 = HK22*P(0,1);
const float HK50 = HK20*P(0,2) + HK24*P(0,0) - HK34 + HK44*P(0,17) + HK47*P(0,18) - HK48*P(0,16) + HK49 + P(0,20);
const float HK51 = HK20*P(2,17) - HK21*P(3,17) + HK22*P(1,17) + HK24*P(0,17) + HK44*P(17,17) + HK47*P(17,18) - HK48*P(16,17) + P(17,20);
const float HK52 = HK20*P(2,16) - HK21*P(3,16) + HK22*P(1,16) + HK24*P(0,16) + HK44*P(16,17) + HK47*P(16,18) - HK48*P(16,16) + P(16,20);
const float HK53 = HK20*P(2,3);
const float HK54 = -HK21*P(3,3) + HK22*P(1,3) + HK25 + HK44*P(3,17) + HK47*P(3,18) - HK48*P(3,16) + HK53 + P(3,20);
const float HK55 = HK20*P(2,18) - HK21*P(3,18) + HK22*P(1,18) + HK24*P(0,18) + HK44*P(17,18) + HK47*P(18,18) - HK48*P(16,18) + P(18,20);
const float HK56 = HK24*P(0,1);
const float HK57 = -HK21*P(1,3) + HK22*P(1,1) + HK29 + HK44*P(1,17) + HK47*P(1,18) - HK48*P(1,16) + HK56 + P(1,20);
const float HK58 = HK21*P(2,3);
const float HK59 = HK20*P(2,2) + HK24*P(0,2) + HK36 + HK44*P(2,17) + HK47*P(2,18) - HK48*P(2,16) - HK58 + P(2,20);
const float HK60 = HK20*P(2,20) - HK21*P(3,20) + HK22*P(1,20) + HK24*P(0,20) + HK44*P(17,20) + HK47*P(18,20) - HK48*P(16,20) + P(20,20);
const float HK61 = 1.0F/(HK20*HK59 - HK21*HK54 + HK22*HK57 + HK24*HK50 + HK44*HK51 + HK47*HK55 - HK48*HK52 + HK60 + R_MAG);
const float HK62 = HK16 + q1*q3;
const float HK63 = HK45 - q2*q3;
const float HK64 = HK10 + HK11 + HK43 + HK9;
const float HK65 = 2*HK62;
const float HK66 = 2*HK63;
const float HK67 = HK20*P(0,3) + HK22*P(0,0) + HK30 - HK56 + HK64*P(0,18) + HK65*P(0,16) - HK66*P(0,17) + P(0,21);
const float HK68 = HK20*P(3,18) + HK21*P(2,18) + HK22*P(0,18) - HK24*P(1,18) + HK64*P(18,18) + HK65*P(16,18) - HK66*P(17,18) + P(18,21);
const float HK69 = HK20*P(3,17) + HK21*P(2,17) + HK22*P(0,17) - HK24*P(1,17) + HK64*P(17,18) + HK65*P(16,17) - HK66*P(17,17) + P(17,21);
const float HK70 = HK21*P(1,2) - HK24*P(1,1) + HK33 + HK49 + HK64*P(1,18) + HK65*P(1,16) - HK66*P(1,17) + P(1,21);
const float HK71 = HK20*P(3,16) + HK21*P(2,16) + HK22*P(0,16) - HK24*P(1,16) + HK64*P(16,18) + HK65*P(16,16) - HK66*P(16,17) + P(16,21);
const float HK72 = HK20*P(3,3) + HK22*P(0,3) - HK37 + HK58 + HK64*P(3,18) + HK65*P(3,16) - HK66*P(3,17) + P(3,21);
const float HK73 = HK21*P(2,2) + HK23 - HK24*P(1,2) + HK53 + HK64*P(2,18) + HK65*P(2,16) - HK66*P(2,17) + P(2,21);
const float HK74 = HK20*P(3,21) + HK21*P(2,21) + HK22*P(0,21) - HK24*P(1,21) + HK64*P(18,21) + HK65*P(16,21) - HK66*P(17,21) + P(21,21);
const float HK75 = 1.0F/(HK20*HK72 + HK21*HK73 + HK22*HK67 - HK24*HK70 + HK64*HK68 + HK65*HK71 - HK66*HK69 + HK74 + R_MAG);


// Observation Jacobians - axis 0
Hfusion.at<0>() = HK1;
Hfusion.at<1>() = HK3;
Hfusion.at<2>() = -2*HK4;
Hfusion.at<3>() = HK6;
Hfusion.at<16>() = HK13;
Hfusion.at<17>() = 2*HK15;
Hfusion.at<18>() = -2*HK17;
Hfusion.at<19>() = 1;


// Kalman gains - axis 0
Kfusion(0) = HK26*HK40;
Kfusion(1) = HK38*HK40;
Kfusion(2) = HK31*HK40;
Kfusion(3) = HK35*HK40;
Kfusion(4) = HK40*(HK13*P(4,16) + HK18*P(4,17) - HK19*P(4,18) + HK20*P(1,4) + HK21*P(0,4) - HK22*P(2,4) + HK24*P(3,4) + P(4,19));
Kfusion(5) = HK40*(HK13*P(5,16) + HK18*P(5,17) - HK19*P(5,18) + HK20*P(1,5) + HK21*P(0,5) - HK22*P(2,5) + HK24*P(3,5) + P(5,19));
Kfusion(6) = HK40*(HK13*P(6,16) + HK18*P(6,17) - HK19*P(6,18) + HK20*P(1,6) + HK21*P(0,6) - HK22*P(2,6) + HK24*P(3,6) + P(6,19));
Kfusion(7) = HK40*(HK13*P(7,16) + HK18*P(7,17) - HK19*P(7,18) + HK20*P(1,7) + HK21*P(0,7) - HK22*P(2,7) + HK24*P(3,7) + P(7,19));
Kfusion(8) = HK40*(HK13*P(8,16) + HK18*P(8,17) - HK19*P(8,18) + HK20*P(1,8) + HK21*P(0,8) - HK22*P(2,8) + HK24*P(3,8) + P(8,19));
Kfusion(9) = HK40*(HK13*P(9,16) + HK18*P(9,17) - HK19*P(9,18) + HK20*P(1,9) + HK21*P(0,9) - HK22*P(2,9) + HK24*P(3,9) + P(9,19));
Kfusion(10) = HK40*(HK13*P(10,16) + HK18*P(10,17) - HK19*P(10,18) + HK20*P(1,10) + HK21*P(0,10) - HK22*P(2,10) + HK24*P(3,10) + P(10,19));
Kfusion(11) = HK40*(HK13*P(11,16) + HK18*P(11,17) - HK19*P(11,18) + HK20*P(1,11) + HK21*P(0,11) - HK22*P(2,11) + HK24*P(3,11) + P(11,19));
Kfusion(12) = HK40*(HK13*P(12,16) + HK18*P(12,17) - HK19*P(12,18) + HK20*P(1,12) + HK21*P(0,12) - HK22*P(2,12) + HK24*P(3,12) + P(12,19));
Kfusion(13) = HK40*(HK13*P(13,16) + HK18*P(13,17) - HK19*P(13,18) + HK20*P(1,13) + HK21*P(0,13) - HK22*P(2,13) + HK24*P(3,13) + P(13,19));
Kfusion(14) = HK40*(HK13*P(14,16) + HK18*P(14,17) - HK19*P(14,18) + HK20*P(1,14) + HK21*P(0,14) - HK22*P(2,14) + HK24*P(3,14) + P(14,19));
Kfusion(15) = HK40*(HK13*P(15,16) + HK18*P(15,17) - HK19*P(15,18) + HK20*P(1,15) + HK21*P(0,15) - HK22*P(2,15) + HK24*P(3,15) + P(15,19));
Kfusion(16) = HK27*HK40;
Kfusion(17) = HK32*HK40;
Kfusion(18) = HK28*HK40;
Kfusion(19) = HK39*HK40;
Kfusion(20) = HK40*(HK13*P(16,20) + HK18*P(17,20) - HK19*P(18,20) + HK20*P(1,20) + HK21*P(0,20) - HK22*P(2,20) + HK24*P(3,20) + P(19,20));
Kfusion(21) = HK40*(HK13*P(16,21) + HK18*P(17,21) - HK19*P(18,21) + HK20*P(1,21) + HK21*P(0,21) - HK22*P(2,21) + HK24*P(3,21) + P(19,21));
Kfusion(22) = HK40*(HK13*P(16,22) + HK18*P(17,22) - HK19*P(18,22) + HK20*P(1,22) + HK21*P(0,22) - HK22*P(2,22) + HK24*P(3,22) + P(19,22));
Kfusion(23) = HK40*(HK13*P(16,23) + HK18*P(17,23) - HK19*P(18,23) + HK20*P(1,23) + HK21*P(0,23) - HK22*P(2,23) + HK24*P(3,23) + P(19,23));


// Observation Jacobians - axis 1
Hfusion.at<0>() = HK6;
Hfusion.at<1>() = HK41;
Hfusion.at<2>() = HK3;
Hfusion.at<3>() = -2*HK0;
Hfusion.at<16>() = -2*HK42;
Hfusion.at<17>() = HK44;
Hfusion.at<18>() = 2*HK46;
Hfusion.at<20>() = 1;


// Kalman gains - axis 1
Kfusion(0) = HK50*HK61;
Kfusion(1) = HK57*HK61;
Kfusion(2) = HK59*HK61;
Kfusion(3) = HK54*HK61;
Kfusion(4) = HK61*(HK20*P(2,4) - HK21*P(3,4) + HK22*P(1,4) + HK24*P(0,4) + HK44*P(4,17) + HK47*P(4,18) - HK48*P(4,16) + P(4,20));
Kfusion(5) = HK61*(HK20*P(2,5) - HK21*P(3,5) + HK22*P(1,5) + HK24*P(0,5) + HK44*P(5,17) + HK47*P(5,18) - HK48*P(5,16) + P(5,20));
Kfusion(6) = HK61*(HK20*P(2,6) - HK21*P(3,6) + HK22*P(1,6) + HK24*P(0,6) + HK44*P(6,17) + HK47*P(6,18) - HK48*P(6,16) + P(6,20));
Kfusion(7) = HK61*(HK20*P(2,7) - HK21*P(3,7) + HK22*P(1,7) + HK24*P(0,7) + HK44*P(7,17) + HK47*P(7,18) - HK48*P(7,16) + P(7,20));
Kfusion(8) = HK61*(HK20*P(2,8) - HK21*P(3,8) + HK22*P(1,8) + HK24*P(0,8) + HK44*P(8,17) + HK47*P(8,18) - HK48*P(8,16) + P(8,20));
Kfusion(9) = HK61*(HK20*P(2,9) - HK21*P(3,9) + HK22*P(1,9) + HK24*P(0,9) + HK44*P(9,17) + HK47*P(9,18) - HK48*P(9,16) + P(9,20));
Kfusion(10) = HK61*(HK20*P(2,10) - HK21*P(3,10) + HK22*P(1,10) + HK24*P(0,10) + HK44*P(10,17) + HK47*P(10,18) - HK48*P(10,16) + P(10,20));
Kfusion(11) = HK61*(HK20*P(2,11) - HK21*P(3,11) + HK22*P(1,11) + HK24*P(0,11) + HK44*P(11,17) + HK47*P(11,18) - HK48*P(11,16) + P(11,20));
Kfusion(12) = HK61*(HK20*P(2,12) - HK21*P(3,12) + HK22*P(1,12) + HK24*P(0,12) + HK44*P(12,17) + HK47*P(12,18) - HK48*P(12,16) + P(12,20));
Kfusion(13) = HK61*(HK20*P(2,13) - HK21*P(3,13) + HK22*P(1,13) + HK24*P(0,13) + HK44*P(13,17) + HK47*P(13,18) - HK48*P(13,16) + P(13,20));
Kfusion(14) = HK61*(HK20*P(2,14) - HK21*P(3,14) + HK22*P(1,14) + HK24*P(0,14) + HK44*P(14,17) + HK47*P(14,18) - HK48*P(14,16) + P(14,20));
Kfusion(15) = HK61*(HK20*P(2,15) - HK21*P(3,15) + HK22*P(1,15) + HK24*P(0,15) + HK44*P(15,17) + HK47*P(15,18) - HK48*P(15,16) + P(15,20));
Kfusion(16) = HK52*HK61;
Kfusion(17) = HK51*HK61;
Kfusion(18) = HK55*HK61;
Kfusion(19) = HK61*(HK20*P(2,19) - HK21*P(3,19) + HK22*P(1,19) + HK24*P(0,19) + HK44*P(17,19) + HK47*P(18,19) - HK48*P(16,19) + P(19,20));
Kfusion(20) = HK60*HK61;
Kfusion(21) = HK61*(HK20*P(2,21) - HK21*P(3,21) + HK22*P(1,21) + HK24*P(0,21) + HK44*P(17,21) + HK47*P(18,21) - HK48*P(16,21) + P(20,21));
Kfusion(22) = HK61*(HK20*P(2,22) - HK21*P(3,22) + HK22*P(1,22) + HK24*P(0,22) + HK44*P(17,22) + HK47*P(18,22) - HK48*P(16,22) + P(20,22));
Kfusion(23) = HK61*(HK20*P(2,23) - HK21*P(3,23) + HK22*P(1,23) + HK24*P(0,23) + HK44*P(17,23) + HK47*P(18,23) - HK48*P(16,23) + P(20,23));


// Observation Jacobians - axis 2
Hfusion.at<0>() = HK41;
Hfusion.at<1>() = -2*HK5;
Hfusion.at<2>() = HK1;
Hfusion.at<3>() = HK3;
Hfusion.at<16>() = 2*HK62;
Hfusion.at<17>() = -2*HK63;
Hfusion.at<18>() = HK64;
Hfusion.at<21>() = 1;


// Kalman gains - axis 2
Kfusion(0) = HK67*HK75;
Kfusion(1) = HK70*HK75;
Kfusion(2) = HK73*HK75;
Kfusion(3) = HK72*HK75;
Kfusion(4) = HK75*(HK20*P(3,4) + HK21*P(2,4) + HK22*P(0,4) - HK24*P(1,4) + HK64*P(4,18) + HK65*P(4,16) - HK66*P(4,17) + P(4,21));
Kfusion(5) = HK75*(HK20*P(3,5) + HK21*P(2,5) + HK22*P(0,5) - HK24*P(1,5) + HK64*P(5,18) + HK65*P(5,16) - HK66*P(5,17) + P(5,21));
Kfusion(6) = HK75*(HK20*P(3,6) + HK21*P(2,6) + HK22*P(0,6) - HK24*P(1,6) + HK64*P(6,18) + HK65*P(6,16) - HK66*P(6,17) + P(6,21));
Kfusion(7) = HK75*(HK20*P(3,7) + HK21*P(2,7) + HK22*P(0,7) - HK24*P(1,7) + HK64*P(7,18) + HK65*P(7,16) - HK66*P(7,17) + P(7,21));
Kfusion(8) = HK75*(HK20*P(3,8) + HK21*P(2,8) + HK22*P(0,8) - HK24*P(1,8) + HK64*P(8,18) + HK65*P(8,16) - HK66*P(8,17) + P(8,21));
Kfusion(9) = HK75*(HK20*P(3,9) + HK21*P(2,9) + HK22*P(0,9) - HK24*P(1,9) + HK64*P(9,18) + HK65*P(9,16) - HK66*P(9,17) + P(9,21));
Kfusion(10) = HK75*(HK20*P(3,10) + HK21*P(2,10) + HK22*P(0,10) - HK24*P(1,10) + HK64*P(10,18) + HK65*P(10,16) - HK66*P(10,17) + P(10,21));
Kfusion(11) = HK75*(HK20*P(3,11) + HK21*P(2,11) + HK22*P(0,11) - HK24*P(1,11) + HK64*P(11,18) + HK65*P(11,16) - HK66*P(11,17) + P(11,21));
Kfusion(12) = HK75*(HK20*P(3,12) + HK21*P(2,12) + HK22*P(0,12) - HK24*P(1,12) + HK64*P(12,18) + HK65*P(12,16) - HK66*P(12,17) + P(12,21));
Kfusion(13) = HK75*(HK20*P(3,13) + HK21*P(2,13) + HK22*P(0,13) - HK24*P(1,13) + HK64*P(13,18) + HK65*P(13,16) - HK66*P(13,17) + P(13,21));
Kfusion(14) = HK75*(HK20*P(3,14) + HK21*P(2,14) + HK22*P(0,14) - HK24*P(1,14) + HK64*P(14,18) + HK65*P(14,16) - HK66*P(14,17) + P(14,21));
Kfusion(15) = HK75*(HK20*P(3,15) + HK21*P(2,15) + HK22*P(0,15) - HK24*P(1,15) + HK64*P(15,18) + HK65*P(15,16) - HK66*P(15,17) + P(15,21));
Kfusion(16) = HK71*HK75;
Kfusion(17) = HK69*HK75;
Kfusion(18) = HK68*HK75;
Kfusion(19) = HK75*(HK20*P(3,19) + HK21*P(2,19) + HK22*P(0,19) - HK24*P(1,19) + HK64*P(18,19) + HK65*P(16,19) - HK66*P(17,19) + P(19,21));
Kfusion(20) = HK75*(HK20*P(3,20) + HK21*P(2,20) + HK22*P(0,20) - HK24*P(1,20) + HK64*P(18,20) + HK65*P(16,20) - HK66*P(17,20) + P(20,21));
Kfusion(21) = HK74*HK75;
Kfusion(22) = HK75*(HK20*P(3,22) + HK21*P(2,22) + HK22*P(0,22) - HK24*P(1,22) + HK64*P(18,22) + HK65*P(16,22) - HK66*P(17,22) + P(21,22));
Kfusion(23) = HK75*(HK20*P(3,23) + HK21*P(2,23) + HK22*P(0,23) - HK24*P(1,23) + HK64*P(18,23) + HK65*P(16,23) - HK66*P(17,23) + P(21,23));


