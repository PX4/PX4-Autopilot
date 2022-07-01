// Sub Expressions
const float HK0 = q2*vd;
const float HK1 = ve - vwe;
const float HK2 = HK1*q3;
const float HK3 = HK0 - HK2;
const float HK4 = 2*Kaccx;
const float HK5 = q3*vd;
const float HK6 = HK1*q2 + HK5;
const float HK7 = q0*vd;
const float HK8 = HK1*q1;
const float HK9 = vn - vwn;
const float HK10 = HK9*q2;
const float HK11 = 2*HK10 + HK7 - HK8;
const float HK12 = q1*vd;
const float HK13 = HK9*q3;
const float HK14 = HK1*q0 + HK12 - 2*HK13;
const float HK15 = 2*(q3)*(q3) - 1;
const float HK16 = HK15 + 2*(q2)*(q2);
const float HK17 = HK16*Kaccx;
const float HK18 = q0*q3;
const float HK19 = q1*q2;
const float HK20 = HK18 + HK19;
const float HK21 = HK20*HK4;
const float HK22 = q0*q2 - q1*q3;
const float HK23 = 2*HK20;
const float HK24 = HK23*P(0,23);
const float HK25 = 2*HK22;
const float HK26 = 2*HK3;
const float HK27 = 2*HK11;
const float HK28 = HK23*P(23,23);
const float HK29 = HK23*P(5,23);
const float HK30 = -2*HK22;
const float HK31 = 2*HK6;
const float HK32 = -HK16;
const float HK33 = -2*HK3;
const float HK34 = 2*HK14;
const float HK35 = -2*HK11;
const float HK36 = (Kaccx)*(Kaccx);
const float HK37 = HK23*HK36;
const float HK38 = -HK29;
const float HK39 = HK23*P(6,23);
const float HK40 = HK23*P(1,23);
const float HK41 = HK23*P(4,23);
const float HK42 = HK16*P(4,22);
const float HK43 = HK16*HK36;
const float HK44 = HK23*P(22,23);
const float HK45 = HK23*P(3,23);
const float HK46 = HK23*P(2,23);
const float HK47 = Kaccx/(HK25*HK36*(HK16*P(6,22) + HK23*P(5,6) + HK30*P(6,6) + HK31*P(1,6) + HK32*P(4,6) + HK33*P(0,6) + HK34*P(3,6) + HK35*P(2,6) - HK39) + HK26*HK36*(HK16*P(0,22) + HK23*P(0,5) - HK24 + HK30*P(0,6) + HK31*P(0,1) + HK32*P(0,4) + HK33*P(0,0) + HK34*P(0,3) + HK35*P(0,2)) + HK27*HK36*(HK16*P(2,22) + HK23*P(2,5) + HK30*P(2,6) + HK31*P(1,2) + HK32*P(2,4) + HK33*P(0,2) + HK34*P(2,3) + HK35*P(2,2) - HK46) - HK31*HK36*(HK16*P(1,22) + HK23*P(1,5) + HK30*P(1,6) + HK31*P(1,1) + HK32*P(1,4) + HK33*P(0,1) + HK34*P(1,3) + HK35*P(1,2) - HK40) - HK34*HK36*(HK16*P(3,22) + HK23*P(3,5) + HK30*P(3,6) + HK31*P(1,3) + HK32*P(3,4) + HK33*P(0,3) + HK34*P(3,3) + HK35*P(2,3) - HK45) + HK37*(HK16*P(22,23) - HK28 + HK29 + HK30*P(6,23) + HK31*P(1,23) + HK32*P(4,23) + HK33*P(0,23) + HK34*P(3,23) + HK35*P(2,23)) - HK37*(HK16*P(5,22) + HK23*P(5,5) + HK30*P(5,6) + HK31*P(1,5) + HK32*P(4,5) + HK33*P(0,5) + HK34*P(3,5) + HK35*P(2,5) + HK38) - HK43*(HK16*P(22,22) + HK23*P(5,22) + HK30*P(6,22) + HK31*P(1,22) + HK32*P(4,22) + HK33*P(0,22) + HK34*P(3,22) + HK35*P(2,22) - HK44) + HK43*(HK23*P(4,5) + HK30*P(4,6) + HK31*P(1,4) + HK32*P(4,4) + HK33*P(0,4) + HK34*P(3,4) + HK35*P(2,4) - HK41 + HK42) - R_ACC);
const float HK48 = HK12 - HK13;
const float HK49 = 2*Kaccy;
const float HK50 = HK10 + HK7 - 2*HK8;
const float HK51 = HK5 + HK9*q1;
const float HK52 = -HK0 + 2*HK2 + HK9*q0;
const float HK53 = HK18 - HK19;
const float HK54 = HK49*HK53;
const float HK55 = HK15 + 2*(q1)*(q1);
const float HK56 = HK55*Kaccy;
const float HK57 = q0*q1 + q2*q3;
const float HK58 = 2*HK52;
const float HK59 = 2*HK53;
const float HK60 = 2*HK48;
const float HK61 = 2*HK50;
const float HK62 = 2*HK51;
const float HK63 = 2*HK57;
const float HK64 = HK55*P(0,23) + HK59*P(0,22) + HK60*P(0,0) + HK61*P(0,1) + HK62*P(0,2) + HK63*P(0,6);
const float HK65 = (Kaccy)*(Kaccy);
const float HK66 = -HK55;
const float HK67 = -2*HK52;
const float HK68 = -2*HK53;
const float HK69 = HK55*P(6,23) + HK59*P(6,22) + HK60*P(0,6) + HK61*P(1,6) + HK62*P(2,6) + HK63*P(6,6);
const float HK70 = HK55*P(22,23) + HK59*P(22,22) + HK60*P(0,22) + HK61*P(1,22) + HK62*P(2,22) + HK63*P(6,22);
const float HK71 = HK59*P(4,22);
const float HK72 = HK55*P(4,23) + HK60*P(0,4) + HK61*P(1,4) + HK62*P(2,4) + HK63*P(4,6) + HK71;
const float HK73 = HK55*P(2,23) + HK59*P(2,22) + HK60*P(0,2) + HK61*P(1,2) + HK62*P(2,2) + HK63*P(2,6);
const float HK74 = HK55*P(23,23) + HK59*P(22,23) + HK60*P(0,23) + HK61*P(1,23) + HK62*P(2,23) + HK63*P(6,23);
const float HK75 = HK55*P(5,23);
const float HK76 = HK59*P(5,22) + HK60*P(0,5) + HK61*P(1,5) + HK62*P(2,5) + HK63*P(5,6) + HK75;
const float HK77 = HK55*P(1,23) + HK59*P(1,22) + HK60*P(0,1) + HK61*P(1,1) + HK62*P(1,2) + HK63*P(1,6);
const float HK78 = HK55*P(3,23) + HK59*P(3,22) + HK60*P(0,3) + HK61*P(1,3) + HK62*P(2,3) + HK63*P(3,6);
const float HK79 = Kaccy/(2*HK52*HK65*(HK66*P(3,5) + HK67*P(3,3) + HK68*P(3,4) + HK78) + 2*HK53*HK65*(HK66*P(4,5) + HK67*P(3,4) + HK68*P(4,4) + HK72) - HK55*HK65*(HK66*P(5,23) + HK67*P(3,23) + HK68*P(4,23) + HK74) + HK55*HK65*(HK66*P(5,5) + HK67*P(3,5) + HK68*P(4,5) + HK76) - HK59*HK65*(HK66*P(5,22) + HK67*P(3,22) + HK68*P(4,22) + HK70) - HK60*HK65*(HK64 + HK66*P(0,5) + HK67*P(0,3) + HK68*P(0,4)) - HK61*HK65*(HK66*P(1,5) + HK67*P(1,3) + HK68*P(1,4) + HK77) - HK62*HK65*(HK66*P(2,5) + HK67*P(2,3) + HK68*P(2,4) + HK73) - HK63*HK65*(HK66*P(5,6) + HK67*P(3,6) + HK68*P(4,6) + HK69) - R_ACC);


// Observation Jacobians - axis 0
Hfusion.at<0>() = HK3*HK4;
Hfusion.at<1>() = -HK4*HK6;
Hfusion.at<2>() = HK11*HK4;
Hfusion.at<3>() = -HK14*HK4;
Hfusion.at<4>() = HK17;
Hfusion.at<5>() = -HK21;
Hfusion.at<6>() = HK22*HK4;
Hfusion.at<22>() = -HK17;
Hfusion.at<23>() = HK21;


// Kalman gains - axis 0
Kfusion(0) = HK47*(2*HK14*P(0,3) + HK16*P(0,22) - HK16*P(0,4) + 2*HK20*P(0,5) - HK24 - HK25*P(0,6) - HK26*P(0,0) - HK27*P(0,2) + 2*HK6*P(0,1));
Kfusion(1) = HK47*(2*HK14*P(1,3) + HK16*P(1,22) - HK16*P(1,4) + 2*HK20*P(1,5) - HK25*P(1,6) - HK26*P(0,1) - HK27*P(1,2) - HK40 + 2*HK6*P(1,1));
Kfusion(2) = HK47*(2*HK14*P(2,3) + HK16*P(2,22) - HK16*P(2,4) + 2*HK20*P(2,5) - HK25*P(2,6) - HK26*P(0,2) - HK27*P(2,2) - HK46 + 2*HK6*P(1,2));
Kfusion(3) = HK47*(2*HK14*P(3,3) + HK16*P(3,22) - HK16*P(3,4) + 2*HK20*P(3,5) - HK25*P(3,6) - HK26*P(0,3) - HK27*P(2,3) - HK45 + 2*HK6*P(1,3));
Kfusion(4) = HK47*(2*HK14*P(3,4) - HK16*P(4,4) + 2*HK20*P(4,5) - HK25*P(4,6) - HK26*P(0,4) - HK27*P(2,4) - HK41 + HK42 + 2*HK6*P(1,4));
Kfusion(5) = HK47*(2*HK14*P(3,5) - HK16*P(4,5) + HK16*P(5,22) + 2*HK20*P(5,5) - HK25*P(5,6) - HK26*P(0,5) - HK27*P(2,5) - HK29 + 2*HK6*P(1,5));
Kfusion(6) = HK47*(2*HK14*P(3,6) - HK16*P(4,6) + HK16*P(6,22) + 2*HK20*P(5,6) - HK25*P(6,6) - HK26*P(0,6) - HK27*P(2,6) - HK39 + 2*HK6*P(1,6));
Kfusion(7) = HK47*(2*HK14*P(3,7) - HK16*P(4,7) + HK16*P(7,22) + 2*HK20*P(5,7) - HK23*P(7,23) - HK25*P(6,7) - HK26*P(0,7) - HK27*P(2,7) + 2*HK6*P(1,7));
Kfusion(8) = HK47*(2*HK14*P(3,8) - HK16*P(4,8) + HK16*P(8,22) + 2*HK20*P(5,8) - HK23*P(8,23) - HK25*P(6,8) - HK26*P(0,8) - HK27*P(2,8) + 2*HK6*P(1,8));
Kfusion(9) = HK47*(2*HK14*P(3,9) - HK16*P(4,9) + HK16*P(9,22) + 2*HK20*P(5,9) - HK23*P(9,23) - HK25*P(6,9) - HK26*P(0,9) - HK27*P(2,9) + 2*HK6*P(1,9));
Kfusion(10) = HK47*(2*HK14*P(3,10) + HK16*P(10,22) - HK16*P(4,10) + 2*HK20*P(5,10) - HK23*P(10,23) - HK25*P(6,10) - HK26*P(0,10) - HK27*P(2,10) + 2*HK6*P(1,10));
Kfusion(11) = HK47*(2*HK14*P(3,11) + HK16*P(11,22) - HK16*P(4,11) + 2*HK20*P(5,11) - HK23*P(11,23) - HK25*P(6,11) - HK26*P(0,11) - HK27*P(2,11) + 2*HK6*P(1,11));
Kfusion(12) = HK47*(2*HK14*P(3,12) + HK16*P(12,22) - HK16*P(4,12) + 2*HK20*P(5,12) - HK23*P(12,23) - HK25*P(6,12) - HK26*P(0,12) - HK27*P(2,12) + 2*HK6*P(1,12));
Kfusion(13) = HK47*(2*HK14*P(3,13) + HK16*P(13,22) - HK16*P(4,13) + 2*HK20*P(5,13) - HK23*P(13,23) - HK25*P(6,13) - HK26*P(0,13) - HK27*P(2,13) + 2*HK6*P(1,13));
Kfusion(14) = HK47*(2*HK14*P(3,14) + HK16*P(14,22) - HK16*P(4,14) + 2*HK20*P(5,14) - HK23*P(14,23) - HK25*P(6,14) - HK26*P(0,14) - HK27*P(2,14) + 2*HK6*P(1,14));
Kfusion(15) = HK47*(2*HK14*P(3,15) + HK16*P(15,22) - HK16*P(4,15) + 2*HK20*P(5,15) - HK23*P(15,23) - HK25*P(6,15) - HK26*P(0,15) - HK27*P(2,15) + 2*HK6*P(1,15));
Kfusion(16) = HK47*(2*HK14*P(3,16) + HK16*P(16,22) - HK16*P(4,16) + 2*HK20*P(5,16) - HK23*P(16,23) - HK25*P(6,16) - HK26*P(0,16) - HK27*P(2,16) + 2*HK6*P(1,16));
Kfusion(17) = HK47*(2*HK14*P(3,17) + HK16*P(17,22) - HK16*P(4,17) + 2*HK20*P(5,17) - HK23*P(17,23) - HK25*P(6,17) - HK26*P(0,17) - HK27*P(2,17) + 2*HK6*P(1,17));
Kfusion(18) = HK47*(2*HK14*P(3,18) + HK16*P(18,22) - HK16*P(4,18) + 2*HK20*P(5,18) - HK23*P(18,23) - HK25*P(6,18) - HK26*P(0,18) - HK27*P(2,18) + 2*HK6*P(1,18));
Kfusion(19) = HK47*(2*HK14*P(3,19) + HK16*P(19,22) - HK16*P(4,19) + 2*HK20*P(5,19) - HK23*P(19,23) - HK25*P(6,19) - HK26*P(0,19) - HK27*P(2,19) + 2*HK6*P(1,19));
Kfusion(20) = HK47*(2*HK14*P(3,20) + HK16*P(20,22) - HK16*P(4,20) + 2*HK20*P(5,20) - HK23*P(20,23) - HK25*P(6,20) - HK26*P(0,20) - HK27*P(2,20) + 2*HK6*P(1,20));
Kfusion(21) = HK47*(2*HK14*P(3,21) + HK16*P(21,22) - HK16*P(4,21) + 2*HK20*P(5,21) - HK23*P(21,23) - HK25*P(6,21) - HK26*P(0,21) - HK27*P(2,21) + 2*HK6*P(1,21));
Kfusion(22) = HK47*(2*HK14*P(3,22) + HK16*P(22,22) + 2*HK20*P(5,22) - HK25*P(6,22) - HK26*P(0,22) - HK27*P(2,22) - HK42 - HK44 + 2*HK6*P(1,22));
Kfusion(23) = HK47*(2*HK14*P(3,23) + HK16*P(22,23) - HK16*P(4,23) - HK25*P(6,23) - HK26*P(0,23) - HK27*P(2,23) - HK28 - HK38 + 2*HK6*P(1,23));


// Observation Jacobians - axis 1
Hfusion.at<0>() = -HK48*HK49;
Hfusion.at<1>() = -HK49*HK50;
Hfusion.at<2>() = -HK49*HK51;
Hfusion.at<3>() = HK49*HK52;
Hfusion.at<4>() = HK54;
Hfusion.at<5>() = HK56;
Hfusion.at<6>() = -HK49*HK57;
Hfusion.at<22>() = -HK54;
Hfusion.at<23>() = -HK56;


// Kalman gains - axis 1
Kfusion(0) = HK79*(-HK55*P(0,5) - HK58*P(0,3) - HK59*P(0,4) + HK64);
Kfusion(1) = HK79*(-HK55*P(1,5) - HK58*P(1,3) - HK59*P(1,4) + HK77);
Kfusion(2) = HK79*(-HK55*P(2,5) - HK58*P(2,3) - HK59*P(2,4) + HK73);
Kfusion(3) = HK79*(-HK55*P(3,5) - HK58*P(3,3) - HK59*P(3,4) + HK78);
Kfusion(4) = HK79*(-HK55*P(4,5) - HK58*P(3,4) - HK59*P(4,4) + HK72);
Kfusion(5) = HK79*(-HK55*P(5,5) - HK58*P(3,5) - HK59*P(4,5) + HK76);
Kfusion(6) = HK79*(-HK55*P(5,6) - HK58*P(3,6) - HK59*P(4,6) + HK69);
Kfusion(7) = HK79*(-HK55*P(5,7) + HK55*P(7,23) - HK58*P(3,7) - HK59*P(4,7) + HK59*P(7,22) + HK60*P(0,7) + HK61*P(1,7) + HK62*P(2,7) + HK63*P(6,7));
Kfusion(8) = HK79*(-HK55*P(5,8) + HK55*P(8,23) - HK58*P(3,8) - HK59*P(4,8) + HK59*P(8,22) + HK60*P(0,8) + HK61*P(1,8) + HK62*P(2,8) + HK63*P(6,8));
Kfusion(9) = HK79*(-HK55*P(5,9) + HK55*P(9,23) - HK58*P(3,9) - HK59*P(4,9) + HK59*P(9,22) + HK60*P(0,9) + HK61*P(1,9) + HK62*P(2,9) + HK63*P(6,9));
Kfusion(10) = HK79*(HK55*P(10,23) - HK55*P(5,10) - HK58*P(3,10) + HK59*P(10,22) - HK59*P(4,10) + HK60*P(0,10) + HK61*P(1,10) + HK62*P(2,10) + HK63*P(6,10));
Kfusion(11) = HK79*(HK55*P(11,23) - HK55*P(5,11) - HK58*P(3,11) + HK59*P(11,22) - HK59*P(4,11) + HK60*P(0,11) + HK61*P(1,11) + HK62*P(2,11) + HK63*P(6,11));
Kfusion(12) = HK79*(HK55*P(12,23) - HK55*P(5,12) - HK58*P(3,12) + HK59*P(12,22) - HK59*P(4,12) + HK60*P(0,12) + HK61*P(1,12) + HK62*P(2,12) + HK63*P(6,12));
Kfusion(13) = HK79*(HK55*P(13,23) - HK55*P(5,13) - HK58*P(3,13) + HK59*P(13,22) - HK59*P(4,13) + HK60*P(0,13) + HK61*P(1,13) + HK62*P(2,13) + HK63*P(6,13));
Kfusion(14) = HK79*(HK55*P(14,23) - HK55*P(5,14) - HK58*P(3,14) + HK59*P(14,22) - HK59*P(4,14) + HK60*P(0,14) + HK61*P(1,14) + HK62*P(2,14) + HK63*P(6,14));
Kfusion(15) = HK79*(HK55*P(15,23) - HK55*P(5,15) - HK58*P(3,15) + HK59*P(15,22) - HK59*P(4,15) + HK60*P(0,15) + HK61*P(1,15) + HK62*P(2,15) + HK63*P(6,15));
Kfusion(16) = HK79*(HK55*P(16,23) - HK55*P(5,16) - HK58*P(3,16) + HK59*P(16,22) - HK59*P(4,16) + HK60*P(0,16) + HK61*P(1,16) + HK62*P(2,16) + HK63*P(6,16));
Kfusion(17) = HK79*(HK55*P(17,23) - HK55*P(5,17) - HK58*P(3,17) + HK59*P(17,22) - HK59*P(4,17) + HK60*P(0,17) + HK61*P(1,17) + HK62*P(2,17) + HK63*P(6,17));
Kfusion(18) = HK79*(HK55*P(18,23) - HK55*P(5,18) - HK58*P(3,18) + HK59*P(18,22) - HK59*P(4,18) + HK60*P(0,18) + HK61*P(1,18) + HK62*P(2,18) + HK63*P(6,18));
Kfusion(19) = HK79*(HK55*P(19,23) - HK55*P(5,19) - HK58*P(3,19) + HK59*P(19,22) - HK59*P(4,19) + HK60*P(0,19) + HK61*P(1,19) + HK62*P(2,19) + HK63*P(6,19));
Kfusion(20) = HK79*(HK55*P(20,23) - HK55*P(5,20) - HK58*P(3,20) + HK59*P(20,22) - HK59*P(4,20) + HK60*P(0,20) + HK61*P(1,20) + HK62*P(2,20) + HK63*P(6,20));
Kfusion(21) = HK79*(HK55*P(21,23) - HK55*P(5,21) - HK58*P(3,21) + HK59*P(21,22) - HK59*P(4,21) + HK60*P(0,21) + HK61*P(1,21) + HK62*P(2,21) + HK63*P(6,21));
Kfusion(22) = HK79*(-HK55*P(5,22) - HK58*P(3,22) + HK70 - HK71);
Kfusion(23) = HK79*(-HK58*P(3,23) - HK59*P(4,23) + HK74 - HK75);


// Observation Jacobians - axis 2


// Kalman gains - axis 2


