// X Axis Equations
// Sub Expressions
const float HK0 = Tbs(1,0)*q2;
const float HK1 = Tbs(1,1)*q1;
const float HK2 = Tbs(1,1)*q3;
const float HK3 = Tbs(1,2)*q2;
const float HK4 = Tbs(1,0)*q3;
const float HK5 = Tbs(1,2)*q1;
const float HK6 = -HK5;
const float HK7 = vd*(HK0 - HK1) - ve*(HK4 + HK6) + vn*(HK2 - HK3);
const float HK8 = 1.0F/(range);
const float HK9 = 2*HK8;
const float HK10 = Tbs(1,1)*q2;
const float HK11 = Tbs(1,2)*q3;
const float HK12 = Tbs(1,1)*q0;
const float HK13 = Tbs(1,2)*q0;
const float HK14 = vd*(HK12 + HK4 - 2*HK5) - ve*(-HK0 + 2*HK1 + HK13) + vn*(HK10 + HK11);
const float HK15 = Tbs(1,0)*q1;
const float HK16 = Tbs(1,0)*q0;
const float HK17 = -vd*(HK16 - HK2 + 2*HK3) + ve*(HK11 + HK15) + vn*(-2*HK0 + HK1 + HK13);
const float HK18 = vd*(HK10 + HK15) + ve*(HK16 - 2*HK2 + HK3) - vn*(HK12 + 2*HK4 + HK6);
const float HK19 = q0*q2;
const float HK20 = q1*q3;
const float HK21 = 2*Tbs(1,2);
const float HK22 = q0*q3;
const float HK23 = q1*q2;
const float HK24 = 2*Tbs(1,1);
const float HK25 = 2*(q2)*(q2);
const float HK26 = 2*(q3)*(q3) - 1;
const float HK27 = -HK21*(HK19 + HK20) + HK24*(HK22 - HK23) + Tbs(1,0)*(HK25 + HK26);
const float HK28 = 2*Tbs(1,0);
const float HK29 = q0*q1;
const float HK30 = q2*q3;
const float HK31 = 2*(q1)*(q1);
const float HK32 = HK21*(HK29 - HK30) - HK28*(HK22 + HK23) + Tbs(1,1)*(HK26 + HK31);
const float HK33 = -HK24*(HK29 + HK30) + HK28*(HK19 - HK20) + Tbs(1,2)*(HK25 + HK31 - 1);
const float HK34 = 2*HK7;
const float HK35 = 2*HK14*P(0,1) + 2*HK17*P(0,2) + 2*HK18*P(0,3) - HK27*P(0,4) - HK32*P(0,5) - HK33*P(0,6) - HK34*P(0,0);
const float HK36 = 1.0F/((range)*(range));
const float HK37 = 2*HK14*P(1,6) + 2*HK17*P(2,6) + 2*HK18*P(3,6) - HK27*P(4,6) - HK32*P(5,6) - HK33*P(6,6) - HK34*P(0,6);
const float HK38 = 2*HK14*P(1,5) + 2*HK17*P(2,5) + 2*HK18*P(3,5) - HK27*P(4,5) - HK32*P(5,5) - HK33*P(5,6) - HK34*P(0,5);
const float HK39 = 2*HK14*P(1,4) + 2*HK17*P(2,4) + 2*HK18*P(3,4) - HK27*P(4,4) - HK32*P(4,5) - HK33*P(4,6) - HK34*P(0,4);
const float HK40 = 2*HK14*P(1,3) + 2*HK17*P(2,3) + 2*HK18*P(3,3) - HK27*P(3,4) - HK32*P(3,5) - HK33*P(3,6) - HK34*P(0,3);
const float HK41 = 2*HK14*P(1,2) + 2*HK17*P(2,2) + 2*HK18*P(2,3) - HK27*P(2,4) - HK32*P(2,5) - HK33*P(2,6) - HK34*P(0,2);
const float HK42 = 2*HK14*P(1,1) + 2*HK17*P(1,2) + 2*HK18*P(1,3) - HK27*P(1,4) - HK32*P(1,5) - HK33*P(1,6) - HK34*P(0,1);
const float HK43 = HK8/(2*HK14*HK36*HK42 + 2*HK17*HK36*HK41 + 2*HK18*HK36*HK40 - HK27*HK36*HK39 - HK32*HK36*HK38 - HK33*HK36*HK37 - HK34*HK35*HK36 + R_LOS);


// Observation Jacobians
Hfusion.at<0>() = -HK7*HK9;
Hfusion.at<1>() = HK14*HK9;
Hfusion.at<2>() = HK17*HK9;
Hfusion.at<3>() = HK18*HK9;
Hfusion.at<4>() = -HK27*HK8;
Hfusion.at<5>() = -HK32*HK8;
Hfusion.at<6>() = -HK33*HK8;


// Kalman gains
Kfusion(0) = HK35*HK43;
Kfusion(1) = HK42*HK43;
Kfusion(2) = HK41*HK43;
Kfusion(3) = HK40*HK43;
Kfusion(4) = HK39*HK43;
Kfusion(5) = HK38*HK43;
Kfusion(6) = HK37*HK43;
Kfusion(7) = HK43*(2*HK14*P(1,7) + 2*HK17*P(2,7) + 2*HK18*P(3,7) - HK27*P(4,7) - HK32*P(5,7) - HK33*P(6,7) - HK34*P(0,7));
Kfusion(8) = HK43*(2*HK14*P(1,8) + 2*HK17*P(2,8) + 2*HK18*P(3,8) - HK27*P(4,8) - HK32*P(5,8) - HK33*P(6,8) - HK34*P(0,8));
Kfusion(9) = HK43*(2*HK14*P(1,9) + 2*HK17*P(2,9) + 2*HK18*P(3,9) - HK27*P(4,9) - HK32*P(5,9) - HK33*P(6,9) - HK34*P(0,9));
Kfusion(10) = HK43*(2*HK14*P(1,10) + 2*HK17*P(2,10) + 2*HK18*P(3,10) - HK27*P(4,10) - HK32*P(5,10) - HK33*P(6,10) - HK34*P(0,10));
Kfusion(11) = HK43*(2*HK14*P(1,11) + 2*HK17*P(2,11) + 2*HK18*P(3,11) - HK27*P(4,11) - HK32*P(5,11) - HK33*P(6,11) - HK34*P(0,11));
Kfusion(12) = HK43*(2*HK14*P(1,12) + 2*HK17*P(2,12) + 2*HK18*P(3,12) - HK27*P(4,12) - HK32*P(5,12) - HK33*P(6,12) - HK34*P(0,12));
Kfusion(13) = HK43*(2*HK14*P(1,13) + 2*HK17*P(2,13) + 2*HK18*P(3,13) - HK27*P(4,13) - HK32*P(5,13) - HK33*P(6,13) - HK34*P(0,13));
Kfusion(14) = HK43*(2*HK14*P(1,14) + 2*HK17*P(2,14) + 2*HK18*P(3,14) - HK27*P(4,14) - HK32*P(5,14) - HK33*P(6,14) - HK34*P(0,14));
Kfusion(15) = HK43*(2*HK14*P(1,15) + 2*HK17*P(2,15) + 2*HK18*P(3,15) - HK27*P(4,15) - HK32*P(5,15) - HK33*P(6,15) - HK34*P(0,15));
Kfusion(16) = HK43*(2*HK14*P(1,16) + 2*HK17*P(2,16) + 2*HK18*P(3,16) - HK27*P(4,16) - HK32*P(5,16) - HK33*P(6,16) - HK34*P(0,16));
Kfusion(17) = HK43*(2*HK14*P(1,17) + 2*HK17*P(2,17) + 2*HK18*P(3,17) - HK27*P(4,17) - HK32*P(5,17) - HK33*P(6,17) - HK34*P(0,17));
Kfusion(18) = HK43*(2*HK14*P(1,18) + 2*HK17*P(2,18) + 2*HK18*P(3,18) - HK27*P(4,18) - HK32*P(5,18) - HK33*P(6,18) - HK34*P(0,18));
Kfusion(19) = HK43*(2*HK14*P(1,19) + 2*HK17*P(2,19) + 2*HK18*P(3,19) - HK27*P(4,19) - HK32*P(5,19) - HK33*P(6,19) - HK34*P(0,19));
Kfusion(20) = HK43*(2*HK14*P(1,20) + 2*HK17*P(2,20) + 2*HK18*P(3,20) - HK27*P(4,20) - HK32*P(5,20) - HK33*P(6,20) - HK34*P(0,20));
Kfusion(21) = HK43*(2*HK14*P(1,21) + 2*HK17*P(2,21) + 2*HK18*P(3,21) - HK27*P(4,21) - HK32*P(5,21) - HK33*P(6,21) - HK34*P(0,21));
Kfusion(22) = HK43*(2*HK14*P(1,22) + 2*HK17*P(2,22) + 2*HK18*P(3,22) - HK27*P(4,22) - HK32*P(5,22) - HK33*P(6,22) - HK34*P(0,22));
Kfusion(23) = HK43*(2*HK14*P(1,23) + 2*HK17*P(2,23) + 2*HK18*P(3,23) - HK27*P(4,23) - HK32*P(5,23) - HK33*P(6,23) - HK34*P(0,23));


// Y Axis Equations
// Sub Expressions
const float HK0 = Tbs(0,0)*q2;
const float HK1 = Tbs(0,1)*q1;
const float HK2 = HK0 - HK1;
const float HK3 = Tbs(0,1)*q3;
const float HK4 = Tbs(0,2)*q2;
const float HK5 = HK3 - HK4;
const float HK6 = Tbs(0,0)*q3;
const float HK7 = Tbs(0,2)*q1;
const float HK8 = -HK7;
const float HK9 = HK6 + HK8;
const float HK10 = 1.0F/(range);
const float HK11 = 2*HK10;
const float HK12 = Tbs(0,2)*q0;
const float HK13 = -HK0 + 2*HK1 + HK12;
const float HK14 = Tbs(0,1)*q0;
const float HK15 = Tbs(0,1)*q2;
const float HK16 = Tbs(0,2)*q3;
const float HK17 = vd*(HK14 + HK6 - 2*HK7) + vn*(HK15 + HK16);
const float HK18 = Tbs(0,0)*q0;
const float HK19 = HK18 - HK3 + 2*HK4;
const float HK20 = Tbs(0,0)*q1;
const float HK21 = ve*(HK16 + HK20) + vn*(-2*HK0 + HK1 + HK12);
const float HK22 = HK14 + 2*HK6 + HK8;
const float HK23 = vd*(HK15 + HK20) + ve*(HK18 - 2*HK3 + HK4);
const float HK24 = q0*q2;
const float HK25 = q1*q3;
const float HK26 = HK24 + HK25;
const float HK27 = q0*q3;
const float HK28 = q1*q2;
const float HK29 = HK27 - HK28;
const float HK30 = 2*Tbs(0,1);
const float HK31 = 2*(q2)*(q2);
const float HK32 = 2*(q3)*(q3) - 1;
const float HK33 = HK31 + HK32;
const float HK34 = HK27 + HK28;
const float HK35 = q0*q1;
const float HK36 = q2*q3;
const float HK37 = HK35 - HK36;
const float HK38 = 2*Tbs(0,2);
const float HK39 = 2*(q1)*(q1);
const float HK40 = HK32 + HK39;
const float HK41 = HK35 + HK36;
const float HK42 = HK24 - HK25;
const float HK43 = 2*Tbs(0,0);
const float HK44 = HK31 + HK39 - 1;
const float HK45 = -2*HK2*vd - 2*HK5*vn + 2*HK9*ve;
const float HK46 = HK26*HK38 - HK29*HK30 - HK33*Tbs(0,0);
const float HK47 = HK34*HK43 - HK37*HK38 - HK40*Tbs(0,1);
const float HK48 = HK30*HK41 - HK42*HK43 - HK44*Tbs(0,2);
const float HK49 = -2*HK13*ve + 2*HK17;
const float HK50 = -2*HK19*vd + 2*HK21;
const float HK51 = -2*HK22*vn + 2*HK23;
const float HK52 = HK45*P(0,0) + HK46*P(0,4) + HK47*P(0,5) + HK48*P(0,6) + HK49*P(0,1) + HK50*P(0,2) + HK51*P(0,3);
const float HK53 = 1.0F/((range)*(range));
const float HK54 = HK45*P(0,6) + HK46*P(4,6) + HK47*P(5,6) + HK48*P(6,6) + HK49*P(1,6) + HK50*P(2,6) + HK51*P(3,6);
const float HK55 = HK45*P(0,5) + HK46*P(4,5) + HK47*P(5,5) + HK48*P(5,6) + HK49*P(1,5) + HK50*P(2,5) + HK51*P(3,5);
const float HK56 = HK45*P(0,4) + HK46*P(4,4) + HK47*P(4,5) + HK48*P(4,6) + HK49*P(1,4) + HK50*P(2,4) + HK51*P(3,4);
const float HK57 = HK45*P(0,3) + HK46*P(3,4) + HK47*P(3,5) + HK48*P(3,6) + HK49*P(1,3) + HK50*P(2,3) + HK51*P(3,3);
const float HK58 = HK45*P(0,2) + HK46*P(2,4) + HK47*P(2,5) + HK48*P(2,6) + HK49*P(1,2) + HK50*P(2,2) + HK51*P(2,3);
const float HK59 = HK45*P(0,1) + HK46*P(1,4) + HK47*P(1,5) + HK48*P(1,6) + HK49*P(1,1) + HK50*P(1,2) + HK51*P(1,3);
const float HK60 = HK10/(HK45*HK52*HK53 + HK46*HK53*HK56 + HK47*HK53*HK55 + HK48*HK53*HK54 + HK49*HK53*HK59 + HK50*HK53*HK58 + HK51*HK53*HK57 + R_LOS);


// Observation Jacobians
Hfusion.at<0>() = -HK11*(-HK2*vd - HK5*vn + HK9*ve);
Hfusion.at<1>() = -HK11*(-HK13*ve + HK17);
Hfusion.at<2>() = -HK11*(-HK19*vd + HK21);
Hfusion.at<3>() = -HK11*(-HK22*vn + HK23);
Hfusion.at<4>() = -HK10*(2*HK26*Tbs(0,2) - HK29*HK30 - HK33*Tbs(0,0));
Hfusion.at<5>() = -HK10*(2*HK34*Tbs(0,0) - HK37*HK38 - HK40*Tbs(0,1));
Hfusion.at<6>() = -HK10*(2*HK41*Tbs(0,1) - HK42*HK43 - HK44*Tbs(0,2));


// Kalman gains
Kfusion(0) = -HK52*HK60;
Kfusion(1) = -HK59*HK60;
Kfusion(2) = -HK58*HK60;
Kfusion(3) = -HK57*HK60;
Kfusion(4) = -HK56*HK60;
Kfusion(5) = -HK55*HK60;
Kfusion(6) = -HK54*HK60;
Kfusion(7) = -HK60*(HK45*P(0,7) + HK46*P(4,7) + HK47*P(5,7) + HK48*P(6,7) + HK49*P(1,7) + HK50*P(2,7) + HK51*P(3,7));
Kfusion(8) = -HK60*(HK45*P(0,8) + HK46*P(4,8) + HK47*P(5,8) + HK48*P(6,8) + HK49*P(1,8) + HK50*P(2,8) + HK51*P(3,8));
Kfusion(9) = -HK60*(HK45*P(0,9) + HK46*P(4,9) + HK47*P(5,9) + HK48*P(6,9) + HK49*P(1,9) + HK50*P(2,9) + HK51*P(3,9));
Kfusion(10) = -HK60*(HK45*P(0,10) + HK46*P(4,10) + HK47*P(5,10) + HK48*P(6,10) + HK49*P(1,10) + HK50*P(2,10) + HK51*P(3,10));
Kfusion(11) = -HK60*(HK45*P(0,11) + HK46*P(4,11) + HK47*P(5,11) + HK48*P(6,11) + HK49*P(1,11) + HK50*P(2,11) + HK51*P(3,11));
Kfusion(12) = -HK60*(HK45*P(0,12) + HK46*P(4,12) + HK47*P(5,12) + HK48*P(6,12) + HK49*P(1,12) + HK50*P(2,12) + HK51*P(3,12));
Kfusion(13) = -HK60*(HK45*P(0,13) + HK46*P(4,13) + HK47*P(5,13) + HK48*P(6,13) + HK49*P(1,13) + HK50*P(2,13) + HK51*P(3,13));
Kfusion(14) = -HK60*(HK45*P(0,14) + HK46*P(4,14) + HK47*P(5,14) + HK48*P(6,14) + HK49*P(1,14) + HK50*P(2,14) + HK51*P(3,14));
Kfusion(15) = -HK60*(HK45*P(0,15) + HK46*P(4,15) + HK47*P(5,15) + HK48*P(6,15) + HK49*P(1,15) + HK50*P(2,15) + HK51*P(3,15));
Kfusion(16) = -HK60*(HK45*P(0,16) + HK46*P(4,16) + HK47*P(5,16) + HK48*P(6,16) + HK49*P(1,16) + HK50*P(2,16) + HK51*P(3,16));
Kfusion(17) = -HK60*(HK45*P(0,17) + HK46*P(4,17) + HK47*P(5,17) + HK48*P(6,17) + HK49*P(1,17) + HK50*P(2,17) + HK51*P(3,17));
Kfusion(18) = -HK60*(HK45*P(0,18) + HK46*P(4,18) + HK47*P(5,18) + HK48*P(6,18) + HK49*P(1,18) + HK50*P(2,18) + HK51*P(3,18));
Kfusion(19) = -HK60*(HK45*P(0,19) + HK46*P(4,19) + HK47*P(5,19) + HK48*P(6,19) + HK49*P(1,19) + HK50*P(2,19) + HK51*P(3,19));
Kfusion(20) = -HK60*(HK45*P(0,20) + HK46*P(4,20) + HK47*P(5,20) + HK48*P(6,20) + HK49*P(1,20) + HK50*P(2,20) + HK51*P(3,20));
Kfusion(21) = -HK60*(HK45*P(0,21) + HK46*P(4,21) + HK47*P(5,21) + HK48*P(6,21) + HK49*P(1,21) + HK50*P(2,21) + HK51*P(3,21));
Kfusion(22) = -HK60*(HK45*P(0,22) + HK46*P(4,22) + HK47*P(5,22) + HK48*P(6,22) + HK49*P(1,22) + HK50*P(2,22) + HK51*P(3,22));
Kfusion(23) = -HK60*(HK45*P(0,23) + HK46*P(4,23) + HK47*P(5,23) + HK48*P(6,23) + HK49*P(1,23) + HK50*P(2,23) + HK51*P(3,23));


