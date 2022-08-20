// Sub Expressions
const float HK0 = sinf(ant_yaw);
const float HK1 = q0*q3;
const float HK2 = q1*q2;
const float HK3 = cosf(ant_yaw);
const float HK4 = (q1)*(q1);
const float HK5 = (q2)*(q2);
const float HK6 = (q0)*(q0) - (q3)*(q3);
const float HK7 = 2*HK0*(HK1 - HK2) - HK3*(HK4 - HK5 + HK6);
const float HK8 = 1.0F/(HK7);
const float HK9 = HK3*q0;
const float HK10 = HK0*q3;
const float HK11 = HK0*(-HK4 + HK5 + HK6) + 2*HK3*(HK1 + HK2);
const float HK12 = HK11*HK8;
const float HK13 = HK0*q0 + HK3*q3;
const float HK14 = HK8*(HK12*(-HK10 + HK9) + HK13);
const float HK15 = (HK11)*(HK11)/(HK7)*(HK7) + 1;
const float HK16 = 2/HK15;
const float HK17 = -1/HK7;
const float HK18 = HK0*q2 + HK3*q1;
const float HK19 = HK0*q1 - HK3*q2;
const float HK20 = HK17*(HK11*HK17*HK18 + HK19);
const float HK21 = HK17*(HK11*HK17*HK19 - HK18);
const float HK22 = HK10 + HK12*HK13 - HK9;
const float HK23 = -HK14*P(0,0) - HK20*P(0,1) - HK21*P(0,2) + HK22*HK8*P(0,3);
const float HK24 = -HK14*P(0,1) - HK20*P(1,1) - HK21*P(1,2) + HK22*HK8*P(1,3);
const float HK25 = 1.0F/((HK15)*(HK15));
const float HK26 = 4*HK25;
const float HK27 = -HK14*P(0,2) - HK20*P(1,2) - HK21*P(2,2) + HK22*HK8*P(2,3);
const float HK28 = -HK14*P(0,3) - HK20*P(1,3) - HK21*P(2,3) + HK22*HK8*P(3,3);
const float HK29 = HK16/(-HK14*HK23*HK26 - HK20*HK24*HK26 - HK21*HK26*HK27 + 4*HK22*HK25*HK28*HK8 + R_YAW);


// Observation Jacobians
Hfusion.at<0>() = -HK14*HK16;
Hfusion.at<1>() = -HK16*HK20;
Hfusion.at<2>() = -HK16*HK21;
Hfusion.at<3>() = HK16*HK22*HK8;


// Kalman gains
Kfusion(0) = HK23*HK29;
Kfusion(1) = HK24*HK29;
Kfusion(2) = HK27*HK29;
Kfusion(3) = HK28*HK29;
Kfusion(4) = HK29*(-HK14*P(0,4) - HK20*P(1,4) - HK21*P(2,4) + HK22*HK8*P(3,4));
Kfusion(5) = HK29*(-HK14*P(0,5) - HK20*P(1,5) - HK21*P(2,5) + HK22*HK8*P(3,5));
Kfusion(6) = HK29*(-HK14*P(0,6) - HK20*P(1,6) - HK21*P(2,6) + HK22*HK8*P(3,6));
Kfusion(7) = HK29*(-HK14*P(0,7) - HK20*P(1,7) - HK21*P(2,7) + HK22*HK8*P(3,7));
Kfusion(8) = HK29*(-HK14*P(0,8) - HK20*P(1,8) - HK21*P(2,8) + HK22*HK8*P(3,8));
Kfusion(9) = HK29*(-HK14*P(0,9) - HK20*P(1,9) - HK21*P(2,9) + HK22*HK8*P(3,9));
Kfusion(10) = HK29*(-HK14*P(0,10) - HK20*P(1,10) - HK21*P(2,10) + HK22*HK8*P(3,10));
Kfusion(11) = HK29*(-HK14*P(0,11) - HK20*P(1,11) - HK21*P(2,11) + HK22*HK8*P(3,11));
Kfusion(12) = HK29*(-HK14*P(0,12) - HK20*P(1,12) - HK21*P(2,12) + HK22*HK8*P(3,12));
Kfusion(13) = HK29*(-HK14*P(0,13) - HK20*P(1,13) - HK21*P(2,13) + HK22*HK8*P(3,13));
Kfusion(14) = HK29*(-HK14*P(0,14) - HK20*P(1,14) - HK21*P(2,14) + HK22*HK8*P(3,14));
Kfusion(15) = HK29*(-HK14*P(0,15) - HK20*P(1,15) - HK21*P(2,15) + HK22*HK8*P(3,15));
Kfusion(16) = HK29*(-HK14*P(0,16) - HK20*P(1,16) - HK21*P(2,16) + HK22*HK8*P(3,16));
Kfusion(17) = HK29*(-HK14*P(0,17) - HK20*P(1,17) - HK21*P(2,17) + HK22*HK8*P(3,17));
Kfusion(18) = HK29*(-HK14*P(0,18) - HK20*P(1,18) - HK21*P(2,18) + HK22*HK8*P(3,18));
Kfusion(19) = HK29*(-HK14*P(0,19) - HK20*P(1,19) - HK21*P(2,19) + HK22*HK8*P(3,19));
Kfusion(20) = HK29*(-HK14*P(0,20) - HK20*P(1,20) - HK21*P(2,20) + HK22*HK8*P(3,20));
Kfusion(21) = HK29*(-HK14*P(0,21) - HK20*P(1,21) - HK21*P(2,21) + HK22*HK8*P(3,21));
Kfusion(22) = HK29*(-HK14*P(0,22) - HK20*P(1,22) - HK21*P(2,22) + HK22*HK8*P(3,22));
Kfusion(23) = HK29*(-HK14*P(0,23) - HK20*P(1,23) - HK21*P(2,23) + HK22*HK8*P(3,23));


// Predicted observation


// Innovation variance


