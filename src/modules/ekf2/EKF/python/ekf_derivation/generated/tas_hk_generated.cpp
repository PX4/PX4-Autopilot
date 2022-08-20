// Sub Expressions
const float HK0 = vn - vwn;
const float HK1 = ve - vwe;
const float HK2 = sqrtf((HK0)*(HK0) + (HK1)*(HK1) + (vd)*(vd));
const float HK3 = 1.0F/(HK2);
const float HK4 = HK0*HK3;
const float HK5 = HK1*HK3;
const float HK6 = HK3*vd;
const float HK7 = -HK0*HK3;
const float HK8 = -HK1*HK3;
const float HK9 = 1.0F/(innov_var);


// Observation Jacobians
Hfusion.at<4>() = HK4;
Hfusion.at<5>() = HK5;
Hfusion.at<6>() = HK6;
Hfusion.at<22>() = HK7;
Hfusion.at<23>() = HK8;


// Kalman gains
Kfusion(0) = HK9*(HK4*P(0,4) + HK5*P(0,5) + HK6*P(0,6) + HK7*P(0,22) + HK8*P(0,23));
Kfusion(1) = HK9*(HK4*P(1,4) + HK5*P(1,5) + HK6*P(1,6) + HK7*P(1,22) + HK8*P(1,23));
Kfusion(2) = HK9*(HK4*P(2,4) + HK5*P(2,5) + HK6*P(2,6) + HK7*P(2,22) + HK8*P(2,23));
Kfusion(3) = HK9*(HK4*P(3,4) + HK5*P(3,5) + HK6*P(3,6) + HK7*P(3,22) + HK8*P(3,23));
Kfusion(4) = HK9*(HK4*P(4,4) + HK5*P(4,5) + HK6*P(4,6) + HK7*P(4,22) + HK8*P(4,23));
Kfusion(5) = HK9*(HK4*P(4,5) + HK5*P(5,5) + HK6*P(5,6) + HK7*P(5,22) + HK8*P(5,23));
Kfusion(6) = HK9*(HK4*P(4,6) + HK5*P(5,6) + HK6*P(6,6) + HK7*P(6,22) + HK8*P(6,23));
Kfusion(7) = HK9*(HK4*P(4,7) + HK5*P(5,7) + HK6*P(6,7) + HK7*P(7,22) + HK8*P(7,23));
Kfusion(8) = HK9*(HK4*P(4,8) + HK5*P(5,8) + HK6*P(6,8) + HK7*P(8,22) + HK8*P(8,23));
Kfusion(9) = HK9*(HK4*P(4,9) + HK5*P(5,9) + HK6*P(6,9) + HK7*P(9,22) + HK8*P(9,23));
Kfusion(10) = HK9*(HK4*P(4,10) + HK5*P(5,10) + HK6*P(6,10) + HK7*P(10,22) + HK8*P(10,23));
Kfusion(11) = HK9*(HK4*P(4,11) + HK5*P(5,11) + HK6*P(6,11) + HK7*P(11,22) + HK8*P(11,23));
Kfusion(12) = HK9*(HK4*P(4,12) + HK5*P(5,12) + HK6*P(6,12) + HK7*P(12,22) + HK8*P(12,23));
Kfusion(13) = HK9*(HK4*P(4,13) + HK5*P(5,13) + HK6*P(6,13) + HK7*P(13,22) + HK8*P(13,23));
Kfusion(14) = HK9*(HK4*P(4,14) + HK5*P(5,14) + HK6*P(6,14) + HK7*P(14,22) + HK8*P(14,23));
Kfusion(15) = HK9*(HK4*P(4,15) + HK5*P(5,15) + HK6*P(6,15) + HK7*P(15,22) + HK8*P(15,23));
Kfusion(16) = HK9*(HK4*P(4,16) + HK5*P(5,16) + HK6*P(6,16) + HK7*P(16,22) + HK8*P(16,23));
Kfusion(17) = HK9*(HK4*P(4,17) + HK5*P(5,17) + HK6*P(6,17) + HK7*P(17,22) + HK8*P(17,23));
Kfusion(18) = HK9*(HK4*P(4,18) + HK5*P(5,18) + HK6*P(6,18) + HK7*P(18,22) + HK8*P(18,23));
Kfusion(19) = HK9*(HK4*P(4,19) + HK5*P(5,19) + HK6*P(6,19) + HK7*P(19,22) + HK8*P(19,23));
Kfusion(20) = HK9*(HK4*P(4,20) + HK5*P(5,20) + HK6*P(6,20) + HK7*P(20,22) + HK8*P(20,23));
Kfusion(21) = HK9*(HK4*P(4,21) + HK5*P(5,21) + HK6*P(6,21) + HK7*P(21,22) + HK8*P(21,23));
Kfusion(22) = HK9*(HK4*P(4,22) + HK5*P(5,22) + HK6*P(6,22) + HK7*P(22,22) + HK8*P(22,23));
Kfusion(23) = HK9*(HK4*P(4,23) + HK5*P(5,23) + HK6*P(6,23) + HK7*P(22,23) + HK8*P(23,23));


// Predicted observation
meas_pred = HK2;


// Innovation variance


