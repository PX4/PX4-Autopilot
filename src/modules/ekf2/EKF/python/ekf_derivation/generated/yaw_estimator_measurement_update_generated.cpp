// Intermediate variables
const float t0 = powf(P(0,1), 2);
const float t1 = P(0,0) + velObsVar;
const float t2 = P(1,1) + velObsVar;
const float t3 = t1*t2;
const float t4 = 1.0F/(t0 - t3);
const float t5 = P(0,1)*t4;
const float t6 = t1*t4;
const float t7 = -P(0,0)*t2 + t0;
const float t8 = t4*t7;
const float t9 = -t5*velObsVar;
const float t10 = P(1,1)*t1;
const float t11 = t0 - t10;
const float t12 = P(0,1)*P(1,2) - P(0,2)*t2;
const float t13 = P(0,1)*P(0,2) - P(1,2)*t1;
const float t14 = -t0;
const float t15 = t14 + t3;
const float t16 = 1.0F/t15;
const float t17 = t16*velObsVar;
const float t18 = t17*t2 + t8;
const float t19 = t0*velObsVar;
const float t20 = t16*t19;
const float t21 = t20 + t6*t7;
const float t22 = t16*t18;
const float t23 = P(0,1)*t13;
const float t24 = powf(t15, -2);
const float t25 = t24*(-t11*t2 + t19);
const float t26 = t1*velObsVar + t10 + t14;
const float t27 = t16*t8;
const float t28 = t24*t26;
const float t29 = P(0,1)*t12;
const float t30 = t24*(t13*t2 + t29);
const float t31 = P(0,1)*velObsVar;
const float t32 = t1*t12 + t23;
const float t33 = t24*t32;


// Equations for NE velocity innovation variance's determinante inverse
_ekf_gsf[model_index].S_det_inverse = -t4;


// Equations for NE velocity innovation variance inverse
_ekf_gsf[model_index].S_inverse(0,0) = -t2*t4;
_ekf_gsf[model_index].S_inverse(0,1) = t5;
_ekf_gsf[model_index].S_inverse(1,1) = -t6;


// Equations for NE velocity Kalman gain
K(0,0) = t8;
K(1,0) = t9;
K(2,0) = t12*t4;
K(0,1) = t9;
K(1,1) = t11*t4;
K(2,1) = t13*t4;


// Equations for covariance matrix update
_ekf_gsf[model_index].P(0,0) = P(0,0) - t18*t20 - t21*t8;
_ekf_gsf[model_index].P(0,1) = P(0,1)*(t11*t22 - t17*t21 + 1);
_ekf_gsf[model_index].P(1,1) = P(1,1) + t11*t25 - t19*t28;
_ekf_gsf[model_index].P(0,2) = P(0,2) + t12*t16*t21 + t22*t23;
_ekf_gsf[model_index].P(1,2) = P(1,2) + t13*t25 + t28*t29;
_ekf_gsf[model_index].P(2,2) = P(2,2) - t12*t33 - t13*t30;
