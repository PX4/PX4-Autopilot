// Intermediate variables
const float t0 = powf(P(0,1), 2);
const float t1 = -t0;
const float t2 = P(0,0)*P(1,1) + P(0,0)*velObsVar + P(1,1)*velObsVar + t1 + powf(velObsVar, 2);
const float t3 = 1.0F/t2;
const float t4 = P(1,1) + velObsVar;
const float t5 = P(0,1)*t3;
const float t6 = -t5;
const float t7 = P(0,0) + velObsVar;
const float t8 = P(0,0)*t4 + t1;
const float t9 = t5*velObsVar;
const float t10 = P(1,1)*t7;
const float t11 = t1 + t10;
const float t12 = P(0,1)*P(1,2);
const float t13 = P(0,2)*t4;
const float t14 = P(0,1)*P(0,2);
const float t15 = P(1,2)*t7;
const float t16 = t0*velObsVar;
const float t17 = powf(t2, -2);
const float t18 = t4*velObsVar + t8;
const float t19 = t17*t18;
const float t20 = t17*(t16 + t7*t8);
const float t21 = t0 - t10;
const float t22 = t17*t21;
const float t23 = t14 - t15;
const float t24 = P(0,1)*t23;
const float t25 = t12 - t13;
const float t26 = t16 - t21*t4;
const float t27 = t17*t26;
const float t28 = t11 + t7*velObsVar;
const float t29 = t17*t8;
const float t30 = t17*t28;
const float t31 = P(0,1)*t25;
const float t32 = t23*t4 + t31;
const float t33 = t17*t32;
const float t34 = P(0,1)*velObsVar;
const float t35 = t24 + t25*t7;
const float t36 = t17*t35;


// Equations for NE velocity innovation variance's determinante inverse
_ekf_gsf[model_index].S_det_inverse = t3;


// Equations for NE velocity innovation variance inverse
_ekf_gsf[model_index].S_inverse(0,0) = t3*t4;
_ekf_gsf[model_index].S_inverse(0,1) = t6;
_ekf_gsf[model_index].S_inverse(1,1) = t3*t7;


// Equations for NE velocity Kalman gain
K(0,0) = t3*t8;
K(1,0) = t9;
K(2,0) = t3*(-t12 + t13);
K(0,1) = t9;
K(1,1) = t11*t3;
K(2,1) = t3*(-t14 + t15);


// Equations for covariance matrix update
_ekf_gsf[model_index].P(0,0) = P(0,0) - t16*t19 - t20*t8;
_ekf_gsf[model_index].P(0,1) = P(0,1)*(t18*t22 - t20*velObsVar + 1);
_ekf_gsf[model_index].P(1,1) = P(1,1) - t16*t30 + t22*t26;
_ekf_gsf[model_index].P(0,2) = P(0,2) + t19*t24 + t20*t25;
_ekf_gsf[model_index].P(1,2) = P(1,2) + t23*t27 + t30*t31;
_ekf_gsf[model_index].P(2,2) = P(2,2) - t23*t33 - t25*t36;
