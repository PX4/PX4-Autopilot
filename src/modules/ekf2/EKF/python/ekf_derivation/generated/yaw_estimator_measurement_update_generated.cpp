// Intermediate variables
const float t0 = (P(0,1))*(P(0,1));
const float t1 = -t0;
const float t2 = P(0,0)*P(1,1) + P(0,0)*velObsVar + P(1,1)*velObsVar + t1 + (velObsVar)*(velObsVar);
const float t3 = 1.0F/(t2);
const float t4 = P(1,1) + velObsVar;
const float t5 = P(0,1)*t3;
const float t6 = -t5;
const float t7 = P(0,0) + velObsVar;
const float t8 = P(0,0)*t4 + t1;
const float t9 = t5*velObsVar;
const float t10 = -P(1,1)*t7 + t0;
const float t11 = P(0,1)*P(1,2) - P(0,2)*t4;
const float t12 = P(0,1)*P(0,2) - P(1,2)*t7;
const float t13 = t0*velObsVar;
const float t14 = 1.0F/((t2)*(t2));
const float t15 = t4*velObsVar + t8;
const float t16 = t14*t15;
const float t17 = t14*(t13 + t7*t8);
const float t18 = t10*t14;
const float t19 = P(0,1)*t12;
const float t20 = -t10*t4 + t13;
const float t21 = t14*t20;
const float t22 = P(1,1)*t7 + t1 + t7*velObsVar;
const float t23 = t14*t8;
const float t24 = t14*t22;
const float t25 = P(0,1)*t11;
const float t26 = t12*t4 + t25;
const float t27 = t14*t26;
const float t28 = P(0,1)*velObsVar;
const float t29 = t11*t7 + t19;
const float t30 = t14*t29;


// Equations for NE velocity innovation variance's determinante inverse
_ekf_gsf[model_index].S_det_inverse = t3;


// Equations for NE velocity innovation variance inverse
_ekf_gsf[model_index].S_inverse(0,0) = t3*t4;
_ekf_gsf[model_index].S_inverse(0,1) = t6;
_ekf_gsf[model_index].S_inverse(1,1) = t3*t7;


// Equations for NE velocity Kalman gain
K(0,0) = t3*t8;
K(1,0) = t9;
K(2,0) = -t11*t3;
K(0,1) = t9;
K(1,1) = -t10*t3;
K(2,1) = -t12*t3;


// Equations for covariance matrix update
_ekf_gsf[model_index].P(0,0) = P(0,0) - t13*t16 - t17*t8;
_ekf_gsf[model_index].P(0,1) = P(0,1)*(t15*t18 - t17*velObsVar + 1);
_ekf_gsf[model_index].P(1,1) = P(1,1) - t13*t24 + t18*t20;
_ekf_gsf[model_index].P(0,2) = P(0,2) + t11*t17 + t16*t19;
_ekf_gsf[model_index].P(1,2) = P(1,2) + t12*t21 + t24*t25;
_ekf_gsf[model_index].P(2,2) = P(2,2) - t11*t30 - t12*t27;


