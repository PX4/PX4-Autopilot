// Sub Expressions
const float IV0 = ve - vwe;
const float IV1 = vn - vwn;
const float IV2 = (IV0)*(IV0) + (IV1)*(IV1) + (vd)*(vd);
const float IV3 = 1.0F/(IV2);
const float IV4 = IV0*P(5,23);
const float IV5 = IV0*IV3;
const float IV6 = IV1*P(4,22);
const float IV7 = IV1*IV3;


// Observation Jacobians


// Kalman gains


// Predicted observation
meas_pred = sqrtf(IV2);


// Innovation variance
innov_var = IV3*vd*(IV0*P(5,6) - IV0*P(6,23) + IV1*P(4,6) - IV1*P(6,22) + P(6,6)*vd) - IV5*(-IV0*P(23,23) - IV1*P(22,23) + IV1*P(4,23) + IV4 + P(6,23)*vd) + IV5*(IV0*P(5,5) + IV1*P(4,5) - IV1*P(5,22) - IV4 + P(5,6)*vd) - IV7*(-IV0*P(22,23) + IV0*P(5,22) - IV1*P(22,22) + IV6 + P(6,22)*vd) + IV7*(-IV0*P(4,23) + IV0*P(4,5) + IV1*P(4,4) - IV6 + P(4,6)*vd) + R_TAS;


