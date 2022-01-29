// Equations for covariance matrix prediction
const float S0 = cosf(psi);
const float S1 = powf(S0, 2);
const float S2 = sinf(psi);
const float S3 = powf(S2, 2);
const float S4 = S0*dvy + S2*dvx;
const float S5 = P(0,2) - P(2,2)*S4;
const float S6 = S0*dvx - S2*dvy;
const float S7 = S0*S2;
const float S8 = P(0,1) + S7*dvxVar - S7*dvyVar;
const float S9 = P(1,2) + P(2,2)*S6;


_ekf_gsf[model_index].P(0,0) = P(0,0) - P(0,2)*S4 + S1*dvxVar + S3*dvyVar - S4*S5;
_ekf_gsf[model_index].P(0,1) = -P(1,2)*S4 + S5*S6 + S8;
_ekf_gsf[model_index].P(1,1) = P(1,1) + P(1,2)*S6 + S1*dvyVar + S3*dvxVar + S6*S9;
_ekf_gsf[model_index].P(0,2) = S5;
_ekf_gsf[model_index].P(1,2) = S9;
_ekf_gsf[model_index].P(2,2) = P(2,2) + dazVar;
