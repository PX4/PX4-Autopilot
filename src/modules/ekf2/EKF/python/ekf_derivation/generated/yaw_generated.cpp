// calculate 321 yaw observation matrix - option A
const float SA0 = 2*q3;
const float SA1 = 2*q2;
const float SA2 = SA0*q0 + SA1*q1;
const float SA3 = (q0)*(q0) + (q1)*(q1) - (q2)*(q2) - (q3)*(q3);
const float SA4 = 1.0F/((SA3)*(SA3));
const float SA5 = 1.0F/((SA2)*(SA2)*SA4 + 1);
const float SA6 = 1.0F/(SA3);
const float SA7 = SA2*SA4;
const float SA8 = 2*SA7;
const float SA9 = 2*SA6;


H_YAW.at<0>() = SA5*(2*SA6*q3 - SA8*q0);
H_YAW.at<1>() = SA5*(2*SA6*q2 - SA8*q1);
H_YAW.at<2>() = SA5*(SA1*SA7 + SA9*q1);
H_YAW.at<3>() = SA5*(SA0*SA7 + SA9*q0);


// calculate 321 yaw observation matrix - option B
const float SB0 = 2*q0;
const float SB1 = 2*q1;
const float SB2 = SB0*q3 + SB1*q2;
const float SB3 = 1.0F/((SB2)*(SB2));
const float SB4 = (q0)*(q0) + (q1)*(q1) - (q2)*(q2) - (q3)*(q3);
const float SB5 = 1.0F/(SB3*(SB4)*(SB4) + 1);
const float SB6 = 1.0F/(SB2);
const float SB7 = SB3*SB4;
const float SB8 = 2*SB7;
const float SB9 = 2*SB6;


H_YAW.at<0>() = -SB5*(SB0*SB6 - SB8*q3);
H_YAW.at<1>() = -SB5*(SB1*SB6 - SB8*q2);
H_YAW.at<2>() = -SB5*(-SB1*SB7 - SB9*q2);
H_YAW.at<3>() = -SB5*(-SB0*SB7 - SB9*q3);


// calculate 312 yaw observation matrix - option A
const float SA0 = 2*q3;
const float SA1 = 2*q2;
const float SA2 = SA0*q0 - SA1*q1;
const float SA3 = (q0)*(q0) - (q1)*(q1) + (q2)*(q2) - (q3)*(q3);
const float SA4 = 1.0F/((SA3)*(SA3));
const float SA5 = 1.0F/((SA2)*(SA2)*SA4 + 1);
const float SA6 = 1.0F/(SA3);
const float SA7 = SA2*SA4;
const float SA8 = 2*SA7;
const float SA9 = 2*SA6;


H_YAW.at<0>() = SA5*(2*SA6*q3 - SA8*q0);
H_YAW.at<1>() = SA5*(-SA1*SA6 + SA8*q1);
H_YAW.at<2>() = SA5*(-SA1*SA7 - SA9*q1);
H_YAW.at<3>() = SA5*(SA0*SA7 + SA9*q0);


// calculate 312 yaw observation matrix - option B
const float SB0 = 2*q0;
const float SB1 = -SB0*q3 + 2*q1*q2;
const float SB2 = 1.0F/((SB1)*(SB1));
const float SB3 = -(q0)*(q0) + (q1)*(q1) - (q2)*(q2) + (q3)*(q3);
const float SB4 = 1.0F/(SB2*(SB3)*(SB3) + 1);
const float SB5 = 1.0F/(SB1);
const float SB6 = 2*q1;
const float SB7 = SB2*SB3;
const float SB8 = 2*SB5;


H_YAW.at<0>() = -SB4*(-SB0*SB5 + 2*SB2*SB3*q3);
H_YAW.at<1>() = -SB4*(SB5*SB6 - 2*SB7*q2);
H_YAW.at<2>() = -SB4*(-SB6*SB7 - SB8*q2);
H_YAW.at<3>() = -SB4*(SB0*SB7 + SB8*q3);


