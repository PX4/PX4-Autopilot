H_VEL[0] = q1*vd*2.0f+q0*ve*2.0f-q3*vn*2.0f;
H_VEL[1] = q0*vd*2.0f-q1*ve*2.0f+q2*vn*2.0f;
H_VEL[2] = q3*vd*2.0f+q2*ve*2.0f+q1*vn*2.0f;
H_VEL[3] = q2*vd*2.0f-q3*ve*2.0f-q0*vn*2.0f;
H_VEL[4] = q0*q3*-2.0f+q1*q2*2.0f;
H_VEL[5] = q0*q0-q1*q1+q2*q2-q3*q3;
H_VEL[6] = q0*q1*2.0f+q2*q3*2.0f;
