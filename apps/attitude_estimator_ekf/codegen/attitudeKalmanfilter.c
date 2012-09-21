/*
 * attitudeKalmanfilter.c
 *
 * Code generation for function 'attitudeKalmanfilter'
 *
 * C source code generated on: Fri Sep 21 13:56:42 2012
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "attitudeKalmanfilter.h"
#include "norm.h"
#include "cross.h"
#include "eye.h"
#include "mrdivide.h"
#include "diag.h"
#include "find.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
static real32_T rt_atan2f_snf(real32_T u0, real32_T u1);

/* Function Definitions */
static real32_T rt_atan2f_snf(real32_T u0, real32_T u1)
{
  real32_T y;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = ((real32_T)rtNaN);
  } else if (rtIsInfF(u0) && rtIsInfF(u1)) {
    y = (real32_T)atan2f(u0 > 0.0F ? 1.0F : -1.0F, u1 > 0.0F ? 1.0F : -1.0F);
  } else if (u1 == 0.0F) {
    if (u0 > 0.0F) {
      y = RT_PIF / 2.0F;
    } else if (u0 < 0.0F) {
      y = -(RT_PIF / 2.0F);
    } else {
      y = 0.0F;
    }
  } else {
    y = (real32_T)atan2f(u0, u1);
  }

  return y;
}

/*
 * function [eulerAngles,Rot_matrix,x_aposteriori,P_aposteriori] = attitudeKalmanfilter(dt,updVect,z_k,u,x_aposteriori_k,P_aposteriori_k,knownConst)
 */
void attitudeKalmanfilter(real32_T dt, const int8_T updVect[9], const real32_T
  z_k_data[9], const int32_T z_k_sizes[1], const real32_T u[4], const real32_T
  x_aposteriori_k[9], const real32_T P_aposteriori_k[81], const real32_T
  knownConst[20], real32_T eulerAngles[3], real32_T Rot_matrix[9], real32_T
  x_aposteriori[9], real32_T P_aposteriori[81])
{
  int32_T udpIndVect_sizes;
  real_T udpIndVect_data[9];
  real32_T R_temp[9];
  real_T dv0[9];
  int32_T ib;
  int32_T i0;
  real32_T fv0[81];
  real32_T b_knownConst[27];
  real32_T fv1[9];
  real32_T c_knownConst[9];
  real_T dv1[9];
  real_T dv2[9];
  real32_T A_lin[81];
  real32_T x_n_b[3];
  real32_T K_k_data[81];
  int32_T i1;
  real32_T b_A_lin[81];
  static const int8_T iv0[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  real32_T P_apriori[81];
  int32_T ia;
  static const int8_T H_k_full[81] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  int8_T H_k_data[81];
  int32_T accUpt;
  int32_T magUpt;
  real32_T y;
  real32_T y_k_data[9];
  int32_T P_apriori_sizes[2];
  int32_T H_k_sizes[2];
  int32_T K_k_sizes[2];
  real32_T b_y[9];
  real_T dv3[81];
  real32_T c_y;
  real32_T z_n_b[3];
  real32_T y_n_b[3];

  /* Extended Attitude Kalmanfilter */
  /*  */
  /* state vector x has the following entries [ax,ay,az||mx,my,mz||wox,woy,woz||wx,wy,wz]' */
  /* measurement vector z has the following entries [ax,ay,az||mx,my,mz||wmx,wmy,wmz]' */
  /* knownConst has the following entries [PrvaA,PrvarM,PrvarWO,PrvarW||MsvarA,MsvarM,MsvarW] */
  /*  */
  /* [x_aposteriori,P_aposteriori] = AttKalman(dt,z_k,x_aposteriori_k,P_aposteriori_k,knownConst) */
  /*  */
  /* Example.... */
  /*  */
  /*  $Author: Tobias Naegeli $    $Date: 2012 $    $Revision: 1 $ */
  /* %define the matrices */
  /*  tpred=0.005; */
  /*   */
  /*  A=[   0,   0,  0,  0, 0, 0, 0, 0, 0;  */
  /*        0,   0,  0,  0, 0, 0, 0, 0, 0; */
  /*        0,   0,  0,  0, 0, 0, 0, 0, 0;   */
  /*        0,   -1,  1,  0, 0, 0, 0, 0, 0;   */
  /*        1,   0,  -1,  0, 0, 0, 0, 0, 0;   */
  /*        -1,   1,  0,  0, 0, 0, 0, 0, 0; */
  /*        0,   -1,  1,  0, 0, 0, 0, 0, 0;   */
  /*        1,   0,  -1,  0, 0, 0, 0, 0, 0;   */
  /*        -1,   1,  0,  0, 0, 0, 0, 0, 0];  */
  /*   */
  /*   */
  /*  b=[70,   0,   0;   */
  /*    0, 70,   0;  */
  /*    0,   0,   0;   */
  /*    0,   0,   0;   */
  /*    0,   0, 0;   */
  /*    0,   0,   0;   */
  /*    0,   0,   0;   */
  /*    0,   0,   0; */
  /*    0,   0,   0];   */
  /*   */
  /*   */
  /*  C=[1,   0,  0,  0, 0, 0, 0, 0, 0; */
  /*      0,   1,  0,  0, 0, 0, 0, 0, 0;   */
  /*      0,   0,  1,  0, 0, 0, 0, 0, 0; */
  /*      0,   0,  0,  1, 0, 0, 0, 0, 0; */
  /*      0,   0,  0,  0, 1, 0, 0, 0, 0; */
  /*      0,   0,  0,  0, 0, 1, 0, 0, 0; */
  /*      0,   0,  0,  0, 0, 0, 1, 0, 0; */
  /*      0,   0,  0,  0, 0, 0, 0, 1, 0; */
  /*      0,   0,  0,  0, 0, 0, 0, 0, 1]; */
  /*  D=[]; */
  /*   */
  /*   */
  /*  sys=ss(A,b,C,D); */
  /*   */
  /*  sysd=c2d(sys,tpred); */
  /*   */
  /*   */
  /*  F=sysd.a; */
  /*   */
  /*  B=sysd.b; */
  /*   */
  /*  H=sysd.c; */
  /* 'attitudeKalmanfilter:68' udpIndVect=find(updVect); */
  find(updVect, udpIndVect_data, &udpIndVect_sizes);

  /* 'attitudeKalmanfilter:71' rates_ProcessNoiseMatrix=diag([knownConst(1),knownConst(1),knownConst(2)]); */
  /* 'attitudeKalmanfilter:72' acc_ProcessNoise=knownConst(3); */
  /* 'attitudeKalmanfilter:73' mag_ProcessNoise=knownConst(4); */
  /* 'attitudeKalmanfilter:74' rates_MeasurementNoise=knownConst(6); */
  /* 'attitudeKalmanfilter:75' acc_MeasurementNoise=knownConst(7); */
  /* 'attitudeKalmanfilter:76' mag_MeasurementNoise=knownConst(8); */
  /* process noise covariance matrix */
  /* 'attitudeKalmanfilter:81' Q = [  rates_ProcessNoiseMatrix,     zeros(3),                                              zeros(3); */
  /* 'attitudeKalmanfilter:82'     zeros(3),                       eye(3)*mag_ProcessNoise,                               zeros(3); */
  /* 'attitudeKalmanfilter:83'     zeros(3),                       zeros(3),                                              eye(3)*acc_ProcessNoise]; */
  /* observation matrix */
  /* 'attitudeKalmanfilter:89' H_k_full=[   eye(3),     zeros(3),      zeros(3); */
  /* 'attitudeKalmanfilter:90'     zeros(3),   eye(3),        zeros(3); */
  /* 'attitudeKalmanfilter:91'     zeros(3),   zeros(3),        eye(3)]; */
  /* compute A(t,w) */
  /* x_aposteriori_k[10,11,12] should be [p,q,r] */
  /* R_temp=[1,-r, q */
  /*         r, 1, -p */
  /*        -q, p, 1] */
  /* 'attitudeKalmanfilter:100' R_temp=[1,                      -dt*x_aposteriori_k(3), dt*x_aposteriori_k(2); */
  /* 'attitudeKalmanfilter:101'         dt*x_aposteriori_k(3),  1,                      -dt*x_aposteriori_k(1); */
  /* 'attitudeKalmanfilter:102'         -dt*x_aposteriori_k(2), dt*x_aposteriori_k(1),  1]; */
  R_temp[0] = 1.0F;
  R_temp[3] = -dt * x_aposteriori_k[2];
  R_temp[6] = dt * x_aposteriori_k[1];
  R_temp[1] = dt * x_aposteriori_k[2];
  R_temp[4] = 1.0F;
  R_temp[7] = -dt * x_aposteriori_k[0];
  R_temp[2] = -dt * x_aposteriori_k[1];
  R_temp[5] = dt * x_aposteriori_k[0];
  R_temp[8] = 1.0F;

  /* 'attitudeKalmanfilter:106' A_pred=[eye(3),     zeros(3),       zeros(3); */
  /* 'attitudeKalmanfilter:107'         zeros(3),   R_temp',        zeros(3); */
  /* 'attitudeKalmanfilter:108'         zeros(3),   zeros(3),       R_temp']; */
  /* 'attitudeKalmanfilter:110' B_pred=[knownConst(9)*dt,   0,                  0; */
  /* 'attitudeKalmanfilter:111'         0,                  knownConst(10)*dt,  0; */
  /* 'attitudeKalmanfilter:112'         0,                  0,                  0; */
  /* 'attitudeKalmanfilter:113'         0,                  0,                  0; */
  /* 'attitudeKalmanfilter:114'         0,                  0,                  0; */
  /* 'attitudeKalmanfilter:115'         0,                  0,                  0; */
  /* 'attitudeKalmanfilter:116'         0,                  0,                  0; */
  /* 'attitudeKalmanfilter:117'         0,                  0,                  0; */
  /* 'attitudeKalmanfilter:118'         0,                  0,                  0]; */
  /* %prediction step */
  /* 'attitudeKalmanfilter:121' x_apriori=A_pred*x_aposteriori_k+B_pred*u(1:3); */
  eye(dv0);
  for (ib = 0; ib < 3; ib++) {
    for (i0 = 0; i0 < 3; i0++) {
      fv0[i0 + 9 * ib] = (real32_T)dv0[i0 + 3 * ib];
    }
  }

  for (ib = 0; ib < 3; ib++) {
    for (i0 = 0; i0 < 3; i0++) {
      fv0[i0 + 9 * (ib + 3)] = 0.0F;
    }
  }

  for (ib = 0; ib < 3; ib++) {
    for (i0 = 0; i0 < 3; i0++) {
      fv0[i0 + 9 * (ib + 6)] = 0.0F;
    }
  }

  for (ib = 0; ib < 3; ib++) {
    for (i0 = 0; i0 < 3; i0++) {
      fv0[(i0 + 9 * ib) + 3] = 0.0F;
    }
  }

  for (ib = 0; ib < 3; ib++) {
    for (i0 = 0; i0 < 3; i0++) {
      fv0[(i0 + 9 * (ib + 3)) + 3] = R_temp[ib + 3 * i0];
    }
  }

  for (ib = 0; ib < 3; ib++) {
    for (i0 = 0; i0 < 3; i0++) {
      fv0[(i0 + 9 * (ib + 6)) + 3] = 0.0F;
    }
  }

  for (ib = 0; ib < 3; ib++) {
    for (i0 = 0; i0 < 3; i0++) {
      fv0[(i0 + 9 * ib) + 6] = 0.0F;
    }
  }

  for (ib = 0; ib < 3; ib++) {
    for (i0 = 0; i0 < 3; i0++) {
      fv0[(i0 + 9 * (ib + 3)) + 6] = 0.0F;
    }
  }

  b_knownConst[0] = knownConst[8] * dt;
  b_knownConst[9] = 0.0F;
  b_knownConst[18] = 0.0F;
  b_knownConst[1] = 0.0F;
  b_knownConst[10] = knownConst[9] * dt;
  b_knownConst[19] = 0.0F;
  for (ib = 0; ib < 3; ib++) {
    for (i0 = 0; i0 < 3; i0++) {
      fv0[(i0 + 9 * (ib + 6)) + 6] = R_temp[ib + 3 * i0];
    }

    b_knownConst[2 + 9 * ib] = 0.0F;
    b_knownConst[3 + 9 * ib] = 0.0F;
    b_knownConst[4 + 9 * ib] = 0.0F;
    b_knownConst[5 + 9 * ib] = 0.0F;
    b_knownConst[6 + 9 * ib] = 0.0F;
    b_knownConst[7 + 9 * ib] = 0.0F;
    b_knownConst[8 + 9 * ib] = 0.0F;
  }

  for (ib = 0; ib < 9; ib++) {
    fv1[ib] = 0.0F;
    for (i0 = 0; i0 < 9; i0++) {
      fv1[ib] += fv0[ib + 9 * i0] * x_aposteriori_k[i0];
    }

    c_knownConst[ib] = 0.0F;
    for (i0 = 0; i0 < 3; i0++) {
      c_knownConst[ib] += b_knownConst[ib + 9 * i0] * u[i0];
    }

    x_aposteriori[ib] = fv1[ib] + c_knownConst[ib];
  }

  /* linearization */
  /* 'attitudeKalmanfilter:125' temp_mat=[  0,      -dt,    dt; */
  /* 'attitudeKalmanfilter:126'             dt,    0,     -dt; */
  /* 'attitudeKalmanfilter:127'             -dt,     dt,   0]; */
  R_temp[0] = 0.0F;
  R_temp[3] = -dt;
  R_temp[6] = dt;
  R_temp[1] = dt;
  R_temp[4] = 0.0F;
  R_temp[7] = -dt;
  R_temp[2] = -dt;
  R_temp[5] = dt;
  R_temp[8] = 0.0F;

  /* 'attitudeKalmanfilter:129' A_lin=[ eye(3),         zeros(3),     zeros(3); */
  /* 'attitudeKalmanfilter:130'         temp_mat,  eye(3) ,     zeros(3); */
  /* 'attitudeKalmanfilter:131'         temp_mat,  zeros(3),     eye(3)]; */
  eye(dv0);
  eye(dv1);
  eye(dv2);
  for (ib = 0; ib < 3; ib++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_lin[i0 + 9 * ib] = (real32_T)dv0[i0 + 3 * ib];
    }
  }

  for (ib = 0; ib < 3; ib++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_lin[i0 + 9 * (ib + 3)] = 0.0F;
    }
  }

  for (ib = 0; ib < 3; ib++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_lin[i0 + 9 * (ib + 6)] = 0.0F;
    }
  }

  for (ib = 0; ib < 3; ib++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_lin[(i0 + 9 * ib) + 3] = R_temp[i0 + 3 * ib];
    }
  }

  for (ib = 0; ib < 3; ib++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_lin[(i0 + 9 * (ib + 3)) + 3] = (real32_T)dv1[i0 + 3 * ib];
    }
  }

  for (ib = 0; ib < 3; ib++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_lin[(i0 + 9 * (ib + 6)) + 3] = 0.0F;
    }
  }

  for (ib = 0; ib < 3; ib++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_lin[(i0 + 9 * ib) + 6] = R_temp[i0 + 3 * ib];
    }
  }

  for (ib = 0; ib < 3; ib++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_lin[(i0 + 9 * (ib + 3)) + 6] = 0.0F;
    }
  }

  for (ib = 0; ib < 3; ib++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_lin[(i0 + 9 * (ib + 6)) + 6] = (real32_T)dv2[i0 + 3 * ib];
    }
  }

  /* 'attitudeKalmanfilter:134' P_apriori=A_lin*P_aposteriori_k*A_lin'+Q; */
  x_n_b[0] = knownConst[0];
  x_n_b[1] = knownConst[0];
  x_n_b[2] = knownConst[1];
  diag(x_n_b, R_temp);
  for (ib = 0; ib < 9; ib++) {
    for (i0 = 0; i0 < 9; i0++) {
      K_k_data[ib + 9 * i0] = 0.0F;
      for (i1 = 0; i1 < 9; i1++) {
        K_k_data[ib + 9 * i0] += A_lin[ib + 9 * i1] * P_aposteriori_k[i1 + 9 *
          i0];
      }
    }

    for (i0 = 0; i0 < 9; i0++) {
      b_A_lin[ib + 9 * i0] = 0.0F;
      for (i1 = 0; i1 < 9; i1++) {
        b_A_lin[ib + 9 * i0] += K_k_data[ib + 9 * i1] * A_lin[i0 + 9 * i1];
      }
    }
  }

  for (ib = 0; ib < 3; ib++) {
    for (i0 = 0; i0 < 3; i0++) {
      K_k_data[i0 + 9 * ib] = R_temp[i0 + 3 * ib];
    }
  }

  for (ib = 0; ib < 3; ib++) {
    for (i0 = 0; i0 < 3; i0++) {
      K_k_data[i0 + 9 * (ib + 3)] = 0.0F;
    }
  }

  for (ib = 0; ib < 3; ib++) {
    for (i0 = 0; i0 < 3; i0++) {
      K_k_data[i0 + 9 * (ib + 6)] = 0.0F;
    }
  }

  for (ib = 0; ib < 3; ib++) {
    for (i0 = 0; i0 < 3; i0++) {
      K_k_data[(i0 + 9 * ib) + 3] = 0.0F;
    }
  }

  for (ib = 0; ib < 3; ib++) {
    for (i0 = 0; i0 < 3; i0++) {
      K_k_data[(i0 + 9 * (ib + 3)) + 3] = (real32_T)iv0[i0 + 3 * ib] *
        knownConst[3];
    }
  }

  for (ib = 0; ib < 3; ib++) {
    for (i0 = 0; i0 < 3; i0++) {
      K_k_data[(i0 + 9 * (ib + 6)) + 3] = 0.0F;
    }
  }

  for (ib = 0; ib < 3; ib++) {
    for (i0 = 0; i0 < 3; i0++) {
      K_k_data[(i0 + 9 * ib) + 6] = 0.0F;
    }
  }

  for (ib = 0; ib < 3; ib++) {
    for (i0 = 0; i0 < 3; i0++) {
      K_k_data[(i0 + 9 * (ib + 3)) + 6] = 0.0F;
    }
  }

  for (ib = 0; ib < 3; ib++) {
    for (i0 = 0; i0 < 3; i0++) {
      K_k_data[(i0 + 9 * (ib + 6)) + 6] = (real32_T)iv0[i0 + 3 * ib] *
        knownConst[2];
    }
  }

  for (ib = 0; ib < 9; ib++) {
    for (i0 = 0; i0 < 9; i0++) {
      P_apriori[i0 + 9 * ib] = b_A_lin[i0 + 9 * ib] + K_k_data[i0 + 9 * ib];
    }
  }

  /* 'attitudeKalmanfilter:137' if ~isempty(udpIndVect)==1 */
  if (!(udpIndVect_sizes == 0) == 1) {
    /* 'attitudeKalmanfilter:138' H_k= H_k_full(udpIndVect,:); */
    for (ib = 0; ib < 9; ib++) {
      ia = udpIndVect_sizes - 1;
      for (i0 = 0; i0 <= ia; i0++) {
        H_k_data[i0 + udpIndVect_sizes * ib] = H_k_full[((int32_T)
          udpIndVect_data[i0] + 9 * ib) - 1];
      }
    }

    /* %update step */
    /* 'attitudeKalmanfilter:140' accUpt=1; */
    accUpt = 1;

    /* 'attitudeKalmanfilter:141' magUpt=1; */
    magUpt = 1;

    /* 'attitudeKalmanfilter:142' y_k=z_k-H_k*x_apriori; */
    ia = udpIndVect_sizes - 1;
    for (ib = 0; ib <= ia; ib++) {
      y = 0.0F;
      for (i0 = 0; i0 < 9; i0++) {
        y += (real32_T)H_k_data[ib + udpIndVect_sizes * i0] * x_aposteriori[i0];
      }

      y_k_data[ib] = z_k_data[ib] - y;
    }

    /* 'attitudeKalmanfilter:143' if updVect(4)==1 */
    if (updVect[3] == 1) {
      /* 'attitudeKalmanfilter:144' if (abs(norm(z_k(4:6))-knownConst(12))>knownConst(14)) */
      for (ib = 0; ib < 3; ib++) {
        x_n_b[ib] = z_k_data[ib + 3];
      }

      if ((real32_T)fabsf(norm(x_n_b) - knownConst[11]) > knownConst[13]) {
        /* 'attitudeKalmanfilter:145' accUpt=10000; */
        accUpt = 10000;
      }
    }

    /* 'attitudeKalmanfilter:149' if updVect(7)==1 */
    if (updVect[6] == 1) {
      /* 'attitudeKalmanfilter:150' if (abs(norm(z_k(7:9))-knownConst(13))>knownConst(15)) */
      for (ib = 0; ib < 3; ib++) {
        x_n_b[ib] = z_k_data[ib + 6];
      }

      if ((real32_T)fabs(norm(x_n_b) - knownConst[12]) > knownConst[14]) {
        /* 'attitudeKalmanfilter:152' magUpt=10000; */
        magUpt = 10000;
      }
    }

    /* measurement noise covariance matrix */
    /* 'attitudeKalmanfilter:157' R = [   eye(3)*rates_MeasurementNoise,       zeros(3),                       zeros(3); */
    /* 'attitudeKalmanfilter:158'             zeros(3),                            eye(3)*acc_MeasurementNoise*accUpt,    zeros(3); */
    /* 'attitudeKalmanfilter:159'             zeros(3),                            zeros(3),                       eye(3)*mag_MeasurementNoise*magUpt]; */
    /* 'attitudeKalmanfilter:161' S_k=H_k*P_apriori*H_k'+R(udpIndVect,udpIndVect); */
    /* 'attitudeKalmanfilter:162' K_k=(P_apriori*H_k'/(S_k)); */
    P_apriori_sizes[0] = 9;
    P_apriori_sizes[1] = udpIndVect_sizes;
    for (ib = 0; ib < 9; ib++) {
      ia = udpIndVect_sizes - 1;
      for (i0 = 0; i0 <= ia; i0++) {
        b_A_lin[ib + 9 * i0] = 0.0F;
        for (i1 = 0; i1 < 9; i1++) {
          b_A_lin[ib + 9 * i0] += P_apriori[ib + 9 * i1] * (real32_T)H_k_data[i0
            + udpIndVect_sizes * i1];
        }
      }
    }

    ia = udpIndVect_sizes - 1;
    for (ib = 0; ib <= ia; ib++) {
      for (i0 = 0; i0 < 9; i0++) {
        K_k_data[ib + udpIndVect_sizes * i0] = 0.0F;
        for (i1 = 0; i1 < 9; i1++) {
          K_k_data[ib + udpIndVect_sizes * i0] += (real32_T)H_k_data[ib +
            udpIndVect_sizes * i1] * P_apriori[i1 + 9 * i0];
        }
      }
    }

    for (ib = 0; ib < 3; ib++) {
      for (i0 = 0; i0 < 3; i0++) {
        fv0[i0 + 9 * ib] = (real32_T)iv0[i0 + 3 * ib] * knownConst[5];
      }
    }

    for (ib = 0; ib < 3; ib++) {
      for (i0 = 0; i0 < 3; i0++) {
        fv0[i0 + 9 * (ib + 3)] = 0.0F;
      }
    }

    for (ib = 0; ib < 3; ib++) {
      for (i0 = 0; i0 < 3; i0++) {
        fv0[i0 + 9 * (ib + 6)] = 0.0F;
      }
    }

    for (ib = 0; ib < 3; ib++) {
      for (i0 = 0; i0 < 3; i0++) {
        fv0[(i0 + 9 * ib) + 3] = 0.0F;
      }
    }

    for (ib = 0; ib < 3; ib++) {
      for (i0 = 0; i0 < 3; i0++) {
        fv0[(i0 + 9 * (ib + 3)) + 3] = (real32_T)iv0[i0 + 3 * ib] * knownConst[6]
          * (real32_T)accUpt;
      }
    }

    for (ib = 0; ib < 3; ib++) {
      for (i0 = 0; i0 < 3; i0++) {
        fv0[(i0 + 9 * (ib + 6)) + 3] = 0.0F;
      }
    }

    for (ib = 0; ib < 3; ib++) {
      for (i0 = 0; i0 < 3; i0++) {
        fv0[(i0 + 9 * ib) + 6] = 0.0F;
      }
    }

    for (ib = 0; ib < 3; ib++) {
      for (i0 = 0; i0 < 3; i0++) {
        fv0[(i0 + 9 * (ib + 3)) + 6] = 0.0F;
      }
    }

    for (ib = 0; ib < 3; ib++) {
      for (i0 = 0; i0 < 3; i0++) {
        fv0[(i0 + 9 * (ib + 6)) + 6] = (real32_T)iv0[i0 + 3 * ib] * knownConst[7]
          * (real32_T)magUpt;
      }
    }

    H_k_sizes[0] = udpIndVect_sizes;
    H_k_sizes[1] = udpIndVect_sizes;
    ia = udpIndVect_sizes - 1;
    for (ib = 0; ib <= ia; ib++) {
      accUpt = udpIndVect_sizes - 1;
      for (i0 = 0; i0 <= accUpt; i0++) {
        y = 0.0F;
        for (i1 = 0; i1 < 9; i1++) {
          y += K_k_data[ib + udpIndVect_sizes * i1] * (real32_T)H_k_data[i0 +
            udpIndVect_sizes * i1];
        }

        A_lin[ib + H_k_sizes[0] * i0] = y + fv0[((int32_T)udpIndVect_data[ib] +
          9 * ((int32_T)udpIndVect_data[i0] - 1)) - 1];
      }
    }

    mrdivide(b_A_lin, P_apriori_sizes, A_lin, H_k_sizes, K_k_data, K_k_sizes);

    /* 'attitudeKalmanfilter:165' x_aposteriori=x_apriori+K_k*y_k; */
    if ((K_k_sizes[1] == 1) || (udpIndVect_sizes == 1)) {
      for (ib = 0; ib < 9; ib++) {
        b_y[ib] = 0.0F;
        ia = udpIndVect_sizes - 1;
        for (i0 = 0; i0 <= ia; i0++) {
          b_y[ib] += K_k_data[ib + K_k_sizes[0] * i0] * y_k_data[i0];
        }
      }
    } else {
      for (accUpt = 0; accUpt < 9; accUpt++) {
        b_y[accUpt] = 0.0F;
      }

      magUpt = -1;
      for (ib = 0; ib + 1 <= K_k_sizes[1]; ib++) {
        if ((real_T)y_k_data[ib] != 0.0) {
          ia = magUpt;
          for (accUpt = 0; accUpt < 9; accUpt++) {
            ia++;
            b_y[accUpt] += y_k_data[ib] * K_k_data[ia];
          }
        }

        magUpt += 9;
      }
    }

    for (ib = 0; ib < 9; ib++) {
      x_aposteriori[ib] += b_y[ib];
    }

    /* 'attitudeKalmanfilter:166' P_aposteriori=(eye(9)-K_k*H_k)*P_apriori; */
    b_eye(dv3);
    for (ib = 0; ib < 9; ib++) {
      for (i0 = 0; i0 < 9; i0++) {
        y = 0.0F;
        ia = K_k_sizes[1] - 1;
        for (i1 = 0; i1 <= ia; i1++) {
          y += K_k_data[ib + K_k_sizes[0] * i1] * (real32_T)H_k_data[i1 +
            udpIndVect_sizes * i0];
        }

        fv0[ib + 9 * i0] = (real32_T)dv3[ib + 9 * i0] - y;
      }
    }

    for (ib = 0; ib < 9; ib++) {
      for (i0 = 0; i0 < 9; i0++) {
        P_aposteriori[ib + 9 * i0] = 0.0F;
        for (i1 = 0; i1 < 9; i1++) {
          P_aposteriori[ib + 9 * i0] += fv0[ib + 9 * i1] * P_apriori[i1 + 9 * i0];
        }
      }
    }
  } else {
    /* 'attitudeKalmanfilter:167' else */
    /* 'attitudeKalmanfilter:168' x_aposteriori=x_apriori; */
    /* 'attitudeKalmanfilter:169' P_aposteriori=P_apriori; */
    memcpy((void *)&P_aposteriori[0], (void *)&P_apriori[0], 81U * sizeof
           (real32_T));
  }

  /*         %% euler anglels extraction */
  /* 'attitudeKalmanfilter:175' z_n_b = -x_aposteriori(4:6)./norm(x_aposteriori(4:6)); */
  y = norm(*(real32_T (*)[3])&x_aposteriori[3]);

  /* 'attitudeKalmanfilter:176' m_n_b = x_aposteriori(7:9)./norm(x_aposteriori(7:9)); */
  c_y = norm(*(real32_T (*)[3])&x_aposteriori[6]);

  /* 'attitudeKalmanfilter:178' y_n_b=cross(z_n_b,m_n_b); */
  for (accUpt = 0; accUpt < 3; accUpt++) {
    z_n_b[accUpt] = -x_aposteriori[accUpt + 3] / y;
    x_n_b[accUpt] = x_aposteriori[accUpt + 6] / c_y;
  }

  cross(z_n_b, x_n_b, y_n_b);

  /* 'attitudeKalmanfilter:179' y_n_b=y_n_b./norm(y_n_b); */
  y = norm(y_n_b);
  for (ib = 0; ib < 3; ib++) {
    y_n_b[ib] /= y;
  }

  /* 'attitudeKalmanfilter:181' x_n_b=(cross(y_n_b,z_n_b)); */
  cross(y_n_b, z_n_b, x_n_b);

  /* 'attitudeKalmanfilter:182' x_n_b=x_n_b./norm(x_n_b); */
  y = norm(x_n_b);
  for (ib = 0; ib < 3; ib++) {
    /* 'attitudeKalmanfilter:188' Rot_matrix=[x_n_b,y_n_b,z_n_b]; */
    Rot_matrix[ib] = x_n_b[ib] / y;
    Rot_matrix[3 + ib] = y_n_b[ib];
    Rot_matrix[6 + ib] = z_n_b[ib];
  }

  /* 'attitudeKalmanfilter:192' phi=atan2(Rot_matrix(2,3),Rot_matrix(3,3)); */
  /* 'attitudeKalmanfilter:193' theta=-asin(Rot_matrix(1,3)); */
  /* 'attitudeKalmanfilter:194' psi=atan2(Rot_matrix(1,2),Rot_matrix(1,1)); */
  /* 'attitudeKalmanfilter:195' eulerAngles=[phi;theta;psi]; */
  eulerAngles[0] = rt_atan2f_snf(Rot_matrix[7], Rot_matrix[8]);
  eulerAngles[1] = -(real32_T)asinf(Rot_matrix[6]);
  eulerAngles[2] = rt_atan2f_snf(Rot_matrix[3], Rot_matrix[0]);
}

/* End of code generation (attitudeKalmanfilter.c) */
