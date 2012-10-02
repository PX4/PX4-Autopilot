/*
 * attitudeKalmanfilter.c
 *
 * Code generation for function 'attitudeKalmanfilter'
 *
 * C source code generated on: Mon Oct 01 19:38:49 2012
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
#include "power.h"

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
 * function [eulerAngles,Rot_matrix,x_aposteriori,P_aposteriori] = attitudeKalmanfilter_wo(updateVect,dt,z,x_aposteriori_k,P_aposteriori_k,q,r)
 */
void attitudeKalmanfilter(const uint8_T updateVect[3], real32_T dt, const
  real32_T z[9], const real32_T x_aposteriori_k[12], const real32_T
  P_aposteriori_k[144], const real32_T q[12], const real32_T r[9], real32_T
  eulerAngles[3], real32_T Rot_matrix[9], real32_T x_aposteriori[12], real32_T
  P_aposteriori[144])
{
  real32_T a[12];
  int32_T i;
  real32_T b_a[12];
  real32_T Q[144];
  real32_T O[9];
  real_T dv0[9];
  real32_T c_a[9];
  real32_T d_a[9];
  real32_T x_n_b[3];
  real32_T z_n_b[3];
  real32_T x_apriori[12];
  real32_T y_n_b[3];
  int32_T i0;
  real32_T e_a[3];
  real_T dv1[144];
  real32_T A_lin[144];
  real32_T b_A_lin[144];
  int32_T i1;
  real32_T y;
  real32_T P_apriori[144];
  real32_T R[81];
  real32_T b_P_apriori[108];
  static const int8_T iv0[108] = { 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  real32_T K_k[108];
  static const int8_T iv1[108] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  real32_T fv0[81];
  real32_T c_P_apriori[36];
  static const int8_T iv2[36] = { 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0 };

  real32_T fv1[36];
  static const int8_T iv3[36] = { 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  real32_T S_k[36];
  real32_T d_P_apriori[72];
  static const int8_T iv4[72] = { 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0 };

  real32_T b_K_k[72];
  static const int8_T iv5[72] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0 };

  real32_T b_r[6];
  static const int8_T iv6[72] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 1 };

  static const int8_T iv7[72] = { 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1 };

  real32_T fv2[6];
  real32_T b_z[6];
  real32_T b_y;

  /*  Extended Attitude Kalmanfilter */
  /*  */
  /*  state vector x has the following entries [ax,ay,az||mx,my,mz||wox,woy,woz||wx,wy,wz]' */
  /*  measurement vector z has the following entries [ax,ay,az||mx,my,mz||wmx,wmy,wmz]' */
  /*  knownConst has the following entries [PrvaA,PrvarM,PrvarWO,PrvarW||MsvarA,MsvarM,MsvarW] */
  /*  */
  /*  [x_aposteriori,P_aposteriori] = AttKalman(dt,z_k,x_aposteriori_k,P_aposteriori_k,knownConst) */
  /*  */
  /*  Example.... */
  /*  */
  /*  $Author: Tobias Naegeli $    $Date: 2012 $    $Revision: 1 $ */
  /* coder.varsize('udpIndVect', [9,1], [1,0]) */
  /* udpIndVect=find(updVect); */
  /* process and measurement noise covariance matrix */
  /* 'attitudeKalmanfilter:27' Q = diag(q.^2*dt); */
  power(q, 2.0, a);
  for (i = 0; i < 12; i++) {
    b_a[i] = a[i] * dt;
  }

  diag(b_a, Q);

  /* observation matrix */
  /* 'attitudeKalmanfilter:37' wx=  x_aposteriori_k(1); */
  /* 'attitudeKalmanfilter:38' wy=  x_aposteriori_k(2); */
  /* 'attitudeKalmanfilter:39' wz=  x_aposteriori_k(3); */
  /* 'attitudeKalmanfilter:41' wox=  x_aposteriori_k(4); */
  /* 'attitudeKalmanfilter:42' woy=  x_aposteriori_k(5); */
  /* 'attitudeKalmanfilter:43' woz=  x_aposteriori_k(6); */
  /* 'attitudeKalmanfilter:45' zex=  x_aposteriori_k(7); */
  /* 'attitudeKalmanfilter:46' zey=  x_aposteriori_k(8); */
  /* 'attitudeKalmanfilter:47' zez=  x_aposteriori_k(9); */
  /* 'attitudeKalmanfilter:49' mux=  x_aposteriori_k(10); */
  /* 'attitudeKalmanfilter:50' muy=  x_aposteriori_k(11); */
  /* 'attitudeKalmanfilter:51' muz=  x_aposteriori_k(12); */
  /* 'attitudeKalmanfilter:54' wk =[wx; */
  /* 'attitudeKalmanfilter:55'      wy; */
  /* 'attitudeKalmanfilter:56'      wz]; */
  /* 'attitudeKalmanfilter:58' wok =[wox;woy;woz]; */
  /* 'attitudeKalmanfilter:59' O=[0,-wz,wy;wz,0,-wx;-wy,wx,0]'; */
  O[0] = 0.0F;
  O[1] = -x_aposteriori_k[2];
  O[2] = x_aposteriori_k[1];
  O[3] = x_aposteriori_k[2];
  O[4] = 0.0F;
  O[5] = -x_aposteriori_k[0];
  O[6] = -x_aposteriori_k[1];
  O[7] = x_aposteriori_k[0];
  O[8] = 0.0F;

  /* 'attitudeKalmanfilter:60' zek =(eye(3)+O*dt)*[zex;zey;zez]; */
  eye(dv0);
  for (i = 0; i < 9; i++) {
    c_a[i] = (real32_T)dv0[i] + O[i] * dt;
  }

  /* 'attitudeKalmanfilter:61' muk =(eye(3)+O*dt)*[mux;muy;muz]; */
  eye(dv0);
  for (i = 0; i < 9; i++) {
    d_a[i] = (real32_T)dv0[i] + O[i] * dt;
  }

  /* 'attitudeKalmanfilter:63' EZ=[0,zez,-zey; */
  /* 'attitudeKalmanfilter:64'     -zez,0,zex; */
  /* 'attitudeKalmanfilter:65'     zey,-zex,0]'; */
  /* 'attitudeKalmanfilter:66' MA=[0,muz,-muy; */
  /* 'attitudeKalmanfilter:67'     -muz,0,mux; */
  /* 'attitudeKalmanfilter:68'     zey,-mux,0]'; */
  /* 'attitudeKalmanfilter:72' E=eye(3); */
  /* 'attitudeKalmanfilter:73' Z=zeros(3); */
  /* 'attitudeKalmanfilter:74' x_apriori=[wk;wok;zek;muk]; */
  x_n_b[0] = x_aposteriori_k[6];
  x_n_b[1] = x_aposteriori_k[7];
  x_n_b[2] = x_aposteriori_k[8];
  z_n_b[0] = x_aposteriori_k[9];
  z_n_b[1] = x_aposteriori_k[10];
  z_n_b[2] = x_aposteriori_k[11];
  x_apriori[0] = x_aposteriori_k[0];
  x_apriori[1] = x_aposteriori_k[1];
  x_apriori[2] = x_aposteriori_k[2];
  x_apriori[3] = x_aposteriori_k[3];
  x_apriori[4] = x_aposteriori_k[4];
  x_apriori[5] = x_aposteriori_k[5];
  for (i = 0; i < 3; i++) {
    y_n_b[i] = 0.0F;
    for (i0 = 0; i0 < 3; i0++) {
      y_n_b[i] += c_a[i + 3 * i0] * x_n_b[i0];
    }

    e_a[i] = 0.0F;
    for (i0 = 0; i0 < 3; i0++) {
      e_a[i] += d_a[i + 3 * i0] * z_n_b[i0];
    }

    x_apriori[i + 6] = y_n_b[i];
  }

  for (i = 0; i < 3; i++) {
    x_apriori[i + 9] = e_a[i];
  }

  /* 'attitudeKalmanfilter:76' A_lin=[ Z,  Z,  Z,  Z */
  /* 'attitudeKalmanfilter:77'         Z,  Z,  Z,  Z */
  /* 'attitudeKalmanfilter:78'         EZ, Z,  O,  Z */
  /* 'attitudeKalmanfilter:79'         MA, Z,  Z,  O]; */
  /* 'attitudeKalmanfilter:82' A_lin=eye(12)+A_lin*dt; */
  b_eye(dv1);
  for (i = 0; i < 12; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_lin[i0 + 12 * i] = 0.0F;
    }

    for (i0 = 0; i0 < 3; i0++) {
      A_lin[(i0 + 12 * i) + 3] = 0.0F;
    }
  }

  A_lin[6] = 0.0F;
  A_lin[7] = x_aposteriori_k[8];
  A_lin[8] = -x_aposteriori_k[7];
  A_lin[18] = -x_aposteriori_k[8];
  A_lin[19] = 0.0F;
  A_lin[20] = x_aposteriori_k[6];
  A_lin[30] = x_aposteriori_k[7];
  A_lin[31] = -x_aposteriori_k[6];
  A_lin[32] = 0.0F;
  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_lin[(i0 + 12 * (i + 3)) + 6] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_lin[(i0 + 12 * (i + 6)) + 6] = O[i0 + 3 * i];
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_lin[(i0 + 12 * (i + 9)) + 6] = 0.0F;
    }
  }

  A_lin[9] = 0.0F;
  A_lin[10] = x_aposteriori_k[11];
  A_lin[11] = -x_aposteriori_k[10];
  A_lin[21] = -x_aposteriori_k[11];
  A_lin[22] = 0.0F;
  A_lin[23] = x_aposteriori_k[9];
  A_lin[33] = x_aposteriori_k[7];
  A_lin[34] = -x_aposteriori_k[9];
  A_lin[35] = 0.0F;
  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_lin[(i0 + 12 * (i + 3)) + 9] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_lin[(i0 + 12 * (i + 6)) + 9] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_lin[(i0 + 12 * (i + 9)) + 9] = O[i0 + 3 * i];
    }
  }

  for (i = 0; i < 12; i++) {
    for (i0 = 0; i0 < 12; i0++) {
      b_A_lin[i0 + 12 * i] = (real32_T)dv1[i0 + 12 * i] + A_lin[i0 + 12 * i] *
        dt;
    }
  }

  /* 'attitudeKalmanfilter:88' P_apriori=A_lin*P_aposteriori_k*A_lin'+Q; */
  for (i = 0; i < 12; i++) {
    for (i0 = 0; i0 < 12; i0++) {
      A_lin[i + 12 * i0] = 0.0F;
      for (i1 = 0; i1 < 12; i1++) {
        A_lin[i + 12 * i0] += b_A_lin[i + 12 * i1] * P_aposteriori_k[i1 + 12 *
          i0];
      }
    }
  }

  for (i = 0; i < 12; i++) {
    for (i0 = 0; i0 < 12; i0++) {
      y = 0.0F;
      for (i1 = 0; i1 < 12; i1++) {
        y += A_lin[i + 12 * i1] * b_A_lin[i0 + 12 * i1];
      }

      P_apriori[i + 12 * i0] = y + Q[i + 12 * i0];
    }
  }

  /* %update */
  /* 'attitudeKalmanfilter:92' if updateVect(1)==1&&updateVect(2)==1&&updateVect(3)==1 */
  if ((updateVect[0] == 1) && (updateVect[1] == 1) && (updateVect[2] == 1)) {
    /* 'attitudeKalmanfilter:93' R=diag(r); */
    b_diag(r, R);

    /* observation matrix */
    /* 'attitudeKalmanfilter:96' H_k=[  E,     E,      Z,    Z; */
    /* 'attitudeKalmanfilter:97'         Z,     Z,      E,    Z; */
    /* 'attitudeKalmanfilter:98'         Z,     Z,      Z,    E]; */
    /* 'attitudeKalmanfilter:100' y_k=z(1:9)-H_k*x_apriori; */
    /* 'attitudeKalmanfilter:102' S_k=H_k*P_apriori*H_k'+R; */
    /* 'attitudeKalmanfilter:103' K_k=(P_apriori*H_k'/(S_k)); */
    for (i = 0; i < 12; i++) {
      for (i0 = 0; i0 < 9; i0++) {
        b_P_apriori[i + 12 * i0] = 0.0F;
        for (i1 = 0; i1 < 12; i1++) {
          b_P_apriori[i + 12 * i0] += P_apriori[i + 12 * i1] * (real32_T)iv0[i1
            + 12 * i0];
        }
      }
    }

    for (i = 0; i < 9; i++) {
      for (i0 = 0; i0 < 12; i0++) {
        K_k[i + 9 * i0] = 0.0F;
        for (i1 = 0; i1 < 12; i1++) {
          K_k[i + 9 * i0] += (real32_T)iv1[i + 9 * i1] * P_apriori[i1 + 12 * i0];
        }
      }
    }

    for (i = 0; i < 9; i++) {
      for (i0 = 0; i0 < 9; i0++) {
        y = 0.0F;
        for (i1 = 0; i1 < 12; i1++) {
          y += K_k[i + 9 * i1] * (real32_T)iv0[i1 + 12 * i0];
        }

        fv0[i + 9 * i0] = y + R[i + 9 * i0];
      }
    }

    mrdivide(b_P_apriori, fv0, K_k);

    /* 'attitudeKalmanfilter:106' x_aposteriori=x_apriori+K_k*y_k; */
    for (i = 0; i < 9; i++) {
      y = 0.0F;
      for (i0 = 0; i0 < 12; i0++) {
        y += (real32_T)iv1[i + 9 * i0] * x_apriori[i0];
      }

      c_a[i] = z[i] - y;
    }

    for (i = 0; i < 12; i++) {
      y = 0.0F;
      for (i0 = 0; i0 < 9; i0++) {
        y += K_k[i + 12 * i0] * c_a[i0];
      }

      x_aposteriori[i] = x_apriori[i] + y;
    }

    /* 'attitudeKalmanfilter:107' P_aposteriori=(eye(12)-K_k*H_k)*P_apriori; */
    b_eye(dv1);
    for (i = 0; i < 12; i++) {
      for (i0 = 0; i0 < 12; i0++) {
        y = 0.0F;
        for (i1 = 0; i1 < 9; i1++) {
          y += K_k[i + 12 * i1] * (real32_T)iv1[i1 + 9 * i0];
        }

        Q[i + 12 * i0] = (real32_T)dv1[i + 12 * i0] - y;
      }
    }

    for (i = 0; i < 12; i++) {
      for (i0 = 0; i0 < 12; i0++) {
        P_aposteriori[i + 12 * i0] = 0.0F;
        for (i1 = 0; i1 < 12; i1++) {
          P_aposteriori[i + 12 * i0] += Q[i + 12 * i1] * P_apriori[i1 + 12 * i0];
        }
      }
    }
  } else {
    /* 'attitudeKalmanfilter:108' else */
    /* 'attitudeKalmanfilter:109' if updateVect(1)==1&&updateVect(2)==0&&updateVect(3)==0 */
    if ((updateVect[0] == 1) && (updateVect[1] == 0) && (updateVect[2] == 0)) {
      /* 'attitudeKalmanfilter:110' R=diag(r(1:3)); */
      c_diag(*(real32_T (*)[3])&r[0], O);

      /* observation matrix */
      /* 'attitudeKalmanfilter:113' H_k=[  E,     E,      Z,    Z]; */
      /* 'attitudeKalmanfilter:115' y_k=z(1:3)-H_k(1:3,1:12)*x_apriori; */
      /* 'attitudeKalmanfilter:117' S_k=H_k(1:3,1:12)*P_apriori*H_k(1:3,1:12)'+R(1:3,1:3); */
      /* 'attitudeKalmanfilter:118' K_k=(P_apriori*H_k(1:3,1:12)'/(S_k)); */
      for (i = 0; i < 12; i++) {
        for (i0 = 0; i0 < 3; i0++) {
          c_P_apriori[i + 12 * i0] = 0.0F;
          for (i1 = 0; i1 < 12; i1++) {
            c_P_apriori[i + 12 * i0] += P_apriori[i + 12 * i1] * (real32_T)
              iv2[i1 + 12 * i0];
          }
        }
      }

      for (i = 0; i < 3; i++) {
        for (i0 = 0; i0 < 12; i0++) {
          fv1[i + 3 * i0] = 0.0F;
          for (i1 = 0; i1 < 12; i1++) {
            fv1[i + 3 * i0] += (real32_T)iv3[i + 3 * i1] * P_apriori[i1 + 12 *
              i0];
          }
        }
      }

      for (i = 0; i < 3; i++) {
        for (i0 = 0; i0 < 3; i0++) {
          y = 0.0F;
          for (i1 = 0; i1 < 12; i1++) {
            y += fv1[i + 3 * i1] * (real32_T)iv2[i1 + 12 * i0];
          }

          c_a[i + 3 * i0] = y + O[i + 3 * i0];
        }
      }

      b_mrdivide(c_P_apriori, c_a, S_k);

      /* 'attitudeKalmanfilter:121' x_aposteriori=x_apriori+K_k*y_k; */
      for (i = 0; i < 3; i++) {
        y = 0.0F;
        for (i0 = 0; i0 < 12; i0++) {
          y += (real32_T)iv3[i + 3 * i0] * x_apriori[i0];
        }

        x_n_b[i] = z[i] - y;
      }

      for (i = 0; i < 12; i++) {
        y = 0.0F;
        for (i0 = 0; i0 < 3; i0++) {
          y += S_k[i + 12 * i0] * x_n_b[i0];
        }

        x_aposteriori[i] = x_apriori[i] + y;
      }

      /* 'attitudeKalmanfilter:122' P_aposteriori=(eye(12)-K_k*H_k(1:3,1:12))*P_apriori; */
      b_eye(dv1);
      for (i = 0; i < 12; i++) {
        for (i0 = 0; i0 < 12; i0++) {
          y = 0.0F;
          for (i1 = 0; i1 < 3; i1++) {
            y += S_k[i + 12 * i1] * (real32_T)iv3[i1 + 3 * i0];
          }

          Q[i + 12 * i0] = (real32_T)dv1[i + 12 * i0] - y;
        }
      }

      for (i = 0; i < 12; i++) {
        for (i0 = 0; i0 < 12; i0++) {
          P_aposteriori[i + 12 * i0] = 0.0F;
          for (i1 = 0; i1 < 12; i1++) {
            P_aposteriori[i + 12 * i0] += Q[i + 12 * i1] * P_apriori[i1 + 12 *
              i0];
          }
        }
      }
    } else {
      /* 'attitudeKalmanfilter:123' else */
      /* 'attitudeKalmanfilter:124' if  updateVect(1)==1&&updateVect(2)==1&&updateVect(3)==0 */
      if ((updateVect[0] == 1) && (updateVect[1] == 1) && (updateVect[2] == 0))
      {
        /* 'attitudeKalmanfilter:125' R=diag(r(1:6)); */
        d_diag(*(real32_T (*)[6])&r[0], S_k);

        /* observation matrix */
        /* 'attitudeKalmanfilter:128' H_k=[  E,     E,      Z,    Z; */
        /* 'attitudeKalmanfilter:129'                 Z,     Z,      E,    Z]; */
        /* 'attitudeKalmanfilter:131' y_k=z(1:6)-H_k(1:6,1:12)*x_apriori; */
        /* 'attitudeKalmanfilter:133' S_k=H_k(1:6,1:12)*P_apriori*H_k(1:6,1:12)'+R(1:6,1:6); */
        /* 'attitudeKalmanfilter:134' K_k=(P_apriori*H_k(1:6,1:12)'/(S_k)); */
        for (i = 0; i < 12; i++) {
          for (i0 = 0; i0 < 6; i0++) {
            d_P_apriori[i + 12 * i0] = 0.0F;
            for (i1 = 0; i1 < 12; i1++) {
              d_P_apriori[i + 12 * i0] += P_apriori[i + 12 * i1] * (real32_T)
                iv4[i1 + 12 * i0];
            }
          }
        }

        for (i = 0; i < 6; i++) {
          for (i0 = 0; i0 < 12; i0++) {
            b_K_k[i + 6 * i0] = 0.0F;
            for (i1 = 0; i1 < 12; i1++) {
              b_K_k[i + 6 * i0] += (real32_T)iv5[i + 6 * i1] * P_apriori[i1 + 12
                * i0];
            }
          }
        }

        for (i = 0; i < 6; i++) {
          for (i0 = 0; i0 < 6; i0++) {
            y = 0.0F;
            for (i1 = 0; i1 < 12; i1++) {
              y += b_K_k[i + 6 * i1] * (real32_T)iv4[i1 + 12 * i0];
            }

            fv1[i + 6 * i0] = y + S_k[i + 6 * i0];
          }
        }

        c_mrdivide(d_P_apriori, fv1, b_K_k);

        /* 'attitudeKalmanfilter:137' x_aposteriori=x_apriori+K_k*y_k; */
        for (i = 0; i < 6; i++) {
          y = 0.0F;
          for (i0 = 0; i0 < 12; i0++) {
            y += (real32_T)iv5[i + 6 * i0] * x_apriori[i0];
          }

          b_r[i] = z[i] - y;
        }

        for (i = 0; i < 12; i++) {
          y = 0.0F;
          for (i0 = 0; i0 < 6; i0++) {
            y += b_K_k[i + 12 * i0] * b_r[i0];
          }

          x_aposteriori[i] = x_apriori[i] + y;
        }

        /* 'attitudeKalmanfilter:138' P_aposteriori=(eye(12)-K_k*H_k(1:6,1:12))*P_apriori; */
        b_eye(dv1);
        for (i = 0; i < 12; i++) {
          for (i0 = 0; i0 < 12; i0++) {
            y = 0.0F;
            for (i1 = 0; i1 < 6; i1++) {
              y += b_K_k[i + 12 * i1] * (real32_T)iv5[i1 + 6 * i0];
            }

            Q[i + 12 * i0] = (real32_T)dv1[i + 12 * i0] - y;
          }
        }

        for (i = 0; i < 12; i++) {
          for (i0 = 0; i0 < 12; i0++) {
            P_aposteriori[i + 12 * i0] = 0.0F;
            for (i1 = 0; i1 < 12; i1++) {
              P_aposteriori[i + 12 * i0] += Q[i + 12 * i1] * P_apriori[i1 + 12 *
                i0];
            }
          }
        }
      } else {
        /* 'attitudeKalmanfilter:139' else */
        /* 'attitudeKalmanfilter:140' if  updateVect(1)==1&&updateVect(2)==0&&updateVect(3)==1 */
        if ((updateVect[0] == 1) && (updateVect[1] == 0) && (updateVect[2] == 1))
        {
          /* 'attitudeKalmanfilter:141' R=diag([r(1:3);r(7:9)]); */
          /* observation matrix */
          /* 'attitudeKalmanfilter:144' H_k=[  E,     E,      Z,    Z; */
          /* 'attitudeKalmanfilter:145'                     Z,     Z,      Z,    E]; */
          /* 'attitudeKalmanfilter:147' y_k=[z(1:3);z(7:9)]-H_k(1:6,1:12)*x_apriori; */
          /* 'attitudeKalmanfilter:149' S_k=H_k(1:6,1:12)*P_apriori*H_k(1:6,1:12)'+R(1:6,1:6); */
          for (i = 0; i < 6; i++) {
            for (i0 = 0; i0 < 12; i0++) {
              b_K_k[i + 6 * i0] = 0.0F;
              for (i1 = 0; i1 < 12; i1++) {
                b_K_k[i + 6 * i0] += (real32_T)iv6[i + 6 * i1] * P_apriori[i1 +
                  12 * i0];
              }
            }
          }

          for (i = 0; i < 3; i++) {
            b_r[i << 1] = r[i];
            b_r[1 + (i << 1)] = r[6 + i];
          }

          for (i = 0; i < 6; i++) {
            for (i0 = 0; i0 < 6; i0++) {
              y = 0.0F;
              for (i1 = 0; i1 < 12; i1++) {
                y += b_K_k[i + 6 * i1] * (real32_T)iv7[i1 + 12 * i0];
              }

              S_k[i + 6 * i0] = y + b_r[3 * (i + i0)];
            }
          }

          /* 'attitudeKalmanfilter:150' K_k=(P_apriori*H_k(1:6,1:12)'/(S_k)); */
          for (i = 0; i < 12; i++) {
            for (i0 = 0; i0 < 6; i0++) {
              d_P_apriori[i + 12 * i0] = 0.0F;
              for (i1 = 0; i1 < 12; i1++) {
                d_P_apriori[i + 12 * i0] += P_apriori[i + 12 * i1] * (real32_T)
                  iv7[i1 + 12 * i0];
              }
            }
          }

          c_mrdivide(d_P_apriori, S_k, b_K_k);

          /* 'attitudeKalmanfilter:153' x_aposteriori=x_apriori+K_k*y_k; */
          for (i = 0; i < 3; i++) {
            b_r[i] = z[i];
          }

          for (i = 0; i < 3; i++) {
            b_r[i + 3] = z[i + 6];
          }

          for (i = 0; i < 6; i++) {
            fv2[i] = 0.0F;
            for (i0 = 0; i0 < 12; i0++) {
              fv2[i] += (real32_T)iv6[i + 6 * i0] * x_apriori[i0];
            }

            b_z[i] = b_r[i] - fv2[i];
          }

          for (i = 0; i < 12; i++) {
            y = 0.0F;
            for (i0 = 0; i0 < 6; i0++) {
              y += b_K_k[i + 12 * i0] * b_z[i0];
            }

            x_aposteriori[i] = x_apriori[i] + y;
          }

          /* 'attitudeKalmanfilter:154' P_aposteriori=(eye(12)-K_k*H_k(1:6,1:12))*P_apriori; */
          b_eye(dv1);
          for (i = 0; i < 12; i++) {
            for (i0 = 0; i0 < 12; i0++) {
              y = 0.0F;
              for (i1 = 0; i1 < 6; i1++) {
                y += b_K_k[i + 12 * i1] * (real32_T)iv6[i1 + 6 * i0];
              }

              Q[i + 12 * i0] = (real32_T)dv1[i + 12 * i0] - y;
            }
          }

          for (i = 0; i < 12; i++) {
            for (i0 = 0; i0 < 12; i0++) {
              P_aposteriori[i + 12 * i0] = 0.0F;
              for (i1 = 0; i1 < 12; i1++) {
                P_aposteriori[i + 12 * i0] += Q[i + 12 * i1] * P_apriori[i1 + 12
                  * i0];
              }
            }
          }
        } else {
          /* 'attitudeKalmanfilter:155' else */
          /* 'attitudeKalmanfilter:156' x_aposteriori=x_apriori; */
          for (i = 0; i < 12; i++) {
            x_aposteriori[i] = x_apriori[i];
          }

          /* 'attitudeKalmanfilter:157' P_aposteriori=P_apriori; */
          memcpy((void *)&P_aposteriori[0], (void *)&P_apriori[0], 144U * sizeof
                 (real32_T));
        }
      }
    }
  }

  /* % euler anglels extraction */
  /* 'attitudeKalmanfilter:166' z_n_b = -x_aposteriori(7:9)./norm(x_aposteriori(7:9)); */
  y = norm(*(real32_T (*)[3])&x_aposteriori[6]);

  /* 'attitudeKalmanfilter:167' m_n_b = x_aposteriori(10:12)./norm(x_aposteriori(10:12)); */
  b_y = norm(*(real32_T (*)[3])&x_aposteriori[9]);

  /* 'attitudeKalmanfilter:169' y_n_b=cross(z_n_b,m_n_b); */
  for (i = 0; i < 3; i++) {
    z_n_b[i] = -x_aposteriori[i + 6] / y;
    x_n_b[i] = x_aposteriori[i + 9] / b_y;
  }

  cross(z_n_b, x_n_b, y_n_b);

  /* 'attitudeKalmanfilter:170' y_n_b=y_n_b./norm(y_n_b); */
  y = norm(y_n_b);
  for (i = 0; i < 3; i++) {
    y_n_b[i] /= y;
  }

  /* 'attitudeKalmanfilter:172' x_n_b=(cross(y_n_b,z_n_b)); */
  cross(y_n_b, z_n_b, x_n_b);

  /* 'attitudeKalmanfilter:173' x_n_b=x_n_b./norm(x_n_b); */
  y = norm(x_n_b);
  for (i = 0; i < 3; i++) {
    /* 'attitudeKalmanfilter:179' Rot_matrix=[x_n_b,y_n_b,z_n_b]; */
    Rot_matrix[i] = x_n_b[i] / y;
    Rot_matrix[3 + i] = y_n_b[i];
    Rot_matrix[6 + i] = z_n_b[i];
  }

  /* 'attitudeKalmanfilter:183' phi=atan2(Rot_matrix(2,3),Rot_matrix(3,3)); */
  /* 'attitudeKalmanfilter:184' theta=-asin(Rot_matrix(1,3)); */
  /* 'attitudeKalmanfilter:185' psi=atan2(Rot_matrix(1,2),Rot_matrix(1,1)); */
  /* 'attitudeKalmanfilter:186' eulerAngles=[phi;theta;psi]; */
  eulerAngles[0] = rt_atan2f_snf(Rot_matrix[7], Rot_matrix[8]);
  eulerAngles[1] = -(real32_T)asinf(Rot_matrix[6]);
  eulerAngles[2] = rt_atan2f_snf(Rot_matrix[3], Rot_matrix[0]);
}

/* End of code generation (attitudeKalmanfilter.c) */
