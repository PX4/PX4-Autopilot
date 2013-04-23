/*
 * attitudeKalmanfilter.c
 *
 * Code generation for function 'attitudeKalmanfilter'
 *
 * C source code generated on: Sat Jan 19 15:25:29 2013
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "attitudeKalmanfilter.h"
#include "rdivide.h"
#include "norm.h"
#include "cross.h"
#include "eye.h"
#include "mrdivide.h"

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
  int32_T b_u0;
  int32_T b_u1;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = ((real32_T)rtNaN);
  } else if (rtIsInfF(u0) && rtIsInfF(u1)) {
    if (u0 > 0.0F) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0F) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = (real32_T)atan2((real32_T)b_u0, (real32_T)b_u1);
  } else if (u1 == 0.0F) {
    if (u0 > 0.0F) {
      y = RT_PIF / 2.0F;
    } else if (u0 < 0.0F) {
      y = -(RT_PIF / 2.0F);
    } else {
      y = 0.0F;
    }
  } else {
    y = (real32_T)atan2(u0, u1);
  }

  return y;
}

/*
 * function [eulerAngles,Rot_matrix,x_aposteriori,P_aposteriori] = attitudeKalmanfilter_wo(updateVect,dt,z,x_aposteriori_k,P_aposteriori_k,q,r)
 */
void attitudeKalmanfilter(const uint8_T updateVect[3], real32_T dt, const
  real32_T z[9], const real32_T x_aposteriori_k[12], const real32_T
  P_aposteriori_k[144], const real32_T q[12], real32_T r[9], real32_T
  eulerAngles[3], real32_T Rot_matrix[9], real32_T x_aposteriori[12], real32_T
  P_aposteriori[144])
{
  real32_T wak[3];
  real32_T O[9];
  real_T dv0[9];
  real32_T a[9];
  int32_T i;
  real32_T b_a[9];
  real32_T x_n_b[3];
  real32_T b_x_aposteriori_k[3];
  real32_T z_n_b[3];
  real32_T c_a[3];
  real32_T d_a[3];
  int32_T i0;
  real32_T x_apriori[12];
  real_T dv1[144];
  real32_T A_lin[144];
  static const int8_T iv0[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  real32_T b_A_lin[144];
  real32_T b_q[144];
  real32_T c_A_lin[144];
  real32_T d_A_lin[144];
  real32_T e_A_lin[144];
  int32_T i1;
  real32_T P_apriori[144];
  real32_T b_P_apriori[108];
  static const int8_T iv1[108] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  real32_T K_k[108];
  real32_T fv0[81];
  static const int8_T iv2[108] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  real32_T b_r[81];
  real32_T fv1[81];
  real32_T f0;
  real32_T c_P_apriori[36];
  static const int8_T iv3[36] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  real32_T fv2[36];
  static const int8_T iv4[36] = { 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  real32_T c_r[9];
  real32_T b_K_k[36];
  real32_T d_P_apriori[72];
  static const int8_T iv5[72] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0 };

  real32_T c_K_k[72];
  static const int8_T iv6[72] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0 };

  real32_T b_z[6];
  static const int8_T iv7[72] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1 };

  static const int8_T iv8[72] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 1 };

  real32_T fv3[6];
  real32_T c_z[6];

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
  /* Q = diag(q.^2*dt); */
  /* observation matrix */
  /* 'attitudeKalmanfilter:33' wx=  x_aposteriori_k(1); */
  /* 'attitudeKalmanfilter:34' wy=  x_aposteriori_k(2); */
  /* 'attitudeKalmanfilter:35' wz=  x_aposteriori_k(3); */
  /* 'attitudeKalmanfilter:37' wax=  x_aposteriori_k(4); */
  /* 'attitudeKalmanfilter:38' way=  x_aposteriori_k(5); */
  /* 'attitudeKalmanfilter:39' waz=  x_aposteriori_k(6); */
  /* 'attitudeKalmanfilter:41' zex=  x_aposteriori_k(7); */
  /* 'attitudeKalmanfilter:42' zey=  x_aposteriori_k(8); */
  /* 'attitudeKalmanfilter:43' zez=  x_aposteriori_k(9); */
  /* 'attitudeKalmanfilter:45' mux=  x_aposteriori_k(10); */
  /* 'attitudeKalmanfilter:46' muy=  x_aposteriori_k(11); */
  /* 'attitudeKalmanfilter:47' muz=  x_aposteriori_k(12); */
  /* % prediction section */
  /* body angular accelerations */
  /* 'attitudeKalmanfilter:51' wak =[wax;way;waz]; */
  wak[0] = x_aposteriori_k[3];
  wak[1] = x_aposteriori_k[4];
  wak[2] = x_aposteriori_k[5];

  /* body angular rates */
  /* 'attitudeKalmanfilter:54' wk =[wx;  wy; wz] + dt*wak; */
  /* derivative of the prediction rotation matrix */
  /* 'attitudeKalmanfilter:57' O=[0,-wz,wy;wz,0,-wx;-wy,wx,0]'; */
  O[0] = 0.0F;
  O[1] = -x_aposteriori_k[2];
  O[2] = x_aposteriori_k[1];
  O[3] = x_aposteriori_k[2];
  O[4] = 0.0F;
  O[5] = -x_aposteriori_k[0];
  O[6] = -x_aposteriori_k[1];
  O[7] = x_aposteriori_k[0];
  O[8] = 0.0F;

  /* prediction of the earth z vector */
  /* 'attitudeKalmanfilter:60' zek =(eye(3)+O*dt)*[zex;zey;zez]; */
  eye(dv0);
  for (i = 0; i < 9; i++) {
    a[i] = (real32_T)dv0[i] + O[i] * dt;
  }

  /* prediction of the magnetic vector */
  /* 'attitudeKalmanfilter:63' muk =(eye(3)+O*dt)*[mux;muy;muz]; */
  eye(dv0);
  for (i = 0; i < 9; i++) {
    b_a[i] = (real32_T)dv0[i] + O[i] * dt;
  }

  /* 'attitudeKalmanfilter:65' EZ=[0,zez,-zey; */
  /* 'attitudeKalmanfilter:66'     -zez,0,zex; */
  /* 'attitudeKalmanfilter:67'     zey,-zex,0]'; */
  /* 'attitudeKalmanfilter:68' MA=[0,muz,-muy; */
  /* 'attitudeKalmanfilter:69'     -muz,0,mux; */
  /* 'attitudeKalmanfilter:70'     zey,-mux,0]'; */
  /* 'attitudeKalmanfilter:74' E=eye(3); */
  /* 'attitudeKalmanfilter:76' Z=zeros(3); */
  /* 'attitudeKalmanfilter:77' x_apriori=[wk;wak;zek;muk]; */
  x_n_b[0] = x_aposteriori_k[0];
  x_n_b[1] = x_aposteriori_k[1];
  x_n_b[2] = x_aposteriori_k[2];
  b_x_aposteriori_k[0] = x_aposteriori_k[6];
  b_x_aposteriori_k[1] = x_aposteriori_k[7];
  b_x_aposteriori_k[2] = x_aposteriori_k[8];
  z_n_b[0] = x_aposteriori_k[9];
  z_n_b[1] = x_aposteriori_k[10];
  z_n_b[2] = x_aposteriori_k[11];
  for (i = 0; i < 3; i++) {
    c_a[i] = 0.0F;
    for (i0 = 0; i0 < 3; i0++) {
      c_a[i] += a[i + 3 * i0] * b_x_aposteriori_k[i0];
    }

    d_a[i] = 0.0F;
    for (i0 = 0; i0 < 3; i0++) {
      d_a[i] += b_a[i + 3 * i0] * z_n_b[i0];
    }

    x_apriori[i] = x_n_b[i] + dt * wak[i];
  }

  for (i = 0; i < 3; i++) {
    x_apriori[i + 3] = wak[i];
  }

  for (i = 0; i < 3; i++) {
    x_apriori[i + 6] = c_a[i];
  }

  for (i = 0; i < 3; i++) {
    x_apriori[i + 9] = d_a[i];
  }

  /* 'attitudeKalmanfilter:81' A_lin=[ Z,  E,  Z,  Z */
  /* 'attitudeKalmanfilter:82'     Z,  Z,  Z,  Z */
  /* 'attitudeKalmanfilter:83'     EZ, Z,  O,  Z */
  /* 'attitudeKalmanfilter:84'     MA, Z,  Z,  O]; */
  /* 'attitudeKalmanfilter:86' A_lin=eye(12)+A_lin*dt; */
  b_eye(dv1);
  for (i = 0; i < 12; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_lin[i0 + 12 * i] = (real32_T)iv0[i0 + 3 * i];
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

  /* 'attitudeKalmanfilter:88' Qtemp=[ q(1),     0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0; */
  /* 'attitudeKalmanfilter:89'         0,     q(1),      0,      0,      0,      0,      0,      0,      0,      0,      0,      0; */
  /* 'attitudeKalmanfilter:90'         0,     0,      q(1),      0,      0,      0,      0,      0,      0,      0,      0,      0; */
  /* 'attitudeKalmanfilter:91'         0,     0,      0,      q(2),   0,      0,     0,      0,      0,      0,      0,      0; */
  /* 'attitudeKalmanfilter:92'         0,     0,      0,      0,      q(2),   0,     0,      0,      0,      0,      0,      0; */
  /* 'attitudeKalmanfilter:93'         0,     0,      0,      0,      0,      q(2),   0,      0,      0,      0,      0,      0; */
  /* 'attitudeKalmanfilter:94'         0,     0,      0,      0,      0,      0,      q(3),   0,      0,      0,      0,      0; */
  /* 'attitudeKalmanfilter:95'         0,     0,      0,      0,      0,      0,      0,      q(3),   0,      0,      0,      0; */
  /* 'attitudeKalmanfilter:96'         0,     0,      0,      0,      0,      0,      0,      0,      q(3),   0,      0,      0; */
  /* 'attitudeKalmanfilter:97'         0,     0,      0,      0,      0,      0,      0,      0,      0,      q(4),   0,      0; */
  /* 'attitudeKalmanfilter:98'         0,     0,      0,      0,      0,      0,      0,      0,      0,      0,      q(4),   0; */
  /* 'attitudeKalmanfilter:99'         0,     0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      q(4)]; */
  /* 'attitudeKalmanfilter:103' Q=A_lin*Qtemp*A_lin'; */
  /* 'attitudeKalmanfilter:106' P_apriori=A_lin*P_aposteriori_k*A_lin'+Q; */
  b_q[0] = q[0];
  b_q[12] = 0.0F;
  b_q[24] = 0.0F;
  b_q[36] = 0.0F;
  b_q[48] = 0.0F;
  b_q[60] = 0.0F;
  b_q[72] = 0.0F;
  b_q[84] = 0.0F;
  b_q[96] = 0.0F;
  b_q[108] = 0.0F;
  b_q[120] = 0.0F;
  b_q[132] = 0.0F;
  b_q[1] = 0.0F;
  b_q[13] = q[0];
  b_q[25] = 0.0F;
  b_q[37] = 0.0F;
  b_q[49] = 0.0F;
  b_q[61] = 0.0F;
  b_q[73] = 0.0F;
  b_q[85] = 0.0F;
  b_q[97] = 0.0F;
  b_q[109] = 0.0F;
  b_q[121] = 0.0F;
  b_q[133] = 0.0F;
  b_q[2] = 0.0F;
  b_q[14] = 0.0F;
  b_q[26] = q[0];
  b_q[38] = 0.0F;
  b_q[50] = 0.0F;
  b_q[62] = 0.0F;
  b_q[74] = 0.0F;
  b_q[86] = 0.0F;
  b_q[98] = 0.0F;
  b_q[110] = 0.0F;
  b_q[122] = 0.0F;
  b_q[134] = 0.0F;
  b_q[3] = 0.0F;
  b_q[15] = 0.0F;
  b_q[27] = 0.0F;
  b_q[39] = q[1];
  b_q[51] = 0.0F;
  b_q[63] = 0.0F;
  b_q[75] = 0.0F;
  b_q[87] = 0.0F;
  b_q[99] = 0.0F;
  b_q[111] = 0.0F;
  b_q[123] = 0.0F;
  b_q[135] = 0.0F;
  b_q[4] = 0.0F;
  b_q[16] = 0.0F;
  b_q[28] = 0.0F;
  b_q[40] = 0.0F;
  b_q[52] = q[1];
  b_q[64] = 0.0F;
  b_q[76] = 0.0F;
  b_q[88] = 0.0F;
  b_q[100] = 0.0F;
  b_q[112] = 0.0F;
  b_q[124] = 0.0F;
  b_q[136] = 0.0F;
  b_q[5] = 0.0F;
  b_q[17] = 0.0F;
  b_q[29] = 0.0F;
  b_q[41] = 0.0F;
  b_q[53] = 0.0F;
  b_q[65] = q[1];
  b_q[77] = 0.0F;
  b_q[89] = 0.0F;
  b_q[101] = 0.0F;
  b_q[113] = 0.0F;
  b_q[125] = 0.0F;
  b_q[137] = 0.0F;
  b_q[6] = 0.0F;
  b_q[18] = 0.0F;
  b_q[30] = 0.0F;
  b_q[42] = 0.0F;
  b_q[54] = 0.0F;
  b_q[66] = 0.0F;
  b_q[78] = q[2];
  b_q[90] = 0.0F;
  b_q[102] = 0.0F;
  b_q[114] = 0.0F;
  b_q[126] = 0.0F;
  b_q[138] = 0.0F;
  b_q[7] = 0.0F;
  b_q[19] = 0.0F;
  b_q[31] = 0.0F;
  b_q[43] = 0.0F;
  b_q[55] = 0.0F;
  b_q[67] = 0.0F;
  b_q[79] = 0.0F;
  b_q[91] = q[2];
  b_q[103] = 0.0F;
  b_q[115] = 0.0F;
  b_q[127] = 0.0F;
  b_q[139] = 0.0F;
  b_q[8] = 0.0F;
  b_q[20] = 0.0F;
  b_q[32] = 0.0F;
  b_q[44] = 0.0F;
  b_q[56] = 0.0F;
  b_q[68] = 0.0F;
  b_q[80] = 0.0F;
  b_q[92] = 0.0F;
  b_q[104] = q[2];
  b_q[116] = 0.0F;
  b_q[128] = 0.0F;
  b_q[140] = 0.0F;
  b_q[9] = 0.0F;
  b_q[21] = 0.0F;
  b_q[33] = 0.0F;
  b_q[45] = 0.0F;
  b_q[57] = 0.0F;
  b_q[69] = 0.0F;
  b_q[81] = 0.0F;
  b_q[93] = 0.0F;
  b_q[105] = 0.0F;
  b_q[117] = q[3];
  b_q[129] = 0.0F;
  b_q[141] = 0.0F;
  b_q[10] = 0.0F;
  b_q[22] = 0.0F;
  b_q[34] = 0.0F;
  b_q[46] = 0.0F;
  b_q[58] = 0.0F;
  b_q[70] = 0.0F;
  b_q[82] = 0.0F;
  b_q[94] = 0.0F;
  b_q[106] = 0.0F;
  b_q[118] = 0.0F;
  b_q[130] = q[3];
  b_q[142] = 0.0F;
  b_q[11] = 0.0F;
  b_q[23] = 0.0F;
  b_q[35] = 0.0F;
  b_q[47] = 0.0F;
  b_q[59] = 0.0F;
  b_q[71] = 0.0F;
  b_q[83] = 0.0F;
  b_q[95] = 0.0F;
  b_q[107] = 0.0F;
  b_q[119] = 0.0F;
  b_q[131] = 0.0F;
  b_q[143] = q[3];
  for (i = 0; i < 12; i++) {
    for (i0 = 0; i0 < 12; i0++) {
      A_lin[i + 12 * i0] = 0.0F;
      for (i1 = 0; i1 < 12; i1++) {
        A_lin[i + 12 * i0] += b_A_lin[i + 12 * i1] * P_aposteriori_k[i1 + 12 *
          i0];
      }

      c_A_lin[i + 12 * i0] = 0.0F;
      for (i1 = 0; i1 < 12; i1++) {
        c_A_lin[i + 12 * i0] += b_A_lin[i + 12 * i1] * b_q[i1 + 12 * i0];
      }
    }

    for (i0 = 0; i0 < 12; i0++) {
      d_A_lin[i + 12 * i0] = 0.0F;
      for (i1 = 0; i1 < 12; i1++) {
        d_A_lin[i + 12 * i0] += A_lin[i + 12 * i1] * b_A_lin[i0 + 12 * i1];
      }

      e_A_lin[i + 12 * i0] = 0.0F;
      for (i1 = 0; i1 < 12; i1++) {
        e_A_lin[i + 12 * i0] += c_A_lin[i + 12 * i1] * b_A_lin[i0 + 12 * i1];
      }
    }
  }

  for (i = 0; i < 12; i++) {
    for (i0 = 0; i0 < 12; i0++) {
      P_apriori[i0 + 12 * i] = d_A_lin[i0 + 12 * i] + e_A_lin[i0 + 12 * i];
    }
  }

  /* % update */
  /* 'attitudeKalmanfilter:110' if updateVect(1)==1&&updateVect(2)==1&&updateVect(3)==1 */
  if ((updateVect[0] == 1) && (updateVect[1] == 1) && (updateVect[2] == 1)) {
    /* 'attitudeKalmanfilter:111' if z(6)<4 || z(5)>15 */
    if ((z[5] < 4.0F) || (z[4] > 15.0F)) {
      /* 'attitudeKalmanfilter:112' r(2)=10000; */
      r[1] = 10000.0F;
    }

    /* 'attitudeKalmanfilter:114' R=[r(1),0,0,0,0,0,0,0,0; */
    /* 'attitudeKalmanfilter:115'         0,r(1),0,0,0,0,0,0,0; */
    /* 'attitudeKalmanfilter:116'         0,0,r(1),0,0,0,0,0,0; */
    /* 'attitudeKalmanfilter:117'         0,0,0,r(2),0,0,0,0,0; */
    /* 'attitudeKalmanfilter:118'         0,0,0,0,r(2),0,0,0,0; */
    /* 'attitudeKalmanfilter:119'         0,0,0,0,0,r(2),0,0,0; */
    /* 'attitudeKalmanfilter:120'         0,0,0,0,0,0,r(3),0,0; */
    /* 'attitudeKalmanfilter:121'         0,0,0,0,0,0,0,r(3),0; */
    /* 'attitudeKalmanfilter:122'         0,0,0,0,0,0,0,0,r(3)]; */
    /* observation matrix */
    /* [zw;ze;zmk]; */
    /* 'attitudeKalmanfilter:125' H_k=[  E,     Z,      Z,    Z; */
    /* 'attitudeKalmanfilter:126'         Z,     Z,      E,    Z; */
    /* 'attitudeKalmanfilter:127'         Z,     Z,      Z,    E]; */
    /* 'attitudeKalmanfilter:129' y_k=z(1:9)-H_k*x_apriori; */
    /* 'attitudeKalmanfilter:132' S_k=H_k*P_apriori*H_k'+R; */
    /* 'attitudeKalmanfilter:133' K_k=(P_apriori*H_k'/(S_k)); */
    for (i = 0; i < 12; i++) {
      for (i0 = 0; i0 < 9; i0++) {
        b_P_apriori[i + 12 * i0] = 0.0F;
        for (i1 = 0; i1 < 12; i1++) {
          b_P_apriori[i + 12 * i0] += P_apriori[i + 12 * i1] * (real32_T)iv1[i1
            + 12 * i0];
        }
      }
    }

    for (i = 0; i < 9; i++) {
      for (i0 = 0; i0 < 12; i0++) {
        K_k[i + 9 * i0] = 0.0F;
        for (i1 = 0; i1 < 12; i1++) {
          K_k[i + 9 * i0] += (real32_T)iv2[i + 9 * i1] * P_apriori[i1 + 12 * i0];
        }
      }

      for (i0 = 0; i0 < 9; i0++) {
        fv0[i + 9 * i0] = 0.0F;
        for (i1 = 0; i1 < 12; i1++) {
          fv0[i + 9 * i0] += K_k[i + 9 * i1] * (real32_T)iv1[i1 + 12 * i0];
        }
      }
    }

    b_r[0] = r[0];
    b_r[9] = 0.0F;
    b_r[18] = 0.0F;
    b_r[27] = 0.0F;
    b_r[36] = 0.0F;
    b_r[45] = 0.0F;
    b_r[54] = 0.0F;
    b_r[63] = 0.0F;
    b_r[72] = 0.0F;
    b_r[1] = 0.0F;
    b_r[10] = r[0];
    b_r[19] = 0.0F;
    b_r[28] = 0.0F;
    b_r[37] = 0.0F;
    b_r[46] = 0.0F;
    b_r[55] = 0.0F;
    b_r[64] = 0.0F;
    b_r[73] = 0.0F;
    b_r[2] = 0.0F;
    b_r[11] = 0.0F;
    b_r[20] = r[0];
    b_r[29] = 0.0F;
    b_r[38] = 0.0F;
    b_r[47] = 0.0F;
    b_r[56] = 0.0F;
    b_r[65] = 0.0F;
    b_r[74] = 0.0F;
    b_r[3] = 0.0F;
    b_r[12] = 0.0F;
    b_r[21] = 0.0F;
    b_r[30] = r[1];
    b_r[39] = 0.0F;
    b_r[48] = 0.0F;
    b_r[57] = 0.0F;
    b_r[66] = 0.0F;
    b_r[75] = 0.0F;
    b_r[4] = 0.0F;
    b_r[13] = 0.0F;
    b_r[22] = 0.0F;
    b_r[31] = 0.0F;
    b_r[40] = r[1];
    b_r[49] = 0.0F;
    b_r[58] = 0.0F;
    b_r[67] = 0.0F;
    b_r[76] = 0.0F;
    b_r[5] = 0.0F;
    b_r[14] = 0.0F;
    b_r[23] = 0.0F;
    b_r[32] = 0.0F;
    b_r[41] = 0.0F;
    b_r[50] = r[1];
    b_r[59] = 0.0F;
    b_r[68] = 0.0F;
    b_r[77] = 0.0F;
    b_r[6] = 0.0F;
    b_r[15] = 0.0F;
    b_r[24] = 0.0F;
    b_r[33] = 0.0F;
    b_r[42] = 0.0F;
    b_r[51] = 0.0F;
    b_r[60] = r[2];
    b_r[69] = 0.0F;
    b_r[78] = 0.0F;
    b_r[7] = 0.0F;
    b_r[16] = 0.0F;
    b_r[25] = 0.0F;
    b_r[34] = 0.0F;
    b_r[43] = 0.0F;
    b_r[52] = 0.0F;
    b_r[61] = 0.0F;
    b_r[70] = r[2];
    b_r[79] = 0.0F;
    b_r[8] = 0.0F;
    b_r[17] = 0.0F;
    b_r[26] = 0.0F;
    b_r[35] = 0.0F;
    b_r[44] = 0.0F;
    b_r[53] = 0.0F;
    b_r[62] = 0.0F;
    b_r[71] = 0.0F;
    b_r[80] = r[2];
    for (i = 0; i < 9; i++) {
      for (i0 = 0; i0 < 9; i0++) {
        fv1[i0 + 9 * i] = fv0[i0 + 9 * i] + b_r[i0 + 9 * i];
      }
    }

    mrdivide(b_P_apriori, fv1, K_k);

    /* 'attitudeKalmanfilter:136' x_aposteriori=x_apriori+K_k*y_k; */
    for (i = 0; i < 9; i++) {
      f0 = 0.0F;
      for (i0 = 0; i0 < 12; i0++) {
        f0 += (real32_T)iv2[i + 9 * i0] * x_apriori[i0];
      }

      O[i] = z[i] - f0;
    }

    for (i = 0; i < 12; i++) {
      f0 = 0.0F;
      for (i0 = 0; i0 < 9; i0++) {
        f0 += K_k[i + 12 * i0] * O[i0];
      }

      x_aposteriori[i] = x_apriori[i] + f0;
    }

    /* 'attitudeKalmanfilter:137' P_aposteriori=(eye(12)-K_k*H_k)*P_apriori; */
    b_eye(dv1);
    for (i = 0; i < 12; i++) {
      for (i0 = 0; i0 < 12; i0++) {
        f0 = 0.0F;
        for (i1 = 0; i1 < 9; i1++) {
          f0 += K_k[i + 12 * i1] * (real32_T)iv2[i1 + 9 * i0];
        }

        b_A_lin[i + 12 * i0] = (real32_T)dv1[i + 12 * i0] - f0;
      }
    }

    for (i = 0; i < 12; i++) {
      for (i0 = 0; i0 < 12; i0++) {
        P_aposteriori[i + 12 * i0] = 0.0F;
        for (i1 = 0; i1 < 12; i1++) {
          P_aposteriori[i + 12 * i0] += b_A_lin[i + 12 * i1] * P_apriori[i1 + 12
            * i0];
        }
      }
    }
  } else {
    /* 'attitudeKalmanfilter:138' else */
    /* 'attitudeKalmanfilter:139' if updateVect(1)==1&&updateVect(2)==0&&updateVect(3)==0 */
    if ((updateVect[0] == 1) && (updateVect[1] == 0) && (updateVect[2] == 0)) {
      /* 'attitudeKalmanfilter:141' R=[r(1),0,0; */
      /* 'attitudeKalmanfilter:142'             0,r(1),0; */
      /* 'attitudeKalmanfilter:143'             0,0,r(1)]; */
      /* observation matrix */
      /* 'attitudeKalmanfilter:146' H_k=[  E,     Z,      Z,    Z]; */
      /* 'attitudeKalmanfilter:148' y_k=z(1:3)-H_k(1:3,1:12)*x_apriori; */
      /* 'attitudeKalmanfilter:150' S_k=H_k(1:3,1:12)*P_apriori*H_k(1:3,1:12)'+R(1:3,1:3); */
      /* 'attitudeKalmanfilter:151' K_k=(P_apriori*H_k(1:3,1:12)'/(S_k)); */
      for (i = 0; i < 12; i++) {
        for (i0 = 0; i0 < 3; i0++) {
          c_P_apriori[i + 12 * i0] = 0.0F;
          for (i1 = 0; i1 < 12; i1++) {
            c_P_apriori[i + 12 * i0] += P_apriori[i + 12 * i1] * (real32_T)
              iv3[i1 + 12 * i0];
          }
        }
      }

      for (i = 0; i < 3; i++) {
        for (i0 = 0; i0 < 12; i0++) {
          fv2[i + 3 * i0] = 0.0F;
          for (i1 = 0; i1 < 12; i1++) {
            fv2[i + 3 * i0] += (real32_T)iv4[i + 3 * i1] * P_apriori[i1 + 12 *
              i0];
          }
        }

        for (i0 = 0; i0 < 3; i0++) {
          O[i + 3 * i0] = 0.0F;
          for (i1 = 0; i1 < 12; i1++) {
            O[i + 3 * i0] += fv2[i + 3 * i1] * (real32_T)iv3[i1 + 12 * i0];
          }
        }
      }

      c_r[0] = r[0];
      c_r[3] = 0.0F;
      c_r[6] = 0.0F;
      c_r[1] = 0.0F;
      c_r[4] = r[0];
      c_r[7] = 0.0F;
      c_r[2] = 0.0F;
      c_r[5] = 0.0F;
      c_r[8] = r[0];
      for (i = 0; i < 3; i++) {
        for (i0 = 0; i0 < 3; i0++) {
          a[i0 + 3 * i] = O[i0 + 3 * i] + c_r[i0 + 3 * i];
        }
      }

      b_mrdivide(c_P_apriori, a, b_K_k);

      /* 'attitudeKalmanfilter:154' x_aposteriori=x_apriori+K_k*y_k; */
      for (i = 0; i < 3; i++) {
        f0 = 0.0F;
        for (i0 = 0; i0 < 12; i0++) {
          f0 += (real32_T)iv4[i + 3 * i0] * x_apriori[i0];
        }

        x_n_b[i] = z[i] - f0;
      }

      for (i = 0; i < 12; i++) {
        f0 = 0.0F;
        for (i0 = 0; i0 < 3; i0++) {
          f0 += b_K_k[i + 12 * i0] * x_n_b[i0];
        }

        x_aposteriori[i] = x_apriori[i] + f0;
      }

      /* 'attitudeKalmanfilter:155' P_aposteriori=(eye(12)-K_k*H_k(1:3,1:12))*P_apriori; */
      b_eye(dv1);
      for (i = 0; i < 12; i++) {
        for (i0 = 0; i0 < 12; i0++) {
          f0 = 0.0F;
          for (i1 = 0; i1 < 3; i1++) {
            f0 += b_K_k[i + 12 * i1] * (real32_T)iv4[i1 + 3 * i0];
          }

          b_A_lin[i + 12 * i0] = (real32_T)dv1[i + 12 * i0] - f0;
        }
      }

      for (i = 0; i < 12; i++) {
        for (i0 = 0; i0 < 12; i0++) {
          P_aposteriori[i + 12 * i0] = 0.0F;
          for (i1 = 0; i1 < 12; i1++) {
            P_aposteriori[i + 12 * i0] += b_A_lin[i + 12 * i1] * P_apriori[i1 +
              12 * i0];
          }
        }
      }
    } else {
      /* 'attitudeKalmanfilter:156' else */
      /* 'attitudeKalmanfilter:157' if  updateVect(1)==1&&updateVect(2)==1&&updateVect(3)==0 */
      if ((updateVect[0] == 1) && (updateVect[1] == 1) && (updateVect[2] == 0))
      {
        /* 'attitudeKalmanfilter:158' if z(6)<4 || z(5)>15 */
        if ((z[5] < 4.0F) || (z[4] > 15.0F)) {
          /* 'attitudeKalmanfilter:159' r(2)=10000; */
          r[1] = 10000.0F;
        }

        /* 'attitudeKalmanfilter:162' R=[r(1),0,0,0,0,0; */
        /* 'attitudeKalmanfilter:163'                 0,r(1),0,0,0,0; */
        /* 'attitudeKalmanfilter:164'                 0,0,r(1),0,0,0; */
        /* 'attitudeKalmanfilter:165'                 0,0,0,r(2),0,0; */
        /* 'attitudeKalmanfilter:166'                 0,0,0,0,r(2),0; */
        /* 'attitudeKalmanfilter:167'                 0,0,0,0,0,r(2)]; */
        /* observation matrix */
        /* 'attitudeKalmanfilter:170' H_k=[  E,     Z,      Z,    Z; */
        /* 'attitudeKalmanfilter:171'                 Z,     Z,      E,    Z]; */
        /* 'attitudeKalmanfilter:173' y_k=z(1:6)-H_k(1:6,1:12)*x_apriori; */
        /* 'attitudeKalmanfilter:175' S_k=H_k(1:6,1:12)*P_apriori*H_k(1:6,1:12)'+R(1:6,1:6); */
        /* 'attitudeKalmanfilter:176' K_k=(P_apriori*H_k(1:6,1:12)'/(S_k)); */
        for (i = 0; i < 12; i++) {
          for (i0 = 0; i0 < 6; i0++) {
            d_P_apriori[i + 12 * i0] = 0.0F;
            for (i1 = 0; i1 < 12; i1++) {
              d_P_apriori[i + 12 * i0] += P_apriori[i + 12 * i1] * (real32_T)
                iv5[i1 + 12 * i0];
            }
          }
        }

        for (i = 0; i < 6; i++) {
          for (i0 = 0; i0 < 12; i0++) {
            c_K_k[i + 6 * i0] = 0.0F;
            for (i1 = 0; i1 < 12; i1++) {
              c_K_k[i + 6 * i0] += (real32_T)iv6[i + 6 * i1] * P_apriori[i1 + 12
                * i0];
            }
          }

          for (i0 = 0; i0 < 6; i0++) {
            fv2[i + 6 * i0] = 0.0F;
            for (i1 = 0; i1 < 12; i1++) {
              fv2[i + 6 * i0] += c_K_k[i + 6 * i1] * (real32_T)iv5[i1 + 12 * i0];
            }
          }
        }

        b_K_k[0] = r[0];
        b_K_k[6] = 0.0F;
        b_K_k[12] = 0.0F;
        b_K_k[18] = 0.0F;
        b_K_k[24] = 0.0F;
        b_K_k[30] = 0.0F;
        b_K_k[1] = 0.0F;
        b_K_k[7] = r[0];
        b_K_k[13] = 0.0F;
        b_K_k[19] = 0.0F;
        b_K_k[25] = 0.0F;
        b_K_k[31] = 0.0F;
        b_K_k[2] = 0.0F;
        b_K_k[8] = 0.0F;
        b_K_k[14] = r[0];
        b_K_k[20] = 0.0F;
        b_K_k[26] = 0.0F;
        b_K_k[32] = 0.0F;
        b_K_k[3] = 0.0F;
        b_K_k[9] = 0.0F;
        b_K_k[15] = 0.0F;
        b_K_k[21] = r[1];
        b_K_k[27] = 0.0F;
        b_K_k[33] = 0.0F;
        b_K_k[4] = 0.0F;
        b_K_k[10] = 0.0F;
        b_K_k[16] = 0.0F;
        b_K_k[22] = 0.0F;
        b_K_k[28] = r[1];
        b_K_k[34] = 0.0F;
        b_K_k[5] = 0.0F;
        b_K_k[11] = 0.0F;
        b_K_k[17] = 0.0F;
        b_K_k[23] = 0.0F;
        b_K_k[29] = 0.0F;
        b_K_k[35] = r[1];
        for (i = 0; i < 6; i++) {
          for (i0 = 0; i0 < 6; i0++) {
            c_P_apriori[i0 + 6 * i] = fv2[i0 + 6 * i] + b_K_k[i0 + 6 * i];
          }
        }

        c_mrdivide(d_P_apriori, c_P_apriori, c_K_k);

        /* 'attitudeKalmanfilter:179' x_aposteriori=x_apriori+K_k*y_k; */
        for (i = 0; i < 6; i++) {
          f0 = 0.0F;
          for (i0 = 0; i0 < 12; i0++) {
            f0 += (real32_T)iv6[i + 6 * i0] * x_apriori[i0];
          }

          b_z[i] = z[i] - f0;
        }

        for (i = 0; i < 12; i++) {
          f0 = 0.0F;
          for (i0 = 0; i0 < 6; i0++) {
            f0 += c_K_k[i + 12 * i0] * b_z[i0];
          }

          x_aposteriori[i] = x_apriori[i] + f0;
        }

        /* 'attitudeKalmanfilter:180' P_aposteriori=(eye(12)-K_k*H_k(1:6,1:12))*P_apriori; */
        b_eye(dv1);
        for (i = 0; i < 12; i++) {
          for (i0 = 0; i0 < 12; i0++) {
            f0 = 0.0F;
            for (i1 = 0; i1 < 6; i1++) {
              f0 += c_K_k[i + 12 * i1] * (real32_T)iv6[i1 + 6 * i0];
            }

            b_A_lin[i + 12 * i0] = (real32_T)dv1[i + 12 * i0] - f0;
          }
        }

        for (i = 0; i < 12; i++) {
          for (i0 = 0; i0 < 12; i0++) {
            P_aposteriori[i + 12 * i0] = 0.0F;
            for (i1 = 0; i1 < 12; i1++) {
              P_aposteriori[i + 12 * i0] += b_A_lin[i + 12 * i1] * P_apriori[i1
                + 12 * i0];
            }
          }
        }
      } else {
        /* 'attitudeKalmanfilter:181' else */
        /* 'attitudeKalmanfilter:182' if  updateVect(1)==1&&updateVect(2)==0&&updateVect(3)==1 */
        if ((updateVect[0] == 1) && (updateVect[1] == 0) && (updateVect[2] == 1))
        {
          /* 'attitudeKalmanfilter:183' R=[r(1),0,0,0,0,0; */
          /* 'attitudeKalmanfilter:184'                     0,r(1),0,0,0,0; */
          /* 'attitudeKalmanfilter:185'                     0,0,r(1),0,0,0; */
          /* 'attitudeKalmanfilter:186'                     0,0,0,r(3),0,0; */
          /* 'attitudeKalmanfilter:187'                     0,0,0,0,r(3),0; */
          /* 'attitudeKalmanfilter:188'                     0,0,0,0,0,r(3)]; */
          /* observation matrix */
          /* 'attitudeKalmanfilter:191' H_k=[  E,     Z,      Z,    Z; */
          /* 'attitudeKalmanfilter:192'                     Z,     Z,      Z,    E]; */
          /* 'attitudeKalmanfilter:194' y_k=[z(1:3);z(7:9)]-H_k(1:6,1:12)*x_apriori; */
          /* 'attitudeKalmanfilter:196' S_k=H_k(1:6,1:12)*P_apriori*H_k(1:6,1:12)'+R(1:6,1:6); */
          /* 'attitudeKalmanfilter:197' K_k=(P_apriori*H_k(1:6,1:12)'/(S_k)); */
          for (i = 0; i < 12; i++) {
            for (i0 = 0; i0 < 6; i0++) {
              d_P_apriori[i + 12 * i0] = 0.0F;
              for (i1 = 0; i1 < 12; i1++) {
                d_P_apriori[i + 12 * i0] += P_apriori[i + 12 * i1] * (real32_T)
                  iv7[i1 + 12 * i0];
              }
            }
          }

          for (i = 0; i < 6; i++) {
            for (i0 = 0; i0 < 12; i0++) {
              c_K_k[i + 6 * i0] = 0.0F;
              for (i1 = 0; i1 < 12; i1++) {
                c_K_k[i + 6 * i0] += (real32_T)iv8[i + 6 * i1] * P_apriori[i1 +
                  12 * i0];
              }
            }

            for (i0 = 0; i0 < 6; i0++) {
              fv2[i + 6 * i0] = 0.0F;
              for (i1 = 0; i1 < 12; i1++) {
                fv2[i + 6 * i0] += c_K_k[i + 6 * i1] * (real32_T)iv7[i1 + 12 *
                  i0];
              }
            }
          }

          b_K_k[0] = r[0];
          b_K_k[6] = 0.0F;
          b_K_k[12] = 0.0F;
          b_K_k[18] = 0.0F;
          b_K_k[24] = 0.0F;
          b_K_k[30] = 0.0F;
          b_K_k[1] = 0.0F;
          b_K_k[7] = r[0];
          b_K_k[13] = 0.0F;
          b_K_k[19] = 0.0F;
          b_K_k[25] = 0.0F;
          b_K_k[31] = 0.0F;
          b_K_k[2] = 0.0F;
          b_K_k[8] = 0.0F;
          b_K_k[14] = r[0];
          b_K_k[20] = 0.0F;
          b_K_k[26] = 0.0F;
          b_K_k[32] = 0.0F;
          b_K_k[3] = 0.0F;
          b_K_k[9] = 0.0F;
          b_K_k[15] = 0.0F;
          b_K_k[21] = r[2];
          b_K_k[27] = 0.0F;
          b_K_k[33] = 0.0F;
          b_K_k[4] = 0.0F;
          b_K_k[10] = 0.0F;
          b_K_k[16] = 0.0F;
          b_K_k[22] = 0.0F;
          b_K_k[28] = r[2];
          b_K_k[34] = 0.0F;
          b_K_k[5] = 0.0F;
          b_K_k[11] = 0.0F;
          b_K_k[17] = 0.0F;
          b_K_k[23] = 0.0F;
          b_K_k[29] = 0.0F;
          b_K_k[35] = r[2];
          for (i = 0; i < 6; i++) {
            for (i0 = 0; i0 < 6; i0++) {
              c_P_apriori[i0 + 6 * i] = fv2[i0 + 6 * i] + b_K_k[i0 + 6 * i];
            }
          }

          c_mrdivide(d_P_apriori, c_P_apriori, c_K_k);

          /* 'attitudeKalmanfilter:200' x_aposteriori=x_apriori+K_k*y_k; */
          for (i = 0; i < 3; i++) {
            b_z[i] = z[i];
          }

          for (i = 0; i < 3; i++) {
            b_z[i + 3] = z[i + 6];
          }

          for (i = 0; i < 6; i++) {
            fv3[i] = 0.0F;
            for (i0 = 0; i0 < 12; i0++) {
              fv3[i] += (real32_T)iv8[i + 6 * i0] * x_apriori[i0];
            }

            c_z[i] = b_z[i] - fv3[i];
          }

          for (i = 0; i < 12; i++) {
            f0 = 0.0F;
            for (i0 = 0; i0 < 6; i0++) {
              f0 += c_K_k[i + 12 * i0] * c_z[i0];
            }

            x_aposteriori[i] = x_apriori[i] + f0;
          }

          /* 'attitudeKalmanfilter:201' P_aposteriori=(eye(12)-K_k*H_k(1:6,1:12))*P_apriori; */
          b_eye(dv1);
          for (i = 0; i < 12; i++) {
            for (i0 = 0; i0 < 12; i0++) {
              f0 = 0.0F;
              for (i1 = 0; i1 < 6; i1++) {
                f0 += c_K_k[i + 12 * i1] * (real32_T)iv8[i1 + 6 * i0];
              }

              b_A_lin[i + 12 * i0] = (real32_T)dv1[i + 12 * i0] - f0;
            }
          }

          for (i = 0; i < 12; i++) {
            for (i0 = 0; i0 < 12; i0++) {
              P_aposteriori[i + 12 * i0] = 0.0F;
              for (i1 = 0; i1 < 12; i1++) {
                P_aposteriori[i + 12 * i0] += b_A_lin[i + 12 * i1] *
                  P_apriori[i1 + 12 * i0];
              }
            }
          }
        } else {
          /* 'attitudeKalmanfilter:202' else */
          /* 'attitudeKalmanfilter:203' x_aposteriori=x_apriori; */
          for (i = 0; i < 12; i++) {
            x_aposteriori[i] = x_apriori[i];
          }

          /* 'attitudeKalmanfilter:204' P_aposteriori=P_apriori; */
          memcpy(&P_aposteriori[0], &P_apriori[0], 144U * sizeof(real32_T));
        }
      }
    }
  }

  /* % euler anglels extraction */
  /* 'attitudeKalmanfilter:213' z_n_b = -x_aposteriori(7:9)./norm(x_aposteriori(7:9)); */
  for (i = 0; i < 3; i++) {
    x_n_b[i] = -x_aposteriori[i + 6];
  }

  rdivide(x_n_b, norm(*(real32_T (*)[3])&x_aposteriori[6]), z_n_b);

  /* 'attitudeKalmanfilter:214' m_n_b = x_aposteriori(10:12)./norm(x_aposteriori(10:12)); */
  rdivide(*(real32_T (*)[3])&x_aposteriori[9], norm(*(real32_T (*)[3])&
           x_aposteriori[9]), wak);

  /* 'attitudeKalmanfilter:216' y_n_b=cross(z_n_b,m_n_b); */
  for (i = 0; i < 3; i++) {
    x_n_b[i] = wak[i];
  }

  cross(z_n_b, x_n_b, wak);

  /* 'attitudeKalmanfilter:217' y_n_b=y_n_b./norm(y_n_b); */
  for (i = 0; i < 3; i++) {
    x_n_b[i] = wak[i];
  }

  rdivide(x_n_b, norm(wak), wak);

  /* 'attitudeKalmanfilter:219' x_n_b=(cross(y_n_b,z_n_b)); */
  cross(wak, z_n_b, x_n_b);

  /* 'attitudeKalmanfilter:220' x_n_b=x_n_b./norm(x_n_b); */
  for (i = 0; i < 3; i++) {
    b_x_aposteriori_k[i] = x_n_b[i];
  }

  rdivide(b_x_aposteriori_k, norm(x_n_b), x_n_b);

  /* 'attitudeKalmanfilter:226' Rot_matrix=[x_n_b,y_n_b,z_n_b]; */
  for (i = 0; i < 3; i++) {
    Rot_matrix[i] = x_n_b[i];
    Rot_matrix[3 + i] = wak[i];
    Rot_matrix[6 + i] = z_n_b[i];
  }

  /* 'attitudeKalmanfilter:230' phi=atan2(Rot_matrix(2,3),Rot_matrix(3,3)); */
  /* 'attitudeKalmanfilter:231' theta=-asin(Rot_matrix(1,3)); */
  /* 'attitudeKalmanfilter:232' psi=atan2(Rot_matrix(1,2),Rot_matrix(1,1)); */
  /* 'attitudeKalmanfilter:233' eulerAngles=[phi;theta;psi]; */
  eulerAngles[0] = rt_atan2f_snf(Rot_matrix[7], Rot_matrix[8]);
  eulerAngles[1] = -(real32_T)asin(Rot_matrix[6]);
  eulerAngles[2] = rt_atan2f_snf(Rot_matrix[3], Rot_matrix[0]);
}

/* End of code generation (attitudeKalmanfilter.c) */
