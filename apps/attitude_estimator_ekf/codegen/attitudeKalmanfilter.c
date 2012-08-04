/*
 * attitudeKalmanfilter.c
 *
 * Code generation for function 'attitudeKalmanfilter'
 *
 * C source code generated on: Wed Jul 11 08:38:35 2012
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "attitudeKalmanfilter.h"
#include "norm.h"
#include "eye.h"
#include "mrdivide.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */

/*
 * function [Rot_matrix,x_aposteriori,P_aposteriori] = attitudeKalmanfilter(dt,z_k,x_aposteriori_k,P_aposteriori_k,knownConst)
 */
void attitudeKalmanfilter(real32_T dt, const real32_T z_k[9], const real32_T
  x_aposteriori_k[12], const real32_T P_aposteriori_k[144], const real32_T
  knownConst[7], real32_T Rot_matrix[9], real32_T x_aposteriori[12], real32_T
  P_aposteriori[144])
{
  real32_T R_temp[9];
  real_T dv0[9];
  real_T dv1[9];
  int32_T i;
  int32_T i0;
  real32_T A_pred[144];
  real32_T x_apriori[12];
  real32_T b_A_pred[144];
  int32_T i1;
  real32_T c_A_pred[144];
  static const int8_T iv0[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  real32_T P_apriori[144];
  real32_T b_P_apriori[108];
  static const int8_T iv1[108] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1 };

  real32_T K_k[108];
  static const int8_T iv2[108] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  real32_T fv0[81];
  real32_T fv1[81];
  real32_T fv2[81];
  real32_T B;
  real_T dv2[144];
  real32_T b_B;
  real32_T earth_z[3];
  real32_T y[3];
  real32_T earth_x[3];

  /* Extended Attitude Kalmanfilter */
  /*     */
  /* state vector x has the following entries [ax,ay,az||mx,my,mz||wox,woy,woz||wx,wy,wz]' */
  /* measurement vector z has the following entries [ax,ay,az||mx,my,mz||wmx,wmy,wmz]' */
  /* knownConst has the following entries [PrvaA,PrvarM,PrvarWO,PrvarW||MsvarA,MsvarM,MsvarW] */
  /*  */
  /* [x_aposteriori,P_aposteriori] = AttKalman(dt,z_k,x_aposteriori_k,P_aposteriori_k,knownConst) */
  /*  */
  /* Example....  */
  /*  */
  /*  $Author: Tobias Naegeli $    $Date: 2012 $    $Revision: 1 $ */
  /* %define the matrices */
  /* 'attitudeKalmanfilter:19' acc_ProcessNoise=knownConst(1); */
  /* 'attitudeKalmanfilter:20' mag_ProcessNoise=knownConst(2); */
  /* 'attitudeKalmanfilter:21' ratesOffset_ProcessNoise=knownConst(3); */
  /* 'attitudeKalmanfilter:22' rates_ProcessNoise=knownConst(4); */
  /* 'attitudeKalmanfilter:25' acc_MeasurementNoise=knownConst(5); */
  /* 'attitudeKalmanfilter:26' mag_MeasurementNoise=knownConst(6); */
  /* 'attitudeKalmanfilter:27' rates_MeasurementNoise=knownConst(7); */
  /* process noise covariance matrix */
  /* 'attitudeKalmanfilter:30' Q = [      eye(3)*acc_ProcessNoise,    zeros(3),                   zeros(3),                           zeros(3); */
  /* 'attitudeKalmanfilter:31'                 zeros(3),                   eye(3)*mag_ProcessNoise,    zeros(3),                           zeros(3); */
  /* 'attitudeKalmanfilter:32'                 zeros(3),                   zeros(3),                   eye(3)*ratesOffset_ProcessNoise,    zeros(3); */
  /* 'attitudeKalmanfilter:33'                 zeros(3),                   zeros(3),                   zeros(3),                           eye(3)*rates_ProcessNoise]; */
  /* measurement noise covariance matrix */
  /* 'attitudeKalmanfilter:36' R = [   eye(3)*acc_MeasurementNoise,       zeros(3),                       zeros(3); */
  /* 'attitudeKalmanfilter:37'                  zeros(3),                          eye(3)*mag_MeasurementNoise,    zeros(3); */
  /* 'attitudeKalmanfilter:38'                  zeros(3),                          zeros(3),                       eye(3)*rates_MeasurementNoise]; */
  /* observation matrix */
  /* 'attitudeKalmanfilter:42' H_k=[   eye(3),     zeros(3),   zeros(3),   zeros(3); */
  /* 'attitudeKalmanfilter:43'             zeros(3),   eye(3),     zeros(3),   zeros(3); */
  /* 'attitudeKalmanfilter:44'             zeros(3),   zeros(3),   eye(3),     eye(3)]; */
  /* compute A(t,w) */
  /* x_aposteriori_k[10,11,12] should be [p,q,r] */
  /* R_temp=[1,-r, q */
  /*         r, 1, -p */
  /*        -q, p, 1] */
  /* 'attitudeKalmanfilter:53' R_temp=[1,-dt*x_aposteriori_k(12),dt*x_aposteriori_k(11); */
  /* 'attitudeKalmanfilter:54'         dt*x_aposteriori_k(12),1,-dt*x_aposteriori_k(10); */
  /* 'attitudeKalmanfilter:55'         -dt*x_aposteriori_k(11), dt*x_aposteriori_k(10),1]; */
  R_temp[0] = 1.0F;
  R_temp[3] = -dt * x_aposteriori_k[11];
  R_temp[6] = dt * x_aposteriori_k[10];
  R_temp[1] = dt * x_aposteriori_k[11];
  R_temp[4] = 1.0F;
  R_temp[7] = -dt * x_aposteriori_k[9];
  R_temp[2] = -dt * x_aposteriori_k[10];
  R_temp[5] = dt * x_aposteriori_k[9];
  R_temp[8] = 1.0F;

  /* strange, should not be transposed */
  /* 'attitudeKalmanfilter:58' A_pred=[R_temp',     zeros(3),   zeros(3),   zeros(3); */
  /* 'attitudeKalmanfilter:59'         zeros(3),   R_temp',     zeros(3),   zeros(3); */
  /* 'attitudeKalmanfilter:60'         zeros(3),   zeros(3),   eye(3),     zeros(3); */
  /* 'attitudeKalmanfilter:61'         zeros(3),   zeros(3),   zeros(3),   eye(3)]; */
  eye(dv0);
  eye(dv1);
  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_pred[i0 + 12 * i] = R_temp[i + 3 * i0];
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_pred[i0 + 12 * (i + 3)] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_pred[i0 + 12 * (i + 6)] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_pred[i0 + 12 * (i + 9)] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_pred[(i0 + 12 * i) + 3] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_pred[(i0 + 12 * (i + 3)) + 3] = R_temp[i + 3 * i0];
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_pred[(i0 + 12 * (i + 6)) + 3] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_pred[(i0 + 12 * (i + 9)) + 3] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_pred[(i0 + 12 * i) + 6] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_pred[(i0 + 12 * (i + 3)) + 6] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_pred[(i0 + 12 * (i + 6)) + 6] = (real32_T)dv0[i0 + 3 * i];
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_pred[(i0 + 12 * (i + 9)) + 6] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_pred[(i0 + 12 * i) + 9] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_pred[(i0 + 12 * (i + 3)) + 9] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_pred[(i0 + 12 * (i + 6)) + 9] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_pred[(i0 + 12 * (i + 9)) + 9] = (real32_T)dv1[i0 + 3 * i];
    }
  }

  /* %prediction step */
  /* 'attitudeKalmanfilter:64' x_apriori=A_pred*x_aposteriori_k; */
  for (i = 0; i < 12; i++) {
    x_apriori[i] = 0.0F;
    for (i0 = 0; i0 < 12; i0++) {
      x_apriori[i] += A_pred[i + 12 * i0] * x_aposteriori_k[i0];
    }
  }

  /* linearization */
  /* 'attitudeKalmanfilter:67' acc_temp_mat=[0,              dt*x_aposteriori_k(3),    -dt*x_aposteriori_k(2); */
  /* 'attitudeKalmanfilter:68'         -dt*x_aposteriori_k(3), 0,                  dt*x_aposteriori_k(1); */
  /* 'attitudeKalmanfilter:69'         dt*x_aposteriori_k(2), -dt*x_aposteriori_k(1),    0]; */
  /* 'attitudeKalmanfilter:71' mag_temp_mat=[0,              dt*x_aposteriori_k(6),    -dt*x_aposteriori_k(5); */
  /* 'attitudeKalmanfilter:72'         -dt*x_aposteriori_k(6), 0,                  dt*x_aposteriori_k(4); */
  /* 'attitudeKalmanfilter:73'         dt*x_aposteriori_k(5), -dt*x_aposteriori_k(4),    0]; */
  /* 'attitudeKalmanfilter:75' A_lin=[R_temp',     zeros(3),   zeros(3),   acc_temp_mat'; */
  /* 'attitudeKalmanfilter:76'         zeros(3),   R_temp',     zeros(3),   mag_temp_mat'; */
  /* 'attitudeKalmanfilter:77'         zeros(3),   zeros(3),   eye(3),     zeros(3); */
  /* 'attitudeKalmanfilter:78'         zeros(3),   zeros(3),   zeros(3),   eye(3)]; */
  eye(dv0);
  eye(dv1);
  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_pred[i0 + 12 * i] = R_temp[i + 3 * i0];
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_pred[i0 + 12 * (i + 3)] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_pred[i0 + 12 * (i + 6)] = 0.0F;
    }
  }

  A_pred[108] = 0.0F;
  A_pred[109] = dt * x_aposteriori_k[2];
  A_pred[110] = -dt * x_aposteriori_k[1];
  A_pred[120] = -dt * x_aposteriori_k[2];
  A_pred[121] = 0.0F;
  A_pred[122] = dt * x_aposteriori_k[0];
  A_pred[132] = dt * x_aposteriori_k[1];
  A_pred[133] = -dt * x_aposteriori_k[0];
  A_pred[134] = 0.0F;
  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_pred[(i0 + 12 * i) + 3] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_pred[(i0 + 12 * (i + 3)) + 3] = R_temp[i + 3 * i0];
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_pred[(i0 + 12 * (i + 6)) + 3] = 0.0F;
    }
  }

  A_pred[111] = 0.0F;
  A_pred[112] = dt * x_aposteriori_k[5];
  A_pred[113] = -dt * x_aposteriori_k[4];
  A_pred[123] = -dt * x_aposteriori_k[5];
  A_pred[124] = 0.0F;
  A_pred[125] = dt * x_aposteriori_k[3];
  A_pred[135] = dt * x_aposteriori_k[4];
  A_pred[136] = -dt * x_aposteriori_k[3];
  A_pred[137] = 0.0F;
  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_pred[(i0 + 12 * i) + 6] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_pred[(i0 + 12 * (i + 3)) + 6] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_pred[(i0 + 12 * (i + 6)) + 6] = (real32_T)dv0[i0 + 3 * i];
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_pred[(i0 + 12 * (i + 9)) + 6] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_pred[(i0 + 12 * i) + 9] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_pred[(i0 + 12 * (i + 3)) + 9] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_pred[(i0 + 12 * (i + 6)) + 9] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_pred[(i0 + 12 * (i + 9)) + 9] = (real32_T)dv1[i0 + 3 * i];
    }
  }

  /* 'attitudeKalmanfilter:81' P_apriori=A_lin*P_aposteriori_k*A_lin'+Q; */
  for (i = 0; i < 12; i++) {
    for (i0 = 0; i0 < 12; i0++) {
      b_A_pred[i + 12 * i0] = 0.0F;
      for (i1 = 0; i1 < 12; i1++) {
        b_A_pred[i + 12 * i0] += A_pred[i + 12 * i1] * P_aposteriori_k[i1 + 12 *
          i0];
      }
    }

    for (i0 = 0; i0 < 12; i0++) {
      c_A_pred[i + 12 * i0] = 0.0F;
      for (i1 = 0; i1 < 12; i1++) {
        c_A_pred[i + 12 * i0] += b_A_pred[i + 12 * i1] * A_pred[i0 + 12 * i1];
      }
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      b_A_pred[i0 + 12 * i] = (real32_T)iv0[i0 + 3 * i] * knownConst[0];
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      b_A_pred[i0 + 12 * (i + 3)] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      b_A_pred[i0 + 12 * (i + 6)] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      b_A_pred[i0 + 12 * (i + 9)] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      b_A_pred[(i0 + 12 * i) + 3] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      b_A_pred[(i0 + 12 * (i + 3)) + 3] = (real32_T)iv0[i0 + 3 * i] *
        knownConst[1];
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      b_A_pred[(i0 + 12 * (i + 6)) + 3] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      b_A_pred[(i0 + 12 * (i + 9)) + 3] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      b_A_pred[(i0 + 12 * i) + 6] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      b_A_pred[(i0 + 12 * (i + 3)) + 6] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      b_A_pred[(i0 + 12 * (i + 6)) + 6] = (real32_T)iv0[i0 + 3 * i] *
        knownConst[2];
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      b_A_pred[(i0 + 12 * (i + 9)) + 6] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      b_A_pred[(i0 + 12 * i) + 9] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      b_A_pred[(i0 + 12 * (i + 3)) + 9] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      b_A_pred[(i0 + 12 * (i + 6)) + 9] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      b_A_pred[(i0 + 12 * (i + 9)) + 9] = (real32_T)iv0[i0 + 3 * i] *
        knownConst[3];
    }
  }

  for (i = 0; i < 12; i++) {
    for (i0 = 0; i0 < 12; i0++) {
      P_apriori[i0 + 12 * i] = c_A_pred[i0 + 12 * i] + b_A_pred[i0 + 12 * i];
    }
  }

  /* %update step */
  /* 'attitudeKalmanfilter:86' y_k=z_k-H_k*x_apriori; */
  /* 'attitudeKalmanfilter:87' S_k=H_k*P_apriori*H_k'+R; */
  /* 'attitudeKalmanfilter:88' K_k=(P_apriori*H_k'/(S_k)); */
  for (i = 0; i < 12; i++) {
    for (i0 = 0; i0 < 9; i0++) {
      b_P_apriori[i + 12 * i0] = 0.0F;
      for (i1 = 0; i1 < 12; i1++) {
        b_P_apriori[i + 12 * i0] += P_apriori[i + 12 * i1] * (real32_T)iv1[i1 +
          12 * i0];
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

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      fv1[i0 + 9 * i] = (real32_T)iv0[i0 + 3 * i] * knownConst[4];
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      fv1[i0 + 9 * (i + 3)] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      fv1[i0 + 9 * (i + 6)] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      fv1[(i0 + 9 * i) + 3] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      fv1[(i0 + 9 * (i + 3)) + 3] = (real32_T)iv0[i0 + 3 * i] * knownConst[5];
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      fv1[(i0 + 9 * (i + 6)) + 3] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      fv1[(i0 + 9 * i) + 6] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      fv1[(i0 + 9 * (i + 3)) + 6] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      fv1[(i0 + 9 * (i + 6)) + 6] = (real32_T)iv0[i0 + 3 * i] * knownConst[6];
    }
  }

  for (i = 0; i < 9; i++) {
    for (i0 = 0; i0 < 9; i0++) {
      fv2[i0 + 9 * i] = fv0[i0 + 9 * i] + fv1[i0 + 9 * i];
    }
  }

  mrdivide(b_P_apriori, fv2, K_k);

  /* 'attitudeKalmanfilter:91' x_aposteriori=x_apriori+K_k*y_k; */
  for (i = 0; i < 9; i++) {
    B = 0.0F;
    for (i0 = 0; i0 < 12; i0++) {
      B += (real32_T)iv2[i + 9 * i0] * x_apriori[i0];
    }

    R_temp[i] = z_k[i] - B;
  }

  for (i = 0; i < 12; i++) {
    B = 0.0F;
    for (i0 = 0; i0 < 9; i0++) {
      B += K_k[i + 12 * i0] * R_temp[i0];
    }

    x_aposteriori[i] = x_apriori[i] + B;
  }

  /* 'attitudeKalmanfilter:92' P_aposteriori=(eye(12)-K_k*H_k)*P_apriori; */
  b_eye(dv2);
  for (i = 0; i < 12; i++) {
    for (i0 = 0; i0 < 12; i0++) {
      B = 0.0F;
      for (i1 = 0; i1 < 9; i1++) {
        B += K_k[i + 12 * i1] * (real32_T)iv2[i1 + 9 * i0];
      }

      A_pred[i + 12 * i0] = (real32_T)dv2[i + 12 * i0] - B;
    }
  }

  for (i = 0; i < 12; i++) {
    for (i0 = 0; i0 < 12; i0++) {
      P_aposteriori[i + 12 * i0] = 0.0F;
      for (i1 = 0; i1 < 12; i1++) {
        P_aposteriori[i + 12 * i0] += A_pred[i + 12 * i1] * P_apriori[i1 + 12 *
          i0];
      }
    }
  }

  /* %Rotation matrix generation */
  /* 'attitudeKalmanfilter:97' earth_z=x_aposteriori(1:3)/norm(x_aposteriori(1:3)); */
  B = norm(*(real32_T (*)[3])&x_aposteriori[0]);

  /* 'attitudeKalmanfilter:98' earth_x=cross(earth_z,x_aposteriori(4:6)/norm(x_aposteriori(4:6))); */
  b_B = norm(*(real32_T (*)[3])&x_aposteriori[3]);
  for (i = 0; i < 3; i++) {
    earth_z[i] = x_aposteriori[i] / B;
    y[i] = x_aposteriori[i + 3] / b_B;
  }

  earth_x[0] = earth_z[1] * y[2] - earth_z[2] * y[1];
  earth_x[1] = earth_z[2] * y[0] - earth_z[0] * y[2];
  earth_x[2] = earth_z[0] * y[1] - earth_z[1] * y[0];

  /* 'attitudeKalmanfilter:99' earth_y=cross(earth_x,earth_z); */
  /* 'attitudeKalmanfilter:101' Rot_matrix=[earth_x,earth_y,earth_z]; */
  y[0] = earth_x[1] * earth_z[2] - earth_x[2] * earth_z[1];
  y[1] = earth_x[2] * earth_z[0] - earth_x[0] * earth_z[2];
  y[2] = earth_x[0] * earth_z[1] - earth_x[1] * earth_z[0];
  for (i = 0; i < 3; i++) {
    Rot_matrix[i] = earth_x[i];
    Rot_matrix[3 + i] = y[i];
    Rot_matrix[6 + i] = earth_z[i];
  }
}

/* End of code generation (attitudeKalmanfilter.c) */
