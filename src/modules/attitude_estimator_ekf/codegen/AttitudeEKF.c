/*
 * AttitudeEKF.c
 *
 * Code generation for function 'AttitudeEKF'
 *
 * C source code generated on: Fri Jul 25 14:06:41 2014
 *
 */

/* Include files */
#include "AttitudeEKF.h"

/* Variable Definitions */
static float Ji[9];
static boolean_T Ji_not_empty;
static float x_apo[12];
static float P_apo[144];
static float Q[144];
static boolean_T Q_not_empty;

/* Function Declarations */
static void AttitudeEKF_init(void);
static void b_mrdivide(const float A[72], const float B[36], float y[72]);
static void inv(const float x[9], float y[9]);
static void mrdivide(const float A[108], const float B[81], float y[108]);
static float norm(const float x[3]);

/* Function Definitions */
static void AttitudeEKF_init(void)
{
  int i;
  static const float fv5[12] = { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    -9.81F, 1.0F, 0.0F, 0.0F };

  for (i = 0; i < 12; i++) {
    x_apo[i] = fv5[i];
  }

  for (i = 0; i < 144; i++) {
    P_apo[i] = 200.0F;
  }
}

/*
 *
 */
static void b_mrdivide(const float A[72], const float B[36], float y[72])
{
  float b_A[36];
  signed char ipiv[6];
  int i1;
  int iy;
  int j;
  int c;
  int ix;
  float temp;
  int k;
  float s;
  int jy;
  int ijA;
  float Y[72];
  for (i1 = 0; i1 < 6; i1++) {
    for (iy = 0; iy < 6; iy++) {
      b_A[iy + 6 * i1] = B[i1 + 6 * iy];
    }

    ipiv[i1] = (signed char)(1 + i1);
  }

  for (j = 0; j < 5; j++) {
    c = j * 7;
    iy = 0;
    ix = c;
    temp = (real32_T)fabs(b_A[c]);
    for (k = 2; k <= 6 - j; k++) {
      ix++;
      s = (real32_T)fabs(b_A[ix]);
      if (s > temp) {
        iy = k - 1;
        temp = s;
      }
    }

    if (b_A[c + iy] != 0.0F) {
      if (iy != 0) {
        ipiv[j] = (signed char)((j + iy) + 1);
        ix = j;
        iy += j;
        for (k = 0; k < 6; k++) {
          temp = b_A[ix];
          b_A[ix] = b_A[iy];
          b_A[iy] = temp;
          ix += 6;
          iy += 6;
        }
      }

      i1 = (c - j) + 6;
      for (jy = c + 1; jy + 1 <= i1; jy++) {
        b_A[jy] /= b_A[c];
      }
    }

    iy = c;
    jy = c + 6;
    for (k = 1; k <= 5 - j; k++) {
      temp = b_A[jy];
      if (b_A[jy] != 0.0F) {
        ix = c + 1;
        i1 = (iy - j) + 12;
        for (ijA = 7 + iy; ijA + 1 <= i1; ijA++) {
          b_A[ijA] += b_A[ix] * -temp;
          ix++;
        }
      }

      jy += 6;
      iy += 6;
    }
  }

  for (i1 = 0; i1 < 12; i1++) {
    for (iy = 0; iy < 6; iy++) {
      Y[iy + 6 * i1] = A[i1 + 12 * iy];
    }
  }

  for (jy = 0; jy < 6; jy++) {
    if (ipiv[jy] != jy + 1) {
      for (j = 0; j < 12; j++) {
        temp = Y[jy + 6 * j];
        Y[jy + 6 * j] = Y[(ipiv[jy] + 6 * j) - 1];
        Y[(ipiv[jy] + 6 * j) - 1] = temp;
      }
    }
  }

  for (j = 0; j < 12; j++) {
    c = 6 * j;
    for (k = 0; k < 6; k++) {
      iy = 6 * k;
      if (Y[k + c] != 0.0F) {
        for (jy = k + 2; jy < 7; jy++) {
          Y[(jy + c) - 1] -= Y[k + c] * b_A[(jy + iy) - 1];
        }
      }
    }
  }

  for (j = 0; j < 12; j++) {
    c = 6 * j;
    for (k = 5; k > -1; k += -1) {
      iy = 6 * k;
      if (Y[k + c] != 0.0F) {
        Y[k + c] /= b_A[k + iy];
        for (jy = 0; jy + 1 <= k; jy++) {
          Y[jy + c] -= Y[k + c] * b_A[jy + iy];
        }
      }
    }
  }

  for (i1 = 0; i1 < 6; i1++) {
    for (iy = 0; iy < 12; iy++) {
      y[iy + 12 * i1] = Y[i1 + 6 * iy];
    }
  }
}

/*
 *
 */
static void inv(const float x[9], float y[9])
{
  float b_x[9];
  int p1;
  int p2;
  int p3;
  float absx11;
  float absx21;
  float absx31;
  int itmp;
  for (p1 = 0; p1 < 9; p1++) {
    b_x[p1] = x[p1];
  }

  p1 = 0;
  p2 = 3;
  p3 = 6;
  absx11 = (real32_T)fabs(x[0]);
  absx21 = (real32_T)fabs(x[1]);
  absx31 = (real32_T)fabs(x[2]);
  if ((absx21 > absx11) && (absx21 > absx31)) {
    p1 = 3;
    p2 = 0;
    b_x[0] = x[1];
    b_x[1] = x[0];
    b_x[3] = x[4];
    b_x[4] = x[3];
    b_x[6] = x[7];
    b_x[7] = x[6];
  } else {
    if (absx31 > absx11) {
      p1 = 6;
      p3 = 0;
      b_x[0] = x[2];
      b_x[2] = x[0];
      b_x[3] = x[5];
      b_x[5] = x[3];
      b_x[6] = x[8];
      b_x[8] = x[6];
    }
  }

  absx11 = b_x[1] / b_x[0];
  b_x[1] /= b_x[0];
  absx21 = b_x[2] / b_x[0];
  b_x[2] /= b_x[0];
  b_x[4] -= absx11 * b_x[3];
  b_x[5] -= absx21 * b_x[3];
  b_x[7] -= absx11 * b_x[6];
  b_x[8] -= absx21 * b_x[6];
  if ((real32_T)fabs(b_x[5]) > (real32_T)fabs(b_x[4])) {
    itmp = p2;
    p2 = p3;
    p3 = itmp;
    b_x[1] = absx21;
    b_x[2] = absx11;
    absx11 = b_x[4];
    b_x[4] = b_x[5];
    b_x[5] = absx11;
    absx11 = b_x[7];
    b_x[7] = b_x[8];
    b_x[8] = absx11;
  }

  absx11 = b_x[5] / b_x[4];
  b_x[5] /= b_x[4];
  b_x[8] -= absx11 * b_x[7];
  absx11 = (b_x[5] * b_x[1] - b_x[2]) / b_x[8];
  absx21 = -(b_x[1] + b_x[7] * absx11) / b_x[4];
  y[p1] = ((1.0F - b_x[3] * absx21) - b_x[6] * absx11) / b_x[0];
  y[p1 + 1] = absx21;
  y[p1 + 2] = absx11;
  absx11 = -b_x[5] / b_x[8];
  absx21 = (1.0F - b_x[7] * absx11) / b_x[4];
  y[p2] = -(b_x[3] * absx21 + b_x[6] * absx11) / b_x[0];
  y[p2 + 1] = absx21;
  y[p2 + 2] = absx11;
  absx11 = 1.0F / b_x[8];
  absx21 = -b_x[7] * absx11 / b_x[4];
  y[p3] = -(b_x[3] * absx21 + b_x[6] * absx11) / b_x[0];
  y[p3 + 1] = absx21;
  y[p3 + 2] = absx11;
}

/*
 *
 */
static void mrdivide(const float A[108], const float B[81], float y[108])
{
  float b_A[81];
  signed char ipiv[9];
  int i0;
  int iy;
  int j;
  int c;
  int ix;
  float temp;
  int k;
  float s;
  int jy;
  int ijA;
  float Y[108];
  for (i0 = 0; i0 < 9; i0++) {
    for (iy = 0; iy < 9; iy++) {
      b_A[iy + 9 * i0] = B[i0 + 9 * iy];
    }

    ipiv[i0] = (signed char)(1 + i0);
  }

  for (j = 0; j < 8; j++) {
    c = j * 10;
    iy = 0;
    ix = c;
    temp = (real32_T)fabs(b_A[c]);
    for (k = 2; k <= 9 - j; k++) {
      ix++;
      s = (real32_T)fabs(b_A[ix]);
      if (s > temp) {
        iy = k - 1;
        temp = s;
      }
    }

    if (b_A[c + iy] != 0.0F) {
      if (iy != 0) {
        ipiv[j] = (signed char)((j + iy) + 1);
        ix = j;
        iy += j;
        for (k = 0; k < 9; k++) {
          temp = b_A[ix];
          b_A[ix] = b_A[iy];
          b_A[iy] = temp;
          ix += 9;
          iy += 9;
        }
      }

      i0 = (c - j) + 9;
      for (jy = c + 1; jy + 1 <= i0; jy++) {
        b_A[jy] /= b_A[c];
      }
    }

    iy = c;
    jy = c + 9;
    for (k = 1; k <= 8 - j; k++) {
      temp = b_A[jy];
      if (b_A[jy] != 0.0F) {
        ix = c + 1;
        i0 = (iy - j) + 18;
        for (ijA = 10 + iy; ijA + 1 <= i0; ijA++) {
          b_A[ijA] += b_A[ix] * -temp;
          ix++;
        }
      }

      jy += 9;
      iy += 9;
    }
  }

  for (i0 = 0; i0 < 12; i0++) {
    for (iy = 0; iy < 9; iy++) {
      Y[iy + 9 * i0] = A[i0 + 12 * iy];
    }
  }

  for (jy = 0; jy < 9; jy++) {
    if (ipiv[jy] != jy + 1) {
      for (j = 0; j < 12; j++) {
        temp = Y[jy + 9 * j];
        Y[jy + 9 * j] = Y[(ipiv[jy] + 9 * j) - 1];
        Y[(ipiv[jy] + 9 * j) - 1] = temp;
      }
    }
  }

  for (j = 0; j < 12; j++) {
    c = 9 * j;
    for (k = 0; k < 9; k++) {
      iy = 9 * k;
      if (Y[k + c] != 0.0F) {
        for (jy = k + 2; jy < 10; jy++) {
          Y[(jy + c) - 1] -= Y[k + c] * b_A[(jy + iy) - 1];
        }
      }
    }
  }

  for (j = 0; j < 12; j++) {
    c = 9 * j;
    for (k = 8; k > -1; k += -1) {
      iy = 9 * k;
      if (Y[k + c] != 0.0F) {
        Y[k + c] /= b_A[k + iy];
        for (jy = 0; jy + 1 <= k; jy++) {
          Y[jy + c] -= Y[k + c] * b_A[jy + iy];
        }
      }
    }
  }

  for (i0 = 0; i0 < 9; i0++) {
    for (iy = 0; iy < 12; iy++) {
      y[iy + 12 * i0] = Y[i0 + 9 * iy];
    }
  }
}

/*
 *
 */
static float norm(const float x[3])
{
  float y;
  float scale;
  int k;
  float absxk;
  float t;
  y = 0.0F;
  scale = 1.17549435E-38F;
  for (k = 0; k < 3; k++) {
    absxk = (real32_T)fabs(x[k]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0F + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * (real32_T)sqrt(y);
}

/*
 * function [xa_apo,Pa_apo,Rot_matrix,eulerAngles,debugOutput]...
 *     = AttitudeEKF(approx_prediction,use_inertia_matrix,zFlag,dt,z,q_rotSpeed,q_rotAcc,q_acc,q_mag,r_gyro,r_accel,r_mag,J)
 */
void AttitudeEKF(unsigned char approx_prediction, unsigned char
                 use_inertia_matrix, const unsigned char zFlag[3], float dt,
                 const float z[9], float q_rotSpeed, float q_rotAcc, float q_acc,
                 float q_mag, float r_gyro, float r_accel, float r_mag, const
                 float J[9], float xa_apo[12], float Pa_apo[144], float
                 Rot_matrix[9], float eulerAngles[3], float debugOutput[4])
{
  int i;
  float fv0[3];
  int r2;
  float zek[3];
  float muk[3];
  float b_muk[3];
  float c_muk[3];
  float fv1[3];
  float wak[3];
  float O[9];
  float b_O[9];
  static const signed char iv0[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  float fv2[3];
  float maxval;
  int r1;
  float fv3[9];
  float fv4[3];
  float x_apr[12];
  signed char I[144];
  float A_lin[144];
  static const signed char iv1[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  float b_A_lin[144];
  float v[12];
  float P_apr[144];
  float b_P_apr[108];
  static const signed char b[108] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  float K_k[108];
  float a[81];
  static const signed char b_a[108] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  float b_r_gyro[81];
  float c_a[81];
  float d_a[36];
  static const signed char e_a[36] = { 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const signed char b_b[36] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  float c_r_gyro[9];
  float b_K_k[36];
  int r3;
  float a21;
  float f_a[36];
  float c_P_apr[72];
  static const signed char c_b[72] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0 };

  float c_K_k[72];
  static const signed char g_a[72] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0 };

  float b_z[6];
  static const signed char d_b[72] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1 };

  static const signed char h_a[72] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1 };

  float i_a[6];
  float c_z[6];

  /* LQG Postion Estimator and Controller */
  /*  Observer: */
  /*         x[n|n]   = x[n|n-1] + M(y[n] - Cx[n|n-1] - Du[n]) */
  /*         x[n+1|n] = Ax[n|n] + Bu[n] */
  /*  */
  /*  $Author: Tobias Naegeli $    $Date: 2014 $    $Revision: 3 $ */
  /*  */
  /*  */
  /*  Arguments: */
  /*  approx_prediction: if 1 then the exponential map is approximated with a */
  /*  first order taylor approximation. has at the moment not a big influence */
  /*  (just 1st or 2nd order approximation) we should change it to rodriquez */
  /*  approximation. */
  /*  use_inertia_matrix: set to true if you have the inertia matrix J for your */
  /*  quadrotor */
  /*  xa_apo_k: old state vectotr */
  /*  zFlag: if sensor measurement is available [gyro, acc, mag] */
  /*  dt: dt in s */
  /*  z: measurements [gyro, acc, mag] */
  /*  q_rotSpeed: process noise gyro */
  /*  q_rotAcc: process noise gyro acceleration */
  /*  q_acc: process noise acceleration */
  /*  q_mag: process noise magnetometer */
  /*  r_gyro: measurement noise gyro */
  /*  r_accel: measurement noise accel */
  /*  r_mag: measurement noise mag */
  /*  J: moment of inertia matrix */
  /*  Output: */
  /*  xa_apo: updated state vectotr */
  /*  Pa_apo: updated state covariance matrix */
  /*  Rot_matrix: rotation matrix */
  /*  eulerAngles: euler angles */
  /*  debugOutput: not used */
  /* % model specific parameters */
  /*  compute once the inverse of the Inertia */
  /* 'AttitudeEKF:48' if isempty(Ji) */
  if (!Ji_not_empty) {
    /* 'AttitudeEKF:49' Ji=single(inv(J)); */
    inv(J, Ji);
    Ji_not_empty = TRUE;
  }

  /* % init */
  /* 'AttitudeEKF:54' if(isempty(x_apo)) */
  /* 'AttitudeEKF:64' if(isempty(P_apo)) */
  /* 'AttitudeEKF:69' debugOutput = single(zeros(4,1)); */
  for (i = 0; i < 4; i++) {
    debugOutput[i] = 0.0F;
  }

  /* % copy the states */
  /* 'AttitudeEKF:72' wx=  x_apo(1); */
  /*  x  body angular rate */
  /* 'AttitudeEKF:73' wy=  x_apo(2); */
  /*  y  body angular rate */
  /* 'AttitudeEKF:74' wz=  x_apo(3); */
  /*  z  body angular rate */
  /* 'AttitudeEKF:76' wax=  x_apo(4); */
  /*  x  body angular acceleration */
  /* 'AttitudeEKF:77' way=  x_apo(5); */
  /*  y  body angular acceleration */
  /* 'AttitudeEKF:78' waz=  x_apo(6); */
  /*  z  body angular acceleration */
  /* 'AttitudeEKF:80' zex=  x_apo(7); */
  /*  x  component gravity vector */
  /* 'AttitudeEKF:81' zey=  x_apo(8); */
  /*  y  component gravity vector */
  /* 'AttitudeEKF:82' zez=  x_apo(9); */
  /*  z  component gravity vector */
  /* 'AttitudeEKF:84' mux=  x_apo(10); */
  /*  x  component magnetic field vector */
  /* 'AttitudeEKF:85' muy=  x_apo(11); */
  /*  y  component magnetic field vector */
  /* 'AttitudeEKF:86' muz=  x_apo(12); */
  /*  z  component magnetic field vector */
  /* % prediction section */
  /*  compute the apriori state estimate from the previous aposteriori estimate */
  /* body angular accelerations */
  /* 'AttitudeEKF:94' if (use_inertia_matrix==1) */
  if (use_inertia_matrix == 1) {
    /* 'AttitudeEKF:95' wak =[wax;way;waz]+Ji*(-cross([wax;way;waz],J*[wax;way;waz]))*dt; */
    fv0[0] = x_apo[3];
    fv0[1] = x_apo[4];
    fv0[2] = x_apo[5];
    for (r2 = 0; r2 < 3; r2++) {
      zek[r2] = 0.0F;
      for (i = 0; i < 3; i++) {
        zek[r2] += J[r2 + 3 * i] * fv0[i];
      }
    }

    muk[0] = x_apo[3];
    muk[1] = x_apo[4];
    muk[2] = x_apo[5];
    b_muk[0] = x_apo[4] * zek[2] - x_apo[5] * zek[1];
    b_muk[1] = x_apo[5] * zek[0] - x_apo[3] * zek[2];
    b_muk[2] = x_apo[3] * zek[1] - x_apo[4] * zek[0];
    for (r2 = 0; r2 < 3; r2++) {
      c_muk[r2] = -b_muk[r2];
    }

    fv1[0] = x_apo[3];
    fv1[1] = x_apo[4];
    fv1[2] = x_apo[5];
    for (r2 = 0; r2 < 3; r2++) {
      fv0[r2] = 0.0F;
      for (i = 0; i < 3; i++) {
        fv0[r2] += Ji[r2 + 3 * i] * c_muk[i];
      }

      wak[r2] = fv1[r2] + fv0[r2] * dt;
    }
  } else {
    /* 'AttitudeEKF:96' else */
    /* 'AttitudeEKF:97' wak =[wax;way;waz]; */
    wak[0] = x_apo[3];
    wak[1] = x_apo[4];
    wak[2] = x_apo[5];
  }

  /* body angular rates */
  /* 'AttitudeEKF:101' wk =[wx;  wy; wz] + dt*wak; */
  /* derivative of the prediction rotation matrix */
  /* 'AttitudeEKF:104' O=[0,-wz,wy;wz,0,-wx;-wy,wx,0]'; */
  O[0] = 0.0F;
  O[1] = -x_apo[2];
  O[2] = x_apo[1];
  O[3] = x_apo[2];
  O[4] = 0.0F;
  O[5] = -x_apo[0];
  O[6] = -x_apo[1];
  O[7] = x_apo[0];
  O[8] = 0.0F;

  /* prediction of the earth z vector */
  /* 'AttitudeEKF:107' if (approx_prediction==1) */
  if (approx_prediction == 1) {
    /* e^(Odt)=I+dt*O+dt^2/2!O^2 */
    /*  so we do a first order approximation of the exponential map */
    /* 'AttitudeEKF:110' zek =(O*dt+single(eye(3)))*[zex;zey;zez]; */
    for (r2 = 0; r2 < 3; r2++) {
      for (i = 0; i < 3; i++) {
        b_O[i + 3 * r2] = O[i + 3 * r2] * dt + (float)iv0[i + 3 * r2];
      }
    }

    fv2[0] = x_apo[6];
    fv2[1] = x_apo[7];
    fv2[2] = x_apo[8];
    for (r2 = 0; r2 < 3; r2++) {
      zek[r2] = 0.0F;
      for (i = 0; i < 3; i++) {
        zek[r2] += b_O[r2 + 3 * i] * fv2[i];
      }
    }
  } else {
    /* 'AttitudeEKF:112' else */
    /* 'AttitudeEKF:113' zek =(single(eye(3))+O*dt+dt^2/2*O^2)*[zex;zey;zez]; */
    maxval = dt * dt / 2.0F;
    for (r2 = 0; r2 < 3; r2++) {
      for (i = 0; i < 3; i++) {
        b_O[r2 + 3 * i] = 0.0F;
        for (r1 = 0; r1 < 3; r1++) {
          b_O[r2 + 3 * i] += O[r2 + 3 * r1] * O[r1 + 3 * i];
        }
      }
    }

    for (r2 = 0; r2 < 3; r2++) {
      for (i = 0; i < 3; i++) {
        fv3[i + 3 * r2] = ((float)iv0[i + 3 * r2] + O[i + 3 * r2] * dt) + maxval
          * b_O[i + 3 * r2];
      }
    }

    fv2[0] = x_apo[6];
    fv2[1] = x_apo[7];
    fv2[2] = x_apo[8];
    for (r2 = 0; r2 < 3; r2++) {
      zek[r2] = 0.0F;
      for (i = 0; i < 3; i++) {
        zek[r2] += fv3[r2 + 3 * i] * fv2[i];
      }
    }

    /* zek =expm2(O*dt)*[zex;zey;zez]; not working because use double */
    /* precision */
  }

  /* prediction of the magnetic vector */
  /* 'AttitudeEKF:121' if (approx_prediction==1) */
  if (approx_prediction == 1) {
    /* e^(Odt)=I+dt*O+dt^2/2!O^2 */
    /*  so we do a first order approximation of the exponential map */
    /* 'AttitudeEKF:124' muk =(O*dt+single(eye(3)))*[mux;muy;muz]; */
    for (r2 = 0; r2 < 3; r2++) {
      for (i = 0; i < 3; i++) {
        b_O[i + 3 * r2] = O[i + 3 * r2] * dt + (float)iv0[i + 3 * r2];
      }
    }

    fv4[0] = x_apo[9];
    fv4[1] = x_apo[10];
    fv4[2] = x_apo[11];
    for (r2 = 0; r2 < 3; r2++) {
      muk[r2] = 0.0F;
      for (i = 0; i < 3; i++) {
        muk[r2] += b_O[r2 + 3 * i] * fv4[i];
      }
    }
  } else {
    /* 'AttitudeEKF:125' else */
    /* 'AttitudeEKF:126' muk =(single(eye(3))+O*dt+dt^2/2*O^2)*[mux;muy;muz]; */
    maxval = dt * dt / 2.0F;
    for (r2 = 0; r2 < 3; r2++) {
      for (i = 0; i < 3; i++) {
        b_O[r2 + 3 * i] = 0.0F;
        for (r1 = 0; r1 < 3; r1++) {
          b_O[r2 + 3 * i] += O[r2 + 3 * r1] * O[r1 + 3 * i];
        }
      }
    }

    for (r2 = 0; r2 < 3; r2++) {
      for (i = 0; i < 3; i++) {
        fv3[i + 3 * r2] = ((float)iv0[i + 3 * r2] + O[i + 3 * r2] * dt) + maxval
          * b_O[i + 3 * r2];
      }
    }

    fv4[0] = x_apo[9];
    fv4[1] = x_apo[10];
    fv4[2] = x_apo[11];
    for (r2 = 0; r2 < 3; r2++) {
      muk[r2] = 0.0F;
      for (i = 0; i < 3; i++) {
        muk[r2] += fv3[r2 + 3 * i] * fv4[i];
      }
    }

    /* muk =expm2(O*dt)*[mux;muy;muz]; not working because use double */
    /* precision */
  }

  /* 'AttitudeEKF:131' x_apr=[wk;wak;zek;muk]; */
  x_apr[0] = x_apo[0] + dt * wak[0];
  x_apr[1] = x_apo[1] + dt * wak[1];
  x_apr[2] = x_apo[2] + dt * wak[2];
  for (i = 0; i < 3; i++) {
    x_apr[i + 3] = wak[i];
  }

  for (i = 0; i < 3; i++) {
    x_apr[i + 6] = zek[i];
  }

  for (i = 0; i < 3; i++) {
    x_apr[i + 9] = muk[i];
  }

  /*  compute the apriori error covariance estimate from the previous */
  /* aposteriori estimate */
  /* 'AttitudeEKF:136' EZ=[0,zez,-zey; */
  /* 'AttitudeEKF:137'     -zez,0,zex; */
  /* 'AttitudeEKF:138'     zey,-zex,0]'; */
  /* 'AttitudeEKF:139' MA=[0,muz,-muy; */
  /* 'AttitudeEKF:140'     -muz,0,mux; */
  /* 'AttitudeEKF:141'     muy,-mux,0]'; */
  /* 'AttitudeEKF:143' E=single(eye(3)); */
  /* 'AttitudeEKF:144' Z=single(zeros(3)); */
  /* 'AttitudeEKF:146' A_lin=[ Z,  E,  Z,  Z */
  /* 'AttitudeEKF:147'     Z,  Z,  Z,  Z */
  /* 'AttitudeEKF:148'     EZ, Z,  O,  Z */
  /* 'AttitudeEKF:149'     MA, Z,  Z,  O]; */
  /* 'AttitudeEKF:151' A_lin=eye(12)+A_lin*dt; */
  memset(&I[0], 0, 144U * sizeof(signed char));
  for (i = 0; i < 12; i++) {
    I[i + 12 * i] = 1;
    for (r2 = 0; r2 < 3; r2++) {
      A_lin[r2 + 12 * i] = iv1[r2 + 3 * i];
    }

    for (r2 = 0; r2 < 3; r2++) {
      A_lin[(r2 + 12 * i) + 3] = 0.0F;
    }
  }

  A_lin[6] = 0.0F;
  A_lin[7] = x_apo[8];
  A_lin[8] = -x_apo[7];
  A_lin[18] = -x_apo[8];
  A_lin[19] = 0.0F;
  A_lin[20] = x_apo[6];
  A_lin[30] = x_apo[7];
  A_lin[31] = -x_apo[6];
  A_lin[32] = 0.0F;
  for (r2 = 0; r2 < 3; r2++) {
    for (i = 0; i < 3; i++) {
      A_lin[(i + 12 * (r2 + 3)) + 6] = 0.0F;
    }
  }

  for (r2 = 0; r2 < 3; r2++) {
    for (i = 0; i < 3; i++) {
      A_lin[(i + 12 * (r2 + 6)) + 6] = O[i + 3 * r2];
    }
  }

  for (r2 = 0; r2 < 3; r2++) {
    for (i = 0; i < 3; i++) {
      A_lin[(i + 12 * (r2 + 9)) + 6] = 0.0F;
    }
  }

  A_lin[9] = 0.0F;
  A_lin[10] = x_apo[11];
  A_lin[11] = -x_apo[10];
  A_lin[21] = -x_apo[11];
  A_lin[22] = 0.0F;
  A_lin[23] = x_apo[9];
  A_lin[33] = x_apo[10];
  A_lin[34] = -x_apo[9];
  A_lin[35] = 0.0F;
  for (r2 = 0; r2 < 3; r2++) {
    for (i = 0; i < 3; i++) {
      A_lin[(i + 12 * (r2 + 3)) + 9] = 0.0F;
    }
  }

  for (r2 = 0; r2 < 3; r2++) {
    for (i = 0; i < 3; i++) {
      A_lin[(i + 12 * (r2 + 6)) + 9] = 0.0F;
    }
  }

  for (r2 = 0; r2 < 3; r2++) {
    for (i = 0; i < 3; i++) {
      A_lin[(i + 12 * (r2 + 9)) + 9] = O[i + 3 * r2];
    }
  }

  for (r2 = 0; r2 < 12; r2++) {
    for (i = 0; i < 12; i++) {
      b_A_lin[i + 12 * r2] = (float)I[i + 12 * r2] + A_lin[i + 12 * r2] * dt;
    }
  }

  /* process covariance matrix */
  /* 'AttitudeEKF:156' if (isempty(Q)) */
  if (!Q_not_empty) {
    /* 'AttitudeEKF:157' Q=diag([ q_rotSpeed,q_rotSpeed,q_rotSpeed,... */
    /* 'AttitudeEKF:158'         q_rotAcc,q_rotAcc,q_rotAcc,... */
    /* 'AttitudeEKF:159'         q_acc,q_acc,q_acc,... */
    /* 'AttitudeEKF:160'         q_mag,q_mag,q_mag]); */
    v[0] = q_rotSpeed;
    v[1] = q_rotSpeed;
    v[2] = q_rotSpeed;
    v[3] = q_rotAcc;
    v[4] = q_rotAcc;
    v[5] = q_rotAcc;
    v[6] = q_acc;
    v[7] = q_acc;
    v[8] = q_acc;
    v[9] = q_mag;
    v[10] = q_mag;
    v[11] = q_mag;
    memset(&Q[0], 0, 144U * sizeof(float));
    for (i = 0; i < 12; i++) {
      Q[i + 12 * i] = v[i];
    }

    Q_not_empty = TRUE;
  }

  /* 'AttitudeEKF:163' P_apr=A_lin*P_apo*A_lin'+Q; */
  for (r2 = 0; r2 < 12; r2++) {
    for (i = 0; i < 12; i++) {
      A_lin[r2 + 12 * i] = 0.0F;
      for (r1 = 0; r1 < 12; r1++) {
        A_lin[r2 + 12 * i] += b_A_lin[r2 + 12 * r1] * P_apo[r1 + 12 * i];
      }
    }
  }

  for (r2 = 0; r2 < 12; r2++) {
    for (i = 0; i < 12; i++) {
      maxval = 0.0F;
      for (r1 = 0; r1 < 12; r1++) {
        maxval += A_lin[r2 + 12 * r1] * b_A_lin[i + 12 * r1];
      }

      P_apr[r2 + 12 * i] = maxval + Q[r2 + 12 * i];
    }
  }

  /* % update */
  /* 'AttitudeEKF:167' if zFlag(1)==1&&zFlag(2)==1&&zFlag(3)==1 */
  if ((zFlag[0] == 1) && (zFlag[1] == 1) && (zFlag[2] == 1)) {
    /* 'AttitudeEKF:169' R=[r_gyro,0,0,0,0,0,0,0,0; */
    /* 'AttitudeEKF:170'         0,r_gyro,0,0,0,0,0,0,0; */
    /* 'AttitudeEKF:171'         0,0,r_gyro,0,0,0,0,0,0; */
    /* 'AttitudeEKF:172'         0,0,0,r_accel,0,0,0,0,0; */
    /* 'AttitudeEKF:173'         0,0,0,0,r_accel,0,0,0,0; */
    /* 'AttitudeEKF:174'         0,0,0,0,0,r_accel,0,0,0; */
    /* 'AttitudeEKF:175'         0,0,0,0,0,0,r_mag,0,0; */
    /* 'AttitudeEKF:176'         0,0,0,0,0,0,0,r_mag,0; */
    /* 'AttitudeEKF:177'         0,0,0,0,0,0,0,0,r_mag]; */
    /* observation matrix */
    /* [zw;ze;zmk]; */
    /* 'AttitudeEKF:180' H_k=[  E,     Z,      Z,    Z; */
    /* 'AttitudeEKF:181'         Z,     Z,      E,    Z; */
    /* 'AttitudeEKF:182'         Z,     Z,      Z,    E]; */
    /* 'AttitudeEKF:184' y_k=z(1:9)-H_k*x_apr; */
    /* 'AttitudeEKF:187' S_k=H_k*P_apr*H_k'+R; */
    /* 'AttitudeEKF:188' K_k=(P_apr*H_k'/(S_k)); */
    for (r2 = 0; r2 < 12; r2++) {
      for (i = 0; i < 9; i++) {
        b_P_apr[r2 + 12 * i] = 0.0F;
        for (r1 = 0; r1 < 12; r1++) {
          b_P_apr[r2 + 12 * i] += P_apr[r2 + 12 * r1] * (float)b[r1 + 12 * i];
        }
      }
    }

    for (r2 = 0; r2 < 9; r2++) {
      for (i = 0; i < 12; i++) {
        K_k[r2 + 9 * i] = 0.0F;
        for (r1 = 0; r1 < 12; r1++) {
          K_k[r2 + 9 * i] += (float)b_a[r2 + 9 * r1] * P_apr[r1 + 12 * i];
        }
      }

      for (i = 0; i < 9; i++) {
        a[r2 + 9 * i] = 0.0F;
        for (r1 = 0; r1 < 12; r1++) {
          a[r2 + 9 * i] += K_k[r2 + 9 * r1] * (float)b[r1 + 12 * i];
        }
      }
    }

    b_r_gyro[0] = r_gyro;
    b_r_gyro[9] = 0.0F;
    b_r_gyro[18] = 0.0F;
    b_r_gyro[27] = 0.0F;
    b_r_gyro[36] = 0.0F;
    b_r_gyro[45] = 0.0F;
    b_r_gyro[54] = 0.0F;
    b_r_gyro[63] = 0.0F;
    b_r_gyro[72] = 0.0F;
    b_r_gyro[1] = 0.0F;
    b_r_gyro[10] = r_gyro;
    b_r_gyro[19] = 0.0F;
    b_r_gyro[28] = 0.0F;
    b_r_gyro[37] = 0.0F;
    b_r_gyro[46] = 0.0F;
    b_r_gyro[55] = 0.0F;
    b_r_gyro[64] = 0.0F;
    b_r_gyro[73] = 0.0F;
    b_r_gyro[2] = 0.0F;
    b_r_gyro[11] = 0.0F;
    b_r_gyro[20] = r_gyro;
    b_r_gyro[29] = 0.0F;
    b_r_gyro[38] = 0.0F;
    b_r_gyro[47] = 0.0F;
    b_r_gyro[56] = 0.0F;
    b_r_gyro[65] = 0.0F;
    b_r_gyro[74] = 0.0F;
    b_r_gyro[3] = 0.0F;
    b_r_gyro[12] = 0.0F;
    b_r_gyro[21] = 0.0F;
    b_r_gyro[30] = r_accel;
    b_r_gyro[39] = 0.0F;
    b_r_gyro[48] = 0.0F;
    b_r_gyro[57] = 0.0F;
    b_r_gyro[66] = 0.0F;
    b_r_gyro[75] = 0.0F;
    b_r_gyro[4] = 0.0F;
    b_r_gyro[13] = 0.0F;
    b_r_gyro[22] = 0.0F;
    b_r_gyro[31] = 0.0F;
    b_r_gyro[40] = r_accel;
    b_r_gyro[49] = 0.0F;
    b_r_gyro[58] = 0.0F;
    b_r_gyro[67] = 0.0F;
    b_r_gyro[76] = 0.0F;
    b_r_gyro[5] = 0.0F;
    b_r_gyro[14] = 0.0F;
    b_r_gyro[23] = 0.0F;
    b_r_gyro[32] = 0.0F;
    b_r_gyro[41] = 0.0F;
    b_r_gyro[50] = r_accel;
    b_r_gyro[59] = 0.0F;
    b_r_gyro[68] = 0.0F;
    b_r_gyro[77] = 0.0F;
    b_r_gyro[6] = 0.0F;
    b_r_gyro[15] = 0.0F;
    b_r_gyro[24] = 0.0F;
    b_r_gyro[33] = 0.0F;
    b_r_gyro[42] = 0.0F;
    b_r_gyro[51] = 0.0F;
    b_r_gyro[60] = r_mag;
    b_r_gyro[69] = 0.0F;
    b_r_gyro[78] = 0.0F;
    b_r_gyro[7] = 0.0F;
    b_r_gyro[16] = 0.0F;
    b_r_gyro[25] = 0.0F;
    b_r_gyro[34] = 0.0F;
    b_r_gyro[43] = 0.0F;
    b_r_gyro[52] = 0.0F;
    b_r_gyro[61] = 0.0F;
    b_r_gyro[70] = r_mag;
    b_r_gyro[79] = 0.0F;
    b_r_gyro[8] = 0.0F;
    b_r_gyro[17] = 0.0F;
    b_r_gyro[26] = 0.0F;
    b_r_gyro[35] = 0.0F;
    b_r_gyro[44] = 0.0F;
    b_r_gyro[53] = 0.0F;
    b_r_gyro[62] = 0.0F;
    b_r_gyro[71] = 0.0F;
    b_r_gyro[80] = r_mag;
    for (r2 = 0; r2 < 9; r2++) {
      for (i = 0; i < 9; i++) {
        c_a[i + 9 * r2] = a[i + 9 * r2] + b_r_gyro[i + 9 * r2];
      }
    }

    mrdivide(b_P_apr, c_a, K_k);

    /* 'AttitudeEKF:191' x_apo=x_apr+K_k*y_k; */
    for (r2 = 0; r2 < 9; r2++) {
      maxval = 0.0F;
      for (i = 0; i < 12; i++) {
        maxval += (float)b_a[r2 + 9 * i] * x_apr[i];
      }

      b_O[r2] = z[r2] - maxval;
    }

    for (r2 = 0; r2 < 12; r2++) {
      maxval = 0.0F;
      for (i = 0; i < 9; i++) {
        maxval += K_k[r2 + 12 * i] * b_O[i];
      }

      x_apo[r2] = x_apr[r2] + maxval;
    }

    /* 'AttitudeEKF:192' P_apo=(eye(12)-K_k*H_k)*P_apr; */
    memset(&I[0], 0, 144U * sizeof(signed char));
    for (i = 0; i < 12; i++) {
      I[i + 12 * i] = 1;
    }

    for (r2 = 0; r2 < 12; r2++) {
      for (i = 0; i < 12; i++) {
        maxval = 0.0F;
        for (r1 = 0; r1 < 9; r1++) {
          maxval += K_k[r2 + 12 * r1] * (float)b_a[r1 + 9 * i];
        }

        A_lin[r2 + 12 * i] = (float)I[r2 + 12 * i] - maxval;
      }
    }

    for (r2 = 0; r2 < 12; r2++) {
      for (i = 0; i < 12; i++) {
        P_apo[r2 + 12 * i] = 0.0F;
        for (r1 = 0; r1 < 12; r1++) {
          P_apo[r2 + 12 * i] += A_lin[r2 + 12 * r1] * P_apr[r1 + 12 * i];
        }
      }
    }
  } else {
    /* 'AttitudeEKF:193' else */
    /* 'AttitudeEKF:194' if zFlag(1)==1&&zFlag(2)==0&&zFlag(3)==0 */
    if ((zFlag[0] == 1) && (zFlag[1] == 0) && (zFlag[2] == 0)) {
      /* 'AttitudeEKF:196' R=[r_gyro,0,0; */
      /* 'AttitudeEKF:197'             0,r_gyro,0; */
      /* 'AttitudeEKF:198'             0,0,r_gyro]; */
      /* observation matrix */
      /* 'AttitudeEKF:201' H_k=[  E,     Z,      Z,    Z]; */
      /* 'AttitudeEKF:203' y_k=z(1:3)-H_k(1:3,1:12)*x_apr; */
      /* 'AttitudeEKF:205' S_k=H_k(1:3,1:12)*P_apr*H_k(1:3,1:12)'+R(1:3,1:3); */
      /* 'AttitudeEKF:206' K_k=(P_apr*H_k(1:3,1:12)'/(S_k)); */
      for (r2 = 0; r2 < 3; r2++) {
        for (i = 0; i < 12; i++) {
          d_a[r2 + 3 * i] = 0.0F;
          for (r1 = 0; r1 < 12; r1++) {
            d_a[r2 + 3 * i] += (float)e_a[r2 + 3 * r1] * P_apr[r1 + 12 * i];
          }
        }
      }

      for (r2 = 0; r2 < 3; r2++) {
        for (i = 0; i < 3; i++) {
          b_O[r2 + 3 * i] = 0.0F;
          for (r1 = 0; r1 < 12; r1++) {
            b_O[r2 + 3 * i] += d_a[i + 3 * r1] * (float)b_b[r1 + 12 * r2];
          }
        }
      }

      c_r_gyro[0] = r_gyro;
      c_r_gyro[1] = 0.0F;
      c_r_gyro[2] = 0.0F;
      c_r_gyro[3] = 0.0F;
      c_r_gyro[4] = r_gyro;
      c_r_gyro[5] = 0.0F;
      c_r_gyro[6] = 0.0F;
      c_r_gyro[7] = 0.0F;
      c_r_gyro[8] = r_gyro;
      for (r2 = 0; r2 < 3; r2++) {
        for (i = 0; i < 3; i++) {
          O[i + 3 * r2] = b_O[i + 3 * r2] + c_r_gyro[i + 3 * r2];
        }

        for (i = 0; i < 12; i++) {
          b_K_k[r2 + 3 * i] = 0.0F;
          for (r1 = 0; r1 < 12; r1++) {
            b_K_k[r2 + 3 * i] += P_apr[i + 12 * r1] * (float)b_b[r1 + 12 * r2];
          }
        }
      }

      r1 = 0;
      r2 = 1;
      r3 = 2;
      maxval = (real32_T)fabs(O[0]);
      a21 = (real32_T)fabs(O[1]);
      if (a21 > maxval) {
        maxval = a21;
        r1 = 1;
        r2 = 0;
      }

      if ((real32_T)fabs(O[2]) > maxval) {
        r1 = 2;
        r2 = 1;
        r3 = 0;
      }

      O[r2] /= O[r1];
      O[r3] /= O[r1];
      O[3 + r2] -= O[r2] * O[3 + r1];
      O[3 + r3] -= O[r3] * O[3 + r1];
      O[6 + r2] -= O[r2] * O[6 + r1];
      O[6 + r3] -= O[r3] * O[6 + r1];
      if ((real32_T)fabs(O[3 + r3]) > (real32_T)fabs(O[3 + r2])) {
        i = r2;
        r2 = r3;
        r3 = i;
      }

      O[3 + r3] /= O[3 + r2];
      O[6 + r3] -= O[3 + r3] * O[6 + r2];
      for (i = 0; i < 12; i++) {
        f_a[3 * i] = b_K_k[r1 + 3 * i];
        f_a[1 + 3 * i] = b_K_k[r2 + 3 * i] - f_a[3 * i] * O[r2];
        f_a[2 + 3 * i] = (b_K_k[r3 + 3 * i] - f_a[3 * i] * O[r3]) - f_a[1 + 3 *
          i] * O[3 + r3];
        f_a[2 + 3 * i] /= O[6 + r3];
        f_a[3 * i] -= f_a[2 + 3 * i] * O[6 + r1];
        f_a[1 + 3 * i] -= f_a[2 + 3 * i] * O[6 + r2];
        f_a[1 + 3 * i] /= O[3 + r2];
        f_a[3 * i] -= f_a[1 + 3 * i] * O[3 + r1];
        f_a[3 * i] /= O[r1];
      }

      for (r2 = 0; r2 < 3; r2++) {
        for (i = 0; i < 12; i++) {
          b_K_k[i + 12 * r2] = f_a[r2 + 3 * i];
        }
      }

      /* 'AttitudeEKF:209' x_apo=x_apr+K_k*y_k; */
      for (r2 = 0; r2 < 3; r2++) {
        maxval = 0.0F;
        for (i = 0; i < 12; i++) {
          maxval += (float)e_a[r2 + 3 * i] * x_apr[i];
        }

        b_muk[r2] = z[r2] - maxval;
      }

      for (r2 = 0; r2 < 12; r2++) {
        maxval = 0.0F;
        for (i = 0; i < 3; i++) {
          maxval += b_K_k[r2 + 12 * i] * b_muk[i];
        }

        x_apo[r2] = x_apr[r2] + maxval;
      }

      /* 'AttitudeEKF:210' P_apo=(eye(12)-K_k*H_k(1:3,1:12))*P_apr; */
      memset(&I[0], 0, 144U * sizeof(signed char));
      for (i = 0; i < 12; i++) {
        I[i + 12 * i] = 1;
      }

      for (r2 = 0; r2 < 12; r2++) {
        for (i = 0; i < 12; i++) {
          maxval = 0.0F;
          for (r1 = 0; r1 < 3; r1++) {
            maxval += b_K_k[r2 + 12 * r1] * (float)e_a[r1 + 3 * i];
          }

          A_lin[r2 + 12 * i] = (float)I[r2 + 12 * i] - maxval;
        }
      }

      for (r2 = 0; r2 < 12; r2++) {
        for (i = 0; i < 12; i++) {
          P_apo[r2 + 12 * i] = 0.0F;
          for (r1 = 0; r1 < 12; r1++) {
            P_apo[r2 + 12 * i] += A_lin[r2 + 12 * r1] * P_apr[r1 + 12 * i];
          }
        }
      }
    } else {
      /* 'AttitudeEKF:211' else */
      /* 'AttitudeEKF:212' if  zFlag(1)==1&&zFlag(2)==1&&zFlag(3)==0 */
      if ((zFlag[0] == 1) && (zFlag[1] == 1) && (zFlag[2] == 0)) {
        /* 'AttitudeEKF:214' R=[r_gyro,0,0,0,0,0; */
        /* 'AttitudeEKF:215'                 0,r_gyro,0,0,0,0; */
        /* 'AttitudeEKF:216'                 0,0,r_gyro,0,0,0; */
        /* 'AttitudeEKF:217'                 0,0,0,r_accel,0,0; */
        /* 'AttitudeEKF:218'                 0,0,0,0,r_accel,0; */
        /* 'AttitudeEKF:219'                 0,0,0,0,0,r_accel]; */
        /* observation matrix */
        /* 'AttitudeEKF:223' H_k=[  E,     Z,      Z,    Z; */
        /* 'AttitudeEKF:224'                 Z,     Z,      E,    Z]; */
        /* 'AttitudeEKF:226' y_k=z(1:6)-H_k(1:6,1:12)*x_apr; */
        /* 'AttitudeEKF:228' S_k=H_k(1:6,1:12)*P_apr*H_k(1:6,1:12)'+R(1:6,1:6); */
        /* 'AttitudeEKF:229' K_k=(P_apr*H_k(1:6,1:12)'/(S_k)); */
        for (r2 = 0; r2 < 12; r2++) {
          for (i = 0; i < 6; i++) {
            c_P_apr[r2 + 12 * i] = 0.0F;
            for (r1 = 0; r1 < 12; r1++) {
              c_P_apr[r2 + 12 * i] += P_apr[r2 + 12 * r1] * (float)c_b[r1 + 12 *
                i];
            }
          }
        }

        for (r2 = 0; r2 < 6; r2++) {
          for (i = 0; i < 12; i++) {
            c_K_k[r2 + 6 * i] = 0.0F;
            for (r1 = 0; r1 < 12; r1++) {
              c_K_k[r2 + 6 * i] += (float)g_a[r2 + 6 * r1] * P_apr[r1 + 12 * i];
            }
          }

          for (i = 0; i < 6; i++) {
            d_a[r2 + 6 * i] = 0.0F;
            for (r1 = 0; r1 < 12; r1++) {
              d_a[r2 + 6 * i] += c_K_k[r2 + 6 * r1] * (float)c_b[r1 + 12 * i];
            }
          }
        }

        b_K_k[0] = r_gyro;
        b_K_k[6] = 0.0F;
        b_K_k[12] = 0.0F;
        b_K_k[18] = 0.0F;
        b_K_k[24] = 0.0F;
        b_K_k[30] = 0.0F;
        b_K_k[1] = 0.0F;
        b_K_k[7] = r_gyro;
        b_K_k[13] = 0.0F;
        b_K_k[19] = 0.0F;
        b_K_k[25] = 0.0F;
        b_K_k[31] = 0.0F;
        b_K_k[2] = 0.0F;
        b_K_k[8] = 0.0F;
        b_K_k[14] = r_gyro;
        b_K_k[20] = 0.0F;
        b_K_k[26] = 0.0F;
        b_K_k[32] = 0.0F;
        b_K_k[3] = 0.0F;
        b_K_k[9] = 0.0F;
        b_K_k[15] = 0.0F;
        b_K_k[21] = r_accel;
        b_K_k[27] = 0.0F;
        b_K_k[33] = 0.0F;
        b_K_k[4] = 0.0F;
        b_K_k[10] = 0.0F;
        b_K_k[16] = 0.0F;
        b_K_k[22] = 0.0F;
        b_K_k[28] = r_accel;
        b_K_k[34] = 0.0F;
        b_K_k[5] = 0.0F;
        b_K_k[11] = 0.0F;
        b_K_k[17] = 0.0F;
        b_K_k[23] = 0.0F;
        b_K_k[29] = 0.0F;
        b_K_k[35] = r_accel;
        for (r2 = 0; r2 < 6; r2++) {
          for (i = 0; i < 6; i++) {
            f_a[i + 6 * r2] = d_a[i + 6 * r2] + b_K_k[i + 6 * r2];
          }
        }

        b_mrdivide(c_P_apr, f_a, c_K_k);

        /* 'AttitudeEKF:232' x_apo=x_apr+K_k*y_k; */
        for (r2 = 0; r2 < 6; r2++) {
          maxval = 0.0F;
          for (i = 0; i < 12; i++) {
            maxval += (float)g_a[r2 + 6 * i] * x_apr[i];
          }

          b_z[r2] = z[r2] - maxval;
        }

        for (r2 = 0; r2 < 12; r2++) {
          maxval = 0.0F;
          for (i = 0; i < 6; i++) {
            maxval += c_K_k[r2 + 12 * i] * b_z[i];
          }

          x_apo[r2] = x_apr[r2] + maxval;
        }

        /* 'AttitudeEKF:233' P_apo=(eye(12)-K_k*H_k(1:6,1:12))*P_apr; */
        memset(&I[0], 0, 144U * sizeof(signed char));
        for (i = 0; i < 12; i++) {
          I[i + 12 * i] = 1;
        }

        for (r2 = 0; r2 < 12; r2++) {
          for (i = 0; i < 12; i++) {
            maxval = 0.0F;
            for (r1 = 0; r1 < 6; r1++) {
              maxval += c_K_k[r2 + 12 * r1] * (float)g_a[r1 + 6 * i];
            }

            A_lin[r2 + 12 * i] = (float)I[r2 + 12 * i] - maxval;
          }
        }

        for (r2 = 0; r2 < 12; r2++) {
          for (i = 0; i < 12; i++) {
            P_apo[r2 + 12 * i] = 0.0F;
            for (r1 = 0; r1 < 12; r1++) {
              P_apo[r2 + 12 * i] += A_lin[r2 + 12 * r1] * P_apr[r1 + 12 * i];
            }
          }
        }
      } else {
        /* 'AttitudeEKF:234' else */
        /* 'AttitudeEKF:235' if  zFlag(1)==1&&zFlag(2)==0&&zFlag(3)==1 */
        if ((zFlag[0] == 1) && (zFlag[1] == 0) && (zFlag[2] == 1)) {
          /* 'AttitudeEKF:236' R=[r_gyro,0,0,0,0,0; */
          /* 'AttitudeEKF:237'                     0,r_gyro,0,0,0,0; */
          /* 'AttitudeEKF:238'                     0,0,r_gyro,0,0,0; */
          /* 'AttitudeEKF:239'                     0,0,0,r_mag,0,0; */
          /* 'AttitudeEKF:240'                     0,0,0,0,r_mag,0; */
          /* 'AttitudeEKF:241'                     0,0,0,0,0,r_mag]; */
          /* observation matrix */
          /* 'AttitudeEKF:244' H_k=[  E,     Z,      Z,    Z; */
          /* 'AttitudeEKF:245'                     Z,     Z,      Z,    E]; */
          /* 'AttitudeEKF:247' y_k=[z(1:3);z(7:9)]-H_k(1:6,1:12)*x_apr; */
          /* 'AttitudeEKF:249' S_k=H_k(1:6,1:12)*P_apr*H_k(1:6,1:12)'+R(1:6,1:6); */
          /* 'AttitudeEKF:250' K_k=(P_apr*H_k(1:6,1:12)'/(S_k)); */
          for (r2 = 0; r2 < 12; r2++) {
            for (i = 0; i < 6; i++) {
              c_P_apr[r2 + 12 * i] = 0.0F;
              for (r1 = 0; r1 < 12; r1++) {
                c_P_apr[r2 + 12 * i] += P_apr[r2 + 12 * r1] * (float)d_b[r1 + 12
                  * i];
              }
            }
          }

          for (r2 = 0; r2 < 6; r2++) {
            for (i = 0; i < 12; i++) {
              c_K_k[r2 + 6 * i] = 0.0F;
              for (r1 = 0; r1 < 12; r1++) {
                c_K_k[r2 + 6 * i] += (float)h_a[r2 + 6 * r1] * P_apr[r1 + 12 * i];
              }
            }

            for (i = 0; i < 6; i++) {
              d_a[r2 + 6 * i] = 0.0F;
              for (r1 = 0; r1 < 12; r1++) {
                d_a[r2 + 6 * i] += c_K_k[r2 + 6 * r1] * (float)d_b[r1 + 12 * i];
              }
            }
          }

          b_K_k[0] = r_gyro;
          b_K_k[6] = 0.0F;
          b_K_k[12] = 0.0F;
          b_K_k[18] = 0.0F;
          b_K_k[24] = 0.0F;
          b_K_k[30] = 0.0F;
          b_K_k[1] = 0.0F;
          b_K_k[7] = r_gyro;
          b_K_k[13] = 0.0F;
          b_K_k[19] = 0.0F;
          b_K_k[25] = 0.0F;
          b_K_k[31] = 0.0F;
          b_K_k[2] = 0.0F;
          b_K_k[8] = 0.0F;
          b_K_k[14] = r_gyro;
          b_K_k[20] = 0.0F;
          b_K_k[26] = 0.0F;
          b_K_k[32] = 0.0F;
          b_K_k[3] = 0.0F;
          b_K_k[9] = 0.0F;
          b_K_k[15] = 0.0F;
          b_K_k[21] = r_mag;
          b_K_k[27] = 0.0F;
          b_K_k[33] = 0.0F;
          b_K_k[4] = 0.0F;
          b_K_k[10] = 0.0F;
          b_K_k[16] = 0.0F;
          b_K_k[22] = 0.0F;
          b_K_k[28] = r_mag;
          b_K_k[34] = 0.0F;
          b_K_k[5] = 0.0F;
          b_K_k[11] = 0.0F;
          b_K_k[17] = 0.0F;
          b_K_k[23] = 0.0F;
          b_K_k[29] = 0.0F;
          b_K_k[35] = r_mag;
          for (r2 = 0; r2 < 6; r2++) {
            for (i = 0; i < 6; i++) {
              f_a[i + 6 * r2] = d_a[i + 6 * r2] + b_K_k[i + 6 * r2];
            }
          }

          b_mrdivide(c_P_apr, f_a, c_K_k);

          /* 'AttitudeEKF:253' x_apo=x_apr+K_k*y_k; */
          for (r2 = 0; r2 < 3; r2++) {
            b_z[r2] = z[r2];
          }

          for (r2 = 0; r2 < 3; r2++) {
            b_z[r2 + 3] = z[6 + r2];
          }

          for (r2 = 0; r2 < 6; r2++) {
            i_a[r2] = 0.0F;
            for (i = 0; i < 12; i++) {
              i_a[r2] += (float)h_a[r2 + 6 * i] * x_apr[i];
            }

            c_z[r2] = b_z[r2] - i_a[r2];
          }

          for (r2 = 0; r2 < 12; r2++) {
            maxval = 0.0F;
            for (i = 0; i < 6; i++) {
              maxval += c_K_k[r2 + 12 * i] * c_z[i];
            }

            x_apo[r2] = x_apr[r2] + maxval;
          }

          /* 'AttitudeEKF:254' P_apo=(eye(12)-K_k*H_k(1:6,1:12))*P_apr; */
          memset(&I[0], 0, 144U * sizeof(signed char));
          for (i = 0; i < 12; i++) {
            I[i + 12 * i] = 1;
          }

          for (r2 = 0; r2 < 12; r2++) {
            for (i = 0; i < 12; i++) {
              maxval = 0.0F;
              for (r1 = 0; r1 < 6; r1++) {
                maxval += c_K_k[r2 + 12 * r1] * (float)h_a[r1 + 6 * i];
              }

              A_lin[r2 + 12 * i] = (float)I[r2 + 12 * i] - maxval;
            }
          }

          for (r2 = 0; r2 < 12; r2++) {
            for (i = 0; i < 12; i++) {
              P_apo[r2 + 12 * i] = 0.0F;
              for (r1 = 0; r1 < 12; r1++) {
                P_apo[r2 + 12 * i] += A_lin[r2 + 12 * r1] * P_apr[r1 + 12 * i];
              }
            }
          }
        } else {
          /* 'AttitudeEKF:255' else */
          /* 'AttitudeEKF:256' x_apo=x_apr; */
          for (i = 0; i < 12; i++) {
            x_apo[i] = x_apr[i];
          }

          /* 'AttitudeEKF:257' P_apo=P_apr; */
          memcpy(&P_apo[0], &P_apr[0], 144U * sizeof(float));
        }
      }
    }
  }

  /* % euler anglels extraction */
  /* 'AttitudeEKF:266' z_n_b = -x_apo(7:9)./norm(x_apo(7:9)); */
  maxval = norm(*(float (*)[3])&x_apo[6]);
  a21 = norm(*(float (*)[3])&x_apo[9]);
  for (i = 0; i < 3; i++) {
    /* 'AttitudeEKF:267' m_n_b = x_apo(10:12)./norm(x_apo(10:12)); */
    muk[i] = -x_apo[i + 6] / maxval;
    zek[i] = x_apo[i + 9] / a21;
  }

  /* 'AttitudeEKF:269' y_n_b=cross(z_n_b,m_n_b); */
  wak[0] = muk[1] * zek[2] - muk[2] * zek[1];
  wak[1] = muk[2] * zek[0] - muk[0] * zek[2];
  wak[2] = muk[0] * zek[1] - muk[1] * zek[0];

  /* 'AttitudeEKF:270' y_n_b=y_n_b./norm(y_n_b); */
  maxval = norm(wak);
  for (r2 = 0; r2 < 3; r2++) {
    wak[r2] /= maxval;
  }

  /* 'AttitudeEKF:272' x_n_b=(cross(y_n_b,z_n_b)); */
  zek[0] = wak[1] * muk[2] - wak[2] * muk[1];
  zek[1] = wak[2] * muk[0] - wak[0] * muk[2];
  zek[2] = wak[0] * muk[1] - wak[1] * muk[0];

  /* 'AttitudeEKF:273' x_n_b=x_n_b./norm(x_n_b); */
  maxval = norm(zek);
  for (r2 = 0; r2 < 3; r2++) {
    zek[r2] /= maxval;
  }

  /* 'AttitudeEKF:276' xa_apo=x_apo; */
  for (i = 0; i < 12; i++) {
    xa_apo[i] = x_apo[i];
  }

  /* 'AttitudeEKF:277' Pa_apo=P_apo; */
  memcpy(&Pa_apo[0], &P_apo[0], 144U * sizeof(float));

  /*  rotation matrix from earth to body system */
  /* 'AttitudeEKF:279' Rot_matrix=[x_n_b,y_n_b,z_n_b]; */
  for (r2 = 0; r2 < 3; r2++) {
    Rot_matrix[r2] = zek[r2];
    Rot_matrix[3 + r2] = wak[r2];
    Rot_matrix[6 + r2] = muk[r2];
  }

  /* 'AttitudeEKF:282' phi=atan2(Rot_matrix(2,3),Rot_matrix(3,3)); */
  /* 'AttitudeEKF:283' theta=-asin(Rot_matrix(1,3)); */
  /* 'AttitudeEKF:284' psi=atan2(Rot_matrix(1,2),Rot_matrix(1,1)); */
  /* 'AttitudeEKF:285' eulerAngles=[phi;theta;psi]; */
  eulerAngles[0] = (real32_T)atan2(Rot_matrix[7], Rot_matrix[8]);
  eulerAngles[1] = -(real32_T)asin(Rot_matrix[6]);
  eulerAngles[2] = (real32_T)atan2(Rot_matrix[3], Rot_matrix[0]);
}

void AttitudeEKF_initialize(void)
{
  Q_not_empty = FALSE;
  Ji_not_empty = FALSE;
  AttitudeEKF_init();
}

void AttitudeEKF_terminate(void)
{
  /* (no terminate code required) */
}

/* End of code generation (AttitudeEKF.c) */
