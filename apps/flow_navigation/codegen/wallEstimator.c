/*
 * wallEstimator.c
 *
 * Code generation for function 'wallEstimator'
 *
 * C source code generated on: Thu Feb 28 10:58:05 2013
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "flowNavigation.h"
#include "frontFlowKalmanFilter.h"
#include "wallEstimator.h"
#include "mldivide.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void wallEstimator(const real32_T flow_left[10], const real32_T flow_right[10],
                   real32_T front_distance, int16_T quality, const real32_T
                   speed[2], const real32_T thresholds[3], real32_T *dist_left,
                   real32_T *dist_right)
{
  int8_T invalid_flow_filter[10];
  int32_T i;
  real_T vectors[20];
  real32_T flow;
  static const real_T dv0[20] = { -0.86602540378443871, -0.7880107536067219,
    -0.69465837045899725, -0.58778525229247314, -0.46947156278589081, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.46947156278589081,
    0.58778525229247314, 0.69465837045899725, 0.7880107536067219,
    0.86602540378443871 };

  int32_T i2;
  static const real_T unit_vectors[40] = { 0.49999999999999994,
    0.61566147532565829, 0.71933980033865119, 0.80901699437494745,
    0.882947592858927, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    0.882947592858927, 0.80901699437494745, 0.71933980033865119,
    0.61566147532565829, 0.49999999999999994, -0.86602540378443871,
    -0.7880107536067219, -0.69465837045899725, -0.58778525229247314,
    -0.46947156278589081, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.46947156278589081, 0.58778525229247314, 0.69465837045899725,
    0.7880107536067219, 0.86602540378443871 };

  real_T y;
  real_T b_vectors[20];
  real_T c_vectors[10];
  real_T wall[2];

  /* WALL_ESTIMATION Summary of this function goes here */
  /*    Detailed explanation goes here */
  /* 0.05; */
  /* 0.05; */
  /* 2; */
  *dist_left = 10.0F;

  /*  maximum distance */
  *dist_right = 10.0F;

  /*  maximum distance */
  if ((real32_T)fabs(speed[0]) > thresholds[0]) {
    /*  LEFT --------------------------------------------------------------- */
    for (i = 0; i < 10; i++) {
      invalid_flow_filter[i] = 1;
    }

    memset(&vectors[0], 0, 20U * sizeof(real_T));
    for (i = 0; i < 10; i++) {
      flow = flow_left[i] / 1000.0F;
      if ((real32_T)fabs(flow) > thresholds[1]) {
        flow = speed[0] / flow * (real32_T)dv0[i];
        if (flow > 0.0F) {
          for (i2 = 0; i2 < 2; i2++) {
            vectors[i + 10 * i2] = (real32_T)unit_vectors[i + 20 * i2] * flow;
          }
        } else {
          invalid_flow_filter[i] = 0;
        }
      } else {
        invalid_flow_filter[i] = 0;
      }
    }

    /*  calc left wall with linear regresstion */
    y = (real_T)invalid_flow_filter[0];
    for (i = 0; i < 9; i++) {
      y += (real_T)invalid_flow_filter[i + 1];
    }

    if (y > (real_T)thresholds[2]) {
      for (i2 = 0; i2 < 10; i2++) {
        b_vectors[i2] = vectors[i2] * (real_T)invalid_flow_filter[i2];
        b_vectors[10 + i2] = (real_T)invalid_flow_filter[i2];
        c_vectors[i2] = vectors[10 + i2] * (real_T)invalid_flow_filter[i2];
      }

      mldivide(b_vectors, c_vectors, wall);
      if (wall[1] < 0.0) {
        /* && wall(2) > -10 % left wall can only be in negative y range... else invalid data (max 10m) */
        *dist_left = (real32_T)-wall[1];
      }
    }

    /*  RIGHT -------------------------------------------------------------- */
    for (i = 0; i < 10; i++) {
      invalid_flow_filter[i] = 1;
    }

    memset(&vectors[0], 0, 20U * sizeof(real_T));
    for (i = 0; i < 10; i++) {
      flow = flow_right[i] / 1000.0F;
      if ((real32_T)fabs(flow) > thresholds[1]) {
        flow = speed[0] / flow * (real32_T)dv0[10 + i];
        if (flow > 0.0F) {
          for (i2 = 0; i2 < 2; i2++) {
            vectors[i + 10 * i2] = (real32_T)unit_vectors[(i + 20 * i2) + 10] *
              flow;
          }
        } else {
          invalid_flow_filter[i] = 0;
        }
      } else {
        invalid_flow_filter[i] = 0;
      }
    }

    /*  calc right wall with linear regresstion */
    y = (real_T)invalid_flow_filter[0];
    for (i = 0; i < 9; i++) {
      y += (real_T)invalid_flow_filter[i + 1];
    }

    if (y > (real_T)thresholds[2]) {
      for (i2 = 0; i2 < 10; i2++) {
        b_vectors[i2] = vectors[i2] * (real_T)invalid_flow_filter[i2];
        b_vectors[10 + i2] = (real_T)invalid_flow_filter[i2];
        c_vectors[i2] = vectors[10 + i2] * (real_T)invalid_flow_filter[i2];
      }

      mldivide(b_vectors, c_vectors, wall);
      if (wall[1] > 0.0) {
        /* && wall(2) < 10 % right wall can only be in positiv y range... else invalid data (max 10m) */
        *dist_right = (real32_T)wall[1];
      }
    }
  }
}

/* End of code generation (wallEstimator.c) */
