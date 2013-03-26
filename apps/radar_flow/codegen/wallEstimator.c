/*
 * wallEstimator.c
 *
 * Code generation for function 'wallEstimator'
 *
 * C source code generated on: Thu Mar 14 15:02:19 2013
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "flowNavigation.h"
#include "frontFlowKalmanFilter.h"
#include "wallEstimationFilter.h"
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
  real32_T vectors[20];
  real32_T flow;
  static const real32_T fv4[20] = { -0.866025388F, -0.788010776F, -0.694658399F,
    -0.587785244F, -0.469471574F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.469471574F, 0.587785244F, 0.694658399F, 0.788010776F,
    0.866025388F };

  int32_T i3;
  static const real32_T unit_vectors[40] = { 0.5F, 0.615661502F, 0.719339788F,
    0.809017F, 0.882947564F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F,
    1.0F, 1.0F, 0.882947564F, 0.809017F, 0.719339788F, 0.615661502F, 0.5F,
    -0.866025388F, -0.788010776F, -0.694658399F, -0.587785244F, -0.469471574F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.469471574F,
    0.587785244F, 0.694658399F, 0.788010776F, 0.866025388F };

  real32_T b_vectors[20];
  real32_T c_vectors[10];
  real32_T wall[2];

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

    memset(&vectors[0], 0, 20U * sizeof(real32_T));
    for (i = 0; i < 10; i++) {
      flow = flow_left[i] / 1000.0F;
      if ((real32_T)fabs(flow) > thresholds[1]) {
        flow = speed[0] / flow * fv4[i];
        if (flow > 0.0F) {
          for (i3 = 0; i3 < 2; i3++) {
            vectors[i + 10 * i3] = unit_vectors[i + 20 * i3] * flow;
          }
        } else {
          invalid_flow_filter[i] = 0;
        }
      } else {
        invalid_flow_filter[i] = 0;
      }
    }

    /*  calc left wall with linear regresstion */
    flow = (real32_T)invalid_flow_filter[0];
    for (i = 0; i < 9; i++) {
      flow += (real32_T)invalid_flow_filter[i + 1];
    }

    if (flow > thresholds[2]) {
      for (i3 = 0; i3 < 10; i3++) {
        b_vectors[i3] = vectors[i3] * (real32_T)invalid_flow_filter[i3];
        b_vectors[10 + i3] = (real32_T)invalid_flow_filter[i3];
        c_vectors[i3] = vectors[10 + i3] * (real32_T)invalid_flow_filter[i3];
      }

      mldivide(b_vectors, c_vectors, wall);
      if (wall[1] < 0.0F) {
        /* && wall(2) > -10 % left wall can only be in negative y range... else invalid data (max 10m) */
        *dist_left = -wall[1];
      }
    }

    /*  RIGHT -------------------------------------------------------------- */
    for (i = 0; i < 10; i++) {
      invalid_flow_filter[i] = 1;
    }

    memset(&vectors[0], 0, 20U * sizeof(real32_T));
    for (i = 0; i < 10; i++) {
      flow = flow_right[i] / 1000.0F;
      if ((real32_T)fabs(flow) > thresholds[1]) {
        flow = speed[0] / flow * fv4[10 + i];
        if (flow > 0.0F) {
          for (i3 = 0; i3 < 2; i3++) {
            vectors[i + 10 * i3] = unit_vectors[(i + 20 * i3) + 10] * flow;
          }
        } else {
          invalid_flow_filter[i] = 0;
        }
      } else {
        invalid_flow_filter[i] = 0;
      }
    }

    /*  calc right wall with linear regresstion */
    flow = (real32_T)invalid_flow_filter[0];
    for (i = 0; i < 9; i++) {
      flow += (real32_T)invalid_flow_filter[i + 1];
    }

    if (flow > thresholds[2]) {
      for (i3 = 0; i3 < 10; i3++) {
        b_vectors[i3] = vectors[i3] * (real32_T)invalid_flow_filter[i3];
        b_vectors[10 + i3] = (real32_T)invalid_flow_filter[i3];
        c_vectors[i3] = vectors[10 + i3] * (real32_T)invalid_flow_filter[i3];
      }

      mldivide(b_vectors, c_vectors, wall);
      if (wall[1] > 0.0F) {
        /* && wall(2) < 10 % right wall can only be in positiv y range... else invalid data (max 10m) */
        *dist_right = wall[1];
      }
    }
  }
}

/* End of code generation (wallEstimator.c) */
