/*
 * wallEstimationFilter.c
 *
 * Code generation for function 'wallEstimationFilter'
 *
 * C source code generated on: Fri Mar  8 13:08:40 2013
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
static void eml_li_find(const boolean_T x[32], int32_T y_data[32], int32_T
  y_size[2]);

/* Function Definitions */
static void eml_li_find(const boolean_T x[32], int32_T y_data[32], int32_T
  y_size[2])
{
  int32_T k;
  int32_T i;
  k = 0;
  for (i = 0; i < 32; i++) {
    if (x[i]) {
      k++;
    }
  }

  y_size[0] = 1;
  y_size[1] = k;
  k = 0;
  for (i = 0; i < 32; i++) {
    if (x[i]) {
      y_data[k] = i + 1;
      k++;
    }
  }
}

void wallEstimationFilter(const real32_T radar_filtered_k[32], const real32_T
  radar_weights_k[32], const real32_T flow_left[10], const real32_T flow_right
  [10], real32_T front_distance, uint16_T quality, const real32_T speed[2],
  const real32_T position_update[2], real32_T attitude_update, const real32_T
  thresholds[3], real32_T radar[32], real32_T radar_filtered[32], real32_T
  radar_weights[32])
{
  real_T x_transition_add[32];
  int32_T i;
  real_T y_transition_add[32];
  int32_T i2;
  real32_T wall_est_left_data[4];
  real32_T wall_est_right_data[4];
  real32_T radar_distance_update_left[32];
  real32_T radar_distance_update_right[32];
  real32_T steps;
  real32_T i_top;
  real32_T i_down;
  real32_T beta_left;
  real32_T gamma_front;
  int8_T invalid_flow_filter[10];
  real32_T vectors[20];
  static const real32_T fv0[20] = { -0.866025388F, -0.788010776F, -0.694658399F,
    -0.587785244F, -0.469471574F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.469471574F, 0.587785244F, 0.694658399F, 0.788010776F,
    0.866025388F };

  static const real32_T unit_vectors[40] = { 0.5F, 0.615661502F, 0.719339788F,
    0.809017F, 0.882947564F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F,
    1.0F, 1.0F, 0.882947564F, 0.809017F, 0.719339788F, 0.615661502F, 0.5F,
    -0.866025388F, -0.788010776F, -0.694658399F, -0.587785244F, -0.469471574F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.469471574F,
    0.587785244F, 0.694658399F, 0.788010776F, 0.866025388F };

  real32_T b_vectors[20];
  real32_T c_vectors[10];
  real32_T wall[2];
  boolean_T exitg4;
  static const real32_T fv1[15] = { 0.196349546F, 0.392699093F, 0.589048624F,
    0.785398185F, 0.981747746F, 1.17809725F, 1.37444675F, 1.57079637F,
    1.76714587F, 1.96349537F, 2.15984488F, 2.3561945F, 2.55254412F, 2.7488935F,
    2.94524312F };

  boolean_T exitg3;
  boolean_T y;
  boolean_T exitg2;
  boolean_T guard1 = FALSE;
  boolean_T exitg1;
  boolean_T b_radar_distance_update_left[32];
  int32_T tmp_size[2];
  int32_T tmp_data[32];

  /* WALL_ESTIMATION Summary of this function goes here */
  /*    Detailed explanation goes here */
  /* 0.05; */
  /* 0.05; */
  /* 2; */
  /*  settings */
  /*  only fix resolution is efficient */
  /*  calc from settings */
  for (i = 0; i < 32; i++) {
    x_transition_add[i] = (real32_T)cos(((1.0F + (real32_T)i) - 1.0F) *
      0.196349546F);
  }

  for (i = 0; i < 32; i++) {
    y_transition_add[i] = x_transition_add[(i - ((int32_T)floor((((1.0 + (real_T)
      i) + 8.0) - 1.0) / 32.0) << 5)) + 8];

    /*  default values */
    radar[i] = 5.0F;
  }

  for (i2 = 0; i2 < 2; i2++) {
    wall_est_left_data[i2] = 0.0F;
    wall_est_right_data[i2] = 0.0F;
  }

  for (i2 = 0; i2 < 32; i2++) {
    radar_distance_update_left[i2] = 0.0F;
    radar_distance_update_right[i2] = 0.0F;
  }

  /*  --------------------------------------------------------------------- */
  /*  ATTITUDE / POSITION UPDATE CALCULATION (flow) */
  /*  --------------------------------------------------------------------- */
  /*  rotation */
  if (attitude_update != 0.0F) {
    steps = attitude_update / 0.196349546F;
    i_top = (real32_T)ceil(steps);
    i_down = (real32_T)floor(steps);
    steps = i_top - steps;
    for (i = 0; i < 32; i++) {
      beta_left = ((1.0F + (real32_T)i) + i_top) - 1.0F;
      gamma_front = ((1.0F + (real32_T)i) + i_down) - 1.0F;
      radar_filtered[i] = radar_filtered_k[(int32_T)((beta_left - (real32_T)
        floor(beta_left / 32.0F) * 32.0F) + 1.0F) - 1] * (1.0F - steps) +
        radar_filtered_k[(int32_T)((gamma_front - (real32_T)floor(gamma_front /
        32.0F) * 32.0F) + 1.0F) - 1] * steps;
    }
  } else {
    memcpy(&radar_filtered[0], &radar_filtered_k[0], sizeof(real32_T) << 5);
  }

  /*  transitions */
  if (position_update[0] != 0.0F) {
    for (i2 = 0; i2 < 32; i2++) {
      radar_filtered[i2] += position_update[0] * (real32_T)x_transition_add[i2];
    }
  }

  if (position_update[1] != 0.0F) {
    for (i2 = 0; i2 < 32; i2++) {
      radar_filtered[i2] += position_update[1] * (real32_T)y_transition_add[i2];
    }
  }

  for (i = 0; i < 32; i++) {
    steps = radar_filtered[i];
    if (radar_filtered[i] > 5.0F) {
      steps = 5.0F;
    } else {
      if (radar_filtered[i] < 0.1F) {
        steps = 0.1F;
      }
    }

    radar_filtered[i] = steps;
  }

  /*  --------------------------------------------------------------------- */
  /*  WALL UPDATE CALCULATION (flow) */
  /*  --------------------------------------------------------------------- */
  if ((real32_T)fabs(speed[0]) > thresholds[0]) {
    /*  LEFT ------------------------------------------------------------ */
    for (i = 0; i < 10; i++) {
      invalid_flow_filter[i] = 1;
    }

    memset(&vectors[0], 0, 20U * sizeof(real32_T));
    for (i = 0; i < 10; i++) {
      steps = flow_left[i] / 1000.0F;
      if ((real32_T)fabs(steps) > thresholds[1]) {
        steps = speed[0] / steps * fv0[i];
        if (steps > 0.0F) {
          for (i2 = 0; i2 < 2; i2++) {
            vectors[i + 10 * i2] = unit_vectors[i + 20 * i2] * steps;
          }
        } else {
          invalid_flow_filter[i] = 0;
        }
      } else {
        invalid_flow_filter[i] = 0;
      }
    }

    /*  calc left wall with linear regresstion */
    steps = (real32_T)invalid_flow_filter[0];
    for (i = 0; i < 9; i++) {
      steps += (real32_T)invalid_flow_filter[i + 1];
    }

    if (steps > thresholds[2]) {
      for (i2 = 0; i2 < 10; i2++) {
        b_vectors[i2] = vectors[i2] * (real32_T)invalid_flow_filter[i2];
        b_vectors[10 + i2] = (real32_T)invalid_flow_filter[i2];
        c_vectors[i2] = vectors[10 + i2] * (real32_T)invalid_flow_filter[i2];
      }

      mldivide(b_vectors, c_vectors, wall);
      if (wall[1] < 0.0F) {
        /* && wall(2) > -10 % left wall can only be in negative y range... else invalid data (max 10m) */
        for (i2 = 0; i2 < 2; i2++) {
          wall_est_left_data[i2] = wall[i2];
        }
      }
    }

    /*  RIGHT ----------------------------------------------------------- */
    for (i = 0; i < 10; i++) {
      invalid_flow_filter[i] = 1;
    }

    memset(&vectors[0], 0, 20U * sizeof(real32_T));
    for (i = 0; i < 10; i++) {
      steps = flow_right[i] / 1000.0F;
      if ((real32_T)fabs(steps) > thresholds[1]) {
        steps = speed[0] / steps * fv0[10 + i];
        if (steps > 0.0F) {
          for (i2 = 0; i2 < 2; i2++) {
            vectors[i + 10 * i2] = unit_vectors[(i + 20 * i2) + 10] * steps;
          }
        } else {
          invalid_flow_filter[i] = 0;
        }
      } else {
        invalid_flow_filter[i] = 0;
      }
    }

    /*  calc right wall with linear regresstion */
    steps = (real32_T)invalid_flow_filter[0];
    for (i = 0; i < 9; i++) {
      steps += (real32_T)invalid_flow_filter[i + 1];
    }

    if (steps > thresholds[2]) {
      for (i2 = 0; i2 < 10; i2++) {
        b_vectors[i2] = vectors[i2] * (real32_T)invalid_flow_filter[i2];
        b_vectors[10 + i2] = (real32_T)invalid_flow_filter[i2];
        c_vectors[i2] = vectors[10 + i2] * (real32_T)invalid_flow_filter[i2];
      }

      mldivide(b_vectors, c_vectors, wall);
      if (wall[1] > 0.0F) {
        /* && wall(2) < 10 % right wall can only be in positiv y range... else invalid data (max 10m) */
        for (i2 = 0; i2 < 2; i2++) {
          wall_est_right_data[i2] = wall[i2];
        }
      }
    }

    /*  calc RADAR ------------------------------------------------------ */
    /*  left */
    if (wall_est_left_data[1] < 0.0F) {
      /*  only calc beta if there is a wall estimated */
      if (wall_est_left_data[0] > 0.0F) {
        beta_left = (real32_T)atan(1.0F / wall_est_left_data[0]);
      } else if (wall_est_left_data[0] < 0.0F) {
        beta_left = 3.14159274F - (real32_T)atan(1.0F / (real32_T)fabs
          (wall_est_left_data[0]));
      } else {
        beta_left = 1.57079637F;
      }

      /*  calc distances left */
      radar_distance_update_left[8] = (real32_T)fabs(wall_est_left_data[1]);

      /*  c */
      i = 0;
      exitg4 = FALSE;
      while ((exitg4 == FALSE) && (i < 15)) {
        gamma_front = (3.14159274F - fv1[i]) - beta_left;
        i_top = (3.14159274F - fv1[i]) - (3.14159274F - beta_left);

        /*  TODO change */
        if (gamma_front > 0.0F) {
          radar_distance_update_left[9 + i] = (real32_T)fabs(wall_est_left_data
            [1]) / (real32_T)sin(gamma_front) * (real32_T)sin(beta_left);
        }

        if (i_top > 0.0F) {
          radar_distance_update_left[7 - (i + ((int32_T)floor((8.0 - (1.0 +
            (real_T)i)) / 32.0) << 5))] = (real32_T)fabs(wall_est_left_data[1]) /
            (real32_T)sin(i_top) * (real32_T)sin(beta_left);
        }

        if ((i_top <= 0.0F) && (gamma_front <= 0.0F)) {
          exitg4 = TRUE;
        } else {
          i++;
        }
      }
    }

    /*  right */
    if (wall_est_right_data[1] > 0.0F) {
      if (wall_est_right_data[0] > 0.0F) {
        steps = 3.14159274F - (real32_T)atan(1.0F / wall_est_right_data[0]);
      } else if (wall_est_right_data[0] < 0.0F) {
        steps = (real32_T)atan(1.0F / (real32_T)fabs(wall_est_right_data[0]));
      } else {
        steps = 1.57079637F;
      }

      /*  calc distances right */
      radar_distance_update_right[24] = wall_est_right_data[1];

      /*  c */
      i = 0;
      exitg3 = FALSE;
      while ((exitg3 == FALSE) && (i < 15)) {
        gamma_front = (3.14159274F - fv1[i]) - steps;
        i_top = (3.14159274F - fv1[i]) - (3.14159274F - steps);
        if (gamma_front > 0.0F) {
          radar_distance_update_right[23 - i] = wall_est_right_data[1] /
            (real32_T)sin(gamma_front) * (real32_T)sin(steps);
        }

        if (i_top > 0.0F) {
          radar_distance_update_right[(i - ((int32_T)floor((24.0 + (1.0 +
            (real_T)i)) / 32.0) << 5)) + 25] = wall_est_right_data[1] /
            (real32_T)sin(i_top) * (real32_T)sin(steps);
        }

        if ((i_top <= 0.0F) && (gamma_front <= 0.0F)) {
          exitg3 = TRUE;
        } else {
          i++;
        }
      }
    }
  }

  /*  FILTERING ----------------------------------------------------------- */
  /*  under development */
  for (i2 = 0; i2 < 32; i2++) {
    x_transition_add[i2] = (real_T)(radar_distance_update_left[i2] > 0.1F) *
      (real_T)(radar_distance_update_left[i2] < 5.0F);
    y_transition_add[i2] = (real_T)(radar_distance_update_right[i2] > 0.1F) *
      (real_T)(radar_distance_update_right[i2] < 5.0F);
  }

  y = FALSE;
  i = 0;
  exitg2 = FALSE;
  while ((exitg2 == FALSE) && (i < 32)) {
    if (!(x_transition_add[i] == 0.0)) {
      y = TRUE;
      exitg2 = TRUE;
    } else {
      i++;
    }
  }

  guard1 = FALSE;
  if (y) {
    guard1 = TRUE;
  } else {
    y = FALSE;
    i = 0;
    exitg1 = FALSE;
    while ((exitg1 == FALSE) && (i < 32)) {
      if (!(y_transition_add[i] == 0.0)) {
        y = TRUE;
        exitg1 = TRUE;
      } else {
        i++;
      }
    }

    if (y) {
      guard1 = TRUE;
    } else {
      /*  without update */
    }
  }

  if (guard1 == TRUE) {
    /*  with updates */
    for (i2 = 0; i2 < 32; i2++) {
      steps = (real32_T)x_transition_add[i2] * radar_distance_update_left[i2];
      b_radar_distance_update_left[i2] = (steps == 0.0F);
      radar_distance_update_left[i2] = steps;
      radar_distance_update_right[i2] *= (real32_T)y_transition_add[i2];
    }

    eml_li_find(b_radar_distance_update_left, tmp_data, tmp_size);
    i = tmp_size[0] * tmp_size[1];
    for (i2 = 0; i2 < i; i2++) {
      radar_distance_update_left[tmp_data[i2] - 1] = 5.0F;
    }

    for (i2 = 0; i2 < 32; i2++) {
      b_radar_distance_update_left[i2] = (radar_distance_update_right[i2] ==
        0.0F);
    }

    eml_li_find(b_radar_distance_update_left, tmp_data, tmp_size);
    i = tmp_size[0] * tmp_size[1];
    for (i2 = 0; i2 < i; i2++) {
      radar_distance_update_right[tmp_data[i2] - 1] = 5.0F;
    }

    for (i = 0; i < 32; i++) {
      steps = radar_distance_update_left[i];
      beta_left = radar_distance_update_right[i];
      if ((steps <= beta_left) || rtIsNaNF(beta_left)) {
      } else {
        steps = beta_left;
      }

      radar_filtered[i] = 5.0F - (0.7F * (5.0F - radar_filtered[i]) + 0.3F *
        (5.0F - steps));
      radar[i] = steps;
    }
  }

  /*      radar_filtered = maximal_distance_threshold - ... */
  /*              ((1 - lp_alpha_threesixty) * (maximal_distance_threshold - single(radar_filtered)) + ... */
  /*              lp_alpha_threesixty * (maximal_distance_threshold - radar)); */
  memcpy(&radar_weights[0], &radar_weights_k[0], sizeof(real32_T) << 5);
}

/* End of code generation (wallEstimationFilter.c) */
