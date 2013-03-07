/*
 * wallEstimationFilter.c
 *
 * Code generation for function 'wallEstimationFilter'
 *
 * C source code generated on: Thu Mar  7 14:09:14 2013
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
  const real32_T thresholds[3], real32_T radar[32], real32_T radar_filtered[32],
  real32_T radar_weights[32])
{
  int32_T i2;
  real32_T wall_est_left_data[4];
  real32_T wall_est_right_data[4];
  real_T radar_distance_update_left[32];
  real_T radar_distance_update_right[32];
  int8_T invalid_flow_filter[10];
  int32_T i;
  real_T vectors[20];
  real32_T beta_left;
  static const real_T dv0[20] = { -0.86602540378443871, -0.7880107536067219,
    -0.69465837045899725, -0.58778525229247314, -0.46947156278589081, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.46947156278589081,
    0.58778525229247314, 0.69465837045899725, 0.7880107536067219,
    0.86602540378443871 };

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
  boolean_T exitg4;
  static const real32_T fv0[15] = { 0.196349546F, 0.392699093F, 0.589048624F,
    0.785398185F, 0.981747746F, 1.17809725F, 1.37444675F, 1.57079637F,
    1.76714587F, 1.96349537F, 2.15984488F, 2.3561945F, 2.55254412F, 2.7488935F,
    2.94524312F };

  real32_T gamma_front;
  real32_T gamma_rear;
  boolean_T exitg3;
  real_T distance_filtering_left[32];
  real_T distance_filtering_right[32];
  boolean_T b_y;
  boolean_T exitg2;
  boolean_T guard1 = FALSE;
  boolean_T exitg1;
  boolean_T b_distance_filtering_left[32];
  int32_T tmp_size[2];
  int32_T tmp_data[32];
  real_T u1;

  /* WALL_ESTIMATION Summary of this function goes here */
  /*    Detailed explanation goes here */
  /* 0.05; */
  /* 0.05; */
  /* 2; */
  /*  ... */
  /*  default values */
  for (i2 = 0; i2 < 32; i2++) {
    radar[i2] = 5.0F;
  }

  for (i2 = 0; i2 < 2; i2++) {
    wall_est_left_data[i2] = 0.0F;
    wall_est_right_data[i2] = 0.0F;
  }

  for (i2 = 0; i2 < 32; i2++) {
    radar_distance_update_left[i2] = 0.0;
    radar_distance_update_right[i2] = 0.0;
  }

  if ((real32_T)fabs(speed[0]) > thresholds[0]) {
    /*  LEFT ------------------------------------------------------------ */
    for (i = 0; i < 10; i++) {
      invalid_flow_filter[i] = 1;
    }

    memset(&vectors[0], 0, 20U * sizeof(real_T));
    for (i = 0; i < 10; i++) {
      beta_left = flow_left[i] / 1000.0F;
      if ((real32_T)fabs(beta_left) > thresholds[1]) {
        beta_left = speed[0] / beta_left * (real32_T)dv0[i];
        if (beta_left > 0.0F) {
          for (i2 = 0; i2 < 2; i2++) {
            vectors[i + 10 * i2] = (real32_T)unit_vectors[i + 20 * i2] *
              beta_left;
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
        for (i2 = 0; i2 < 2; i2++) {
          wall_est_left_data[i2] = (real32_T)wall[i2];
        }
      }
    }

    /*  RIGHT ----------------------------------------------------------- */
    for (i = 0; i < 10; i++) {
      invalid_flow_filter[i] = 1;
    }

    memset(&vectors[0], 0, 20U * sizeof(real_T));
    for (i = 0; i < 10; i++) {
      beta_left = flow_right[i] / 1000.0F;
      if ((real32_T)fabs(beta_left) > thresholds[1]) {
        beta_left = speed[0] / beta_left * (real32_T)dv0[10 + i];
        if (beta_left > 0.0F) {
          for (i2 = 0; i2 < 2; i2++) {
            vectors[i + 10 * i2] = (real32_T)unit_vectors[(i + 20 * i2) + 10] *
              beta_left;
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
        for (i2 = 0; i2 < 2; i2++) {
          wall_est_right_data[i2] = (real32_T)wall[i2];
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
        gamma_front = (3.14159274F - fv0[i]) - beta_left;
        gamma_rear = (3.14159274F - fv0[i]) - (3.14159274F - beta_left);

        /*  TODO change */
        if (gamma_front > 0.0F) {
          radar_distance_update_left[9 + i] = (real32_T)fabs(wall_est_left_data
            [1]) / (real32_T)sin(gamma_front) * (real32_T)sin(beta_left);
        }

        if (gamma_rear > 0.0F) {
          radar_distance_update_left[7 - (i + ((int32_T)floor((8.0 - (1.0 +
            (real_T)i)) / 32.0) << 5))] = (real32_T)fabs(wall_est_left_data[1]) /
            (real32_T)sin(gamma_rear) * (real32_T)sin(beta_left);
        }

        if ((gamma_rear <= 0.0F) && (gamma_front <= 0.0F)) {
          exitg4 = TRUE;
        } else {
          i++;
        }
      }
    }

    /*  right */
    if (wall_est_right_data[1] > 0.0F) {
      if (wall_est_right_data[0] > 0.0F) {
        beta_left = 3.14159274F - (real32_T)atan(1.0F / wall_est_right_data[0]);
      } else if (wall_est_right_data[0] < 0.0F) {
        beta_left = (real32_T)atan(1.0F / (real32_T)fabs(wall_est_right_data[0]));
      } else {
        beta_left = 1.57079637F;
      }

      /*  calc distances right */
      radar_distance_update_right[24] = wall_est_right_data[1];

      /*  c */
      i = 0;
      exitg3 = FALSE;
      while ((exitg3 == FALSE) && (i < 15)) {
        gamma_front = (3.14159274F - fv0[i]) - beta_left;
        gamma_rear = (3.14159274F - fv0[i]) - (3.14159274F - beta_left);
        if (gamma_front > 0.0F) {
          radar_distance_update_right[23 - i] = wall_est_right_data[1] /
            (real32_T)sin(gamma_front) * (real32_T)sin(beta_left);
        }

        if (gamma_rear > 0.0F) {
          radar_distance_update_right[(i - ((int32_T)floor((24.0 + (1.0 +
            (real_T)i)) / 32.0) << 5)) + 25] = wall_est_right_data[1] /
            (real32_T)sin(gamma_rear) * (real32_T)sin(beta_left);
        }

        if ((gamma_rear <= 0.0F) && (gamma_front <= 0.0F)) {
          exitg3 = TRUE;
        } else {
          i++;
        }
      }
    }
  }

  /*  FILTERING ----------------------------------------------------------- */
  for (i2 = 0; i2 < 32; i2++) {
    distance_filtering_left[i2] = (real_T)(radar_distance_update_left[i2] >
      0.10000000149011612) * (real_T)(radar_distance_update_left[i2] < 5.0);
    distance_filtering_right[i2] = (real_T)(radar_distance_update_right[i2] >
      0.10000000149011612) * (real_T)(radar_distance_update_right[i2] < 5.0);
  }

  b_y = FALSE;
  i = 0;
  exitg2 = FALSE;
  while ((exitg2 == FALSE) && (i < 32)) {
    if (!(distance_filtering_left[i] == 0.0)) {
      b_y = TRUE;
      exitg2 = TRUE;
    } else {
      i++;
    }
  }

  guard1 = FALSE;
  if (b_y) {
    guard1 = TRUE;
  } else {
    b_y = FALSE;
    i = 0;
    exitg1 = FALSE;
    while ((exitg1 == FALSE) && (i < 32)) {
      if (!(distance_filtering_right[i] == 0.0)) {
        b_y = TRUE;
        exitg1 = TRUE;
      } else {
        i++;
      }
    }

    if (b_y) {
      guard1 = TRUE;
    } else {
      /*  without update */
    }
  }

  if (guard1 == TRUE) {
    /*  with updates */
    for (i2 = 0; i2 < 32; i2++) {
      y = distance_filtering_left[i2] * radar_distance_update_left[i2];
      b_distance_filtering_left[i2] = (y == 0.0);
      distance_filtering_left[i2] = y;
      distance_filtering_right[i2] *= radar_distance_update_right[i2];
    }

    eml_li_find(b_distance_filtering_left, tmp_data, tmp_size);
    i = tmp_size[0] * tmp_size[1];
    for (i2 = 0; i2 < i; i2++) {
      distance_filtering_left[tmp_data[i2] - 1] = 5.0;
    }

    for (i2 = 0; i2 < 32; i2++) {
      b_distance_filtering_left[i2] = (distance_filtering_right[i2] == 0.0);
    }

    eml_li_find(b_distance_filtering_left, tmp_data, tmp_size);
    i = tmp_size[0] * tmp_size[1];
    for (i2 = 0; i2 < i; i2++) {
      distance_filtering_right[tmp_data[i2] - 1] = 5.0;
    }

    for (i = 0; i < 32; i++) {
      y = distance_filtering_left[i];
      u1 = distance_filtering_right[i];
      if ((y <= u1) || rtIsNaN(u1)) {
      } else {
        y = u1;
      }

      radar[i] = (real32_T)y;
    }
  }

  for (i2 = 0; i2 < 32; i2++) {
    radar_filtered[i2] = 5.0F - (0.7F * (5.0F - radar_filtered_k[i2]) + 0.3F *
      (5.0F - radar[i2]));
    radar_weights[i2] = radar_weights_k[i2];
  }
}

/* End of code generation (wallEstimationFilter.c) */
