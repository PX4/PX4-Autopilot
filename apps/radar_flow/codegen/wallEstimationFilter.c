/*
 * wallEstimationFilter.c
 *
 * Code generation for function 'wallEstimationFilter'
 *
 * C source code generated on: Tue Apr  9 09:52:43 2013
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "frontFlowKalmanFilter.h"
#include "wallEstimationFilter.h"
#include "any.h"
#include "sum.h"
#include "power.h"
#include "mean.h"
#include "mldivide.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
static void b_eml_li_find(const boolean_T x[32], int32_T y_data[32], int32_T
  y_size[2]);
static void eml_li_find(const boolean_T x[10], int32_T y_data[10], int32_T
  y_size[1]);

/* Function Definitions */
static void b_eml_li_find(const boolean_T x[32], int32_T y_data[32], int32_T
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

static void eml_li_find(const boolean_T x[10], int32_T y_data[10], int32_T
  y_size[1])
{
  int32_T k;
  int32_T i;
  k = 0;
  for (i = 0; i < 10; i++) {
    if (x[i]) {
      k++;
    }
  }

  y_size[0] = k;
  k = 0;
  for (i = 0; i < 10; i++) {
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
  settings[8], real32_T radar[32], real32_T radar_filtered[32], real32_T
  radar_weights[32])
{
  real32_T x_transition_add[32];
  int32_T i;
  real32_T y_transition_add[32];
  int32_T i2;
  real32_T wall_est_left_data[4];
  real32_T wall_est_right_data[4];
  real32_T det_coef_left;
  real32_T det_coef_right;
  real32_T radar_distance_update_left[32];
  real32_T radar_distance_update_right[32];
  real32_T radar_distance_update_front[32];
  real32_T steps;
  real32_T i_top;
  real32_T i_down;
  real32_T ss_tot;
  real32_T gamma_rear;
  real32_T invalid_flow_filter[10];
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

  real32_T y[10];
  real32_T A[20];
  real32_T wall[2];
  boolean_T b_y[10];
  boolean_T c_y[10];
  int32_T tmp_size[1];
  int32_T tmp_data[10];
  int32_T b_tmp_size[1];
  int32_T b_tmp_data[10];
  int32_T y_size[1];
  int32_T b_y_size[1];
  real32_T c_tmp_data[10];
  real32_T fv1[10];
  int32_T c_y_size[1];
  int32_T d_y_size[1];
  real32_T fv2[10];
  boolean_T exitg2;
  static const real32_T fv3[15] = { 0.196349546F, 0.392699093F, 0.589048624F,
    0.785398185F, 0.981747746F, 1.17809725F, 1.37444675F, 1.57079637F,
    1.76714587F, 1.96349537F, 2.15984488F, 2.3561945F, 2.55254412F, 2.7488935F,
    2.94524312F };

  boolean_T exitg1;
  real32_T distance_filtering_front[32];
  boolean_T b_radar_distance_update_left[32];
  int32_T c_tmp_size[2];
  int32_T d_tmp_data[32];
  real32_T lp_alpha[32];

  /* WALL_ESTIMATION Summary of this function goes here */
  /*    Detailed explanation goes here */
  /*  const settings */
  /*  only fix resolution is efficient */
  /*  calc from settings */
  /*  flow/speed thresholds */
  /* 0.05; */
  /* 0.05; */
  /* 2; */
  /*  radar/weights lp settings */
  /*  0.01 */
  /*  0.3 */
  /*  0.05 */
  /*  0.01 */
  /*  0.002 */
  /*      % radar/weights lp settings */
  /*      lp_alpha_default = single(0.05); */
  /*      lp_alpha_flow_max = single(0.3); */
  /*      lp_alpha_flow_min = lp_alpha_default; */
  /*      lp_alpha_front = lp_alpha_flow_max; */
  /*      lp_alpha_scale_min = single(0.3); */
  /*      lp_weight_down = single(0.005); */
  /*      lp_no_update_max = single(1.0); */
  /*      lp_no_update_min = single(0.9); */
  for (i = 0; i < 32; i++) {
    x_transition_add[i] = (real32_T)cos(((1.0F + (real32_T)i) - 1.0F) *
      0.196349546F);
  }

  for (i = 0; i < 32; i++) {
    y_transition_add[i] = x_transition_add[(i - ((int32_T)(real32_T)floor
      ((((1.0F + (real32_T)i) - 8.0F) - 1.0F) / 32.0F) << 5)) - 8];

    /*  default values */
    radar[i] = 5.0F;
  }

  for (i2 = 0; i2 < 2; i2++) {
    wall_est_left_data[i2] = 0.0F;
    wall_est_right_data[i2] = 0.0F;
  }

  det_coef_left = 0.0F;
  det_coef_right = 0.0F;
  for (i2 = 0; i2 < 32; i2++) {
    radar_distance_update_left[i2] = 0.0F;
    radar_distance_update_right[i2] = 0.0F;
    radar_distance_update_front[i2] = 0.0F;
  }

  /*  wall estimation filter */
  /* wall_model =  */
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
      ss_tot = ((1.0F + (real32_T)i) + i_top) - 1.0F;
      gamma_rear = ss_tot - (real32_T)floor(ss_tot / 32.0F) * 32.0F;
      ss_tot = ((1.0F + (real32_T)i) + i_down) - 1.0F;
      ss_tot -= (real32_T)floor(ss_tot / 32.0F) * 32.0F;
      radar_filtered[i] = radar_filtered_k[(int32_T)(gamma_rear + 1.0F) - 1] *
        (1.0F - steps) + radar_filtered_k[(int32_T)(ss_tot + 1.0F) - 1] * steps;
      radar_weights[i] = radar_weights_k[(int32_T)(gamma_rear + 1.0F) - 1] *
        (1.0F - steps) + radar_weights_k[(int32_T)(ss_tot + 1.0F) - 1] * steps;
    }
  } else {
    for (i2 = 0; i2 < 32; i2++) {
      radar_filtered[i2] = radar_filtered_k[i2];
      radar_weights[i2] = radar_weights_k[i2];
    }
  }

  /*  transitions */
  if (position_update[0] != 0.0F) {
    for (i2 = 0; i2 < 32; i2++) {
      radar_filtered[i2] += position_update[0] * x_transition_add[i2];
    }

    /*  TODO these updates are totaly wrong!!! */
    /*          radar_weights = radar_weights + (position_update(1) * x_transition_add); */
  }

  if (position_update[1] != 0.0F) {
    for (i2 = 0; i2 < 32; i2++) {
      radar_filtered[i2] += position_update[1] * y_transition_add[i2];
    }

    /*  TODO these updates are totaly wrong!!! */
    /*          radar_weights = radar_weights + (position_update(2) * y_transition_add); */
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

    /*  TODO these updates are totaly wrong!!! */
    /*          if radar_weights(i) > maximal_weight_threshold */
    /*              radar_weights(i) = maximal_weight_threshold; */
    /*          elseif radar_weights(i) < minimal_weight_threshold */
    /*              radar_weights(i) = minimal_weight_threshold; */
    /*          end */
    radar_filtered[i] = steps;
  }

  /*  --------------------------------------------------------------------- */
  /*  WALL UPDATE CALCULATION (flow) */
  /*  --------------------------------------------------------------------- */
  if ((real32_T)fabs(speed[0]) > settings[0]) {
    /*  LEFT ------------------------------------------------------------ */
    for (i = 0; i < 10; i++) {
      invalid_flow_filter[i] = 1.0F;
    }

    memset(&vectors[0], 0, 20U * sizeof(real32_T));
    for (i = 0; i < 10; i++) {
      steps = flow_left[i] / 1000.0F;
      if ((real32_T)fabs(steps) > settings[1]) {
        steps = speed[0] / steps * fv0[i];
        if (steps > 0.0F) {
          for (i2 = 0; i2 < 2; i2++) {
            vectors[i + 10 * i2] = unit_vectors[i + 20 * i2] * steps;
          }
        } else {
          invalid_flow_filter[i] = 0.0F;
        }
      } else {
        invalid_flow_filter[i] = 0.0F;
      }
    }

    /*  calc left wall with linear regresstion */
    if (sum(invalid_flow_filter) > settings[2]) {
      for (i2 = 0; i2 < 10; i2++) {
        y[i2] = vectors[10 + i2] * invalid_flow_filter[i2];
        A[i2] = vectors[i2] * invalid_flow_filter[i2];
        A[10 + i2] = invalid_flow_filter[i2];
      }

      mldivide(A, y, wall);
      if (wall[1] < 0.0F) {
        /* && wall(2) > -10 % left wall can only be in negative y range... else invalid data (max 10m) */
        for (i = 0; i < 10; i++) {
          b_y[i] = (y[i] != 0.0F);
          c_y[i] = (y[i] != 0.0F);
        }

        eml_li_find(b_y, tmp_data, tmp_size);
        eml_li_find(c_y, b_tmp_data, b_tmp_size);
        y_size[0] = tmp_size[0];
        i = tmp_size[0];
        for (i2 = 0; i2 < i; i2++) {
          invalid_flow_filter[i2] = y[tmp_data[i2] - 1];
        }

        steps = mean(invalid_flow_filter, y_size);
        b_y_size[0] = b_tmp_size[0];
        i = b_tmp_size[0];
        for (i2 = 0; i2 < i; i2++) {
          invalid_flow_filter[i2] = y[b_tmp_data[i2] - 1] - steps;
        }

        b_power(invalid_flow_filter, b_y_size, c_tmp_data, tmp_size);
        ss_tot = b_sum(c_tmp_data, tmp_size);

        /*  TODO verify that y>0 and not ~= 0... */
        for (i2 = 0; i2 < 10; i2++) {
          steps = 0.0F;
          for (i = 0; i < 2; i++) {
            steps += A[i2 + 10 * i] * wall[i];
          }

          invalid_flow_filter[i2] = y[i2] - steps;
        }

        power(invalid_flow_filter, fv1);
        det_coef_left = 1.0F - sum(fv1) / ss_tot;
        for (i2 = 0; i2 < 2; i2++) {
          wall_est_left_data[i2] = wall[i2];
        }
      }
    }

    /*  RIGHT ----------------------------------------------------------- */
    for (i = 0; i < 10; i++) {
      invalid_flow_filter[i] = 1.0F;
    }

    memset(&vectors[0], 0, 20U * sizeof(real32_T));
    for (i = 0; i < 10; i++) {
      steps = flow_right[i] / 1000.0F;
      if ((real32_T)fabs(steps) > settings[1]) {
        steps = speed[0] / steps * fv0[10 + i];
        if (steps > 0.0F) {
          for (i2 = 0; i2 < 2; i2++) {
            vectors[i + 10 * i2] = unit_vectors[(i + 20 * i2) + 10] * steps;
          }
        } else {
          invalid_flow_filter[i] = 0.0F;
        }
      } else {
        invalid_flow_filter[i] = 0.0F;
      }
    }

    /*  calc right wall with linear regresstion */
    if (sum(invalid_flow_filter) > settings[2]) {
      for (i2 = 0; i2 < 10; i2++) {
        y[i2] = vectors[10 + i2] * invalid_flow_filter[i2];
        A[i2] = vectors[i2] * invalid_flow_filter[i2];
        A[10 + i2] = invalid_flow_filter[i2];
      }

      mldivide(A, y, wall);
      if (wall[1] > 0.0F) {
        /* && wall(2) < 10 % right wall can only be in positiv y range... else invalid data (max 10m) */
        for (i = 0; i < 10; i++) {
          b_y[i] = (y[i] != 0.0F);
          c_y[i] = (y[i] != 0.0F);
        }

        eml_li_find(b_y, tmp_data, tmp_size);
        eml_li_find(c_y, b_tmp_data, b_tmp_size);
        c_y_size[0] = tmp_size[0];
        i = tmp_size[0];
        for (i2 = 0; i2 < i; i2++) {
          invalid_flow_filter[i2] = y[tmp_data[i2] - 1];
        }

        steps = mean(invalid_flow_filter, c_y_size);
        d_y_size[0] = b_tmp_size[0];
        i = b_tmp_size[0];
        for (i2 = 0; i2 < i; i2++) {
          invalid_flow_filter[i2] = y[b_tmp_data[i2] - 1] - steps;
        }

        b_power(invalid_flow_filter, d_y_size, c_tmp_data, tmp_size);
        ss_tot = b_sum(c_tmp_data, tmp_size);
        for (i2 = 0; i2 < 10; i2++) {
          steps = 0.0F;
          for (i = 0; i < 2; i++) {
            steps += A[i2 + 10 * i] * wall[i];
          }

          invalid_flow_filter[i2] = y[i2] - steps;
        }

        power(invalid_flow_filter, fv2);
        det_coef_right = 1.0F - sum(fv2) / ss_tot;
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
        ss_tot = (real32_T)atan(1.0F / wall_est_left_data[0]);
      } else if (wall_est_left_data[0] < 0.0F) {
        ss_tot = 3.14159274F - (real32_T)atan(1.0F / (real32_T)fabs
          (wall_est_left_data[0]));
      } else {
        ss_tot = 1.57079637F;
      }

      /*  calc distances left */
      radar_distance_update_left[8] = (real32_T)fabs(wall_est_left_data[1]);

      /*  c */
      i = 0;
      exitg2 = FALSE;
      while ((exitg2 == FALSE) && (i < 15)) {
        steps = (3.14159274F - fv3[i]) - ss_tot;
        gamma_rear = (3.14159274F - fv3[i]) - (3.14159274F - ss_tot);

        /*  TODO change */
        if (steps > 0.0F) {
          radar_distance_update_left[9 + i] = (real32_T)fabs(wall_est_left_data
            [1]) / (real32_T)sin(steps) * (real32_T)sin(ss_tot);
        }

        if (gamma_rear > 0.0F) {
          radar_distance_update_left[7 - (i + ((int32_T)(real32_T)floor((8.0F -
            (1.0F + (real32_T)i)) / 32.0F) << 5))] = (real32_T)fabs
            (wall_est_left_data[1]) / (real32_T)sin(gamma_rear) * (real32_T)sin
            (ss_tot);
        }

        if ((gamma_rear <= 0.0F) && (steps <= 0.0F)) {
          exitg2 = TRUE;
        } else {
          i++;
        }
      }
    }

    /*  right */
    if (wall_est_right_data[1] > 0.0F) {
      if (wall_est_right_data[0] > 0.0F) {
        ss_tot = 3.14159274F - (real32_T)atan(1.0F / wall_est_right_data[0]);
      } else if (wall_est_right_data[0] < 0.0F) {
        ss_tot = (real32_T)atan(1.0F / (real32_T)fabs(wall_est_right_data[0]));
      } else {
        ss_tot = 1.57079637F;
      }

      /*  calc distances right */
      radar_distance_update_right[24] = wall_est_right_data[1];

      /*  c */
      i = 0;
      exitg1 = FALSE;
      while ((exitg1 == FALSE) && (i < 15)) {
        steps = (3.14159274F - fv3[i]) - ss_tot;
        gamma_rear = (3.14159274F - fv3[i]) - (3.14159274F - ss_tot);
        if (steps > 0.0F) {
          radar_distance_update_right[23 - i] = wall_est_right_data[1] /
            (real32_T)sin(steps) * (real32_T)sin(ss_tot);
        }

        if (gamma_rear > 0.0F) {
          radar_distance_update_right[(i - ((int32_T)(real32_T)floor((24.0F +
            (1.0F + (real32_T)i)) / 32.0F) << 5)) + 25] = wall_est_right_data[1]
            / (real32_T)sin(gamma_rear) * (real32_T)sin(ss_tot);
        }

        if ((gamma_rear <= 0.0F) && (steps <= 0.0F)) {
          exitg1 = TRUE;
        } else {
          i++;
        }
      }
    }
  }

  /*  --------------------------------------------------------------------- */
  /*  FRONT DISTANCE */
  /*  --------------------------------------------------------------------- */
  if (front_distance < 5.0F) {
    for (i2 = 0; i2 < 5; i2++) {
      radar_distance_update_front[14 + i2] = front_distance;
    }
  }

  /*  FILTERING ----------------------------------------------------------- */
  /*  under development: do not update if only sonar has a new distance */
  /*  TODO */
  for (i2 = 0; i2 < 32; i2++) {
    x_transition_add[i2] = (real32_T)(radar_distance_update_left[i2] > 0.1F) *
      (real32_T)(radar_distance_update_left[i2] < 5.0F);
    y_transition_add[i2] = (real32_T)(radar_distance_update_right[i2] > 0.1F) *
      (real32_T)(radar_distance_update_right[i2] < 5.0F);
    distance_filtering_front[i2] = (real32_T)(radar_distance_update_front[i2] >
      0.1F) * (real32_T)(radar_distance_update_front[i2] < 5.0F);
  }

  if (any(x_transition_add) || any(y_transition_add) || any
      (distance_filtering_front)) {
    /*  with updates */
    for (i2 = 0; i2 < 32; i2++) {
      steps = x_transition_add[i2] * radar_distance_update_left[i2];
      b_radar_distance_update_left[i2] = (steps == 0.0F);
      radar_distance_update_left[i2] = steps;
      radar_distance_update_front[i2] *= distance_filtering_front[i2];
      radar_distance_update_right[i2] *= y_transition_add[i2];
    }

    b_eml_li_find(b_radar_distance_update_left, d_tmp_data, c_tmp_size);
    i = c_tmp_size[0] * c_tmp_size[1];
    for (i2 = 0; i2 < i; i2++) {
      radar_distance_update_left[d_tmp_data[i2] - 1] = 5.0F;
    }

    for (i2 = 0; i2 < 32; i2++) {
      b_radar_distance_update_left[i2] = (radar_distance_update_right[i2] ==
        0.0F);
    }

    b_eml_li_find(b_radar_distance_update_left, d_tmp_data, c_tmp_size);
    i = c_tmp_size[0] * c_tmp_size[1];
    for (i2 = 0; i2 < i; i2++) {
      radar_distance_update_right[d_tmp_data[i2] - 1] = 5.0F;
    }

    for (i2 = 0; i2 < 32; i2++) {
      b_radar_distance_update_left[i2] = (radar_distance_update_front[i2] ==
        0.0F);
    }

    b_eml_li_find(b_radar_distance_update_left, d_tmp_data, c_tmp_size);
    i = c_tmp_size[0] * c_tmp_size[1];
    for (i2 = 0; i2 < i; i2++) {
      radar_distance_update_front[d_tmp_data[i2] - 1] = 5.0F;
    }

    /*  calc update lowpass alpha, radar values and new weights */
    memset(&lp_alpha[0], 0, sizeof(real32_T) << 5);
    for (i = 0; i < 32; i++) {
      gamma_rear = 0.0F;
      if ((1 + i < 8) || (1 + i > 24)) {
        /*  no real measurements possible */
        i_top = settings[3];

        /*  new values at the back have default update gain. */
        ss_tot = radar_distance_update_left[i];
        steps = radar_distance_update_right[i];
        if ((ss_tot <= steps) || rtIsNaNF(steps)) {
        } else {
          ss_tot = steps;
        }

        radar[i] = ss_tot;
      } else {
        /*  front distance available */
        if (distance_filtering_front[i] != 0.0F) {
          i_top = settings[4];
          radar[i] = radar_distance_update_front[i];
          gamma_rear = 1.0F;

          /*  right and left available */
        } else if ((x_transition_add[i] != 0.0F) && (y_transition_add[i] != 0.0F))
        {
          if (1 + i < 17) {
            /*  believe left */
            i_top = settings[5] + (settings[4] - settings[5]) * det_coef_left;
            radar[i] = radar_distance_update_left[i];
            gamma_rear = 0.2F + 0.8F * det_coef_left;
          } else {
            /*  believe right */
            i_top = settings[5] + (settings[4] - settings[5]) * det_coef_right;
            radar[i] = radar_distance_update_right[i];
            gamma_rear = 0.2F + 0.8F * det_coef_right;
          }

          /*  only left available */
        } else if (x_transition_add[i] != 0.0F) {
          i_top = settings[5] + (settings[4] - settings[5]) * det_coef_left;
          radar[i] = radar_distance_update_left[i];
          gamma_rear = 0.2F + 0.8F * det_coef_left;

          /*  only right available */
        } else if (y_transition_add[i] != 0.0F) {
          i_top = settings[5] + (settings[4] - settings[5]) * det_coef_right;
          radar[i] = radar_distance_update_right[i];
          gamma_rear = 0.2F + 0.8F * det_coef_right;

          /*  no distance update available */
        } else {
          i_top = settings[5];
          gamma_rear = 0.2F;

          /*  radar on max distance */
        }
      }

      /*  weight update */
      if (gamma_rear != 0.0F) {
        /*  new weight (low passed down, instant up) */
        /*  if no new measurements means weight and lp_alpha on  */
        /*  lowest level (settings) -> TODO is this correct */
        if (radar_weights[i] > gamma_rear) {
          /*  if old value has bigger weight -> lower new lp_alpha */
          i_top -= (i_top - settings[5]) * (radar_weights[i] - gamma_rear);
          gamma_rear = (1.0F - settings[6]) * radar_weights[i] + settings[6] *
            gamma_rear;
        } else {
          /*  if new value has bigger weight -> take lp_alpha */
        }
      } else {
        /*  no new value possible (slow fade out) */
        gamma_rear = (1.0F - settings[7]) * radar_weights[i] + settings[7] *
          0.0F;
      }

      radar_weights[i] = gamma_rear;
      lp_alpha[i] = i_top;
    }

    for (i2 = 0; i2 < 32; i2++) {
      radar_filtered[i2] = 5.0F - ((1.0F - lp_alpha[i2]) * (5.0F -
        radar_filtered[i2]) + lp_alpha[i2] * (5.0F - radar[i2]));
    }
  } else {
    /*  without update */
    /*  TODO this almost never happens because of sonar input */
    for (i2 = 0; i2 < 32; i2++) {
      steps = 5.0F - (0.9F + 0.100000024F * radar_weights[i2]) * (5.0F -
        radar_filtered[i2]);
      radar_weights[i2] *= 1.0F - settings[7];
      radar_filtered[i2] = steps;
    }
  }
}

/* End of code generation (wallEstimationFilter.c) */
