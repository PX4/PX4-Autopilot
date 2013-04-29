/*
 * radarControl.c
 *
 * Code generation for function 'radarControl'
 *
 * C source code generated on: Mon Apr 29 11:05:59 2013
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "radarControl.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
static void frontSituationUpdate(const int16_T front_situation_old[17], real32_T
  yaw_update_rad, real32_T x_update_mm, real32_T y_update_mm, int16_T
  front_situation_new[17]);
static real32_T rt_powf_snf(real32_T u0, real32_T u1);
static real32_T rt_roundf_snf(real32_T u);

/* Function Definitions */
static void frontSituationUpdate(const int16_T front_situation_old[17], real32_T
  yaw_update_rad, real32_T x_update_mm, real32_T y_update_mm, int16_T
  front_situation_new[17])
{
  real32_T x_transition_add[17];
  real32_T y_transition_add[17];
  int32_T i;
  real32_T steps;
  real32_T i_top;
  real32_T i_down;
  int32_T value_top;
  int32_T value_down;
  real32_T f0;
  int16_T i0;

  /*  mm */
  /*  mm */
  for (i = 0; i < 17; i++) {
    x_transition_add[i] = -(real32_T)sin(((1.0F + (real32_T)i) - 1.0F) *
      0.196349546F);
    y_transition_add[i] = (real32_T)cos(((1.0F + (real32_T)i) - 1.0F) *
      0.196349546F);
    front_situation_new[i] = 0;
  }

  /*  rotation */
  if (yaw_update_rad != 0.0F) {
    steps = yaw_update_rad / 0.196349546F;

    /*  will mostly be 0 and 1 or -1 and 0 */
    i_top = (real32_T)ceil(steps);
    i_down = (real32_T)floor(steps);
    steps = i_top - steps;
    for (i = 0; i < 17; i++) {
      if (((1.0F + (real32_T)i) + i_top < 1.0F) || ((1.0F + (real32_T)i) + i_top
           > 17.0F)) {
        value_top = 5000;
      } else {
        value_top = front_situation_old[(int32_T)((1.0F + (real32_T)i) + i_top)
          - 1];
      }

      if (((1.0F + (real32_T)i) + i_down < 1.0F) || ((1.0F + (real32_T)i) +
           i_down > 17.0F)) {
        value_down = 5000;
      } else {
        value_down = front_situation_old[(int32_T)((1.0F + (real32_T)i) + i_down)
          - 1];
      }

      f0 = rt_roundf_snf((real32_T)value_top * (1.0F - steps) + (real32_T)
                         value_down * steps);
      if (f0 < 32768.0F) {
        if (f0 >= -32768.0F) {
          i0 = (int16_T)f0;
        } else {
          i0 = MIN_int16_T;
        }
      } else if (f0 >= 32768.0F) {
        i0 = MAX_int16_T;
      } else {
        i0 = 0;
      }

      front_situation_new[i] = i0;
    }
  } else {
    for (i = 0; i < 17; i++) {
      front_situation_new[i] = front_situation_old[i];
    }
  }

  /*  transitions */
  if (x_update_mm != 0.0F) {
    for (i = 0; i < 17; i++) {
      f0 = rt_roundf_snf(x_update_mm * x_transition_add[i]);
      if (f0 < 32768.0F) {
        if (f0 >= -32768.0F) {
          i0 = (int16_T)f0;
        } else {
          i0 = MIN_int16_T;
        }
      } else if (f0 >= 32768.0F) {
        i0 = MAX_int16_T;
      } else {
        i0 = 0;
      }

      value_top = front_situation_new[i] + i0;
      if (value_top > 32767) {
        value_top = 32767;
      } else {
        if (value_top < -32768) {
          value_top = -32768;
        }
      }

      front_situation_new[i] = (int16_T)value_top;
    }
  }

  if (y_update_mm != 0.0F) {
    for (i = 0; i < 17; i++) {
      f0 = rt_roundf_snf(y_update_mm * y_transition_add[i]);
      if (f0 < 32768.0F) {
        if (f0 >= -32768.0F) {
          i0 = (int16_T)f0;
        } else {
          i0 = MIN_int16_T;
        }
      } else if (f0 >= 32768.0F) {
        i0 = MAX_int16_T;
      } else {
        i0 = 0;
      }

      value_top = front_situation_new[i] + i0;
      if (value_top > 32767) {
        value_top = 32767;
      } else {
        if (value_top < -32768) {
          value_top = -32768;
        }
      }

      front_situation_new[i] = (int16_T)value_top;
    }
  }
}

static real32_T rt_powf_snf(real32_T u0, real32_T u1)
{
  real32_T y;
  real32_T f1;
  real32_T f2;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = ((real32_T)rtNaN);
  } else {
    f1 = (real32_T)fabs(u0);
    f2 = (real32_T)fabs(u1);
    if (rtIsInfF(u1)) {
      if (f1 == 1.0F) {
        y = ((real32_T)rtNaN);
      } else if (f1 > 1.0F) {
        if (u1 > 0.0F) {
          y = ((real32_T)rtInf);
        } else {
          y = 0.0F;
        }
      } else if (u1 > 0.0F) {
        y = 0.0F;
      } else {
        y = ((real32_T)rtInf);
      }
    } else if (f2 == 0.0F) {
      y = 1.0F;
    } else if (f2 == 1.0F) {
      if (u1 > 0.0F) {
        y = u0;
      } else {
        y = 1.0F / u0;
      }
    } else if (u1 == 2.0F) {
      y = u0 * u0;
    } else if ((u1 == 0.5F) && (u0 >= 0.0F)) {
      y = (real32_T)sqrt(u0);
    } else if ((u0 < 0.0F) && (u1 > (real32_T)floor(u1))) {
      y = ((real32_T)rtNaN);
    } else {
      y = (real32_T)pow(u0, u1);
    }
  }

  return y;
}

static real32_T rt_roundf_snf(real32_T u)
{
  real32_T y;
  if ((real32_T)fabs(u) < 8.388608E+6F) {
    if (u >= 0.5F) {
      y = (real32_T)floor(u + 0.5F);
    } else if (u > -0.5F) {
      y = u * 0.0F;
    } else {
      y = (real32_T)ceil(u - 0.5F);
    }
  } else {
    y = u;
  }

  return y;
}

boolean_T radarControl(int16_T radar[32], int16_T front_distance, int16_T
  front_situation[17], const real32_T settings[9], real32_T *yaw_control,
  real32_T *x_control, real32_T *y_control)
{
  boolean_T free_environment;
  real32_T free_sectors_left;
  int16_T side_threshold;
  int16_T front_side_threshold;
  int16_T min_front_side_dist;
  int16_T dist_edge_right;
  int32_T react_direction;
  int16_T side_range;
  int16_T front_side_range;
  int16_T dist_edge_left;
  int16_T front_range;
  real_T b_free_sectors_left;
  real_T free_sectors_right;
  real32_T b_free_sectors_right;
  boolean_T exitg2;
  boolean_T exitg1;
  real32_T free_sectors_diff;
  int32_T yaw_direction;
  static const real32_T fv0[32] = { 3.14159274F, 2.94524312F, 2.74889374F,
    2.55254412F, 2.3561945F, 2.15984488F, 1.96349549F, 1.76714587F, 1.57079637F,
    1.37444687F, 1.17809725F, 0.981747746F, 0.785398185F, 0.589048624F,
    0.392699093F, 0.196349546F, 0.0F, 0.196349546F, 0.392699093F, 0.589048624F,
    0.785398185F, 0.981747746F, 1.17809725F, 1.37444675F, 1.57079637F,
    1.76714587F, 1.96349537F, 2.15984488F, 2.3561945F, 2.55254412F, 2.7488935F,
    2.94524312F };

  real32_T b_free_sectors_diff;
  real32_T c_free_sectors_left;
  int16_T iv0[17];

  /* RADARCONTROLL calculates the controlling output based on radar input */
  /*  there are five parts, left, front_left, front, front_right, right */
  /*  for yaw controlling only front_left, front and front_right are concerned */
  /*  for y controlling only left <-> right are compared... and in extreme */
  /*  situation also front part can influent y controlling... */
  /*  radar variables */
  /*  mm */
  /*  mm */
  /*  front situation variables */
  /*  !!! index + 1 because of matlab */
  /*  maximum controls */
  /*  thresholds for begin with reaction */
  free_sectors_left = rt_roundf_snf(settings[3]);
  if (free_sectors_left < 32768.0F) {
    if (free_sectors_left >= -32768.0F) {
      side_threshold = (int16_T)free_sectors_left;
    } else {
      side_threshold = MIN_int16_T;
    }
  } else if (free_sectors_left >= 32768.0F) {
    side_threshold = MAX_int16_T;
  } else {
    side_threshold = 0;
  }

  free_sectors_left = rt_roundf_snf(settings[4]);
  if (free_sectors_left < 32768.0F) {
    if (free_sectors_left >= -32768.0F) {
      front_side_threshold = (int16_T)free_sectors_left;
    } else {
      front_side_threshold = MIN_int16_T;
    }
  } else if (free_sectors_left >= 32768.0F) {
    front_side_threshold = MAX_int16_T;
  } else {
    front_side_threshold = 0;
  }

  /*  minimum distance for full reaction */
  /*  TODO settings... */
  /* 1000; */
  free_sectors_left = rt_roundf_snf(settings[7]);
  if (free_sectors_left < 32768.0F) {
    if (free_sectors_left >= -32768.0F) {
      min_front_side_dist = (int16_T)free_sectors_left;
    } else {
      min_front_side_dist = MIN_int16_T;
    }
  } else if (free_sectors_left >= 32768.0F) {
    min_front_side_dist = MAX_int16_T;
  } else {
    min_front_side_dist = 0;
  }

  /* 1500; */
  /* 1000; */
  /*  ranges */
  free_sectors_left = rt_roundf_snf(settings[8]);
  if (free_sectors_left < 32768.0F) {
    if (free_sectors_left >= -32768.0F) {
      dist_edge_right = (int16_T)free_sectors_left;
    } else {
      dist_edge_right = MIN_int16_T;
    }
  } else if (free_sectors_left >= 32768.0F) {
    dist_edge_right = MAX_int16_T;
  } else {
    dist_edge_right = 0;
  }

  react_direction = side_threshold - dist_edge_right;
  if (react_direction > 32767) {
    react_direction = 32767;
  } else {
    if (react_direction < -32768) {
      react_direction = -32768;
    }
  }

  side_range = (int16_T)react_direction;
  react_direction = front_side_threshold - min_front_side_dist;
  if (react_direction > 32767) {
    react_direction = 32767;
  } else {
    if (react_direction < -32768) {
      react_direction = -32768;
    }
  }

  front_side_range = (int16_T)react_direction;
  free_sectors_left = rt_roundf_snf(settings[5]);
  if (free_sectors_left < 32768.0F) {
    if (free_sectors_left >= -32768.0F) {
      dist_edge_right = (int16_T)free_sectors_left;
    } else {
      dist_edge_right = MIN_int16_T;
    }
  } else if (free_sectors_left >= 32768.0F) {
    dist_edge_right = MAX_int16_T;
  } else {
    dist_edge_right = 0;
  }

  free_sectors_left = rt_roundf_snf(settings[6]);
  if (free_sectors_left < 32768.0F) {
    if (free_sectors_left >= -32768.0F) {
      dist_edge_left = (int16_T)free_sectors_left;
    } else {
      dist_edge_left = MIN_int16_T;
    }
  } else if (free_sectors_left >= 32768.0F) {
    dist_edge_left = MAX_int16_T;
  } else {
    dist_edge_left = 0;
  }

  react_direction = dist_edge_right - dist_edge_left;
  if (react_direction > 32767) {
    react_direction = 32767;
  } else {
    if (react_direction < -32768) {
      react_direction = -32768;
    }
  }

  front_range = (int16_T)react_direction;

  /*  --------------------------------------------------------------------- */
  /*  calculate front situation */
  /*  --------------------------------------------------------------------- */
  /*  evaluate current front situation, need an update? */
  if ((front_distance < front_situation[8]) && (front_distance < front_range) &&
      (front_distance < radar[16])) {
    /*  create new front situation */
    /*  only create new front situation if wall detection has not */
    /*  recognized the obstacle */
    /*  clear current front reaction */
    for (react_direction = 0; react_direction < 17; react_direction++) {
      front_situation[react_direction] = 5000;
    }

    /* right or left? */
    if (*yaw_control != 0.0F) {
      if (*yaw_control < 0.0F) {
        /*  left */
        react_direction = -1;
      } else {
        /*  right */
        react_direction = 1;
      }
    } else if (*y_control != 0.0F) {
      if (*y_control < 0.0F) {
        /*  left */
        react_direction = -1;
      } else {
        /*  right */
        react_direction = 1;
      }
    } else {
      /*  situation check with right and left sample */
      /*  TODO find a better way */
      b_free_sectors_left = 0.0;
      free_sectors_right = 0.0;

      /*  begin to show from middle */
      react_direction = 0;
      while ((react_direction < 8) && (radar[15 - react_direction] >
              front_side_threshold)) {
        b_free_sectors_left++;
        react_direction++;
      }

      react_direction = 0;
      while ((react_direction < 8) && (radar[react_direction + 17] >
              front_side_threshold)) {
        free_sectors_right++;
        react_direction++;
      }

      if (b_free_sectors_left > free_sectors_right) {
        /*  left */
        react_direction = -1;
      } else {
        /*  right */
        react_direction = 1;
      }
    }

    if (react_direction < 0) {
      /*  left */
      react_direction = front_distance + 100;
      if (react_direction > 32767) {
        react_direction = 32767;
      }

      front_situation[7] = (int16_T)react_direction;
      front_situation[8] = front_distance;
      react_direction = front_distance - 100;
      if (react_direction < -32768) {
        react_direction = -32768;
      }

      front_situation[9] = (int16_T)react_direction;
    } else {
      /*  right */
      react_direction = front_distance - 100;
      if (react_direction < -32768) {
        react_direction = -32768;
      }

      front_situation[7] = (int16_T)react_direction;
      front_situation[8] = front_distance;
      react_direction = front_distance + 100;
      if (react_direction > 32767) {
        react_direction = 32767;
      }

      front_situation[9] = (int16_T)react_direction;
    }
  }

  /*  merge front situation with radar */
  for (react_direction = 0; react_direction < 18; react_direction++) {
    if (radar[react_direction + 7] > front_situation[react_direction - 1]) {
      radar[react_direction + 7] = front_situation[react_direction - 1];
    }
  }

  /*  --------------------------------------------------------------------- */
  /*  calculate yaw control */
  /*  --------------------------------------------------------------------- */
  /*  we control through free sectors */
  /*  the #free sectors to the frontal sides defines the controlling output of yaw */
  free_sectors_left = 0.0F;
  b_free_sectors_right = 0.0F;
  dist_edge_left = 5000;
  dist_edge_right = 5000;

  /*  begin to show from middle */
  react_direction = 0;
  exitg2 = FALSE;
  while ((exitg2 == FALSE) && ((react_direction < 8) && (!(1 + react_direction >
            4)))) {
    if (radar[15 - react_direction] > front_side_threshold) {
      free_sectors_left++;
      react_direction++;
    } else {
      dist_edge_left = radar[15 - react_direction];
      exitg2 = TRUE;
    }
  }

  react_direction = 0;
  exitg1 = FALSE;
  while ((exitg1 == FALSE) && ((react_direction < 8) && (!(1 + react_direction >
            4)))) {
    if (radar[react_direction + 17] > front_side_threshold) {
      b_free_sectors_right++;
      react_direction++;
    } else {
      dist_edge_right = radar[react_direction + 17];
      exitg1 = TRUE;
    }
  }

  if ((free_sectors_left < 4.0F) || (b_free_sectors_right < 4.0F)) {
    /*  difference of free sectors */
    free_sectors_diff = free_sectors_left - b_free_sectors_right;
    if (dist_edge_left < 5000) {
    } else {
      dist_edge_left = front_side_threshold;
    }

    if (dist_edge_right < 5000) {
    } else {
      dist_edge_right = front_side_threshold;
    }

    /*  which side is near */
    /*  scaling and direction */
    /*  0 to 1 scaling */
    /*  left or right */
    if ((free_sectors_diff == 0.0F) && ((free_sectors_left == 0.0F) ||
         (b_free_sectors_right == 0.0F))) {
      /* wall is almost frontal */
      /*  almost frontal,   */
      react_direction = dist_edge_left - dist_edge_right;
      if (react_direction > 32767) {
        react_direction = 32767;
      } else {
        if (react_direction < -32768) {
          react_direction = -32768;
        }
      }

      if (react_direction > 0) {
        /*  turn left */
        yaw_direction = -1;
        react_direction = dist_edge_right - min_front_side_dist;
        if (react_direction > 32767) {
          react_direction = 32767;
        } else {
          if (react_direction < -32768) {
            react_direction = -32768;
          }
        }

        dist_edge_right = (int16_T)react_direction;
      } else {
        /*  turn right */
        yaw_direction = 1;
        react_direction = dist_edge_left - min_front_side_dist;
        if (react_direction > 32767) {
          react_direction = 32767;
        } else {
          if (react_direction < -32768) {
            react_direction = -32768;
          }
        }

        dist_edge_right = (int16_T)react_direction;
      }

      /*  only distance to wall defines scaling */
      if (dist_edge_right <= 0) {
        free_sectors_left = 1.0F;
      } else {
        /*  0..1 */
        react_direction = front_side_range - dist_edge_right;
        if (react_direction < -32768) {
          react_direction = -32768;
        }

        free_sectors_left = rt_powf_snf((real32_T)react_direction / (real32_T)
          front_side_range, 2.0F);
      }
    } else if ((free_sectors_left < 4.0F) && (b_free_sectors_right < 4.0F)) {
      /*  calc perfect yaw */
      /*  ------------------------------------------------------------- */
      free_sectors_left = fv0[15 - (int32_T)free_sectors_left] - fv0[(int32_T)
        b_free_sectors_right + 17];
      if (free_sectors_left > 0.0F) {
        /*  turn left */
        yaw_direction = -1;
      } else {
        /*  turn right */
        yaw_direction = 1;
      }

      /*  calc circle sector length (perimeter) */
      /*  calc scale */
      react_direction = dist_edge_left + dist_edge_right;
      if (react_direction > 32767) {
        react_direction = 32767;
      } else {
        if (react_direction < -32768) {
          react_direction = -32768;
        }
      }

      free_sectors_left = (real32_T)fabs(free_sectors_left) / ((real32_T)
        react_direction / 2.0F / (real32_T)sin((real32_T)fabs(free_sectors_left))
        * (real32_T)fabs(free_sectors_left) / settings[0]) / settings[2];
    } else {
      /*  full reaction is possible but not always needed */
      if (free_sectors_diff > 0.0F) {
        /*  turn left */
        yaw_direction = -1;
        react_direction = dist_edge_right - min_front_side_dist;
        if (react_direction > 32767) {
          react_direction = 32767;
        } else {
          if (react_direction < -32768) {
            react_direction = -32768;
          }
        }

        dist_edge_right = (int16_T)react_direction;
      } else {
        /*  turn right */
        yaw_direction = 1;
        react_direction = dist_edge_left - min_front_side_dist;
        if (react_direction > 32767) {
          react_direction = 32767;
        } else {
          if (react_direction < -32768) {
            react_direction = -32768;
          }
        }

        dist_edge_right = (int16_T)react_direction;
      }

      if (dist_edge_right <= 0) {
        free_sectors_left = free_sectors_diff / 4.0F;
      } else {
        /*  0..1 */
        react_direction = front_side_range - dist_edge_right;
        if (react_direction < -32768) {
          react_direction = -32768;
        }

        free_sectors_left = rt_powf_snf((real32_T)react_direction / (real32_T)
          front_side_range, 2.0F) * (free_sectors_diff / 4.0F);
      }
    }

    free_environment = FALSE;
    *yaw_control = (real32_T)yaw_direction * (settings[2] * free_sectors_left);
  } else {
    free_environment = TRUE;
    *yaw_control = 0.0F;
  }

  /*  --------------------------------------------------------------------- */
  /*  calculate y control */
  /*  --------------------------------------------------------------------- */
  /*  for y controlling we just compare the average of right and left sector */
  /*  distances and take the difference as controlling input. */
  /*  */
  free_sectors_left = 0.0F;
  b_free_sectors_right = 0.0F;
  for (react_direction = 0; react_direction < 3; react_direction++) {
    free_sectors_left += (real32_T)radar[react_direction + 8] * (real32_T)sin
      (fv0[react_direction + 8]);
    b_free_sectors_right += (real32_T)radar[24 - react_direction] * (real32_T)
      sin(fv0[24 - react_direction]);
  }

  free_sectors_diff = free_sectors_left / 3.0F;
  free_sectors_left = b_free_sectors_right / 3.0F;
  if (free_sectors_diff < (real32_T)side_threshold) {
    b_free_sectors_diff = free_sectors_diff;
  } else {
    b_free_sectors_diff = (real32_T)side_threshold;
  }

  if (free_sectors_left < (real32_T)side_threshold) {
    c_free_sectors_left = free_sectors_left;
  } else {
    c_free_sectors_left = (real32_T)side_threshold;
  }

  free_sectors_left = b_free_sectors_diff - c_free_sectors_left;

  /*  scale from -1 to 1 */
  if (free_sectors_left > 0.0F) {
    /*  turn left */
    if (free_sectors_left > (real32_T)side_range) {
      free_sectors_left = -1.0F;
    } else {
      free_sectors_left = -rt_powf_snf(free_sectors_left / (real32_T)side_range,
        2.0F);
    }

    free_environment = FALSE;
  } else if (free_sectors_left < 0.0F) {
    /*  turn right */
    if ((real32_T)fabs(free_sectors_left) > (real32_T)side_range) {
      free_sectors_left = 1.0F;
    } else {
      free_sectors_left = rt_powf_snf(free_sectors_left / (real32_T)side_range,
        2.0F);
    }

    free_environment = FALSE;
  } else {
    /*  nothing on both sides */
    free_sectors_left = 0.0F;
  }

  *y_control = settings[1] * free_sectors_left;

  /*  --------------------------------------------------------------------- */
  /*  calculate x control */
  /*  --------------------------------------------------------------------- */
  /*  adjust x control (TODO half it if a obstacle is there...) */
  *x_control = settings[0];

  /*  --------------------------------------------------------------------- */
  /*  update front situation for next iteration */
  /*  --------------------------------------------------------------------- */
  /*  x and y offset in mm, yaw control in rad */
  frontSituationUpdate(front_situation, *yaw_control, settings[0] * 1000.0F,
                       *y_control * 1000.0F, iv0);
  for (react_direction = 0; react_direction < 17; react_direction++) {
    front_situation[react_direction] = iv0[react_direction];
  }

  /*  delete unimportant opstacles and minimum correction */
  for (react_direction = 0; react_direction < 32; react_direction++) {
    if (front_situation[react_direction] > front_range) {
      front_situation[react_direction] = 5000;
    } else {
      if (front_situation[react_direction] < 100) {
        front_situation[react_direction] = 100;
      }
    }
  }

  return free_environment;
}

/* End of code generation (radarControl.c) */
