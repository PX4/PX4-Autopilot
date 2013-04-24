/*
 * radarControl.c
 *
 * Code generation for function 'radarControl'
 *
 * C source code generated on: Wed Apr 24 15:06:04 2013
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
static real32_T rt_powf_snf(real32_T u0, real32_T u1);

/* Function Definitions */
static real32_T rt_powf_snf(real32_T u0, real32_T u1)
{
  real32_T y;
  real32_T f0;
  real32_T f1;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = ((real32_T)rtNaN);
  } else {
    f0 = (real32_T)fabs(u0);
    f1 = (real32_T)fabs(u1);
    if (rtIsInfF(u1)) {
      if (f0 == 1.0F) {
        y = ((real32_T)rtNaN);
      } else if (f0 > 1.0F) {
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
    } else if (f1 == 0.0F) {
      y = 1.0F;
    } else if (f1 == 1.0F) {
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

void radarControl(const int16_T radar[32], real32_T front_distance, const
                  real32_T settings[9], real32_T *yaw_control, real32_T
                  *x_control, real32_T *y_control)
{
  real32_T side_range;
  real32_T front_side_range;
  real32_T free_sectors_left;
  real32_T free_sectors_right;
  int32_T dist_edge_left;
  int32_T dist_edge_right;
  int32_T i;
  boolean_T exitg2;
  boolean_T exitg1;
  real32_T free_sectors_diff;
  real32_T front_left;
  real32_T front_right;
  static const real32_T fv0[32] = { 3.14159274F, 2.94524312F, 2.74889374F,
    2.55254412F, 2.3561945F, 2.15984488F, 1.96349549F, 1.76714587F, 1.57079637F,
    1.37444687F, 1.17809725F, 0.981747746F, 0.785398185F, 0.589048624F,
    0.392699093F, 0.196349546F, 0.0F, 0.196349546F, 0.392699093F, 0.589048624F,
    0.785398185F, 0.981747746F, 1.17809725F, 1.37444675F, 1.57079637F,
    1.76714587F, 1.96349537F, 2.15984488F, 2.3561945F, 2.55254412F, 2.7488935F,
    2.94524312F };

  real32_T b_front_side_range;
  real32_T b_free_sectors_left;

  /* RADARCONTROLL calculates the controlling output based on radar input */
  /*  there are five parts, left, front_left, front, front_right, right */
  /*  for yaw controlling only front_left, front and front_right are concerned */
  /*  for y controlling only left <-> right are compared... and in extreme */
  /*  situation also front part can influent y controlling... */
  /*  !!! index + 1 because of matlab */
  /*  setting */
  /*  maximum controls */
  /*  thresholds for begin with reaction */
  /*  minimum distance for full reaction */
  /*  TODO settings... */
  /* single(1000); */
  /* single(1500); */
  /* single(1000); */
  /*  ranges */
  side_range = settings[3] - settings[8];
  front_side_range = settings[4] - settings[7];

  /*   */
  /*  % calc from settings */
  /*  radar_left_pos = radar_resolution / 4; */
  /*  radar_right_pos = radar_resolution / 4 * 3; */
  /*  radar_front_pos = radar_resolution / 2; */
  /*  resolution_step = single(2 * pi / radar_resolution); */
  /*  alpha_steps = resolution_step:resolution_step:single(pi)-resolution_step; */
  /*  ------------------------------------------------------------------------- */
  /*  calculate yaw control */
  /*  ------------------------------------------------------------------------- */
  /*  we control through free sectors */
  /*  the #free sectors to the frontal sides defines the controlling output of yaw */
  free_sectors_left = 0.0F;
  free_sectors_right = 0.0F;
  dist_edge_left = 5000;
  dist_edge_right = 5000;

  /*  begin to show from middle */
  i = 0;
  exitg2 = FALSE;
  while ((exitg2 == FALSE) && ((i < 8) && (!(1 + i > 4)))) {
    if ((real32_T)radar[15 - i] > settings[4]) {
      free_sectors_left++;
      i++;
    } else {
      dist_edge_left = radar[15 - i];
      exitg2 = TRUE;
    }
  }

  i = 0;
  exitg1 = FALSE;
  while ((exitg1 == FALSE) && ((i < 8) && (!(1 + i > 4)))) {
    if ((real32_T)radar[i + 17] > settings[4]) {
      free_sectors_right++;
      i++;
    } else {
      dist_edge_right = radar[i + 17];
      exitg1 = TRUE;
    }
  }

  if ((free_sectors_left < 4.0F) || (free_sectors_right < 4.0F)) {
    /*  difference of free sectors */
    free_sectors_diff = free_sectors_left - free_sectors_right;
    if (dist_edge_left < 5000) {
      front_left = (real32_T)dist_edge_left;
    } else {
      front_left = settings[4];
    }

    if (dist_edge_right < 5000) {
      front_right = (real32_T)dist_edge_right;
    } else {
      front_right = settings[4];
    }

    /*  which side is near */
    /*  scaling and direction */
    /*  0 to 1 scaling */
    /*  left or right */
    if ((free_sectors_diff == 0.0F) && ((free_sectors_left == 0.0F) ||
         (free_sectors_right == 0.0F))) {
      /* wall is almost frontal */
      /*  almost frontal,   */
      if (front_left - front_right > 0.0F) {
        /*  turn left */
        dist_edge_left = -1;
        free_sectors_left = front_right - settings[7];
      } else {
        /*  turn right */
        dist_edge_left = 1;
        free_sectors_left = front_left - settings[7];
      }

      /*  only distance to wall defines scaling */
      if (free_sectors_left <= 0.0F) {
        free_sectors_left = 1.0F;
      } else {
        /*  0..1 */
        free_sectors_left = rt_powf_snf((front_side_range - free_sectors_left) /
          front_side_range, 2.0F);
      }
    } else if ((free_sectors_left < 4.0F) && (free_sectors_right < 4.0F)) {
      /*  calc perfect yaw */
      /*  ------------------------------------------------------------- */
      free_sectors_left = fv0[15 - (int32_T)free_sectors_left] - fv0[(int32_T)
        free_sectors_right + 17];
      if (free_sectors_left > 0.0F) {
        /*  turn left */
        dist_edge_left = -1;
      } else {
        /*  turn right */
        dist_edge_left = 1;
      }

      /*  calc circle sector length (perimeter) */
      /*  calc scale */
      free_sectors_left = (real32_T)fabs(free_sectors_left) / ((front_left +
        front_right) / 2.0F / (real32_T)sin((real32_T)fabs(free_sectors_left)) *
        (real32_T)fabs(free_sectors_left) / settings[0]) / settings[2];
    } else {
      /*  full reaction is possible but not always needed */
      if (free_sectors_diff > 0.0F) {
        /*  turn left */
        dist_edge_left = -1;
        free_sectors_left = front_right - settings[7];
      } else {
        /*  turn right */
        dist_edge_left = 1;
        free_sectors_left = front_left - settings[7];
      }

      if (free_sectors_left <= 0.0F) {
        free_sectors_left = free_sectors_diff / 4.0F;
      } else {
        /*  0..1 */
        free_sectors_left = rt_powf_snf((front_side_range - free_sectors_left) /
          front_side_range, 2.0F) * (free_sectors_diff / 4.0F);
      }
    }

    *yaw_control = (real32_T)dist_edge_left * (settings[2] * free_sectors_left);
  } else {
    *yaw_control = 0.0F;
  }

  /*  ------------------------------------------------------------------------- */
  /*  calculate y control */
  /*  ------------------------------------------------------------------------- */
  /*  for y controlling we just compare the average of right and left sector */
  /*  distances and take the difference as controlling input. */
  /*  */
  free_sectors_left = 0.0F;
  free_sectors_right = 0.0F;
  for (i = 0; i < 3; i++) {
    free_sectors_left += (real32_T)radar[i + 8] * (real32_T)sin(fv0[i + 8]);
    free_sectors_right += (real32_T)radar[24 - i] * (real32_T)sin(fv0[24 - i]);
  }

  front_side_range = free_sectors_left / 3.0F;
  free_sectors_left = free_sectors_right / 3.0F;
  if (front_side_range < settings[3]) {
    b_front_side_range = front_side_range;
  } else {
    b_front_side_range = settings[3];
  }

  if (free_sectors_left < settings[3]) {
    b_free_sectors_left = free_sectors_left;
  } else {
    b_free_sectors_left = settings[3];
  }

  free_sectors_left = b_front_side_range - b_free_sectors_left;

  /*  scale from -1 to 1 */
  if (free_sectors_left > 0.0F) {
    /*  turn left */
    if (free_sectors_left > side_range) {
      free_sectors_left = -1.0F;
    } else {
      free_sectors_left = -rt_powf_snf(free_sectors_left / side_range, 2.0F);
    }
  } else if (free_sectors_left < 0.0F) {
    /*  turn right */
    if ((real32_T)fabs(free_sectors_left) > side_range) {
      free_sectors_left = 1.0F;
    } else {
      free_sectors_left = rt_powf_snf(free_sectors_left / side_range, 2.0F);
    }
  } else {
    /*  nothing on both sides */
    free_sectors_left = 0.0F;
  }

  *y_control = settings[1] * free_sectors_left;

  /*  ------------------------------------------------------------------------- */
  /*  calculate x control */
  /*  ------------------------------------------------------------------------- */
  /*  adjust x control (TODO half it if a obstacle is there...) */
  *x_control = settings[0];
}

/* End of code generation (radarControl.c) */
