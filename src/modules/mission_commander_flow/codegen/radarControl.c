/*
 * radarControl.c
 *
 * Code generation for function 'radarControl'
 *
 * C source code generated on: Sun May 19 21:16:10 2013
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "radarControl.h"
#include "sum.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
static real32_T rt_powf_snf(real32_T u0, real32_T u1);
static real32_T rt_roundf_snf(real32_T u);

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

static real32_T rt_roundf_snf(real32_T u)
{
  real32_T y;
  if ((real32_T)fabs(u) < 8.388608E+6F) {
    if (u >= 0.5F) {
      y = (real32_T)floor(u + 0.5F);
    } else if (u > -0.5F) {
      y = -0.0F;
    } else {
      y = (real32_T)ceil(u - 0.5F);
    }
  } else {
    y = u;
  }

  return y;
}

boolean_T radarControl(int16_T radar[32], real32_T front_distance, real32_T
  sonar_obstacle[3], boolean_T sonar_flags[2], const real32_T settings[10],
  real32_T *x_control, real32_T *y_control, real32_T *yaw_control)
{
  boolean_T free_environment;
  real32_T free_sectors_right;
  int16_T side_threshold;
  int16_T front_side_threshold;
  real32_T front_threshold;
  int16_T min_front_side_dist;
  int16_T dist_edge_left;
  int32_T i;
  int16_T side_range;
  int16_T front_side_range;
  real_T free_sectors_left;
  real_T b_free_sectors_right;
  int16_T b_radar[3];
  int16_T sonar_pitch;
  boolean_T exitg2;
  boolean_T exitg1;
  real32_T free_sectors_diff;
  int16_T front_left;
  real32_T yaw_scale;
  int32_T yaw_direction;
  static const real32_T fv0[32] = { 3.14159274F, 2.94524312F, 2.74889374F,
    2.55254412F, 2.3561945F, 2.15984488F, 1.96349549F, 1.76714587F, 1.57079637F,
    1.37444687F, 1.17809725F, 0.981747746F, 0.785398185F, 0.589048624F,
    0.392699093F, 0.196349546F, 0.0F, 0.196349546F, 0.392699093F, 0.589048624F,
    0.785398185F, 0.981747746F, 1.17809725F, 1.37444675F, 1.57079637F,
    1.76714587F, 1.96349537F, 2.15984488F, 2.3561945F, 2.55254412F, 2.7488935F,
    2.94524312F };

  real32_T b_free_sectors_diff;
  real32_T b_front_threshold;

  /* RADARCONTROLL calculates the controlling output based on radar input */
  /*  there are five parts, left, front_left, front, front_right, right */
  /*  for yaw controlling only front_left, front and front_right are concerned */
  /*  for y controlling only left <-> right are compared... and in extreme */
  /*  situation also front part can influent y controlling... */
  /*  */
  free_environment = TRUE;

  /*  radar variables */
  /*  mm */
  /*  mm */
  /*  front situation variables */
  /*  !!! index + 1 because of matlab */
  /*  maximum controls */
  /*  thresholds for begin with reaction */
  free_sectors_right = rt_roundf_snf(settings[3]);
  if (free_sectors_right < 32768.0F) {
    if (free_sectors_right >= -32768.0F) {
      side_threshold = (int16_T)free_sectors_right;
    } else {
      side_threshold = MIN_int16_T;
    }
  } else if (free_sectors_right >= 32768.0F) {
    side_threshold = MAX_int16_T;
  } else {
    side_threshold = 0;
  }

  /*  in mm */
  free_sectors_right = rt_roundf_snf(settings[4]);
  if (free_sectors_right < 32768.0F) {
    if (free_sectors_right >= -32768.0F) {
      front_side_threshold = (int16_T)free_sectors_right;
    } else {
      front_side_threshold = MIN_int16_T;
    }
  } else if (free_sectors_right >= 32768.0F) {
    front_side_threshold = MAX_int16_T;
  } else {
    front_side_threshold = 0;
  }

  /*  in mm */
  front_threshold = settings[5] / 1000.0F;

  /*  in meter */
  /*  minimum distance for full reaction */
  /*  TODO settings... */
  /* 1000; */
  free_sectors_right = rt_roundf_snf(settings[7]);
  if (free_sectors_right < 32768.0F) {
    if (free_sectors_right >= -32768.0F) {
      min_front_side_dist = (int16_T)free_sectors_right;
    } else {
      min_front_side_dist = MIN_int16_T;
    }
  } else if (free_sectors_right >= 32768.0F) {
    min_front_side_dist = MAX_int16_T;
  } else {
    min_front_side_dist = 0;
  }

  /* 1500; */
  /* 1000; */
  /* 1000; */
  /*  ranges */
  free_sectors_right = rt_roundf_snf(settings[8]);
  if (free_sectors_right < 32768.0F) {
    if (free_sectors_right >= -32768.0F) {
      dist_edge_left = (int16_T)free_sectors_right;
    } else {
      dist_edge_left = MIN_int16_T;
    }
  } else if (free_sectors_right >= 32768.0F) {
    dist_edge_left = MAX_int16_T;
  } else {
    dist_edge_left = 0;
  }

  i = side_threshold - dist_edge_left;
  if (i > 32767) {
    i = 32767;
  } else {
    if (i < -32768) {
      i = -32768;
    }
  }

  side_range = (int16_T)i;
  i = front_side_threshold - min_front_side_dist;
  if (i > 32767) {
    i = 32767;
  } else {
    if (i < -32768) {
      i = -32768;
    }
  }

  front_side_range = (int16_T)i;

  /*  --------------------------------------------------------------------- */
  /*  calculate front situation */
  /*  --------------------------------------------------------------------- */
  if (settings[9] != 0.0F) {
    if ((front_distance < front_threshold) && (front_distance < (real32_T)radar
         [16] / 1000.0F)) {
      /*  only create new front situation if wall detection has not */
      /*  recognized the obstacle */
      if (sonar_flags[0]) {
        /*  take same situation */
        front_threshold = sonar_obstacle[2];
      } else {
        /*  check new situation */
        /*  right or left? */
        /*  TODO not ideal... */
        if (*yaw_control != 0.0F) {
          if (*yaw_control < 0.0F) {
            /*  left */
            front_threshold = -1.0F;
          } else {
            /*  right */
            front_threshold = 1.0F;
          }
        } else {
          /*  situation check with right and left free sectors */
          free_sectors_left = 0.0;
          b_free_sectors_right = 0.0;

          /*  begin to show from middle */
          i = 0;
          while ((i < 8) && (radar[15 - i] > front_side_threshold)) {
            free_sectors_left++;
            i++;
          }

          i = 0;
          while ((i < 8) && (radar[i + 17] > front_side_threshold)) {
            b_free_sectors_right++;
            i++;
          }

          if (free_sectors_left == b_free_sectors_right) {
            /*  front is free, side samples decides */
            front_threshold = (real32_T)sum(*(int16_T (*)[3])&radar[8]);
            for (i = 0; i < 3; i++) {
              b_radar[i] = radar[24 - i];
            }

            if (front_threshold / 3.0F > (real32_T)sum(b_radar) / 3.0F) {
              /*  left */
              front_threshold = -1.0F;
            } else {
              /*  right */
              front_threshold = 1.0F;
            }
          } else {
            /*  sectors decides */
            if (free_sectors_left > b_free_sectors_right) {
              /*  left */
              front_threshold = -1.0F;
            } else {
              /*  right */
              front_threshold = 1.0F;
            }
          }
        }
      }

      sonar_obstacle[0] = front_distance;
      sonar_obstacle[1] = 0.0F;
      sonar_obstacle[2] = front_threshold;
      sonar_flags[0] = TRUE;
      sonar_flags[1] = TRUE;
    } else {
      if (sonar_flags[0] && (((real32_T)fabs(sonar_obstacle[1]) > 1.04719758F) ||
           (sonar_obstacle[0] < 0.5F) || (sonar_obstacle[0] > front_threshold)))
      {
        /*  update sonar obstacle valididy */
        sonar_flags[0] = FALSE;
      }
    }

    /*  calc front sonar sector and merge it */
    if (sonar_flags[0]) {
      front_threshold = rt_roundf_snf(sonar_obstacle[1] / 0.196349546F);
      free_sectors_right = rt_roundf_snf(sonar_obstacle[2]);
      if (free_sectors_right < 32768.0F) {
        if (free_sectors_right >= -32768.0F) {
          dist_edge_left = (int16_T)free_sectors_right;
        } else {
          dist_edge_left = MIN_int16_T;
        }
      } else if (free_sectors_right >= 32768.0F) {
        dist_edge_left = MAX_int16_T;
      } else {
        dist_edge_left = 0;
      }

      i = dist_edge_left * 100;
      if (i > 32767) {
        i = 32767;
      } else {
        if (i < -32768) {
          i = -32768;
        }
      }

      sonar_pitch = (int16_T)i;

      /*  merge sonar update with radar */
      free_sectors_right = rt_roundf_snf(front_distance * 1000.0F);
      if (free_sectors_right < 32768.0F) {
        if (free_sectors_right >= -32768.0F) {
          dist_edge_left = (int16_T)free_sectors_right;
        } else {
          dist_edge_left = MIN_int16_T;
        }
      } else if (free_sectors_right >= 32768.0F) {
        dist_edge_left = MAX_int16_T;
      } else {
        dist_edge_left = 0;
      }

      i = dist_edge_left - sonar_pitch;
      if (i > 32767) {
        i = 32767;
      } else {
        if (i < -32768) {
          i = -32768;
        }
      }

      if (radar[(int32_T)((17.0F + front_threshold) - 1.0F) - 1] > i) {
        free_sectors_right = rt_roundf_snf(front_distance * 1000.0F);
        if (free_sectors_right < 32768.0F) {
          if (free_sectors_right >= -32768.0F) {
            dist_edge_left = (int16_T)free_sectors_right;
          } else {
            dist_edge_left = MIN_int16_T;
          }
        } else if (free_sectors_right >= 32768.0F) {
          dist_edge_left = MAX_int16_T;
        } else {
          dist_edge_left = 0;
        }

        i = dist_edge_left - sonar_pitch;
        if (i > 32767) {
          i = 32767;
        } else {
          if (i < -32768) {
            i = -32768;
          }
        }

        radar[(int32_T)((17.0F + front_threshold) - 1.0F) - 1] = (int16_T)i;
      }

      free_sectors_right = rt_roundf_snf(front_distance * 1000.0F);
      if (free_sectors_right < 32768.0F) {
        if (free_sectors_right >= -32768.0F) {
          dist_edge_left = (int16_T)free_sectors_right;
        } else {
          dist_edge_left = MIN_int16_T;
        }
      } else if (free_sectors_right >= 32768.0F) {
        dist_edge_left = MAX_int16_T;
      } else {
        dist_edge_left = 0;
      }

      if (radar[(int32_T)(17.0F + front_threshold) - 1] > dist_edge_left) {
        free_sectors_right = rt_roundf_snf(front_distance * 1000.0F);
        if (free_sectors_right < 32768.0F) {
          if (free_sectors_right >= -32768.0F) {
            dist_edge_left = (int16_T)free_sectors_right;
          } else {
            dist_edge_left = MIN_int16_T;
          }
        } else if (free_sectors_right >= 32768.0F) {
          dist_edge_left = MAX_int16_T;
        } else {
          dist_edge_left = 0;
        }

        radar[(int32_T)(17.0F + front_threshold) - 1] = dist_edge_left;
      }

      free_sectors_right = rt_roundf_snf(front_distance * 1000.0F);
      if (free_sectors_right < 32768.0F) {
        if (free_sectors_right >= -32768.0F) {
          dist_edge_left = (int16_T)free_sectors_right;
        } else {
          dist_edge_left = MIN_int16_T;
        }
      } else if (free_sectors_right >= 32768.0F) {
        dist_edge_left = MAX_int16_T;
      } else {
        dist_edge_left = 0;
      }

      i = dist_edge_left + sonar_pitch;
      if (i > 32767) {
        i = 32767;
      } else {
        if (i < -32768) {
          i = -32768;
        }
      }

      if (radar[(int32_T)((17.0F + front_threshold) + 1.0F) - 1] > i) {
        free_sectors_right = rt_roundf_snf(front_distance * 1000.0F);
        if (free_sectors_right < 32768.0F) {
          if (free_sectors_right >= -32768.0F) {
            dist_edge_left = (int16_T)free_sectors_right;
          } else {
            dist_edge_left = MIN_int16_T;
          }
        } else if (free_sectors_right >= 32768.0F) {
          dist_edge_left = MAX_int16_T;
        } else {
          dist_edge_left = 0;
        }

        i = dist_edge_left + sonar_pitch;
        if (i > 32767) {
          i = 32767;
        } else {
          if (i < -32768) {
            i = -32768;
          }
        }

        radar[(int32_T)((17.0F + front_threshold) + 1.0F) - 1] = (int16_T)i;
      }

      free_environment = FALSE;
    }
  }

  /*  --------------------------------------------------------------------- */
  /*  calculate yaw control */
  /*  --------------------------------------------------------------------- */
  /*  we control through free sectors */
  /*  the #free sectors to the frontal sides defines the controlling output of yaw */
  front_threshold = 0.0F;
  free_sectors_right = 0.0F;
  dist_edge_left = 5000;
  sonar_pitch = 5000;

  /*  begin to show from middle */
  i = 0;
  exitg2 = FALSE;
  while ((exitg2 == FALSE) && ((i < 8) && (!(1 + i > 4)))) {
    if (radar[15 - i] > front_side_threshold) {
      front_threshold++;
      i++;
    } else {
      dist_edge_left = radar[15 - i];
      exitg2 = TRUE;
    }
  }

  i = 0;
  exitg1 = FALSE;
  while ((exitg1 == FALSE) && ((i < 8) && (!(1 + i > 4)))) {
    if (radar[i + 17] > front_side_threshold) {
      free_sectors_right++;
      i++;
    } else {
      sonar_pitch = radar[i + 17];
      exitg1 = TRUE;
    }
  }

  if ((front_threshold < 4.0F) || (free_sectors_right < 4.0F)) {
    /*  difference of free sectors */
    free_sectors_diff = front_threshold - free_sectors_right;
    if (dist_edge_left < 5000) {
      front_left = dist_edge_left;
    } else {
      front_left = front_side_threshold;
    }

    if (sonar_pitch < 5000) {
      dist_edge_left = sonar_pitch;
    } else {
      dist_edge_left = front_side_threshold;
    }

    /*  which side is near */
    /*  scaling and direction */
    yaw_scale = 0.0F;

    /*  0 to 1 scaling */
    /*  left or right */
    if ((free_sectors_diff == 0.0F) && ((front_threshold == 0.0F) ||
         (free_sectors_right == 0.0F))) {
      /* wall is almost frontal */
      /*  almost frontal,   */
      i = front_left - dist_edge_left;
      if (i > 32767) {
        i = 32767;
      } else {
        if (i < -32768) {
          i = -32768;
        }
      }

      if (i > 0) {
        /*  turn left */
        yaw_direction = -1;
        i = dist_edge_left - min_front_side_dist;
        if (i > 32767) {
          i = 32767;
        } else {
          if (i < -32768) {
            i = -32768;
          }
        }

        dist_edge_left = (int16_T)i;
      } else {
        /*  turn right */
        yaw_direction = 1;
        i = front_left - min_front_side_dist;
        if (i > 32767) {
          i = 32767;
        } else {
          if (i < -32768) {
            i = -32768;
          }
        }

        dist_edge_left = (int16_T)i;
      }

      /*  only distance to wall defines scaling */
      if (dist_edge_left <= 0) {
        yaw_scale = 1.0F;
      } else {
        /*  0..1 */
        i = front_side_range - dist_edge_left;
        if (i < -32768) {
          i = -32768;
        }

        yaw_scale = rt_powf_snf((real32_T)i / (real32_T)front_side_range, 2.0F);

        /*                  yaw_scale = (single(front_side_range - dist_front_side) / single(front_side_range)); */
      }
    } else if ((front_threshold < 4.0F) && (free_sectors_right < 4.0F)) {
      /*  calc perfect yaw */
      /*  ------------------------------------------------------------- */
      front_threshold = fv0[15 - (int32_T)front_threshold] - fv0[(int32_T)
        free_sectors_right + 17];
      if (front_threshold > 0.0F) {
        /*  turn left */
        yaw_direction = -1;
      } else if (front_threshold < 0.0F) {
        /*  turn right */
        yaw_direction = 1;
      } else {
        /*  straight */
        yaw_direction = 0;
      }

      if (yaw_direction != 0) {
        /*  calc circle sector length (perimeter) */
        /*  calc scale */
        i = front_left + dist_edge_left;
        if (i > 32767) {
          i = 32767;
        } else {
          if (i < -32768) {
            i = -32768;
          }
        }

        yaw_scale = (real32_T)fabs(front_threshold) / ((real32_T)i / 2.0F /
          (real32_T)sin((real32_T)fabs(front_threshold)) * (real32_T)fabs
          (front_threshold) / settings[0]) / settings[2];
      }

      /*  for x throttle */
    } else {
      /*  full reaction is possible but not always needed */
      if (free_sectors_diff > 0.0F) {
        /*  turn left */
        yaw_direction = -1;
        i = dist_edge_left - min_front_side_dist;
        if (i > 32767) {
          i = 32767;
        } else {
          if (i < -32768) {
            i = -32768;
          }
        }

        dist_edge_left = (int16_T)i;
      } else {
        /*  turn right */
        yaw_direction = 1;
        i = front_left - min_front_side_dist;
        if (i > 32767) {
          i = 32767;
        } else {
          if (i < -32768) {
            i = -32768;
          }
        }

        dist_edge_left = (int16_T)i;
      }

      if (dist_edge_left <= 0) {
        yaw_scale = (real32_T)fabs(free_sectors_diff) / 4.0F;
      } else {
        /*  0..1 */
        /*                 yaw_scale = (single(front_side_range - dist_front_side) / single(front_side_range)) * ... */
        /*                     (abs(free_sectors_diff) / front_side_sectors); */
        i = front_side_range - dist_edge_left;
        if (i < -32768) {
          i = -32768;
        }

        yaw_scale = rt_powf_snf((real32_T)i / (real32_T)front_side_range, 2.0F) *
          ((real32_T)fabs(free_sectors_diff) / 4.0F);
      }
    }

    free_environment = FALSE;
    *yaw_control = (real32_T)yaw_direction * (settings[2] * yaw_scale);
  } else {
    *yaw_control = 0.0F;
  }

  /*  --------------------------------------------------------------------- */
  /*  calculate y control */
  /*  --------------------------------------------------------------------- */
  /*  for y controlling we just compare the average of right and left sector */
  /*  distances and take the difference as controlling input. */
  /*  */
  front_threshold = 0.0F;
  free_sectors_right = 0.0F;
  for (i = 0; i < 3; i++) {
    front_threshold += (real32_T)radar[i + 8] * (real32_T)sin(fv0[i + 8]);
    free_sectors_right += (real32_T)radar[24 - i] * (real32_T)sin(fv0[24 - i]);
  }

  free_sectors_diff = front_threshold / 3.0F;
  front_threshold = free_sectors_right / 3.0F;
  if (free_sectors_diff < (real32_T)side_threshold) {
    b_free_sectors_diff = free_sectors_diff;
  } else {
    b_free_sectors_diff = (real32_T)side_threshold;
  }

  if (front_threshold < (real32_T)side_threshold) {
    b_front_threshold = front_threshold;
  } else {
    b_front_threshold = (real32_T)side_threshold;
  }

  front_threshold = b_free_sectors_diff - b_front_threshold;

  /*  scale from -1 to 1 */
  if (front_threshold > 0.0F) {
    /*  turn left */
    if (front_threshold > (real32_T)side_range) {
      front_threshold = -1.0F;
    } else {
      front_threshold = -rt_powf_snf(front_threshold / (real32_T)side_range,
        2.0F);
    }

    free_environment = FALSE;
  } else if (front_threshold < 0.0F) {
    /*  turn right */
    if ((real32_T)fabs(front_threshold) > (real32_T)side_range) {
      front_threshold = 1.0F;
    } else {
      front_threshold = rt_powf_snf(front_threshold / (real32_T)side_range, 2.0F);
    }

    free_environment = FALSE;
  } else {
    /*  nothing on both sides */
    front_threshold = 0.0F;
  }

  /*  only change y set if no yaw controll */
  /*      if yaw_control == 0 */
  /*          y_control = single(max_y_control * y_scale); */
  /*      end */
  *y_control = settings[1] * front_threshold;

  /*  --------------------------------------------------------------------- */
  /*  calculate x control */
  /*  --------------------------------------------------------------------- */
  /*  adjust x control (TODO half it if a obstacle is there...) */
  /*      if free_sectors_left == 0 || free_sectors_right == 0 */
  /*          if dist_front_side < x_throttle_range */
  /*              if dist_front_side <= 0 */
  /*                  x_control_scale = x_control_scale - 0.5; */
  /*              else  */
  /*                  x_control_scale = x_control_scale - (single(x_throttle_range - dist_front_side) / single(x_throttle_range)) * 0.5; */
  /*              end */
  /*          end */
  /*      end */
  *x_control = settings[0];
  return free_environment;
}

/* End of code generation (radarControl.c) */
