/*
 * radarControl.c
 *
 * Code generation for function 'radarControl'
 *
 * C source code generated on: Tue Apr 30 22:00:18 2013
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
  front_situation[4], const real32_T settings[9], real32_T x_update, real32_T
  y_update, real32_T yaw_update, real32_T *x_control, real32_T *y_control,
  real32_T *yaw_control)
{
  boolean_T free_environment;
  real32_T y;
  int16_T side_threshold;
  int16_T front_side_threshold;
  real32_T front_threshold;
  int16_T min_front_side_dist;
  int16_T dist_edge_left;
  int32_T i;
  int16_T side_range;
  int16_T front_side_range;
  real_T free_sectors_left;
  real_T free_sectors_right;
  real32_T x;
  real32_T r;
  int16_T sonar_pitch;
  int16_T front_left;
  boolean_T exitg2;
  boolean_T exitg1;
  int32_T yaw_direction;
  static const real32_T fv0[32] = { 3.14159274F, 2.94524312F, 2.74889374F,
    2.55254412F, 2.3561945F, 2.15984488F, 1.96349549F, 1.76714587F, 1.57079637F,
    1.37444687F, 1.17809725F, 0.981747746F, 0.785398185F, 0.589048624F,
    0.392699093F, 0.196349546F, 0.0F, 0.196349546F, 0.392699093F, 0.589048624F,
    0.785398185F, 0.981747746F, 1.17809725F, 1.37444675F, 1.57079637F,
    1.76714587F, 1.96349537F, 2.15984488F, 2.3561945F, 2.55254412F, 2.7488935F,
    2.94524312F };

  real32_T b_r;
  real32_T b_y;

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
  y = rt_roundf_snf(settings[3]);
  if (y < 32768.0F) {
    if (y >= -32768.0F) {
      side_threshold = (int16_T)y;
    } else {
      side_threshold = MIN_int16_T;
    }
  } else if (y >= 32768.0F) {
    side_threshold = MAX_int16_T;
  } else {
    side_threshold = 0;
  }

  /*  in mm */
  y = rt_roundf_snf(settings[4]);
  if (y < 32768.0F) {
    if (y >= -32768.0F) {
      front_side_threshold = (int16_T)y;
    } else {
      front_side_threshold = MIN_int16_T;
    }
  } else if (y >= 32768.0F) {
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
  y = rt_roundf_snf(settings[7]);
  if (y < 32768.0F) {
    if (y >= -32768.0F) {
      min_front_side_dist = (int16_T)y;
    } else {
      min_front_side_dist = MIN_int16_T;
    }
  } else if (y >= 32768.0F) {
    min_front_side_dist = MAX_int16_T;
  } else {
    min_front_side_dist = 0;
  }

  /* 1500; */
  /* 1000; */
  /*  ranges */
  y = rt_roundf_snf(settings[8]);
  if (y < 32768.0F) {
    if (y >= -32768.0F) {
      dist_edge_left = (int16_T)y;
    } else {
      dist_edge_left = MIN_int16_T;
    }
  } else if (y >= 32768.0F) {
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
  if ((front_distance < front_threshold) && (front_distance < (real32_T)radar[16]
       / 1000.0F)) {
    /*  only create new front situation if wall detection has not */
    /*  recognized the obstacle */
    if (front_situation[3] != 0.0F) {
      /*  take same situation */
      y = front_situation[2];
    } else {
      /*  check new situation */
      /*  right or left? */
      /*  TODO not ideal... */
      if (*yaw_control != 0.0F) {
        if (*yaw_control < 0.0F) {
          /*  left */
          y = -1.0F;
        } else {
          /*  right */
          y = 1.0F;
        }
      } else {
        /*  situation check with right and left free sectors */
        free_sectors_left = 0.0;
        free_sectors_right = 0.0;

        /*  begin to show from middle */
        i = 0;
        while ((i < 8) && (radar[15 - i] > front_side_threshold)) {
          free_sectors_left++;
          i++;
        }

        i = 0;
        while ((i < 8) && (radar[i + 17] > front_side_threshold)) {
          free_sectors_right++;
          i++;
        }

        if (free_sectors_left > free_sectors_right) {
          /*  left */
          y = -1.0F;
        } else {
          /*  right */
          y = 1.0F;
        }
      }
    }

    front_situation[0] = front_distance;
    front_situation[1] = 0.0F;
    front_situation[2] = y;
    front_situation[3] = 1.0F;
  } else {
    if (front_situation[3] != 0.0F) {
      /*  update front situation with old controls */
      x = front_situation[0] * (real32_T)cos(front_situation[1]) - x_update;
      y = front_situation[0] * (real32_T)sin(front_situation[1]) - y_update;
      if (x > 0.0F) {
        r = (real32_T)sqrt(rt_powf_snf(x, 2.0F) + rt_powf_snf(y, 2.0F));
        y = (real32_T)atan(y / x) - yaw_update;
        if (((real32_T)fabs(y) < 1.04719758F) && (r > 0.5F) && (r <
             front_threshold)) {
          front_situation[0] = r;
          front_situation[1] = y;
        } else {
          front_situation[3] = 0.0F;
        }
      } else {
        front_situation[3] = 0.0F;
      }
    }
  }

  /*  calc front sonar sector and merge it */
  if (front_situation[3] != 0.0F) {
    x = rt_roundf_snf(front_situation[1] / 0.196349546F);
    y = rt_roundf_snf(front_situation[2]);
    if (y < 32768.0F) {
      if (y >= -32768.0F) {
        dist_edge_left = (int16_T)y;
      } else {
        dist_edge_left = MIN_int16_T;
      }
    } else if (y >= 32768.0F) {
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
    y = rt_roundf_snf(front_distance * 1000.0F);
    if (y < 32768.0F) {
      if (y >= -32768.0F) {
        dist_edge_left = (int16_T)y;
      } else {
        dist_edge_left = MIN_int16_T;
      }
    } else if (y >= 32768.0F) {
      dist_edge_left = MAX_int16_T;
    } else {
      dist_edge_left = 0;
    }

    if (sonar_pitch > 16383) {
      front_left = MAX_int16_T;
    } else if (sonar_pitch <= -16384) {
      front_left = MIN_int16_T;
    } else {
      front_left = (int16_T)(sonar_pitch << 1);
    }

    i = dist_edge_left - front_left;
    if (i > 32767) {
      i = 32767;
    } else {
      if (i < -32768) {
        i = -32768;
      }
    }

    if (radar[(int32_T)((17.0F + x) - 1.0F) - 1] > i) {
      y = rt_roundf_snf(front_distance * 1000.0F);
      if (y < 32768.0F) {
        if (y >= -32768.0F) {
          dist_edge_left = (int16_T)y;
        } else {
          dist_edge_left = MIN_int16_T;
        }
      } else if (y >= 32768.0F) {
        dist_edge_left = MAX_int16_T;
      } else {
        dist_edge_left = 0;
      }

      if (sonar_pitch > 16383) {
        front_left = MAX_int16_T;
      } else if (sonar_pitch <= -16384) {
        front_left = MIN_int16_T;
      } else {
        front_left = (int16_T)(sonar_pitch << 1);
      }

      i = dist_edge_left - front_left;
      if (i > 32767) {
        i = 32767;
      } else {
        if (i < -32768) {
          i = -32768;
        }
      }

      radar[(int32_T)((17.0F + x) - 1.0F) - 1] = (int16_T)i;
    }

    y = rt_roundf_snf(front_distance * 1000.0F);
    if (y < 32768.0F) {
      if (y >= -32768.0F) {
        dist_edge_left = (int16_T)y;
      } else {
        dist_edge_left = MIN_int16_T;
      }
    } else if (y >= 32768.0F) {
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

    if (radar[(int32_T)((17.0F + x) - 1.0F) - 1] > i) {
      y = rt_roundf_snf(front_distance * 1000.0F);
      if (y < 32768.0F) {
        if (y >= -32768.0F) {
          dist_edge_left = (int16_T)y;
        } else {
          dist_edge_left = MIN_int16_T;
        }
      } else if (y >= 32768.0F) {
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

      radar[(int32_T)((17.0F + x) - 1.0F) - 1] = (int16_T)i;
    }

    y = rt_roundf_snf(front_distance * 1000.0F);
    if (y < 32768.0F) {
      if (y >= -32768.0F) {
        dist_edge_left = (int16_T)y;
      } else {
        dist_edge_left = MIN_int16_T;
      }
    } else if (y >= 32768.0F) {
      dist_edge_left = MAX_int16_T;
    } else {
      dist_edge_left = 0;
    }

    if (radar[(int32_T)(17.0F + x) - 1] > dist_edge_left) {
      y = rt_roundf_snf(front_distance * 1000.0F);
      if (y < 32768.0F) {
        if (y >= -32768.0F) {
          dist_edge_left = (int16_T)y;
        } else {
          dist_edge_left = MIN_int16_T;
        }
      } else if (y >= 32768.0F) {
        dist_edge_left = MAX_int16_T;
      } else {
        dist_edge_left = 0;
      }

      radar[(int32_T)(17.0F + x) - 1] = dist_edge_left;
    }

    y = rt_roundf_snf(front_distance * 1000.0F);
    if (y < 32768.0F) {
      if (y >= -32768.0F) {
        dist_edge_left = (int16_T)y;
      } else {
        dist_edge_left = MIN_int16_T;
      }
    } else if (y >= 32768.0F) {
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

    if (radar[(int32_T)((17.0F + x) + 1.0F) - 1] > i) {
      y = rt_roundf_snf(front_distance * 1000.0F);
      if (y < 32768.0F) {
        if (y >= -32768.0F) {
          dist_edge_left = (int16_T)y;
        } else {
          dist_edge_left = MIN_int16_T;
        }
      } else if (y >= 32768.0F) {
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

      radar[(int32_T)((17.0F + x) + 1.0F) - 1] = (int16_T)i;
    }

    y = rt_roundf_snf(front_distance * 1000.0F);
    if (y < 32768.0F) {
      if (y >= -32768.0F) {
        dist_edge_left = (int16_T)y;
      } else {
        dist_edge_left = MIN_int16_T;
      }
    } else if (y >= 32768.0F) {
      dist_edge_left = MAX_int16_T;
    } else {
      dist_edge_left = 0;
    }

    if (sonar_pitch > 16383) {
      front_left = MAX_int16_T;
    } else if (sonar_pitch <= -16384) {
      front_left = MIN_int16_T;
    } else {
      front_left = (int16_T)(sonar_pitch << 1);
    }

    i = dist_edge_left + front_left;
    if (i > 32767) {
      i = 32767;
    } else {
      if (i < -32768) {
        i = -32768;
      }
    }

    if (radar[(int32_T)((17.0F + x) + 2.0F) - 1] > i) {
      y = rt_roundf_snf(front_distance * 1000.0F);
      if (y < 32768.0F) {
        if (y >= -32768.0F) {
          dist_edge_left = (int16_T)y;
        } else {
          dist_edge_left = MIN_int16_T;
        }
      } else if (y >= 32768.0F) {
        dist_edge_left = MAX_int16_T;
      } else {
        dist_edge_left = 0;
      }

      if (sonar_pitch > 16383) {
        front_left = MAX_int16_T;
      } else if (sonar_pitch <= -16384) {
        front_left = MIN_int16_T;
      } else {
        front_left = (int16_T)(sonar_pitch << 1);
      }

      i = dist_edge_left + front_left;
      if (i > 32767) {
        i = 32767;
      } else {
        if (i < -32768) {
          i = -32768;
        }
      }

      radar[(int32_T)((17.0F + x) + 2.0F) - 1] = (int16_T)i;
    }
  }

  /*  --------------------------------------------------------------------- */
  /*  calculate yaw control */
  /*  --------------------------------------------------------------------- */
  /*  we control through free sectors */
  /*  the #free sectors to the frontal sides defines the controlling output of yaw */
  y = 0.0F;
  x = 0.0F;
  dist_edge_left = 5000;
  sonar_pitch = 5000;

  /*  begin to show from middle */
  i = 0;
  exitg2 = FALSE;
  while ((exitg2 == FALSE) && ((i < 8) && (!(1 + i > 4)))) {
    if (radar[15 - i] > front_side_threshold) {
      y++;
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
      x++;
      i++;
    } else {
      sonar_pitch = radar[i + 17];
      exitg1 = TRUE;
    }
  }

  if ((y < 4.0F) || (x < 4.0F)) {
    /*  difference of free sectors */
    r = y - x;
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
    /*  0 to 1 scaling */
    /*  left or right */
    if ((r == 0.0F) && ((y == 0.0F) || (x == 0.0F))) {
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
        y = 1.0F;
      } else {
        /*  0..1 */
        i = front_side_range - dist_edge_left;
        if (i < -32768) {
          i = -32768;
        }

        y = rt_powf_snf((real32_T)i / (real32_T)front_side_range, 2.0F);
      }
    } else if ((y < 4.0F) && (x < 4.0F)) {
      /*  calc perfect yaw */
      /*  ------------------------------------------------------------- */
      y = fv0[15 - (int32_T)y] - fv0[(int32_T)x + 17];
      if (y > 0.0F) {
        /*  turn left */
        yaw_direction = -1;
      } else {
        /*  turn right */
        yaw_direction = 1;
      }

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

      y = (real32_T)fabs(y) / ((real32_T)i / 2.0F / (real32_T)sin((real32_T)fabs
                                (y)) * (real32_T)fabs(y) / settings[0]) /
        settings[2];
    } else {
      /*  full reaction is possible but not always needed */
      if (r > 0.0F) {
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
        y = r / 4.0F;
      } else {
        /*  0..1 */
        i = front_side_range - dist_edge_left;
        if (i < -32768) {
          i = -32768;
        }

        y = rt_powf_snf((real32_T)i / (real32_T)front_side_range, 2.0F) * (r /
          4.0F);
      }
    }

    free_environment = FALSE;
    *yaw_control = (real32_T)yaw_direction * (settings[2] * y);
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
  y = 0.0F;
  x = 0.0F;
  for (i = 0; i < 3; i++) {
    y += (real32_T)radar[i + 8] * (real32_T)sin(fv0[i + 8]);
    x += (real32_T)radar[24 - i] * (real32_T)sin(fv0[24 - i]);
  }

  r = y / 3.0F;
  y = x / 3.0F;
  if (r < (real32_T)side_threshold) {
    b_r = r;
  } else {
    b_r = (real32_T)side_threshold;
  }

  if (y < (real32_T)side_threshold) {
    b_y = y;
  } else {
    b_y = (real32_T)side_threshold;
  }

  y = b_r - b_y;

  /*  scale from -1 to 1 */
  if (y > 0.0F) {
    /*  turn left */
    if (y > (real32_T)side_range) {
      y = -1.0F;
    } else {
      y = -rt_powf_snf(y / (real32_T)side_range, 2.0F);
    }

    free_environment = FALSE;
  } else if (y < 0.0F) {
    /*  turn right */
    if ((real32_T)fabs(y) > (real32_T)side_range) {
      y = 1.0F;
    } else {
      y = rt_powf_snf(y / (real32_T)side_range, 2.0F);
    }

    free_environment = FALSE;
  } else {
    /*  nothing on both sides */
    y = 0.0F;
  }

  *y_control = settings[1] * y;

  /*  --------------------------------------------------------------------- */
  /*  calculate x control */
  /*  --------------------------------------------------------------------- */
  /*  adjust x control (TODO half it if a obstacle is there...) */
  *x_control = settings[0];
  return free_environment;
}

/* End of code generation (radarControl.c) */
