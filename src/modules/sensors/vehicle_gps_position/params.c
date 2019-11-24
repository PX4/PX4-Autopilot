/**
 * Multi GPS Blending Control Mask.
 *
 * Set bits in the following positions to set which GPS accuracy metrics will be used to calculate the blending weight. Set to zero to disable and always used first GPS instance.
 * 0 : Set to true to use speed accuracy
 * 1 : Set to true to use horizontal position accuracy
 * 2 : Set to true to use vertical position accuracy
 *
 * @group EKF2
 * @min 0
 * @max 7
 * @bit 0 use speed accuracy
 * @bit 1 use hpos accuracy
 * @bit 2 use vpos accuracy
 */
PARAM_DEFINE_INT32(SENS_GPS_MASK, 0);

/**
 * Multi GPS Blending Time Constant
 *
 * Sets the longest time constant that will be applied to the calculation of GPS position and height offsets used to correct data from multiple GPS data for steady state position differences.
 *
 *
 * @group EKF2
 * @min 1.0
 * @max 100.0
 * @unit s
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(SENS_GPS_TAU, 10.0f);
