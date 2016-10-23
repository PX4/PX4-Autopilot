/**
 * @file rcrecover_params.c
 *
 * Parameters for RC Recover mode
 */

/*
 * RC Recover parameters, accessible via MAVLink
 */

/**
 * RTL delay
 *
 * Delay after returning along recent path before switching to RTL mode.
 * If set to -1 the system will not switch to RTL but loiter at the earliest known position.
 *
 * @unit s
 * @min -1
 * @max 300
 * @decimal 1
 * @increment 0.5
 * @group RC Recover
 */
PARAM_DEFINE_FLOAT(RCRCVR_RTL_DELAY, -1.0f);
