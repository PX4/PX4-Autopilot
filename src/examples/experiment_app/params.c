/*
 * @file params.c
 *
 * Parameters for fixedwing system id
 */

/* Experiment new parameters*/

/**
 * For magnitude of input
 *
 *
 * @min 0
 * @max 1
 *
 */
PARAM_DEFINE_FLOAT(EXP_MAG, 0.1);

/**
 * For duration of input
 *
 *
 * @min 0
 * @max 20
 *
 */
PARAM_DEFINE_FLOAT(EXP_DUR, 5);


/**
 * For start frequency of sine sweep
 *
 *
 * @min 0.1
 * @max 6.9
 *
 */
PARAM_DEFINE_FLOAT(EXP_FSTART, 0.1);

/**
 * For end frequency of sine sweep
 *
 *
 * @min 0.1
 * @max 7
 *
 */
PARAM_DEFINE_FLOAT(EXP_FEND, 1);

/**
 * To select input to be given 100 - step, 010 - doublet, 001 - sine_sweep.
 *
 *
 * @min 0
 * @max 2000
 *
 */
PARAM_DEFINE_INT32(EXP_INPUT, 000);

/**

 * To select the actuator for signal injection 10 - Throttle and 01 - Elevator
 *
 *
 * @min 0
 * @max 2000
 *
 */
PARAM_DEFINE_INT32(EXP_OUTPUT, 00);

/**
 * Flag to check if experiment is active 1 - active, 0 - not active.
 *
 * @min 0
 * @max 1
 *
 */
PARAM_DEFINE_INT32(EXP_FLAG, 0);
