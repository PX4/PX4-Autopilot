/*
 * @file params.c
 *
 * Parameters for fixedwing system id
 */

/* Experiment new parameters*/

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