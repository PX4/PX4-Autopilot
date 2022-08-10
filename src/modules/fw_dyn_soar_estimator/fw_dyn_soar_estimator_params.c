/**
 * @file fw_dyn_soar_estimator_params.c
 *
 * Parameters defined by the INDI position controller
 *
 * @author Marvin Harms <marv@teleport.ch>
 */

/*
 * Controller parameters, accessible via MAVLink
 */

/**
 * Standard deviation of velicity state in shear model
 * 
 * This is the std dev of the wind velocity in each direction
 * 
 * @unit m
 * @min 0.0
 * @max 10
 * @decimal 6
 * @increment 0.000001
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_SIGMA_Q_V, 0.1f);

/**
 * Standard deviation of vertical shear position
 * 
 * This is the std dev of the shear vertical position
 * 
 * @unit m
 * @min 0.0
 * @max 10
 * @decimal 6
 * @increment 0.000001
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_SIGMA_Q_H, 1.f);

/**
 * Standard deviation of velicity state in shear model
 * 
 * This is the std dev of the shear strenght param
 * 
 * @unit m
 * @min 0.0
 * @max 10
 * @decimal 6
 * @increment 0.000001
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_SIGMA_Q_A, 0.03f);

/**
 * Standard deviation of velicity measurement (wind)
 * 
 * This is the std dev of the wind pseudomeasurement passed to the EKF
 * 
 * @unit m
 * @min 0.0
 * @max 10
 * @decimal 6
 * @increment 0.000001
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_SIGMA_R_V, 1.f);

/**
 * Initial guess  of the vertical shear position
 * 
 * This is the filter initial value of the shear vertical position
 * 
 * @unit m
 * @min -200
 * @max 200
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_INIT_H, 60.f);