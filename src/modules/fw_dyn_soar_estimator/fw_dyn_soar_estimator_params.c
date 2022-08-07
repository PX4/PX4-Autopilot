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
 * @unit kg
 * @min 0.01
 * @max 10
 * @decimal 2
 * @increment 0.01
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_SIGMA_Q_V, 1.f);

/**
 * Standard deviation of vertical shear position
 * 
 * This is the std dev of the shear vertical position
 * 
 * @unit kg
 * @min 0.01
 * @max 10
 * @decimal 2
 * @increment 0.01
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_SIGMA_Q_H, 1.f);

/**
 * Standard deviation of velicity state in shear model
 * 
 * This is the std dev of the shear strenght param
 * 
 * @unit kg
 * @min 0.01
 * @max 10
 * @decimal 2
 * @increment 0.01
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_SIGMA_Q_A, 1.f);

/**
 * Standard deviation of velicity measurement (wind)
 * 
 * This is the std dev of the wind pseudomeasurement passed to the EKF
 * 
 * @unit kg
 * @min 0.01
 * @max 10
 * @decimal 2
 * @increment 0.01
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_SIGMA_R_V, 1.f);