/**
 * @file fw_dyn_soar_control_params.c
 *
 * Parameters defined by the INDI position controller
 *
 * @author Marvin Harms <marv@teleport.ch>
 */

/*
 * Controller parameters, accessible via MAVLink
 */

/**
 * total takeoff mass
 * 
 * This is the mass of the aircraft, used for the INDI
 * 
 * @unit kg
 * @min 1.0
 * @max 2.0
 * @decimal 2
 * @increment 0.01
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(FW_MASS, 1.4f);

/**
 * total wing area used for lift and drag computation
 * 
 * @unit m^2
 * @min 0.1
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(FW_WING_AREA, 0.4f);


/**
 * air density used for lift and drag computation
 * 
 * @unit 
 * @min 0.5
 * @max 1.225
 * @decimal 3
 * @increment 0.001
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(RHO, 1.223f);

/**
 * estimated lift coefficients used for lift and drag computation
 * 
 * Used as L = C_l0 + C_l1*alpha,
 * where alpha is the angle of attack.
 * 
 * @unit 
 * @min -100
 * @max 100
 * @decimal 3
 * @increment 0.001
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(C_l0, 0.356f);

/**
 * estimated lift coefficients used for lift and drag computation
 * 
 * Used as L = C_l0 + C_l1*alpha,
 * where alpha is the angle of attack.
 * 
 * @unit 
 * @min -100
 * @max 100
 * @decimal 3
 * @increment 0.001
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(C_l1, 2.354f);


/**
 * estimated drag coefficients used for lift and drag computation
 * 
 * Used as D = C_d0 + C_d1*alpha + C_d2*alpha**2,
 * where alpha is the angle of attack.
 * 
 * @unit 
 * @min -100
 * @max 100
 * @decimal 4
 * @increment 0.0001
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(C_d0, 0.0288f);

/**
 * estimated drag coefficients used for lift and drag computation
 * 
 * Used as D = C_d0 + C_d1*alpha + C_d2*alpha**2,
 * where alpha is the angle of attack.
 * 
 * @unit 
 * @min -100
 * @max 100
 * @decimal 4
 * @increment 0.0001
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(C_d1, 0.3783f);

/**
 * estimated drag coefficients used for lift and drag computation
 * 
 * Used as D = C_d0 + C_d1*alpha + C_d2*alpha**2,
 * where alpha is the angle of attack.
 * 
 * @unit 
 * @min -100
 * @max 100
 * @decimal 4
 * @increment 0.0001
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(C_d2, 1.984f);


/**
 * coefficients of the butterworth filter used for smoothing the IMU
 * 
 * @unit 
 * @min -100
 * @max 100
 * @decimal 4
 * @increment 0.0001
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(FILTER_A1, 0.0f);

/**
 * coefficients of the butterworth filter used for smoothing the IMU
 * 
 * @unit 
 * @min -100
 * @max 100
 * @decimal 4
 * @increment 0.0001
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(FILTER_A2, 0.0f);

/**
 * coefficients of the butterworth filter used for smoothing the IMU
 * 
 * @unit 
 * @min -100
 * @max 100
 * @decimal 4
 * @increment 0.0001
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(FILTER_B1, 0.0f);

/**
 * coefficients of the butterworth filter used for smoothing the IMU
 * 
 * @unit 
 * @min -100
 * @max 100
 * @decimal 4
 * @increment 0.0001
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(FILTER_B2, 0.0f);

/**
 * coefficients of the butterworth filter used for smoothing the IMU
 * 
 * @unit 
 * @min -100
 * @max 100
 * @decimal 4
 * @increment 0.0001
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(FILTER_B3, 0.0f);


