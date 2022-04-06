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
 * This is the mass of the aircraft, used for the INDI
 * @unit kg
 */
PARAM_DEFINE_FLOAT(FW_MASS, 1.4f);

/**
 * total wing area used for lift and drag computation
 * @unit m2
 */
PARAM_DEFINE_FLOAT(FW_WING_AREA, 0.4f);

/**
 * total wing area used for lift and drag computation
 * @unit m2
 */
PARAM_DEFINE_FLOAT(FW_WING_AREA, 0.4f);

/**
 * air density used for lift and drag computation
 * @unit kg/m3
 */
PARAM_DEFINE_FLOAT(RHO, 1.223f);

/**
 * estimated lift coefficients used for lift and drag computation
 * Used as L = C_l0 + C_l1*alpha,
 * where alpha is the angle of attack.
 * @unit ()
 */
PARAM_DEFINE_FLOAT(C_l0, 0.356f);
PARAM_DEFINE_FLOAT(C_l1, 2.354f);


/**
 * estimated drag coefficients used for lift and drag computation
 * Used as D = C_d0 + C_d1*alpha + C_d2*alpha**2,
 * where alpha is the angle of attack.
 * @unit ()
 */
PARAM_DEFINE_FLOAT(C_d0, 0.0288f);
PARAM_DEFINE_FLOAT(C_d1, 0.3783f);
PARAM_DEFINE_FLOAT(C_d2, 1.984f);

/**
 * air density used for lift and drag computation
 * @unit kg/m3
 */
PARAM_DEFINE_FLOAT(RHO, 1.223f);

/**
 * coefficients of the butterworth filter used for smoothing the IMU
 * @unit ()
 */
PARAM_DEFINE_FLOAT(FILTER_A1, 0.0f);
PARAM_DEFINE_FLOAT(FILTER_A2, 0.0f);
PARAM_DEFINE_FLOAT(FILTER_B1, 0.0f);
PARAM_DEFINE_FLOAT(FILTER_B2, 0.0f);
PARAM_DEFINE_FLOAT(FILTER_B3, 0.0f);


