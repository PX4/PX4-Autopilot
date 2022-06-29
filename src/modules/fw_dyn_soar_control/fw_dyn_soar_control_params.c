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
 * inertia around body x-axis
 * 
 * This is the inertia of the aircraft, used for the INDI
 * 
 * @unit kg
 * @min 0.01
 * @max 10
 * @decimal 2
 * @increment 0.01
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_INERTIA_ROLL, 0.197563f);

/**
 * inertia around body y-axis
 * 
 * This is the inertia of the aircraft, used for the INDI
 * 
 * @unit kg
 * @min 0.01
 * @max 10
 * @decimal 2
 * @increment 0.01
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_INERTIA_PITCH, 0.1458929f);

/**
 * inertia around body z-axis
 * 
 * This is the inertia of the aircraft, used for the INDI
 * 
 * @unit kg
 * @min 0.01
 * @max 10
 * @decimal 2
 * @increment 0.01
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_INERTIA_YAW, 0.1477f);

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
PARAM_DEFINE_FLOAT(DS_MASS, 1.4f);

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
PARAM_DEFINE_FLOAT(DS_WING_AREA, 0.4f);


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
PARAM_DEFINE_FLOAT(DS_RHO, 1.223f);

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
PARAM_DEFINE_FLOAT(DS_C_L0, 0.356f);

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
PARAM_DEFINE_FLOAT(DS_C_L1, 2.354f);


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
PARAM_DEFINE_FLOAT(DS_C_D0, 0.0288f);

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
PARAM_DEFINE_FLOAT(DS_C_D1, 0.3783f);

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
PARAM_DEFINE_FLOAT(DS_C_D2, 1.984f);

/**
 * offset angle between body frame (Pixhawk) and the wing chord
 * 
 * Used to compute the AoA
 * 
 * @unit rad
 * @min 0
 * @max 0.1
 * @decimal 2
 * @increment 0.01
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_AOA_OFFSET, 0.03f);

/**
 * stall speed of the aircraft
 * 
 * @unit rad
 * @min 5
 * @max 10
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_STALL_SPEED, 9.0f);


// ========================================================
// =================== CONTROL GAINS ======================
// ========================================================
/**
 * control gain of position PD-controller (body x-direction)
 * 
 * @unit 
 * @min 0
 * @max 100
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_LIN_K_X, 1.0f);

/**
 * control gain of position PD-controller (body y-direction)
 * 
 * @unit 
 * @min 0
 * @max 100
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_LIN_K_Y, 1.0f);

/**
 * control gain of position PD-controller (body z-direction)
 * 
 * @unit 
 * @min 0
 * @max 100
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_LIN_K_Z, 1.0f);

/**
 * normalized damping coefficient of position PD-controller (body x-direction)
 * 
 * @unit 
 * @min 0
 * @max 2
 * @decimal 2
 * @increment 0.01
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_LIN_C_X, 0.75f);

/**
 * normalized damping coefficient of position PD-controller (body y-direction)
 * 
 * @unit 
 * @min 0
 * @max 2
 * @decimal 2
 * @increment 0.01
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_LIN_C_Y, 0.75f);

/**
 * normalized damping coefficient of position PD-controller (body z-direction)
 * 
* @unit 
 * @min 0
 * @max 2
 * @decimal 2
 * @increment 0.01
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_LIN_C_Z, 0.75f);

/**
 * acceleration feedback gain of position PD-controller (body x-direction)
 * 
 * @unit 
 * @min 0
 * @max 10
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_LIN_FF_X, 0.5f);

/**
 * acceleration feedback gain of position PD-controller (body y-direction)
 * 
 * @unit 
 * @min 0
 * @max 10
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_LIN_FF_Y, 0.5f);

/**
 * acceleration feedback gain of position PD-controller (body z-direction)
 * 
 * @unit 
 * @min 0
 * @max 10
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_LIN_FF_Z, 0.5f);

/**
 * control gain of attitude PD-controller (body roll-direction)
 * 
 * @unit 
 * @min 0
 * @max 20
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_ROT_K_ROLL, 10.0f);

/**
 * control gain of attitude PD-controller (body pitch-direction)
 * 
 * @unit 
 * @min 0
 * @max 20
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_ROT_K_PITCH, 10.0f);


/**
 * normalized damping coefficient of attitude PD-controller (body roll-direction)
 * 
 * @unit 
 * @min 0
 * @max 2
 * @decimal 2
 * @increment 0.01
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_ROT_C_ROLL, 0.75f);

/**
 * normalized damping coefficient of attitude PD-controller (body pitch-direction)
 * 
 * @unit 
 * @min 0
 * @max 2
 * @decimal 2
 * @increment 0.01
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_ROT_C_PITCH, 0.75f);


// =============================
// low level INDI control params
// =============================

/**
 * roll gain of K_ACT (actuator deflection gain)
 * 
 * @unit 
 * @min 0
 * @max 100
 * @decimal 3
 * @increment 0.001
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_K_ACT_ROLL, 0.25f);

/**
 * pitch gain of K_ACT (actuator deflection gain)
 * 
 * @unit 
 * @min 0
 * @max 100
 * @decimal 2
 * @increment 0.01
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_K_ACT_PITCH, 0.05f);

/**
 * yaw gain of K_ACT (actuator deflection gain)
 * 
 * @unit 
 * @min 0
 * @max 100
 * @decimal 4
 * @increment 0.0001
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_K_ACT_YAW, 0.1f);

/**
 * roll gain of K_ACT_DAMPING (actuator damping gain)
 * 
 * @unit 
 * @min 0
 * @max 100
 * @decimal 3
 * @increment 0.001
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_K_DAMP_ROLL, 0.04f);

/**
 * pitch gain of K_ACT_DAMPING (actuator damping gain)
 * 
 * @unit 
 * @min 0
 * @max 100
 * @decimal 3
 * @increment 0.001
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_K_DAMP_PITCH, 0.02f);

/**
 * yaw gain of K_ACT_DAMPING (actuator damping gain)
 * 
 * @unit 
 * @min 0
 * @max 100
 * @decimal 4
 * @increment 0.0001
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_K_DAMP_YAW, 0.0f);

// ===================================================
// ==============  trajectory center =================
// ===================================================

/**
 * latitude of trajectory start point (WGS84)
 * 
 * @unit 
 * @min -90
 * @max 90
 * @decimal 7
 * @increment 0.0000001
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_ORIGIN_LAT, 47.3130000f);

/**
 * longitude of trajectory start point (WGS84)
 * 
 * @unit 
 * @min -180
 * @max 180
 * @decimal 7
 * @increment 0.0000001
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_ORIGIN_LON, 8.8100000f);

/**
 * altitude of trajectory start point (WGS84)
 * 
 * @unit 
 * @min 0
 * @max 2000
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_ORIGIN_ALT, 537.0f);

// ======================================================
// ==============  loiter circle number =================
// ======================================================

/**
 * integer in {0,1,2,3,4,5} defining the loiter trajectory
 * 
 * @unit 
 * @min 0
 * @max 5
 * @decimal 1
 * @increment 1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_INT32(DS_LOITER, 0);

// ==============================================
// ============== engine thrust =================
// ==============================================
/**
 * float in [0,1] corresponding to the engine thrust
 * 
 * @unit 
 * @min 0
 * @max 1
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_THRUST, 0);

// ======================================================
// ================ Stick feedthorugh ===================
// ======================================================

/**
 * integer in {0,1} defining if manual attitude setpoints are commanded by the pilot, 0=DS-controller, 1=manual feedthrough
 * 
 * @unit 
 * @min 0
 * @max 1
 * @decimal 1
 * @increment 1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_INT32(DS_SWITCH_MANUAL, 0);