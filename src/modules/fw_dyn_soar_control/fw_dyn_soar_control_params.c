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
 * inertia tensor term in body xz-axis (roll-yaw coupling)
 *
 * This is the inertia of the aircraft, used for the INDI
 *
 * @unit kg
 * @min -0.5
 * @max 0
 * @decimal 2
 * @increment 0.01
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_INERTIA_RP, -0.0f);

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
 * estimated sideslip sensitivity coefficients used for wind estimation
 *
 * Used as F_y = 0.5 * DS_RHO * ASPD^2 * DS_C_B1,
 * where alpha is the angle of attack.
 *
 * @unit
 * @min -100
 * @max -0.01
 * @decimal 4
 * @increment 0.0001
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_C_B1, -3.32f);

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
PARAM_DEFINE_FLOAT(DS_AOA_OFFSET, 0.07f);

/**
 * stall speed of the aircraft
 *
 * @unit m/s
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
PARAM_DEFINE_FLOAT(DS_LIN_C_X, 1.0f);

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
PARAM_DEFINE_FLOAT(DS_LIN_C_Y, 1.0f);

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
PARAM_DEFINE_FLOAT(DS_LIN_C_Z, 1.0f);

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
 * @max 100
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_ROT_K_ROLL, 30.0f);

/**
 * control gain of attitude PD-controller (body pitch-direction)
 *
 * @unit
 * @min 0
 * @max 100
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_ROT_K_PITCH, 30.0f);

/**
 * rudder turn coordination FF-gain
 *
 * @unit
 * @min 0
 * @max 100
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_ROT_FF_YAW, 1.0f);

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
PARAM_DEFINE_FLOAT(DS_ROT_C_ROLL, 1.0f);

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
PARAM_DEFINE_FLOAT(DS_ROT_C_PITCH, 1.0f);

/**
 * rudder turn coordination P-gain
 *
 * @unit
 * @min 0
 * @max 100
 * @decimal 2
 * @increment 0.01
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_ROT_P_YAW, 1.0f);


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
 * @max 7
 * @decimal 1
 * @increment 1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_INT32(DS_LOITER, 0);

// ======================================================
// ============ wind shear heading ======================
// ======================================================

/**
 * integer defining wind heading
 *
 * @unit deg
 * @min -180
 * @max 180
 * @decimal 1
 * @increment 1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_INT32(DS_W_HEADING, 0);

// ======================================================
// ======= wind shear height in soaring frame ===========
// ======================================================

/**
 * float defining wind shear vertical postition in soaring frame
 *
 * @unit
 * @min 0
 * @max 100
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_W_HEIGHT, 100.f);

// ==============================================
// ============== engine thrust =================
// ==============================================
/**
 * float in [0,1] corresponding to the engine thrust
 *
 * @unit
 * @min 0
 * @max 1
 * @decimal 2
 * @increment 0.01
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_THRUST, 0);

// ======================================================
// ============= controller force saturation ============
// ======================================================

/**
 * integer in {0,1} defining if the commanded force has an upper bound (saturates), 0=no saturation, 1=saturation
 *
 * @unit
 * @min 0
 * @max 1
 * @decimal 1
 * @increment 1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_INT32(DS_SWITCH_SAT, 1);


// ======================================================
// ============= hardcoded trajectory center ============
// ======================================================

/**
 * integer in {0,1} defining if the trajectory origin is taken from hardcoded params or shear estimate, 1=params, 0=estimate
 * @unit
 * @min 0
 * @max 1
 * @decimal 1
 * @increment 1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_INT32(DS_SWITCH_ORI_HC, 1);

// =================================================
// ============= loiter trajectory test ============
// =================================================

/**
 * integer in {0,1} defining if the loiter circle defined by param DS_LOITER shall be used, 0=soaring, 1=loiter
 * @unit
 * @min 0
 * @max 1
 * @decimal 1
 * @increment 1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_INT32(DS_SWITCH_LOITER, 1);

// ====================================================
// ============= manual feedthrough switch ============
// ====================================================

/**
 * integer in {0,1} defining if we are using manual feedthrough, only used in SITL mode
 * @unit
 * @min 0
 * @max 1
 * @decimal 1
 * @increment 1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_INT32(DS_SWITCH_MANUAL, 1);

// =====================================================
// ============= open loop / closed loop DS ============
// =====================================================

/**
 * integer in {0,1} defining if the shear parameters are changed by the estimator while soaring (closed loop). 0=fixed shear, 1=changing shear
 * @unit
 * @min 0
 * @max 1
 * @decimal 1
 * @increment 1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_INT32(DS_SWITCH_CLOOP, 0);

// =====================================================
// ============= open loop / closed loop DS ============
// =====================================================

/**
 * integer in {0,1} defining if we are running the controller in SITL. 0=hardware, 1=sitl
 * @unit
 * @min 0
 * @max 1
 * @decimal 1
 * @increment 1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_INT32(DS_SWITCH_SITL, 0);

/**
 * float in [0,1] corresponding to the engine thrust
 *
 * @unit
 * @min 0
 * @max 2000
 * @decimal 2
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_WINDEST_HGHT, 0);

/**
 * float in [0,1] corresponding to the engine thrust
 *
 * @unit
 * @min -90
 * @max 90
 * @decimal 2
 * @increment 0.01
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_WINDEST_HDG, 0);

/**
 * float in [0,1] corresponding to the engine thrust
 *
 * @unit
 * @min 0
 * @max 1
 * @decimal 2
 * @increment 0.01
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_WINDEST_ALPHA, 0);

/**
 * float in [0,1] corresponding to the engine thrust
 *
 * @unit
 * @min 0
 * @max 1
 * @decimal 2
 * @increment 0.01
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(DS_WINDEST_VMAX, 0);
