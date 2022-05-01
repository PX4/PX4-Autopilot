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
PARAM_DEFINE_FLOAT(FW_INERTIA_ROLL, 0.197563f);

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
PARAM_DEFINE_FLOAT(FW_INERTIA_PITCH, 0.1458929f);

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
PARAM_DEFINE_FLOAT(FW_INERTIA_YAW, 0.1477f);



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
PARAM_DEFINE_FLOAT(C_L0, 0.356f);

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
PARAM_DEFINE_FLOAT(C_L1, 2.354f);


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
PARAM_DEFINE_FLOAT(C_D0, 0.0288f);

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
PARAM_DEFINE_FLOAT(C_D1, 0.3783f);

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
PARAM_DEFINE_FLOAT(C_D2, 1.984f);

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
PARAM_DEFINE_FLOAT(AOA_OFFSET, 0.01f);

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
PARAM_DEFINE_FLOAT(FILTER_A1, 1.16826067f);

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
PARAM_DEFINE_FLOAT(FILTER_A2, -0.42411821f);

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
PARAM_DEFINE_FLOAT(FILTER_B1, 0.06396438f);

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
PARAM_DEFINE_FLOAT(FILTER_B2, 0.12792877f);

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
PARAM_DEFINE_FLOAT(FILTER_B3, 0.06396438f);

// ========================================================
// =================== CONTROL GAINS ======================
// ========================================================
/**
 * roll gain of K_x (position error gain)
 * 
 * @unit 
 * @min -10
 * @max 10
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(K_X_ROLL, 1.0f);

/**
 * pitch gain of K_x (position error gain)
 * 
 * @unit 
 * @min -10
 * @max 10
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(K_X_PITCH, 1.0f);

/**
 * yaw gain of K_x (position error gain)
 * 
 * @unit 
 * @min -10
 * @max 10
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(K_X_YAW, 1.0f);

/**
 * roll gain of K_v (velocity error gain)
 * 
 * @unit 
 * @min -10
 * @max 10
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(K_V_ROLL, 1.0f);

/**
 * pitch gain of K_v (velocity error gain)
 * 
 * @unit 
 * @min -10
 * @max 10
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(K_V_PITCH, 1.0f);

/**
 * yaw gain of K_v (velocity error gain)
 * 
 * @unit 
 * @min -10
 * @max 10
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(K_V_YAW, 1.0f);

/**
 * roll gain of K_A (acceleration error gain)
 * 
 * @unit 
 * @min -10
 * @max 10
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(K_A_ROLL, 0.5f);

/**
 * pitch gain of K_A (acceleration error gain)
 * 
 * @unit 
 * @min -10
 * @max 10
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(K_A_PITCH, 0.5f);

/**
 * yaw gain of K_A (acceleration error gain)
 * 
 * @unit 
 * @min -10
 * @max 10
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(K_A_YAW, 0.5f);

/**
 * roll gain of K_Q (attitude error gain)
 * 
 * @unit 
 * @min 0
 * @max 20
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(K_Q_ROLL, 10.0f);

/**
 * pitch gain of K_Q (attitude error gain)
 * 
 * @unit 
 * @min 0
 * @max 20
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(K_Q_PITCH, 10.0f);

/**
 * yaw gain of K_Q (attitude error gain)
 * 
 * @unit 
 * @min 0
 * @max 20
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(K_Q_YAW, 5.0f);

/**
 * roll gain of K_W (angular velocity error gain)
 * 
 * @unit 
 * @min 0
 * @max 20
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(K_W_ROLL, 5.0f);

/**
 * pitch gain of K_W (angular velocity error gain)
 * 
 * @unit 
 * @min 0
 * @max 20
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(K_W_PITCH, 5.0f);

/**
 * yaw gain of K_W (angular velocity error gain)
 * 
 * @unit 
 * @min 0
 * @max 20
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(K_W_YAW, 5.0f);

/**
 * roll gain of K_ACT (actuator deflection gain)
 * 
 * @unit 
 * @min 0
 * @max 100
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(K_ACT_ROLL, 0.5f);

/**
 * pitch gain of K_ACT (actuator deflection gain)
 * 
 * @unit 
 * @min 0
 * @max 100
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(K_ACT_PITCH, 0.1f);

/**
 * yaw gain of K_ACT (actuator deflection gain)
 * 
 * @unit 
 * @min 0
 * @max 100
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(K_ACT_YAW, 0.1f);

// ===================================================
// ==============  trajectory center =================
// ===================================================

/**
 * latitude of trajectory start point (WGS84)
 * 
 * @unit 
 * @min -180
 * @max 180
 * @decimal 7
 * @increment 0.0000001
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(ORIGIN_LAT, 47.39797f);

/**
 * longitude of trajectory start point (WGS84)
 * 
 * @unit 
 * @min -90
 * @max 90
 * @decimal 7
 * @increment 0.0000001
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(ORIGIN_LON, 8.54554f);

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
PARAM_DEFINE_FLOAT(ORIGIN_ALT, 488.0f);
