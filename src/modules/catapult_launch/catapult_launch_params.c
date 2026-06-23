/****************************************************************************
 *
 * Catapult Launch Automation Parameters
 *
 ****************************************************************************/

/**
 * Catapult launch sequence enable
 *
 * Enables the catapult launch automation sequence.
 * When enabled, the module monitors IMU acceleration for launch detection
 * and controls tail lock servos (MAIN5/6) and propulsion motor (MAIN3).
 *
 * @boolean
 * @reboot_required true
 * @group Catapult Launch
 */
PARAM_DEFINE_INT32(CAT_EN, 0);

/**
 * Catapult launch trigger mode
 *
 * Selects the trigger source for launch detection.
 * 0: Manual (via RC failsafe switch only)
 * 1: IMU acceleration (default)
 * 2: Airspeed (future)
 * 3: Combined IMU + airspeed (future)
 *
 * @min 0
 * @max 3
 * @value 0 Manual
 * @value 1 IMU acceleration
 * @value 2 Airspeed (future)
 * @value 3 Combined (future)
 * @group Catapult Launch
 */
PARAM_DEFINE_INT32(CAT_TRIG_MODE, 1);

/**
 * IMU launch detection acceleration threshold
 *
 * Launch is detected when the selected body-axis acceleration exceeds
 * this value (in G) for at least CAT_ACC_HOLD_MS milliseconds.
 * 4.0 G = 39.2 m/s^2.
 *
 * @min 0.0
 * @max 20.0
 * @decimal 1
 * @increment 0.5
 * @group Catapult Launch
 */
PARAM_DEFINE_FLOAT(CAT_ACC_THR_G, 4.0f);

/**
 * IMU launch detection axis
 *
 * Selects which acceleration axis is used for launch detection.
 * 0: Magnitude (sqrt(x^2+y^2+z^2))
 * 1: Body X axis (forward, default for tube launch)
 * 2: Body Y axis
 * 3: Body Z axis
 * 4: Vehicle forward estimate (future)
 *
 * @min 0
 * @max 4
 * @value 0 Magnitude
 * @value 1 Body X (forward)
 * @value 2 Body Y
 * @value 3 Body Z
 * @value 4 Forward estimate (future)
 * @group Catapult Launch
 */
PARAM_DEFINE_INT32(CAT_ACC_AXIS, 1);

/**
 * IMU launch detection minimum hold time
 *
 * The acceleration must exceed CAT_ACC_THR_G for at least this many
 * milliseconds before launch is confirmed.
 *
 * @unit ms
 * @min 0
 * @max 500
 * @group Catapult Launch
 */
PARAM_DEFINE_INT32(CAT_ACC_HOLD_MS, 20);

/**
 * Tail lock release delay after launch detection
 *
 * Time after launch detection (T0) before MAIN5/MAIN6 move to release PWM.
 *
 * @unit s
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.05
 * @group Catapult Launch
 */
PARAM_DEFINE_FLOAT(CAT_TAIL_DLY, 1.0f);

/**
 * Motor start delay after launch detection
 *
 * Time after launch detection (T0) before motor start is permitted.
 * When equal to CAT_TAIL_DLY, tail release executes first in the same cycle.
 *
 * @unit s
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.05
 * @group Catapult Launch
 */
PARAM_DEFINE_FLOAT(CAT_MOT_DLY, 1.0f);

/**
 * Motor auto-start enable
 *
 * Enables automatic motor start after launch detection.
 * When disabled (0), MAIN3 is not commanded by the catapult sequence.
 * All RC/MAVLink gates must also be satisfied when enabled.
 *
 * @boolean
 * @group Catapult Launch
 */
PARAM_DEFINE_INT32(CAT_MOT_AUTO, 0);

/**
 * Motor auto-start RC gate channel
 *
 * RC channel used as an additional gate for motor auto-start permission.
 * Set to -1 to disable RC gate (channel not required).
 * Channel is assignable after vehicle setup.
 *
 * @min -1
 * @max 18
 * @group Catapult Launch
 */
PARAM_DEFINE_INT32(CAT_MOT_RC_CH, -1);

/**
 * Motor auto-start RC gate threshold
 *
 * Normalized RC channel value above which the RC gate is considered active.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 2
 * @group Catapult Launch
 */
PARAM_DEFINE_FLOAT(CAT_MOT_RC_THR, 0.5f);

/**
 * Motor auto-start RC gate minimum hold time
 *
 * RC channel must exceed CAT_MOT_RC_THR for at least this many milliseconds.
 *
 * @unit ms
 * @min 0
 * @max 3000
 * @group Catapult Launch
 */
PARAM_DEFINE_INT32(CAT_MOT_RC_HOLD, 300);

/**
 * Motor auto-start MAVLink gate enable
 *
 * When enabled, MAVLink commands can grant or revoke motor auto-start permission.
 * When disabled, the MAVLink gate is always satisfied (non-blocking).
 *
 * @boolean
 * @group Catapult Launch
 */
PARAM_DEFINE_INT32(CAT_MAV_EN, 1);

/**
 * Motor auto-start QGC gate enable
 *
 * When enabled, QGC parameter changes to CAT_MOT_AUTO are respected.
 *
 * @boolean
 * @group Catapult Launch
 */
PARAM_DEFINE_INT32(CAT_QGC_EN, 1);

/**
 * MAIN5 tail lock servo PWM (locked position)
 *
 * PWM value commanding the tail lock hold position for MAIN5.
 * Adjust per airframe. Range 1000-2000 us.
 *
 * @unit us
 * @min 1000
 * @max 2000
 * @group Catapult Launch
 */
PARAM_DEFINE_INT32(CAT_TAIL5_LOCK, 1000);

/**
 * MAIN5 tail lock servo PWM (released position)
 *
 * PWM value commanding the tail lock release position for MAIN5.
 * Adjust per airframe. Range 1000-2000 us.
 *
 * @unit us
 * @min 1000
 * @max 2000
 * @group Catapult Launch
 */
PARAM_DEFINE_INT32(CAT_TAIL5_REL, 2000);

/**
 * MAIN6 tail lock servo PWM (locked position)
 *
 * PWM value commanding the tail lock hold position for MAIN6.
 * Adjust per airframe. Can be opposite of MAIN5 for reversed servo direction.
 *
 * @unit us
 * @min 1000
 * @max 2000
 * @group Catapult Launch
 */
PARAM_DEFINE_INT32(CAT_TAIL6_LOCK, 1000);

/**
 * MAIN6 tail lock servo PWM (released position)
 *
 * PWM value commanding the tail lock release position for MAIN6.
 * Adjust per airframe. Can be opposite of MAIN5 for reversed servo direction.
 *
 * @unit us
 * @min 1000
 * @max 2000
 * @group Catapult Launch
 */
PARAM_DEFINE_INT32(CAT_TAIL6_REL, 2000);

/**
 * Motor stop PWM override
 *
 * PWM value used as motor stop command. Set to 0 to use the system disarm PWM
 * (recommended, typically 1000 us for standard ESC).
 *
 * @unit us
 * @min 0
 * @max 2200
 * @group Catapult Launch
 */
PARAM_DEFINE_INT32(CAT_MOT_STOP_PWM, 0);

/**
 * Motor start PWM
 *
 * PWM value used to command MAIN3 motor start.
 * Verify actual motor response with ESC calibration before flight.
 *
 * @unit us
 * @min 1000
 * @max 2000
 * @group Catapult Launch
 */
PARAM_DEFINE_INT32(CAT_MOT_STRT_PWM, 1500);

/**
 * Motor start ramp time
 *
 * Time to ramp from stop PWM to start PWM.
 *
 * @unit s
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.05
 * @group Catapult Launch
 */
PARAM_DEFINE_FLOAT(CAT_MOT_RAMP, 0.5f);

/**
 * Propulsion motor output channel (MAIN index)
 *
 * Physical MAIN output channel for the propulsion motor.
 * This corresponds to the Peripheral_via_Actuator_Set index in QGC.
 *
 * @min 1
 * @max 16
 * @group Catapult Launch
 */
PARAM_DEFINE_INT32(CAT_MAIN3_CH, 3);

/**
 * Tail servo 1 output channel (MAIN index)
 *
 * Physical MAIN output channel for tail lock servo 1.
 * In QGC, assign this MAIN pin to Peripheral_via_Actuator_Set1.
 *
 * @min 1
 * @max 16
 * @group Catapult Launch
 */
PARAM_DEFINE_INT32(CAT_MAIN5_CH, 5);

/**
 * Tail servo 2 output channel (MAIN index)
 *
 * Physical MAIN output channel for tail lock servo 2.
 * In QGC, assign this MAIN pin to Peripheral_via_Actuator_Set2.
 *
 * @min 1
 * @max 16
 * @group Catapult Launch
 */
PARAM_DEFINE_INT32(CAT_MAIN6_CH, 6);

/**
 * Require tail release before motor start
 *
 * When enabled, motor auto-start is blocked until tail lock servos have
 * moved to the release position.
 *
 * @boolean
 * @group Catapult Launch
 */
PARAM_DEFINE_INT32(CAT_MOT_REQ_TAIL, 1);

/**
 * Abort sequence on PX4 failsafe
 *
 * When enabled, the catapult sequence is aborted if vehicle_status.failsafe
 * becomes true.
 *
 * @boolean
 * @group Catapult Launch
 */
PARAM_DEFINE_INT32(CAT_ABORT_FS, 1);

/**
 * Manual failsafe RC channel
 *
 * RC channel used to trigger manual tail release + Stabilized mode transition
 * when IMU launch detection fails. Set to -1 to disable.
 * Channel is assignable after vehicle setup.
 *
 * @min -1
 * @max 18
 * @group Catapult Launch
 */
PARAM_DEFINE_INT32(CAT_FS_RC_CH, -1);

/**
 * Manual failsafe RC threshold
 *
 * Normalized RC channel value above which the failsafe switch is active.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 2
 * @group Catapult Launch
 */
PARAM_DEFINE_FLOAT(CAT_FS_RC_THR, 0.5f);

/**
 * Manual failsafe RC minimum hold time
 *
 * Failsafe switch must exceed CAT_FS_RC_THR for at least this many milliseconds
 * to prevent inadvertent activation.
 *
 * @unit ms
 * @min 0
 * @max 3000
 * @group Catapult Launch
 */
PARAM_DEFINE_INT32(CAT_FS_RC_HOLD, 500);

/**
 * Manual failsafe: request Stabilized mode transition
 *
 * When enabled, the module requests a transition to Stabilized mode
 * when manual failsafe is triggered. Transition may be rejected by Commander.
 * Tail release is maintained regardless of transition result.
 *
 * @boolean
 * @group Catapult Launch
 */
PARAM_DEFINE_INT32(CAT_FS_TO_STAB, 1);

/**
 * Manual failsafe: allow motor auto-start
 *
 * When enabled, motor auto-start is permitted during manual failsafe.
 * Default is disabled (safe side).
 *
 * @boolean
 * @group Catapult Launch
 */
PARAM_DEFINE_INT32(CAT_FS_MOT_EN, 0);
