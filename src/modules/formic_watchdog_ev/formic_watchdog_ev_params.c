#include <px4_platform_common/px4_config.h>
#include <parameters/param.h>

/**
 * Enable the Formic Watchdog EV module.
 *
 * @min 0
 * @max 1
 * @group Formic Watchdog EV
 * @category Standard
 * @value 0 Disabled
 * @value 1 Enabled
 */
PARAM_DEFINE_INT32(FORMIC_WDEV_EN, 0);



/**
 * EV initialization delay.
 *
 * Time to wait after startup before the external vision (EV)
 * watchdog begins monitoring / declares the EV stream initialized.
 *
 * @min 0
 * @max 60
 * @unit s
 * @group Formic Watchdog EV
 * @category Standard
 */
PARAM_DEFINE_INT32(FORMIC_WDEV_INIT, 2);


/**
 * EV heading reset threshold.
 *
 * Minimum heading difference (radians) between the local estimate and the
 * EV yaw observation required to trigger a heading reset counter increment.
 * When the difference is smaller than this value the heading is considered
 * aligned and no reset is counted for this session. The reset is still
 * throttled to at most one increment every 3 seconds.
 *
 * @min 0.0
 * @max 3.14
 * @unit rad
 * @group Formic Watchdog EV
 * @category Standard
 */
PARAM_DEFINE_FLOAT(FORMIC_WDEV_DYAW, 0.5f);

/**
 * EV position reset threshold.
 *
 * Minimum horizontal distance (meters) between the local position estimate and
 * the EV position observation required to keep the session in the reset phase.
 * When the distance is smaller than this value the position is considered
 * aligned with the EV. The reset phase ends only once BOTH the heading
 * (FORMIC_WDEV_DYAW) and the position are aligned. Reset increments are still
 * throttled to at most one every 2 seconds.
 *
 * @min 0.0
 * @max 50.0
 * @unit m
 * @group Formic Watchdog EV
 * @category Standard
 */
PARAM_DEFINE_FLOAT(FORMIC_WDEV_DPOS, 1.0f);

/**
 * EV init velocity-average threshold.
 *
 * Maximum allowed mean EV 3D speed (||vx,vy,vz||) accumulated during the
 * FORMIC_WDEV_INIT settle window. If the average exceeds this value the EV
 * data is considered unreliable, the init check fails and the EV stream is
 * not forwarded for this session.
 *
 * @min 0.0
 * @max 50.0
 * @unit m/s
 * @group Formic Watchdog EV
 * @category Standard
 */
PARAM_DEFINE_FLOAT(FORMIC_WDEV_VINI, 10.0f);




/**
 * EV roll/pitch alignment threshold.
 *
 * Maximum allowed difference (degrees) between the EV (VIO) attitude and the
 * local estimate in roll and pitch. When the difference exceeds this value the
 * EV attitude is considered misaligned with the drone and the data is treated
 * as unreliable for this session.
 *
 * @min 0.0
 * @max 50.0
 * @unit deg
 * @group Formic Watchdog EV
 * @category Standard
 */
PARAM_DEFINE_FLOAT(FORMIC_WDEV_DATT, 25.0f);





/**
 * User VIO init quality value.
 *
 * This parameter defines the initial quality value for the vision odometry (VIO) system.
 *
 * @min 0
 * @max 100
 * @unit %
 * @decimal 0
 * @increment 1
 * @group Formic Watchdog EV
 * @category Standard
 */
PARAM_DEFINE_INT32(FORMIC_WDEV_QV, 95);




/**
 * EV position max dist from init.
 *
 * Maximum allowed distance (meters) between the current position and the
 * initial position. If the distance exceeds this value, the EV data is
 * considered unreliable, and the init check fails for this session.
 *
 * @min 0.0
 * @max 100.0
 * @unit m
 * @group Formic Watchdog EV
 * @category Standard
 */
PARAM_DEFINE_FLOAT(FORMIC_WDEV_DIP, 1.0f);


