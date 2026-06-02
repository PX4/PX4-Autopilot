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
PARAM_DEFINE_INT32(FORMIC_WDEV_INIT, 5);

/**
 * User VIO activation AUX channel.
 *
 * Selects which RC AUX channel lets the user control activation of the
 * vision odometry (VIO). When set to "None", no AUX channel is used and
 * the VIO is treated as active all the time.
 *
 * @value 0 None (VIO always active)
 * @value 1 AUX1
 * @value 2 AUX2
 * @value 3 AUX3
 * @value 4 AUX4
 * @value 5 AUX5
 * @value 6 AUX6
 * @group Formic Watchdog EV
 * @category Standard
 */
PARAM_DEFINE_INT32(FORMIC_WDEV_AUX, 0);

