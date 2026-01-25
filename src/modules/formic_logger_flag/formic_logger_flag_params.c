#include <px4_platform_common/px4_config.h>
#include <parameters/param.h>

/**
 * Aux channel assigned to Formic Logger Flag module need to take a pop-up click at the manual control.
 *
 * Set via QGroundControl so the module knows which aux channel
 * should trigger its behavior (1-6, 0 disables).
 *
 * @min 0
 * @max 6
 * @group Formic Debug
 * @category Standard
 * @value 0 Disabled
 * @value 1 Aux 1
 * @value 2 Aux 2
 * @value 3 Aux 3
 * @value 4 Aux 4
 * @value 5 Aux 5
 * @value 6 Aux 6
 */
PARAM_DEFINE_INT32(FORMIC_LG_AUX, 1);



/**

 * This parameter sets the time duration of the pop-up click.
 *
 * @unit ms
 * @min 0
 * @max 1
 * @decimal 1
 * @reboot_required true
 * @group Formic Logger Flag
 */
PARAM_DEFINE_FLOAT(FORMIC_LG_INVERT, 0);

