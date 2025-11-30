#include <px4_platform_common/px4_config.h>
#include <parameters/param.h>

/**
 * RC channel assigned to Formic Logger Flag module need to take a pop-up click at the rc .
 *
 * Set via QGroundControl so the module knows which RC channel
 * should trigger its behavior (1-18, 0 disables).
 *
 * @min 0
 * @max 18
 * @group Formic Debug
 * @category Standard
 * @value 0 Disabled
//  * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(FORMIC_LG_CH, 1);



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

