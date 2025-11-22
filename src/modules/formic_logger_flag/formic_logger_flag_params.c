#include <px4_platform_common/px4_config.h>
#include <parameters/param.h>

/**
 * RC channel assigned to Formic Logger Flag module.
 *
 * Set via QGroundControl so the module knows which RC channel
 * should trigger its behavior (1-18, 0 disables).
 *
 * @min 0
 * @max 18
 * @value 0 Disabled
 * @group Formic Debug
 */
PARAM_DEFINE_INT32(FLF_RC_CH, 0);

