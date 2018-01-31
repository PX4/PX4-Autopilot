#include <systemlib/param/param.h>

/**
 * Satellite radio read interval
 *
 * @unit s
 * @min 0
 * @max 300
 * @group Iridium SBD
 */
PARAM_DEFINE_INT32(ISBD_READ_INT, 60);

/**
 * Iridium SBD session timeout
 *
 * @unit s
 * @min 0
 * @max 300
 * @group Iridium SBD
 */
PARAM_DEFINE_INT32(ISBD_SBD_TIMEOUT, 60);

/**
 * Time [ms] the Iridium driver will wait for additional mavlink messages to combine them into one SBD message
 * Value 0 turns the functionality off
 *
 * @unit ms
 * @min 0
 * @max 500
 * @group Iridium SBD
 */
PARAM_DEFINE_INT32(ISBD_STACK_TIME, 0);
