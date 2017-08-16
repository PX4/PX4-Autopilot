#include <systemlib/param/param.h>

/**
 * Satellite radio read interval
 *
 * @unit s
 * @min 0
 * @max 300
 * @group Iridium SBD
 */
PARAM_DEFINE_INT32(ISBD_READINT, 10);
