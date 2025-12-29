/**
 * @file winch_params.c
 * EayLoad Winch Driver Parameters
 */

/**
 * Enable EayLoad Winch Driver
 *
 * @boolean
 * @reboot_required true
 * @group Winch
 */
PARAM_DEFINE_INT32(WINCH_EN, 0);

/**
 * Winch Device Address
 *
 * Modbus device address of the winch (default 0x01)
 *
 * @min 1
 * @max 255
 * @group Winch
 */
PARAM_DEFINE_INT32(WINCH_ADDR, 1);

/**
 * Winch Communication Mode
 *
 * 0: Polling mode (request status periodically)
 * 1: Fixed frame mode (winch sends status at 10Hz automatically)
 *
 * @min 0
 * @max 1
 * @group Winch
 */
PARAM_DEFINE_INT32(WINCH_MODE, 1);
