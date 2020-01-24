/**
 * PCF8583 rotorfreq (i2c) pool interval
 *
 * @reboot_required true
 * @group Sensors
 * @unit us
 */
PARAM_DEFINE_INT32(PCF8583_POOL, 1000000);

/**
 * PCF8583 rotorfreq (i2c) i2c address
 *
 * @reboot_required true
 * @group Sensors
 * @value 80 0x50
 * @value 81 0x51
 */
PARAM_DEFINE_INT32(PCF8583_ADDR, 80);

/**
 * PCF8583 rotorfreq (i2c) counter reset value
 *
 * Internal device counter is reset to 0 when overun this value
 *
 * @reboot_required true
 * @group Sensors
 * @value 0 - reset avter every measurement
 */
PARAM_DEFINE_INT32(PCF8583_RESET, 0);

/**
 * PCF8583 rotorfreq (i2c) magnet count
 *
 * Nmumber of signals per rotation of rotor
 *
 * @reboot_required true
 * @min 1
 */
PARAM_DEFINE_INT32(PCF8583_MAGNET, 2);
