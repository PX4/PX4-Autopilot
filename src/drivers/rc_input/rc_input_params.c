/**
 * ADC Device ID for Analogue RC RSSI
 *
 * Which ADC backend the RSSI comes from.
 * Invalid device ID will disable this function.
 *
 * @group RC Input
 */
PARAM_DEFINE_INT32(RC_RSSI_DEVID, -1);

/**
 * ADC Channel for RC RSSI
 *
 * Which ADC channel the RSSI comes from.
 * A value of -1 disables RSSI function.
 *
 * @group RC Input
 */
PARAM_DEFINE_INT32(RC_RSSI_CH, -1);