/**
 * GPS Yaw Baseline Length Accuracy Requirement
 *
 * Discard any GPS yaw data if the baseline length deviates from the expected length by more than this value.
 * Expected length is calculated from the SENS_GNSSREL_P{X,Y,Z} parameters.
 * A value of 0 disables the gate. (This is not recommended.)
 *
 * @group Sensors
 * @unit m
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(SENS_GNSSHDG_LRQ, 0.1f);

/**
 * Required carrier phase range solution for GPS Yaw
 *
 * @value 0 None
 * @value 1 Floating or Fixed
 * @value 2 Fixed
 * @group Sensors
 * @min 0
 * @max 2
 */
PARAM_DEFINE_INT32(SENS_GNSSHDG_CRQ, 2);
