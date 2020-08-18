/**
 * Scale factor for ADC to convert raw to units
 */
PARAM_DEFINE_FLOAT(ADC1_SCALE, 1.0);
PARAM_DEFINE_FLOAT(ADC2_SCALE, 1.0);
PARAM_DEFINE_FLOAT(ADC3_SCALE, 1.0);
PARAM_DEFINE_FLOAT(ADC4_SCALE, 1.0);

/**
 * Unit type:
 * 0: none
 * 1: mV
 * 2: mA
 * 3: cK
 *
 * @min 0
 * @max 3
 * @group Analog Measurement
 */
PARAM_DEFINE_INT32(ADC1_UNIT, 0);
PARAM_DEFINE_INT32(ADC2_UNIT, 0);
PARAM_DEFINE_INT32(ADC3_UNIT, 0);
PARAM_DEFINE_INT32(ADC4_UNIT, 0);


