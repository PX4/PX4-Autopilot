/**
 * @file	Generic control value mixing library.
 *
 * This library implements a generic mixer function that can be used
 * by any driver or subsytem that wants to combine several control signals
 * into a single output.
 *
 * Terminology
 * ===========
 *
 * control
 *	A mixer input value, typically provided by some controlling
 *	component of the system.
 *
 * actuator
 *	The mixer output value.
 *
 * Mixing basics
 * =============
 *
 * An actuator derives its value from the combination of one or more
 * control values. Each of the control values is scaled according to
 * the actuator's configuration and then combined to produce the 
 * actuator value, which may then be further scaled to suit the specific
 * output type.
 *
 * Internally, all scaling is performed using floating point values.
 * Inputs and outputs are clamped to the range -1.0 to 1.0.
 *
 * control    control   control
 *    |          |         |
 *    v          v         v
 *  scale      scale     scale
 *    |          |         |
 *    |          v         |
 *    +-------> mix <------+
 *               |
 *             scale
 *               |
 *               v
 *              out
 *
 * Scaling
 * -------
 *
 * Each scaler allows the input value to be scaled independently for
 * inputs greater/less than zero. An offset can be applied to the output,
 * as well as lower and upper boundary constraints.
 * Negative scaling factors cause the output to be inverted (negative input
 * produces positive output).
 *
 * Scaler pseudocode:
 *
 * if (input < 0)
 *     output = (input * NEGATIVE_SCALE) + OFFSET
 * else
 *     output = (input * POSITIVE_SCALE) + OFFSET
 *
 * if (output < LOWER_LIMIT)
 *     output = LOWER_LIMIT
 * if (output > UPPER_LIMIT)
 *     output = UPPER_LIMIT
 *
 *
 * Mixing
 * ------
 *
 * Mixing is performed by summing the scaled control values.  
 *
 *
 * Controls
 * --------
 *
 * Each mixer is presented with an array of controls from which it
 * selects the set that will be mixed for each actuator.
 *
 * The precise assignment of controls may vary depending on the 
 * application, but the following assignments should be used
 * when appropriate.
 *
 * control | standard meaning
 * --------+-----------------------
 *     0   | roll
 *     1   | pitch
 *     2   | yaw
 *     3   | primary thrust
 */

 struct MixScaler
 {
 	unsigned	control;	/**< control consumed by this scaler */
 	float		negative_scale;	/**< scale for inputs < 0 */
 	float		positive_scale;	/**< scale for inputs > 0 */
 	float		offset;		/**< bias applied to output */
 	float		lower_limit;	/**< minimum output value */
 	float		upper_limit;	/**< maximum output value */
 };

 struct MixMixer
 {
 	unsigned		control_count;	/**< number of control scalers */
 	struct MixScaler	output_scaler;	/**< scaler applied to mixer output */
 	struct MixScaler	control_scaler[0]; /**< array of control scalers */
 };

__BEGIN_DECLS

/**
 * Perform a mixer calculation.
 *
 * Note that the controls array is assumed to be sufficiently large for any control
 * index in the mixer.
 *
 * @param mixer			Mixer configuration.
 * @param controls		Array of input control values.
 * @return			The mixed output.
 */
 __EXPORT float	mixer_mix(struct MixMixer *mixer, float *controls);

 /**
  * Check a mixer configuration for sanity.
  *
  * @param mixer		The mixer configuration to be checked.
  * @param control_count	The number of controls in the system.
  * @return			Zero if the mixer configuration is sane,
  *				nonzero otherwise.
  */
 __EXPORT int	mixer_check(struct MixMixer *mixer, unsigned control_count);

__END_DECLS
