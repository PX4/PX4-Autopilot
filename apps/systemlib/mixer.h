/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef _SYSTEMLIB_MIXER_H
#define _SYSTEMLIB_MIXER_H

/**
 * @file mixer.h
 * 
 * Generic control value mixing library.
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

struct MixScaler {
	unsigned	control;	/**< control consumed by this scaler */
	float		negative_scale;	/**< scale for inputs < 0 */
	float		positive_scale;	/**< scale for inputs > 0 */
	float		offset;		/**< bias applied to output */
	float		lower_limit;	/**< minimum output value */
	float		upper_limit;	/**< maximum output value */
};

struct MixMixer {
	unsigned		control_count;	/**< number of control scalers */
	struct MixScaler	output_scaler;	/**< scaler applied to mixer output */
	struct MixScaler	control_scaler[0]; /**< array of control scalers */
};

/**
 * Handy macro for determining the allocation size of a mixer.
 */
#define MIXER_SIZE(_num_scalers)	(sizeof(struct MixMixer) + ((_num_scalers) * sizeof(struct MixScaler)))

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
 * @param mixer			The mixer configuration to be checked.
 * @param control_count		The number of controls in the system.
 * @return			Zero if the mixer configuration is sane,
 *				nonzero otherwise.
 */
__EXPORT int	mixer_check(struct MixMixer *mixer, unsigned control_count);

/**
 * Read a mixer definition from a file.
 *
 * A mixer definition is a text representation of the configuration of a
 * mixer.  The definition consists of a single-line header indicating the
 * number of scalers and then one line defining each scaler.  The first
 * scaler in the file is always the output scaler, followed by the input
 * scalers.
 *
 * M: <scaler count>
 * S: <control> <negative_scale> <positive_scale> <offset> <lower_limit> <upper_limit>
 * S: ...
 *
 * The <control> value for the output scaler is ignored by the mixer.
 *
 * Multiple mixer definitions may be stored in a single file; it is assumed that
 * the reader will know how many to expect and read accordingly. 
 *
 * A mixer entry with a scaler count of zero indicates a disabled mixer. This
 * will return NULL for the mixer when processed by this function, and will be
 * generated by passing NULL as the mixer to mixer_save.
 *
 * @param fd			The file to read the definitions from.
 * @param mixer			Mixer is returned here.
 * @return			1 if a mixer was read, zero on EOF or negative on error.
 */
__EXPORT int	mixer_load(int fd, struct MixMixer **mixer);

/**
 * Save a mixer definition to a file.
 *
 * @param fd			The file to write the definitions to.
 * @param mixer			The mixer definition to save.
 * @return			Zero on success, negative on error.
 */
__EXPORT int	mixer_save(int fd, struct MixMixer *mixers);


__END_DECLS

#endif /* _SYSTEMLIB_MIXER_H */
