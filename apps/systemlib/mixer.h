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
 * control value
 *	A mixer input value, typically provided by some controlling
 *	component of the system.
 *
 * control group
 * 	A collection of controls provided by a single controlling component.
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

struct scaler_s {
	float		negative_scale;	/**< scale for inputs < 0 */
	float		positive_scale;	/**< scale for inputs > 0 */
	float		offset;		/**< bias applied to output */
	float		lower_limit;	/**< minimum output value */
	float		upper_limit;	/**< maximum output value */
	uint8_t		control_group;	/**< control group this scaler reads from */
	uint8_t		control_index;	/**< control index within the group */
};

struct mixer_s {
	unsigned	control_count;	/**< number of control scalers */
	struct scaler_s	output_scaler;	/**< scaler applied to mixer output */
	struct scaler_s	control_scaler[0]; /**< array of control scalers */
};

/**
 * Handy macro for determining the allocation size of a mixer.
 */
#define MIXER_SIZE(_num_scalers)	(sizeof(struct mixer_s) + ((_num_scalers) * sizeof(struct scaler_s)))

__BEGIN_DECLS

/**
 * Perform a mixer calculation.
 *
 * Note that the controls array, and the arrays it indexes, are assumed
 * to be sufficiently large for any control index in the mixer.
 *
 * @param mixer			Mixer configuration.
 * @param controls		Array of pointers to control group values.
 * @return			The mixed output.
 */
__EXPORT float	mixer_mix(struct mixer_s *mixer, float **controls);

/**
 * Check a mixer configuration for sanity.
 *
 * @param mixer			The mixer configuration to be checked.
 * @param group_count		The highest-numbered control group that
 *				should be considered legal.
 * @param control_count		The highest control index that should be
 *				considered legal.
 * @return			Zero if the mixer configuration is sane,
 *				nonzero otherwise.
 */
__EXPORT int	mixer_check(struct mixer_s *mixer, unsigned group_count, unsigned control_count);

/**
 * Evaluate the control inputs to a mixer and update the bitmask of
 * required control groups.
 *
 * This function allows an actuator driver to selectively fetch just
 * the control groups required to support a particular mixer or set of
 * mixers.
 *
 * @param mixer			The mixer being evaluated.
 * @param groups		Pointer to a bitmask to be updated with set bits
 *				corresponding to the control groups used by the
 *				mixer.
 */
__EXPORT void	mixer_requires(struct mixer_s *mixer, uint32_t *groups);

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
 * S: <control group> <control index> <negative_scale*> <positive_scale*> <offset*> <lower_limit*> <upper_limit*>
 * S: ...
 *
 * The <control ...> values for the output scaler are ignored by the mixer.
 *
 * Values marked * are integers representing floating point values; values are
 * scaled by 10000 on load/save.
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
__EXPORT int	mixer_load(int fd, struct mixer_s **mixer);

/**
 * Save a mixer definition to a file.
 *
 * @param fd			The file to write the definitions to.
 * @param mixer			The mixer definition to save.
 * @return			Zero on success, negative on error.
 */
__EXPORT int	mixer_save(int fd, struct mixer_s *mixers);


__END_DECLS

#endif /* _SYSTEMLIB_MIXER_H */
