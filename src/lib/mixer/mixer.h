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

/**
 * @file mixer.h
 *
 * Generic, programmable, procedural control signal mixers.
 *
 * This library implements a generic mixer interface that can be used
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
 * Mixing behaviour varies based on the specific mixer class; each
 * mixer class describes its behaviour in more detail.
 *
 *
 * Controls
 * --------
 *
 * The precise assignment of controls may vary depending on the
 * application, but the following assignments should be used
 * when appropriate.  Some mixer classes have specific assumptions
 * about the assignment of controls.
 *
 * control | standard meaning
 * --------+-----------------------
 *     0   | roll
 *     1   | pitch
 *     2   | yaw
 *     3   | primary thrust
 */


#pragma once

#include <stdint.h>

/** simple channel scaler */
struct mixer_scaler_s {
	float			negative_scale;
	float			positive_scale;
	float			offset;
	float			min_output;
	float			max_output;
};

/** mixer input */
struct mixer_control_s {
	uint8_t			control_group;	/**< group from which the input reads */
	uint8_t			control_index;	/**< index within the control group */
	struct mixer_scaler_s 	scaler;		/**< scaling applied to the input before use */
};

/** simple mixer */
struct mixer_simple_s {
	uint8_t			control_count;	/**< number of inputs */
	struct mixer_scaler_s	output_scaler;	/**< scaling for the output */
	struct mixer_control_s	controls[0];	/**< actual size of the array is set by control_count */
};

#define MIXER_SIMPLE_SIZE(_icount)	(sizeof(struct mixer_simple_s) + (_icount) * sizeof(struct mixer_control_s))


/**
 * Abstract class defining a mixer mixing zero or more inputs to
 * one or more outputs.
 */
class Mixer
{
public:
	enum class Airmode : int32_t {
		disabled = 0,
		roll_pitch = 1,
		roll_pitch_yaw = 2
	};

	/** next mixer in a list */
	Mixer				*_next;

	/**
	 * Fetch a control value.
	 *
	 * @param handle		Token passed when the callback is registered.
	 * @param control_group		The group to fetch the control from.
	 * @param control_index		The group-relative index to fetch the control from.
	 * @param control		The returned control
	 * @return			Zero if the value was fetched, nonzero otherwise.
	 */
	typedef int	(* ControlCallback)(uintptr_t handle,
					    uint8_t control_group,
					    uint8_t control_index,
					    float &control);

	/**
	 * Constructor.
	 *
	 * @param control_cb		Callback invoked when reading controls.
	 */
	Mixer(ControlCallback control_cb, uintptr_t cb_handle);
	virtual ~Mixer() {}

	/**
	 * Perform the mixing function.
	 *
	 * @param outputs		Array into which mixed output(s) should be placed.
	 * @param space			The number of available entries in the output array;
	 * @return			The number of entries in the output array that were populated.
	 */
	virtual unsigned		mix(float *outputs, unsigned space) = 0;

	/**
	 * Get the saturation status.
	 *
	 * @return			Integer bitmask containing saturation_status from multirotor_motor_limits.msg.
	 */
	virtual uint16_t		get_saturation_status(void) = 0;

	/**
	 * Analyses the mix configuration and updates a bitmask of groups
	 * that are required.
	 *
	 * @param groups		A bitmask of groups (0-31) that the mixer requires.
	 */
	virtual void			groups_required(uint32_t &groups) = 0;

	/**
	 * @brief      Empty method, only implemented for MultirotorMixer and MixerGroup class.
	 *
	 * @param[in]  delta_out_max  Maximum delta output.
	 *
	 */
	virtual void 			set_max_delta_out_once(float delta_out_max) {}

	/**
	 * @brief Set trim offset for this mixer
	 *
	 * @return the number of outputs this mixer feeds to
	 */
	virtual unsigned set_trim(float trim) = 0;

	/**
	 * @brief Get trim offset for this mixer
	 *
	 * @return the number of outputs this mixer feeds to
	 */
	virtual unsigned get_trim(float *trim) = 0;

	/*
	 * @brief      Sets the thrust factor used to calculate mapping from desired thrust to pwm.
	 *
	 * @param[in]  val   The value
	 */
	virtual void 			set_thrust_factor(float val) {}

	/**
	 * @brief Set airmode. Airmode allows the mixer to increase the total thrust in order to unsaturate the motors.
	 *
	 * @param[in]  airmode   Select airmode type (0 = disabled, 1 = roll/pitch, 2 = roll/pitch/yaw)
	 */
	virtual void set_airmode(Airmode airmode) {};

protected:
	/** client-supplied callback used when fetching control values */
	ControlCallback			_control_cb;
	uintptr_t			_cb_handle;

	/**
	 * Invoke the client callback to fetch a control value.
	 *
	 * @param group			Control group to fetch from.
	 * @param index			Control index to fetch.
	 * @return			The control value.
	 */
	float				get_control(uint8_t group, uint8_t index);

	/**
	 * Perform simpler linear scaling.
	 *
	 * @param scaler		The scaler configuration.
	 * @param input			The value to be scaled.
	 * @return			The scaled value.
	 */
	static float			scale(const mixer_scaler_s &scaler, float input);

	/**
	 * Validate a scaler
	 *
	 * @param scaler		The scaler to be validated.
	 * @return			Zero if good, nonzero otherwise.
	 */
	static int			scale_check(struct mixer_scaler_s &scaler);

	/**
	 * Find a tag
	 *
	 * @param buf			The buffer to operate on.
	 * @param buflen		length of the buffer.
	 * @param tag			character to search for.
	 */
	static const char 		*findtag(const char *buf, unsigned &buflen, char tag);

	/**
	 * Find next tag and return it (0 is returned if no tag is found)
	 *
	 * @param buf			The buffer to operate on.
	 * @param buflen		length of the buffer.
	 */
	static char 			findnexttag(const char *buf, unsigned buflen);

	/**
	 * Skip a line
	 *
	 * @param buf			The buffer to operate on.
	 * @param buflen		length of the buffer.
	 * @return			0 / OK if a line could be skipped, 1 else
	 */
	static const char 		*skipline(const char *buf, unsigned &buflen);

	/**
	 * Check wether the string is well formed and suitable for parsing
	 */
	static bool				string_well_formed(const char *buf, unsigned &buflen);

private:

	/* do not allow to copy due to pointer data members */
	Mixer(const Mixer &);
	Mixer &operator=(const Mixer &);
};

/**
 * Group of mixers, built up from single mixers and processed
 * in order when mixing.
 */
class MixerGroup : public Mixer
{
public:
	MixerGroup(ControlCallback control_cb, uintptr_t cb_handle);
	~MixerGroup();

	unsigned		mix(float *outputs, unsigned space) override;
	uint16_t		get_saturation_status(void) override;
	void			groups_required(uint32_t &groups) override;

	/**
	 * Add a mixer to the group.
	 *
	 * @param mixer			The mixer to be added.
	 */
	void				add_mixer(Mixer *mixer);

	/**
	 * Remove all the mixers from the group.
	 */
	void				reset();

	/**
	 * Count the mixers in the group.
	 */
	unsigned			count();

	/**
	 * Adds mixers to the group based on a text description in a buffer.
	 *
	 * Mixer definitions begin with a single capital letter and a colon.
	 * The actual format of the mixer definition varies with the individual
	 * mixers; they are summarised here, but see ROMFS/mixers/README for
	 * more details.
	 *
	 * Null Mixer
	 * ..........
	 *
	 * The null mixer definition has the form:
	 *
	 *   Z:
	 *
	 * Simple Mixer
	 * ............
	 *
	 * A simple mixer definition begins with:
	 *
	 *   M: <control count>
	 *   O: <-ve scale> <+ve scale> <offset> <lower limit> <upper limit>
	 *
	 * The second line O: can be omitted. In that case 'O: 10000 10000 0 -10000 10000' is used.
	 * The definition continues with <control count> entries describing the control
	 * inputs and their scaling, in the form:
	 *
	 *   S: <group> <index> <-ve scale> <+ve scale> <offset> <lower limit> <upper limit>
	 *
	 * Multirotor Mixer
	 * ................
	 *
	 * The multirotor mixer definition is a single line of the form:
	 *
	 * R: <geometry> <roll scale> <pitch scale> <yaw scale> <deadband>
	 *
	 * Helicopter Mixer
	 * ................
	 *
	 * The helicopter mixer includes throttle and pitch curves
	 *
	 * H: <swash plate servo count>
	 * T: <0> <2500> <5000> <7500> <10000>
	 * P: <-10000> <-5000> <0> <5000> <10000>
	 *
	 * The definition continues with <swash plate servo count> entries describing
	 * the position of the servo, in the following form:
	 *
	 *   S: <angle (deg)> <normalized arm length> <scale> <offset> <lower limit> <upper limit>
	 *
	 * @param buf			The mixer configuration buffer.
	 * @param buflen		The length of the buffer, updated to reflect
	 *				bytes as they are consumed.
	 * @return			Zero on successful load, nonzero otherwise.
	 */
	int				load_from_buf(const char *buf, unsigned &buflen);

	/**
	 * @brief      Update slew rate parameter. This tells instances of the class MultirotorMixer
	 *             the maximum allowed change of the output values per cycle.
	 *             The value is only valid for one cycle, in order to have continuous
	 *             slew rate limiting this function needs to be called before every call
	 *             to mix().
	 *
	 * @param[in]  delta_out_max  Maximum delta output.
	 *
	 */
	void 			set_max_delta_out_once(float delta_out_max) override;

	/*
	 * Invoke the set_offset method of each mixer in the group
	 * for each value in page r_page_servo_control_trim
	 */
	unsigned set_trims(int16_t *v, unsigned n);

	unsigned set_trim(float trim) override
	{
		return 0;
	}

	unsigned get_trims(int16_t *values);

	unsigned get_trim(float *trim) override
	{
		return 0;
	}

	/**
	 * @brief      Sets the thrust factor used to calculate mapping from desired thrust to pwm.
	 *
	 * @param[in]  val   The value
	 */
	void	set_thrust_factor(float val) override;

	void 	set_airmode(Airmode airmode) override;

private:
	Mixer				*_first;	/**< linked list of mixers */

	/* do not allow to copy due to pointer data members */
	MixerGroup(const MixerGroup &);
	MixerGroup operator=(const MixerGroup &);
};

/**
 * Null mixer; returns zero.
 *
 * Used as a placeholder for output channels that are unassigned in groups.
 */
class NullMixer : public Mixer
{
public:
	NullMixer();
	~NullMixer() {}

	/**
	 * Factory method.
	 *
	 * Given a pointer to a buffer containing a text description of the mixer,
	 * returns a pointer to a new instance of the mixer.
	 *
	 * @param buf			Buffer containing a text description of
	 *				the mixer.
	 * @param buflen		Length of the buffer in bytes, adjusted
	 *				to reflect the bytes consumed.
	 * @return			A new NullMixer instance, or nullptr
	 *				if the text format is bad.
	 */
	static NullMixer		*from_text(const char *buf, unsigned &buflen);

	unsigned		mix(float *outputs, unsigned space) override;
	uint16_t		get_saturation_status(void) override;
	void			groups_required(uint32_t &groups) override;
	unsigned set_trim(float trim) override
	{
		return 1;
	}

	unsigned get_trim(float *trim) override
	{
		return 1;
	}

};

/**
 * Simple summing mixer.
 *
 * Collects zero or more inputs and mixes them to a single output.
 */
class SimpleMixer : public Mixer
{
public:
	/**
	 * Constructor
	 *
	 * @param mixinfo		Mixer configuration.  The pointer passed
	 *				becomes the property of the mixer and
	 *				will be freed when the mixer is deleted.
	 */
	SimpleMixer(ControlCallback control_cb,
		    uintptr_t cb_handle,
		    mixer_simple_s *mixinfo);
	~SimpleMixer();

	/**
	 * Factory method with full external configuration.
	 *
	 * Given a pointer to a buffer containing a text description of the mixer,
	 * returns a pointer to a new instance of the mixer.
	 *
	 * @param control_cb		The callback to invoke when fetching a
	 *				control value.
	 * @param cb_handle		Handle passed to the control callback.
	 * @param buf			Buffer containing a text description of
	 *				the mixer.
	 * @param buflen		Length of the buffer in bytes, adjusted
	 *				to reflect the bytes consumed.
	 * @return			A new SimpleMixer instance, or nullptr
	 *				if the text format is bad.
	 */
	static SimpleMixer		*from_text(Mixer::ControlCallback control_cb,
			uintptr_t cb_handle,
			const char *buf,
			unsigned &buflen);

	/**
	 * Factory method for PWM/PPM input to internal float representation.
	 *
	 * @param control_cb		The callback to invoke when fetching a
	 *				control value.
	 * @param cb_handle		Handle passed to the control callback.
	 * @param input			The control index used when fetching the input.
	 * @param min			The PWM/PPM value considered to be "minimum" (gives -1.0 out)
	 * @param mid			The PWM/PPM value considered to be the midpoint (gives 0.0 out)
	 * @param max			The PWM/PPM value considered to be "maximum" (gives 1.0 out)
	 * @return			A new SimpleMixer instance, or nullptr if one could not be
	 *				allocated.
	 */
	static SimpleMixer		*pwm_input(Mixer::ControlCallback control_cb, uintptr_t cb_handle, unsigned input, uint16_t min,
			uint16_t mid, uint16_t max);

	unsigned		mix(float *outputs, unsigned space) override;
	uint16_t		get_saturation_status(void) override;
	void			groups_required(uint32_t &groups) override;

	/**
	 * Check that the mixer configuration as loaded is sensible.
	 *
	 * Note that this function will call control_cb, but only cares about
	 * error returns, not the input value.
	 *
	 * @return			Zero if the mixer makes sense, nonzero otherwise.
	 */
	int				check();

	unsigned set_trim(float trim) override;

	unsigned get_trim(float *trim) override;

protected:

private:
	mixer_simple_s			*_pinfo;

	static int			parse_output_scaler(const char *buf, unsigned &buflen, mixer_scaler_s &scaler);
	static int			parse_control_scaler(const char *buf,
			unsigned &buflen,
			mixer_scaler_s &scaler,
			uint8_t &control_group,
			uint8_t &control_index);

	/* do not allow to copy due to ptr data members */
	SimpleMixer(const SimpleMixer &);
	SimpleMixer operator=(const SimpleMixer &);
};

/**
 * Supported multirotor geometries.
 *
 * Values are generated by the px_generate_mixers.py script and placed to mixer_multirotor_normalized.generated.h
 */
typedef unsigned int MultirotorGeometryUnderlyingType;
enum class MultirotorGeometry : MultirotorGeometryUnderlyingType;

/**
 * Multi-rotor mixer for pre-defined vehicle geometries.
 *
 * Collects four inputs (roll, pitch, yaw, thrust) and mixes them to
 * a set of outputs based on the configured geometry.
 */
class MultirotorMixer : public Mixer
{
public:
	/**

	 * Precalculated rotor mix.
	 */
	struct Rotor {
		float	roll_scale;		/**< scales roll for this rotor */
		float	pitch_scale;	/**< scales pitch for this rotor */
		float	yaw_scale;		/**< scales yaw for this rotor */
		float	thrust_scale;	/**< scales thrust for this rotor */
	};

	/**
	 * Constructor.
	 *
	 * @param control_cb		Callback invoked to read inputs.
	 * @param cb_handle		Passed to control_cb.
	 * @param geometry		The selected geometry.
	 * @param roll_scale		Scaling factor applied to roll inputs
	 *				compared to thrust.
	 * @param pitch_scale		Scaling factor applied to pitch inputs
	 *				compared to thrust.
	 * @param yaw_wcale		Scaling factor applied to yaw inputs compared
	 *				to thrust.
	 * @param idle_speed		Minimum rotor control output value; usually
	 *				tuned to ensure that rotors never stall at the
	 * 				low end of their control range.
	 */
	MultirotorMixer(ControlCallback control_cb,
			uintptr_t cb_handle,
			MultirotorGeometry geometry,
			float roll_scale,
			float pitch_scale,
			float yaw_scale,
			float idle_speed);

	/**
	 * Constructor (for testing).
	 *
	 * @param control_cb		Callback invoked to read inputs.
	 * @param cb_handle		Passed to control_cb.
	 * @param rotors		control allocation matrix
	 * @param rotor_count		length of rotors array (= number of motors)
	 */
	MultirotorMixer(ControlCallback control_cb,
			uintptr_t cb_handle,
			Rotor *rotors,
			unsigned rotor_count);

	~MultirotorMixer();

	/**
	 * Factory method.
	 *
	 * Given a pointer to a buffer containing a text description of the mixer,
	 * returns a pointer to a new instance of the mixer.
	 *
	 * @param control_cb		The callback to invoke when fetching a
	 *				control value.
	 * @param cb_handle		Handle passed to the control callback.
	 * @param buf			Buffer containing a text description of
	 *				the mixer.
	 * @param buflen		Length of the buffer in bytes, adjusted
	 *				to reflect the bytes consumed.
	 * @return			A new MultirotorMixer instance, or nullptr
	 *				if the text format is bad.
	 */
	static MultirotorMixer		*from_text(Mixer::ControlCallback control_cb, uintptr_t cb_handle, const char *buf,
			unsigned &buflen);

	unsigned		mix(float *outputs, unsigned space) override;
	uint16_t		get_saturation_status(void) override;
	void			groups_required(uint32_t &groups) override;

	/**
	 * @brief      Update slew rate parameter. This tells the multicopter mixer
	 *             the maximum allowed change of the output values per cycle.
	 *             The value is only valid for one cycle, in order to have continuous
	 *             slew rate limiting this function needs to be called before every call
	 *             to mix().
	 *
	 * @param[in]  delta_out_max  Maximum delta output.
	 *
	 */
	void 			set_max_delta_out_once(float delta_out_max) override { _delta_out_max = delta_out_max; }

	unsigned set_trim(float trim) override
	{
		return _rotor_count;
	}

	unsigned get_trim(float *trim) override
	{
		return _rotor_count;
	}

	/**
	 * @brief      Sets the thrust factor used to calculate mapping from desired thrust to pwm.
	 *
	 * @param[in]  val   The value
	 */
	void			set_thrust_factor(float val) override { _thrust_factor = val; }

	void 			set_airmode(Airmode airmode) override;

	union saturation_status {
		struct {
			uint16_t valid		: 1; // 0 - true when the saturation status is used
			uint16_t motor_pos	: 1; // 1 - true when any motor has saturated in the positive direction
			uint16_t motor_neg	: 1; // 2 - true when any motor has saturated in the negative direction
			uint16_t roll_pos	: 1; // 3 - true when a positive roll demand change will increase saturation
			uint16_t roll_neg	: 1; // 4 - true when a negative roll demand change will increase saturation
			uint16_t pitch_pos	: 1; // 5 - true when a positive pitch demand change will increase saturation
			uint16_t pitch_neg	: 1; // 6 - true when a negative pitch demand change will increase saturation
			uint16_t yaw_pos	: 1; // 7 - true when a positive yaw demand change will increase saturation
			uint16_t yaw_neg	: 1; // 8 - true when a negative yaw demand change will increase saturation
			uint16_t thrust_pos	: 1; // 9 - true when a positive thrust demand change will increase saturation
			uint16_t thrust_neg	: 1; //10 - true when a negative thrust demand change will increase saturation
		} flags;
		uint16_t value;
	};

private:
	/**
	 * Computes the gain k by which desaturation_vector has to be multiplied
	 * in order to unsaturate the output that has the greatest saturation.
	 * @see also minimize_saturation().
	 *
	 * @return desaturation gain
	 */
	float compute_desaturation_gain(const float *desaturation_vector, const float *outputs, saturation_status &sat_status,
					float min_output, float max_output) const;

	/**
	 * Minimize the saturation of the actuators by adding or substracting a fraction of desaturation_vector.
	 * desaturation_vector is the vector that added to the output outputs, modifies the thrust or angular
	 * acceleration on a specific axis.
	 * For example, if desaturation_vector is given to slide along the vertical thrust axis (thrust_scale), the
	 * saturation will be minimized by shifting the vertical thrust setpoint, without changing the
	 * roll/pitch/yaw accelerations.
	 *
	 * Note that as we only slide along the given axis, in extreme cases outputs can still contain values
	 * outside of [min_output, max_output].
	 *
	 * @param desaturation_vector vector that is added to the outputs, e.g. thrust_scale
	 * @param outputs output vector that is modified
	 * @param sat_status saturation status output
	 * @param min_output minimum desired value in outputs
	 * @param max_output maximum desired value in outputs
	 * @param reduce_only if true, only allow to reduce (substract) a fraction of desaturation_vector
	 */
	void minimize_saturation(const float *desaturation_vector, float *outputs, saturation_status &sat_status,
				 float min_output = 0.f, float max_output = 1.f, bool reduce_only = false) const;

	/**
	 * Mix roll, pitch, yaw, thrust and set the outputs vector.
	 *
	 * Desaturation behavior: airmode for roll/pitch:
	 * thrust is increased/decreased as much as required to meet the demanded roll/pitch.
	 * Yaw is not allowed to increase the thrust, @see mix_yaw() for the exact behavior.
	 */
	inline void mix_airmode_rp(float roll, float pitch, float yaw, float thrust, float *outputs);

	/**
	 * Mix roll, pitch, yaw, thrust and set the outputs vector.
	 *
	 * Desaturation behavior: full airmode for roll/pitch/yaw:
	 * thrust is increased/decreased as much as required to meet demanded the roll/pitch/yaw.
	 */
	inline void mix_airmode_rpy(float roll, float pitch, float yaw, float thrust, float *outputs);

	/**
	 * Mix roll, pitch, yaw, thrust and set the outputs vector.
	 *
	 * Desaturation behavior: no airmode, thrust is NEVER increased to meet the demanded
	 * roll/pitch/yaw. Instead roll/pitch/yaw is reduced as much as needed.
	 * Thrust can be reduced to unsaturate the upper side.
	 * @see mix_yaw() for the exact yaw behavior.
	 */
	inline void mix_airmode_disabled(float roll, float pitch, float yaw, float thrust, float *outputs);

	/**
	 * Mix yaw by updating an existing output vector (that already contains roll/pitch/thrust).
	 *
	 * Desaturation behavior: thrust is allowed to be decreased up to 15% in order to allow
	 * some yaw control on the upper end. On the lower end thrust will never be increased,
	 * but yaw is decreased as much as required.
	 *
	 * @param yaw demanded yaw
	 * @param outputs output vector that is updated
	 */
	inline void mix_yaw(float yaw, float *outputs);

	void update_saturation_status(unsigned index, bool clipping_high, bool clipping_low_roll_pitch, bool clipping_low_yaw);

	float				_roll_scale;
	float				_pitch_scale;
	float				_yaw_scale;
	float				_idle_speed;
	float 				_delta_out_max;
	float 				_thrust_factor;

	Airmode				_airmode;

	saturation_status _saturation_status;

	unsigned			_rotor_count;
	const Rotor			*_rotors;

	float 				*_outputs_prev = nullptr;
	float 				*_tmp_array = nullptr;

	/* do not allow to copy due to ptr data members */
	MultirotorMixer(const MultirotorMixer &);
	MultirotorMixer operator=(const MultirotorMixer &);
};

/** helicopter swash servo mixer */
struct mixer_heli_servo_s {
	float angle;
	float arm_length;
	float scale;
	float offset;
	float min_output;
	float max_output;
};

#define HELI_CURVES_NR_POINTS 5

/** helicopter swash plate mixer */
struct mixer_heli_s {
	uint8_t				control_count;	/**< number of inputs */
	float				throttle_curve[HELI_CURVES_NR_POINTS];
	float				pitch_curve[HELI_CURVES_NR_POINTS];
	struct mixer_heli_servo_s	servos[4];	/**< up to four inputs */
};

/**
 * Generic helicopter mixer for helicopters with swash plate.
 *
 * Collects four inputs (roll, pitch, yaw, thrust) and mixes them to servo commands
 * for swash plate tilting and throttle- and pitch curves.
 */
class HelicopterMixer : public Mixer
{
public:
	/**
	 * Constructor.
	 *
	 * @param control_cb		Callback invoked to read inputs.
	 * @param cb_handle		Passed to control_cb.
	 * @param mixer_info		Pointer to heli mixer configuration
	 */
	HelicopterMixer(ControlCallback control_cb,
			uintptr_t cb_handle,
			mixer_heli_s *mixer_info);

	~HelicopterMixer() = default;

	/**
	 * Factory method.
	 *
	 * Given a pointer to a buffer containing a text description of the mixer,
	 * returns a pointer to a new instance of the mixer.
	 *
	 * @param control_cb		The callback to invoke when fetching a
	 *				control value.
	 * @param cb_handle		Handle passed to the control callback.
	 * @param buf			Buffer containing a text description of
	 *				the mixer.
	 * @param buflen		Length of the buffer in bytes, adjusted
	 *				to reflect the bytes consumed.
	 * @return			A new HelicopterMixer instance, or nullptr
	 *				if the text format is bad.
	 */
	static HelicopterMixer		*from_text(Mixer::ControlCallback control_cb, uintptr_t cb_handle, const char *buf,
			unsigned &buflen);

	unsigned		mix(float *outputs, unsigned space) override;
	void			groups_required(uint32_t &groups) override;

	uint16_t		get_saturation_status(void) override { return 0; }

	unsigned set_trim(float trim) override
	{
		return 4;
	}

	unsigned get_trim(float *trim) override
	{
		return 4;
	}

private:
	mixer_heli_s			_mixer_info;

	/* do not allow to copy */
	HelicopterMixer(const HelicopterMixer &);
	HelicopterMixer operator=(const HelicopterMixer &);
};
