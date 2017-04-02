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
 * @file drv_mixer.h
 *
 * Mixer ioctl interfaces.
 *
 * Normal workflow is:
 *
 * - open mixer device
 * - add mixer(s)
 * loop:
 *  - mix actuators to array
 *
 * Each client has its own configuration.
 *
 * When mixing, outputs are produced by mixers in the order they are
 * added.  A simple mixer produces one output; a multirotor mixer will
 * produce several outputs, etc.
 */

#ifndef _DRV_MIXER_H
#define _DRV_MIXER_H

#include <px4_defines.h>
#include <stdint.h>
#include <sys/ioctl.h>

#define MIXER0_DEVICE_PATH		"/dev/mixer0"

/*
 * ioctl() definitions
 */
#define _MIXERIOCBASE		(0x2500)
#define _MIXERIOC(_n)		(_PX4_IOC(_MIXERIOCBASE, _n))

/** get the number of mixable outputs */
#define MIXERIOCGETOUTPUTCOUNT	_MIXERIOC(0)

/** reset (clear) the mixer configuration */
#define MIXERIOCRESET		_MIXERIOC(1)

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
 * add a simple mixer in (struct mixer_simple_s *)arg
 */
#define MIXERIOCADDSIMPLE	_MIXERIOC(2)

/* _MIXERIOC(3) was deprecated */
/* _MIXERIOC(4) was deprecated */

/**
 * Add mixer(s) from the buffer in (const char *)arg
 */
#define MIXERIOCLOADBUF		_MIXERIOC(5)

/*
 * XXX Thoughts for additional operations:
 *
 * - get/set output scale, for tuning center/limit values.
 * - save/serialise for saving tuned mixers.
 */


/**
 * Get the count of parameters in the group as (unsigned *)arg
 */
#define   MIXERIOCGETPARAMCOUNT         _MIXERIOC(10)


/**
 * Get the parameter at index
 * return the value and metadata
 */
struct mixer_param_s {
	int16_t         index;
	int16_t         mix_index;
	int16_t         mix_sub_index;
	int16_t         param_index;
	float           values[6];
	char            name[17];
	int8_t          mix_type;
	int8_t          param_type;
	int8_t          array_size;
	uint8_t         flags;
};
#define   MIXERIOCGETPARAM		_MIXERIOC(11)


/**
 * Set the parameter at index for a mixer at index (mixer_param_s *)arg
 * set the the mixer index and parameter index int the struct
 * return 0 if success, -1 if fail.  Fail can indicate out of range.
 * If failed then real value MAY be set in the mixer_param_s struct.
 */
#define MIXERIOCSETPARAM                 _MIXERIOC(14)


/**
 * Get mixer configuration text for serialization
 */
struct mixer_config_s {
	char           *buff;
	unsigned        size;
};
#define MIXERIOCGETCONFIG               _MIXERIOC(15)


#endif /* _DRV_ACCEL_H */
