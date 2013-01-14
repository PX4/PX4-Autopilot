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
 * @file mixer.cpp
 *
 * Control channel input/output mixer and failsafe.
 */

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>

#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>

#include <systemlib/mixer/mixer.h>

extern "C" {
//#define DEBUG
#include "px4io.h"
}

/*
 * Maximum interval in us before FMU signal is considered lost
 */
#define FMU_INPUT_DROP_LIMIT_US		200000

/* XXX need to move the RC_CHANNEL_FUNCTION out of rc_channels.h and into systemlib */
#define ROLL     0
#define PITCH    1
#define YAW      2
#define THROTTLE 3
#define OVERRIDE 4

/* current servo arm/disarm state */
bool mixer_servos_armed = false;

/* selected control values and count for mixing */
enum mixer_source {
	MIX_FMU,
	MIX_OVERRIDE,
	MIX_FAILSAFE
};
static mixer_source source;

static int	mixer_callback(uintptr_t handle,
			       uint8_t control_group,
			       uint8_t control_index,
			       float &control);

static MixerGroup mixer_group(mixer_callback, 0);

void
mixer_tick(void)
{
	/* check that we are receiving fresh data from the FMU */
	if ((hrt_absolute_time() - system_state.fmu_data_received_time) > FMU_INPUT_DROP_LIMIT_US) {

		/* too many frames without FMU input, time to go to failsafe */
		r_status_flags |= PX4IO_P_STATUS_FLAGS_OVERRIDE;
		r_status_flags &= ~PX4IO_P_STATUS_FLAGS_FMU_OK;
		r_status_alarms |= PX4IO_P_STATUS_ALARMS_FMU_LOST;
		debug("AP RX timeout");
	}

	/*
	 * Decide which set of controls we're using.
	 */
	if ((r_setup_features & PX4IO_P_FEAT_ARMING_MANUAL_OVERRIDE_OK) && 
		(r_status_flags & PX4IO_P_STATUS_FLAGS_OVERRIDE)) {
	 	/* this is for planes, where manual override makes sense */
		source = MIX_OVERRIDE;
	} else if (r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK) {
		source = MIX_FMU;
	} else {
		source = MIX_FAILSAFE;
	}

#if 0
		/* if everything is ok */

		if (!system_state.mixer_manual_override && system_state.mixer_fmu_available) {
			/* we have recent control data from the FMU */
			control_count = PX4IO_CONTROL_CHANNELS;
			control_values = &system_state.fmu_channel_data[0];

		} else if (system_state.rc_channels > 0) {
			/* when override is on or the fmu is not available, but RC is present */
			control_count = system_state.rc_channels;

			sched_lock();

			/* remap roll, pitch, yaw and throttle from RC specific to internal ordering */
			rc_channel_data[ROLL]     = system_state.rc_channel_data[system_state.rc_map[ROLL] - 1];
			rc_channel_data[PITCH]    = system_state.rc_channel_data[system_state.rc_map[PITCH] - 1];
			rc_channel_data[YAW]      = system_state.rc_channel_data[system_state.rc_map[YAW] - 1];
			rc_channel_data[THROTTLE] = system_state.rc_channel_data[system_state.rc_map[THROTTLE] - 1];
			//rc_channel_data[OVERRIDE] = system_state.rc_channel_data[system_state.rc_map[OVERRIDE] - 1];

			/* get the remaining channels, no remapping needed */
			for (unsigned i = 4; i < system_state.rc_channels; i++) {
				rc_channel_data[i] = system_state.rc_channel_data[i];
			}

			/* scale the control inputs */ 
			rc_channel_data[THROTTLE] = ((float)(rc_channel_data[THROTTLE] - system_state.rc_min[THROTTLE]) / 
				(float)(system_state.rc_max[THROTTLE] - system_state.rc_min[THROTTLE])) * 1000.0f + 1000;

			if (rc_channel_data[THROTTLE] > 2000) {
				rc_channel_data[THROTTLE] = 2000;
			}

			if (rc_channel_data[THROTTLE] < 1000) {
				rc_channel_data[THROTTLE] = 1000;
			}
			
			// lib_lowprintf("Tmin: %d Ttrim: %d Tmax: %d T: %d \n",
			// 	(int)(system_state.rc_min[THROTTLE]), (int)(system_state.rc_trim[THROTTLE]),
			// 	(int)(system_state.rc_max[THROTTLE]), (int)(rc_channel_data[THROTTLE]));

			control_values = &rc_channel_data[0];
			sched_unlock();
		} else {
			/* we have no control input (no FMU, no RC) */

			// XXX builtin failsafe would activate here
			control_count = 0;
		}
		//lib_lowprintf("R: %d P: %d Y: %d T: %d \n", control_values[0], control_values[1], control_values[2], control_values[3]);

	/* this is for multicopters, etc. where manual override does not make sense */
	} else {
		/* if the fmu is available whe are good */
		if (system_state.mixer_fmu_available) {
			control_count = PX4IO_CONTROL_CHANNELS;
			control_values = &system_state.fmu_channel_data[0];
		/* we better shut everything off */
		} else {
			control_count = 0;
		}
	}
#endif
	/*
	 * Run the mixers.
	 */
	float	outputs[IO_SERVO_COUNT];
	unsigned mixed;

	/* mix */
	mixed = mixer_group.mix(&outputs[0], IO_SERVO_COUNT);

	/* scale to PWM and update the servo outputs as required */
	for (unsigned i = 0; i < mixed; i++) {

		/* save actuator values for FMU readback */
		r_page_actuators[i] = FLOAT_TO_REG(outputs[i]);

		/* scale to servo output */
		r_page_servos[i] = (outputs[i] * 500.0f) + 1500;

	}
	for (unsigned i = mixed; i < IO_SERVO_COUNT; i++)
		r_page_servos[i] = 0;

	/*
	 * Update the servo outputs.
	 */
	for (unsigned i = 0; i < IO_SERVO_COUNT; i++)
		up_pwm_servo_set(i, r_page_servos[i]);

	/*
	 * Decide whether the servos should be armed right now.
	 */

	bool should_arm = (r_status_flags & PX4IO_P_STATUS_FLAGS_ARMED);

	if (should_arm && !mixer_servos_armed) {
		/* need to arm, but not armed */
		up_pwm_servo_arm(true);
		mixer_servos_armed = true;

	} else if (!should_arm && mixer_servos_armed) {
		/* armed but need to disarm */
		up_pwm_servo_arm(false);
		mixer_servos_armed = false;
	}
}

static int
mixer_callback(uintptr_t handle,
	       uint8_t control_group,
	       uint8_t control_index,
	       float &control)
{
	if (control_group != 0)
		return -1;

	switch (source) {
	case MIX_FMU:
		if (control_index < PX4IO_CONTROL_CHANNELS) {
			control = REG_TO_FLOAT(r_page_controls[control_index]);
			break;
		}
		return -1;

	case MIX_OVERRIDE:
		if (r_page_rc_input[PX4IO_P_RC_VALID] & (1 << control_index)) {
			control = REG_TO_FLOAT(r_page_rc_input[PX4IO_P_RC_BASE + control_index]);
			break;
		}
		return -1;

	case MIX_FAILSAFE:
		/* XXX we could allow for configuration of per-output failsafe values */
		return -1;
	}

	return 0;
}

static char mixer_text[256];		/* large enough for one mixer */
static unsigned mixer_text_length = 0;

void
mixer_handle_text(const void *buffer, size_t length)
{

	px4io_mixdata	*msg = (px4io_mixdata *)buffer;

	debug("mixer text %u", length);

	if (length < sizeof(px4io_mixdata))
		return;

	unsigned	text_length = length - sizeof(px4io_mixdata);

	switch (msg->action) {
	case F2I_MIXER_ACTION_RESET:
		debug("reset");
		mixer_group.reset();
		mixer_text_length = 0;

		/* FALLTHROUGH */
	case F2I_MIXER_ACTION_APPEND:
		debug("append %d", length);

		/* check for overflow - this is really fatal */
		/* XXX could add just what will fit & try to parse, then repeat... */
		if ((mixer_text_length + text_length + 1) > sizeof(mixer_text))
			return;

		/* append mixer text and nul-terminate */
		memcpy(&mixer_text[mixer_text_length], msg->text, text_length);
		mixer_text_length += text_length;
		mixer_text[mixer_text_length] = '\0';
		debug("buflen %u", mixer_text_length);

		/* process the text buffer, adding new mixers as their descriptions can be parsed */
		unsigned resid = mixer_text_length;
		mixer_group.load_from_buf(&mixer_text[0], resid);

		/* if anything was parsed */
		if (resid != mixer_text_length) {
			debug("used %u", mixer_text_length - resid);

			/* copy any leftover text to the base of the buffer for re-use */
			if (resid > 0)
				memcpy(&mixer_text[0], &mixer_text[mixer_text_length - resid], resid);

			mixer_text_length = resid;
		}

		break;
	}
}
