/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
#include <syslog.h>

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>

#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>

#include <systemlib/pwm_limit/pwm_limit.h>
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
static bool mixer_servos_armed = false;
static bool should_arm = false;
static bool should_always_enable_pwm = false;
static volatile bool in_mixer = false;

/* selected control values and count for mixing */
enum mixer_source {
	MIX_NONE,
	MIX_FMU,
	MIX_OVERRIDE,
	MIX_FAILSAFE,
	MIX_OVERRIDE_FMU_OK
};
static mixer_source source;

static int	mixer_callback(uintptr_t handle,
			       uint8_t control_group,
			       uint8_t control_index,
			       float &control);

static MixerGroup mixer_group(mixer_callback, 0);

/* Set the failsafe values of all mixed channels (based on zero throttle, controls centered) */
static void mixer_set_failsafe();

void
mixer_tick(void)
{

	/* check that we are receiving fresh data from the FMU */
	if (hrt_elapsed_time(&system_state.fmu_data_received_time) > FMU_INPUT_DROP_LIMIT_US) {

		/* too long without FMU input, time to go to failsafe */
		if (r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK) {
			isr_debug(1, "AP RX timeout");
		}
		r_status_flags &= ~(PX4IO_P_STATUS_FLAGS_FMU_OK);
		r_status_alarms |= PX4IO_P_STATUS_ALARMS_FMU_LOST;

	} else {
		r_status_flags |= PX4IO_P_STATUS_FLAGS_FMU_OK;
	}

	/* default to failsafe mixing */
	source = MIX_FAILSAFE;

	/*
	 * Decide which set of controls we're using.
	 */

	/* do not mix if RAW_PWM mode is on and FMU is good */
	if ((r_status_flags & PX4IO_P_STATUS_FLAGS_RAW_PWM) &&
	        (r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK)) {

		/* don't actually mix anything - we already have raw PWM values */
		source = MIX_NONE;

	} else {

		if (!(r_status_flags & PX4IO_P_STATUS_FLAGS_OVERRIDE) &&
		     (r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK) &&
		     (r_status_flags & PX4IO_P_STATUS_FLAGS_MIXER_OK)) {

			/* mix from FMU controls */
			source = MIX_FMU;
		}

		if ( (r_status_flags & PX4IO_P_STATUS_FLAGS_OVERRIDE) &&
		     (r_status_flags & PX4IO_P_STATUS_FLAGS_RC_OK) &&
		     (r_status_flags & PX4IO_P_STATUS_FLAGS_MIXER_OK) &&
		     !(r_setup_arming & PX4IO_P_SETUP_ARMING_RC_HANDLING_DISABLED) &&
		     !(r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK)) {

		 	/* if allowed, mix from RC inputs directly */
			source = MIX_OVERRIDE;
		} else 	if ( (r_status_flags & PX4IO_P_STATUS_FLAGS_OVERRIDE) &&
		     (r_status_flags & PX4IO_P_STATUS_FLAGS_RC_OK) &&
		     (r_status_flags & PX4IO_P_STATUS_FLAGS_MIXER_OK) &&
		     !(r_setup_arming & PX4IO_P_SETUP_ARMING_RC_HANDLING_DISABLED) &&
		     (r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK)) {

			/* if allowed, mix from RC inputs directly up to available rc channels */
			source = MIX_OVERRIDE_FMU_OK;
		}
	}

	/*
	 * Set failsafe status flag depending on mixing source
	 */
	if (source == MIX_FAILSAFE) {
		r_status_flags |= PX4IO_P_STATUS_FLAGS_FAILSAFE;
	} else {
		r_status_flags &= ~(PX4IO_P_STATUS_FLAGS_FAILSAFE);
	}

	/*
	 * Decide whether the servos should be armed right now.
	 *
	 * We must be armed, and we must have a PWM source; either raw from
	 * FMU or from the mixer.
	 *
	 * XXX correct behaviour for failsafe may require an additional case
	 * here.
	 */
	should_arm = (
		/* IO initialised without error */   (r_status_flags & PX4IO_P_STATUS_FLAGS_INIT_OK)
		/* and IO is armed */ 		  && (r_status_flags & PX4IO_P_STATUS_FLAGS_SAFETY_OFF)
		/* and FMU is armed */ 		  && (
							    ((r_setup_arming & PX4IO_P_SETUP_ARMING_FMU_ARMED)
		/* and there is valid input via or mixer */         &&   (r_status_flags & PX4IO_P_STATUS_FLAGS_MIXER_OK) )
		/* or direct PWM is set */               || (r_status_flags & PX4IO_P_STATUS_FLAGS_RAW_PWM)
		/* or failsafe was set manually */	 || ((r_setup_arming & PX4IO_P_SETUP_ARMING_FAILSAFE_CUSTOM) && !(r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK))
						     )
	);

	should_always_enable_pwm = (r_setup_arming & PX4IO_P_SETUP_ARMING_ALWAYS_PWM_ENABLE)
						&& (r_status_flags & PX4IO_P_STATUS_FLAGS_INIT_OK)
						&& (r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK);

	/*
	 * Run the mixers.
	 */
	if (source == MIX_FAILSAFE) {

		/* copy failsafe values to the servo outputs */
		for (unsigned i = 0; i < PX4IO_SERVO_COUNT; i++) {
			r_page_servos[i] = r_page_servo_failsafe[i];

			/* safe actuators for FMU feedback */
			r_page_actuators[i] = FLOAT_TO_REG((r_page_servos[i] - 1500) / 600.0f);
		}


	} else if (source != MIX_NONE && (r_status_flags & PX4IO_P_STATUS_FLAGS_MIXER_OK)) {

		float	outputs[PX4IO_SERVO_COUNT];
		unsigned mixed;

		/* mix */

		/* poor mans mutex */
		in_mixer = true;
		mixed = mixer_group.mix(&outputs[0], PX4IO_SERVO_COUNT);
		in_mixer = false;

		/* the pwm limit call takes care of out of band errors */
		pwm_limit_calc(should_arm, mixed, r_page_servo_disarmed, r_page_servo_control_min, r_page_servo_control_max, outputs, r_page_servos, &pwm_limit);

		for (unsigned i = mixed; i < PX4IO_SERVO_COUNT; i++)
			r_page_servos[i] = 0;

		for (unsigned i = 0; i < PX4IO_SERVO_COUNT; i++) {
			r_page_actuators[i] = FLOAT_TO_REG(outputs[i]);
		}
	}

	/* set arming */
	bool needs_to_arm = (should_arm || should_always_enable_pwm);

	/* check any conditions that prevent arming */
	if (r_setup_arming & PX4IO_P_SETUP_ARMING_LOCKDOWN) {
		needs_to_arm = false;
	}
	if (!should_arm && !should_always_enable_pwm) {
		needs_to_arm = false;
	}

	if (needs_to_arm && !mixer_servos_armed) {
		/* need to arm, but not armed */
		up_pwm_servo_arm(true);
		mixer_servos_armed = true;
		r_status_flags |= PX4IO_P_STATUS_FLAGS_OUTPUTS_ARMED;
		isr_debug(5, "> PWM enabled");

	} else if (!needs_to_arm && mixer_servos_armed) {
		/* armed but need to disarm */
		up_pwm_servo_arm(false);
		mixer_servos_armed = false;
		r_status_flags &= ~(PX4IO_P_STATUS_FLAGS_OUTPUTS_ARMED);
		isr_debug(5, "> PWM disabled");
	}

	if (mixer_servos_armed && should_arm) {
		/* update the servo outputs. */
		for (unsigned i = 0; i < PX4IO_SERVO_COUNT; i++)
			up_pwm_servo_set(i, r_page_servos[i]);

		/* set S.BUS1 or S.BUS2 outputs */

		if (r_setup_features & PX4IO_P_SETUP_FEATURES_SBUS2_OUT) {
			sbus2_output(r_page_servos, PX4IO_SERVO_COUNT);
		} else if (r_setup_features & PX4IO_P_SETUP_FEATURES_SBUS1_OUT) {
			sbus1_output(r_page_servos, PX4IO_SERVO_COUNT);
		}

	} else if (mixer_servos_armed && should_always_enable_pwm) {
		/* set the disarmed servo outputs. */
		for (unsigned i = 0; i < PX4IO_SERVO_COUNT; i++)
			up_pwm_servo_set(i, r_page_servo_disarmed[i]);

		/* set S.BUS1 or S.BUS2 outputs */
		if (r_setup_features & PX4IO_P_SETUP_FEATURES_SBUS1_OUT)
			sbus1_output(r_page_servos, PX4IO_SERVO_COUNT);

		if (r_setup_features & PX4IO_P_SETUP_FEATURES_SBUS2_OUT)
			sbus2_output(r_page_servos, PX4IO_SERVO_COUNT);
	}
}

static int
mixer_callback(uintptr_t handle,
	       uint8_t control_group,
	       uint8_t control_index,
	       float &control)
{
	if (control_group >= PX4IO_CONTROL_GROUPS)
		return -1;

	switch (source) {
	case MIX_FMU:
		if (control_index < PX4IO_CONTROL_CHANNELS && control_group < PX4IO_CONTROL_GROUPS ) {
			control = REG_TO_FLOAT(r_page_controls[CONTROL_PAGE_INDEX(control_group, control_index)]);
			break;
		}
		return -1;

	case MIX_OVERRIDE:
		if (r_page_rc_input[PX4IO_P_RC_VALID] & (1 << CONTROL_PAGE_INDEX(control_group, control_index))) {
			control = REG_TO_FLOAT(r_page_rc_input[PX4IO_P_RC_BASE + control_index]);
			break;
		}
		return -1;

	case MIX_OVERRIDE_FMU_OK:
		/* FMU is ok but we are in override mode, use direct rc control for the available rc channels. The remaining channels are still controlled by the fmu */
		if (r_page_rc_input[PX4IO_P_RC_VALID] & (1 << CONTROL_PAGE_INDEX(control_group, control_index))) {
			control = REG_TO_FLOAT(r_page_rc_input[PX4IO_P_RC_BASE + control_index]);
			break;
		} else if (control_index < PX4IO_CONTROL_CHANNELS && control_group < PX4IO_CONTROL_GROUPS) {
			control = REG_TO_FLOAT(r_page_controls[CONTROL_PAGE_INDEX(control_group, control_index)]);
			break;
		}
		return -1;

	case MIX_FAILSAFE:
	case MIX_NONE:
		control = 0.0f;
		return -1;
	}

	return 0;
}

/*
 * XXX error handling here should be more aggressive; currently it is
 * possible to get STATUS_FLAGS_MIXER_OK set even though the mixer has
 * not loaded faithfully.
 */

static char mixer_text[256];		/* large enough for one mixer */
static unsigned mixer_text_length = 0;

int
mixer_handle_text(const void *buffer, size_t length)
{
	/* do not allow a mixer change while safety off */
	if ((r_status_flags & PX4IO_P_STATUS_FLAGS_SAFETY_OFF)) {
		return 1;
	}

	/* abort if we're in the mixer */
	if (in_mixer) {
		return 1;
	}

	px4io_mixdata	*msg = (px4io_mixdata *)buffer;

	isr_debug(2, "mix txt %u", length);

	if (length < sizeof(px4io_mixdata))
		return 0;

	unsigned	text_length = length - sizeof(px4io_mixdata);

	switch (msg->action) {
	case F2I_MIXER_ACTION_RESET:
		isr_debug(2, "reset");

		/* FIRST mark the mixer as invalid */
		r_status_flags &= ~PX4IO_P_STATUS_FLAGS_MIXER_OK;
		/* THEN actually delete it */
		mixer_group.reset();
		mixer_text_length = 0;

		/* FALLTHROUGH */
	case F2I_MIXER_ACTION_APPEND:
		isr_debug(2, "append %d", length);

		/* disable mixing during the update */
		r_status_flags &= ~PX4IO_P_STATUS_FLAGS_MIXER_OK;

		/* check for overflow - this would be really fatal */
		if ((mixer_text_length + text_length + 1) > sizeof(mixer_text)) {
			r_status_flags &= ~PX4IO_P_STATUS_FLAGS_MIXER_OK;
			return 0;
		}

		/* append mixer text and nul-terminate, guard against overflow */
		memcpy(&mixer_text[mixer_text_length], msg->text, text_length);
		mixer_text_length += text_length;
		mixer_text[mixer_text_length] = '\0';
		isr_debug(2, "buflen %u", mixer_text_length);

		/* process the text buffer, adding new mixers as their descriptions can be parsed */
		unsigned resid = mixer_text_length;
		mixer_group.load_from_buf(&mixer_text[0], resid);

		/* if anything was parsed */
		if (resid != mixer_text_length) {

			/* only set mixer ok if no residual is left over */
			if (resid == 0) {
				r_status_flags |= PX4IO_P_STATUS_FLAGS_MIXER_OK;
			} else {
				/* not yet reached the end of the mixer, set as not ok */
				r_status_flags &= ~PX4IO_P_STATUS_FLAGS_MIXER_OK;
			}

			isr_debug(2, "used %u", mixer_text_length - resid);

			/* copy any leftover text to the base of the buffer for re-use */
			if (resid > 0)
				memcpy(&mixer_text[0], &mixer_text[mixer_text_length - resid], resid);

			mixer_text_length = resid;

			/* update failsafe values */
			mixer_set_failsafe();
		}

		break;
	}

	return 0;
}

static void
mixer_set_failsafe()
{
	/* 
	 * Check if a custom failsafe value has been written,
	 * or if the mixer is not ok and bail out.
	 */

	if ((r_setup_arming & PX4IO_P_SETUP_ARMING_FAILSAFE_CUSTOM) ||
		!(r_status_flags & PX4IO_P_STATUS_FLAGS_MIXER_OK))
		return;

	/* set failsafe defaults to the values for all inputs = 0 */
	float	outputs[PX4IO_SERVO_COUNT];
	unsigned mixed;

	/* mix */
	mixed = mixer_group.mix(&outputs[0], PX4IO_SERVO_COUNT);

	/* scale to PWM and update the servo outputs as required */
	for (unsigned i = 0; i < mixed; i++) {

		/* scale to servo output */
		r_page_servo_failsafe[i] = (outputs[i] * 600.0f) + 1500;

	}

	/* disable the rest of the outputs */
	for (unsigned i = mixed; i < PX4IO_SERVO_COUNT; i++)
		r_page_servo_failsafe[i] = 0;

}
