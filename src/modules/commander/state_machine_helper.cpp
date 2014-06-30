/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file state_machine_helper.cpp
 * State machine helper functions implementations
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <julian@oes.ch>
 */

#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <dirent.h>
#include <fcntl.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/differential_pressure.h>
#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_airspeed.h>
#include <drivers/drv_device.h>
#include <mavlink/mavlink_log.h>

#include "state_machine_helper.h"
#include "commander_helper.h"

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

static int prearm_check(const struct vehicle_status_s *status, const int mavlink_fd);

// This array defines the arming state transitions. The rows are the new state, and the columns
// are the current state. Using new state and current  state you can index into the array which
// will be true for a valid transition or false for a invalid transition. In some cases even
// though the transition is marked as true additional checks must be made. See arming_state_transition
// code for those checks.
static const bool arming_transitions[ARMING_STATE_MAX][ARMING_STATE_MAX] = {
	//                                  INIT,  STANDBY, ARMED, ARMED_ERROR, STANDBY_ERROR, REBOOT, IN_AIR_RESTORE
	{ /* ARMING_STATE_INIT */           true,  true,    false, false,       false,         false,  false },
	{ /* ARMING_STATE_STANDBY */        true,  true,    true,  true,        false,         false,  false },
	{ /* ARMING_STATE_ARMED */          false, true,    true,  false,       false,         false,  true },
	{ /* ARMING_STATE_ARMED_ERROR */    false, false,   true,  true,        false,         false,  false },
	{ /* ARMING_STATE_STANDBY_ERROR */  true,  true,    false, true,        true,          false,  false },
	{ /* ARMING_STATE_REBOOT */         true,  true,    false, false,       true,          true,   true },
	{ /* ARMING_STATE_IN_AIR_RESTORE */ false, false,   false, false,       false,         false,  false }, // NYI
};

// You can index into the array with an arming_state_t in order to get it's textual representation
static const char *state_names[ARMING_STATE_MAX] = {
	"ARMING_STATE_INIT",
	"ARMING_STATE_STANDBY",
	"ARMING_STATE_ARMED",
	"ARMING_STATE_ARMED_ERROR",
	"ARMING_STATE_STANDBY_ERROR",
	"ARMING_STATE_REBOOT",
	"ARMING_STATE_IN_AIR_RESTORE",
};

transition_result_t
arming_state_transition(struct vehicle_status_s *status,            /// current vehicle status
			const struct safety_s   *safety,            /// current safety settings
			arming_state_t          new_arming_state,   /// arming state requested
			struct actuator_armed_s *armed,             /// current armed status
			const int               mavlink_fd)         /// mavlink fd for error reporting, 0 for none
{
	// Double check that our static arrays are still valid
	ASSERT(ARMING_STATE_INIT == 0);
	ASSERT(ARMING_STATE_IN_AIR_RESTORE == ARMING_STATE_MAX - 1);

	transition_result_t ret = TRANSITION_DENIED;

	arming_state_t current_arming_state = status->arming_state;

	/* only check transition if the new state is actually different from the current one */
	if (new_arming_state == current_arming_state) {
		ret = TRANSITION_NOT_CHANGED;

	} else {

		/*
		 * Get sensing state if necessary
		 */
		int prearm_ret = OK;

		/* only perform the check if we have to */
		if (new_arming_state == ARMING_STATE_ARMED) {
			prearm_ret = prearm_check(status, mavlink_fd);
		}

		/*
		 * Perform an atomic state update
		 */
		irqstate_t flags = irqsave();

		/* enforce lockdown in HIL */
		if (status->hil_state == HIL_STATE_ON) {
			armed->lockdown = true;

		} else {
			armed->lockdown = false;
		}

		// Check that we have a valid state transition
		bool valid_transition = arming_transitions[new_arming_state][status->arming_state];

		if (valid_transition) {
			// We have a good transition. Now perform any secondary validation.
			if (new_arming_state == ARMING_STATE_ARMED) {

				//      Do not perform pre-arm checks if coming from in air restore
				//      Allow if HIL_STATE_ON
				if (status->arming_state != ARMING_STATE_IN_AIR_RESTORE &&
					status->hil_state == HIL_STATE_OFF) {

					// Fail transition if pre-arm check fails
					if (prearm_ret) {
						valid_transition = false;

					// Fail transition if we need safety switch press
					} else if (safety->safety_switch_available && !safety->safety_off) {

						mavlink_log_critical(mavlink_fd, "#audio: NOT ARMING: Press safety switch!");

						valid_transition = false;
					}

					// Perform power checks only if circuit breaker is not
					// engaged for these checks
					if (!status->circuit_breaker_engaged_power_check) {
						// Fail transition if power is not good
						if (!status->condition_power_input_valid) {

							mavlink_log_critical(mavlink_fd, "#audio: NOT ARMING: Connect power module.");
							valid_transition = false;
						}

						// Fail transition if power levels on the avionics rail
						// are measured but are insufficient
						if (status->condition_power_input_valid && (status->avionics_power_rail_voltage > 0.0f) &&
							(status->avionics_power_rail_voltage < 4.9f)) {

							mavlink_log_critical(mavlink_fd, "#audio: NOT ARMING: Avionics power low: %6.2f V.", status->avionics_power_rail_voltage);
							valid_transition = false;
						}
					}

				}

			} else if (new_arming_state == ARMING_STATE_STANDBY && status->arming_state == ARMING_STATE_ARMED_ERROR) {
				new_arming_state = ARMING_STATE_STANDBY_ERROR;
			}
		}

		// HIL can always go to standby
		if (status->hil_state == HIL_STATE_ON && new_arming_state == ARMING_STATE_STANDBY) {
			valid_transition = true;
		}

		/* Sensors need to be initialized for STANDBY state */
		if (new_arming_state == ARMING_STATE_STANDBY && !status->condition_system_sensors_initialized) {
			valid_transition = false;
		}

		// Finish up the state transition
		if (valid_transition) {
			armed->armed = new_arming_state == ARMING_STATE_ARMED || new_arming_state == ARMING_STATE_ARMED_ERROR;
			armed->ready_to_arm = new_arming_state == ARMING_STATE_ARMED || new_arming_state == ARMING_STATE_STANDBY;
			ret = TRANSITION_CHANGED;
			status->arming_state = new_arming_state;
		}

		/* end of atomic state update */
		irqrestore(flags);
	}

	if (ret == TRANSITION_DENIED) {
		static const char *errMsg = "INVAL: %s - %s";

		mavlink_log_critical(mavlink_fd, errMsg, state_names[status->arming_state], state_names[new_arming_state]);

		warnx(errMsg, state_names[status->arming_state], state_names[new_arming_state]);
	}

	return ret;
}

bool is_safe(const struct vehicle_status_s *status, const struct safety_s *safety, const struct actuator_armed_s *armed)
{
	// System is safe if:
	// 1) Not armed
	// 2) Armed, but in software lockdown (HIL)
	// 3) Safety switch is present AND engaged -> actuators locked
	if (!armed->armed || (armed->armed && armed->lockdown) || (safety->safety_switch_available && !safety->safety_off)) {
		return true;

	} else {
		return false;
	}
}

transition_result_t
main_state_transition(struct vehicle_status_s *status, main_state_t new_main_state)
{
	transition_result_t ret = TRANSITION_DENIED;

	/* transition may be denied even if the same state is requested because conditions may have changed */
	switch (new_main_state) {
	case MAIN_STATE_MANUAL:
	case MAIN_STATE_ACRO:
		ret = TRANSITION_CHANGED;
		break;

	case MAIN_STATE_ALTCTL:
		/* need at minimum altitude estimate */
		/* TODO: add this for fixedwing as well */
		if (!status->is_rotary_wing ||
		    (status->condition_local_altitude_valid ||
		     status->condition_global_position_valid)) {
			ret = TRANSITION_CHANGED;
		}
		break;

	case MAIN_STATE_POSCTL:
		/* need at minimum local position estimate */
		if (status->condition_local_position_valid ||
		    status->condition_global_position_valid) {
			ret = TRANSITION_CHANGED;
		}
		break;

	case MAIN_STATE_AUTO_MISSION:
	case MAIN_STATE_AUTO_LOITER:
		/* need global position estimate */
		if (status->condition_global_position_valid) {
			ret = TRANSITION_CHANGED;
		}
		break;

	case MAIN_STATE_AUTO_RTL:
		/* need global position and home position */
		if (status->condition_global_position_valid && status->condition_home_position_valid) {
			ret = TRANSITION_CHANGED;
		}
		break;

	case MAIN_STATE_MAX:
	default:
		break;
	}
	if (ret == TRANSITION_CHANGED) {
		if (status->main_state != new_main_state) {
			status->main_state = new_main_state;
		} else {
			ret = TRANSITION_NOT_CHANGED;
		}
	}

	return ret;
}

/**
 * Transition from one hil state to another
 */
transition_result_t hil_state_transition(hil_state_t new_state, int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd)
{
	transition_result_t ret = TRANSITION_DENIED;

	if (current_status->hil_state == new_state) {
		ret = TRANSITION_NOT_CHANGED;

	} else {
		switch (new_state) {
		case HIL_STATE_OFF:
			/* we're in HIL and unexpected things can happen if we disable HIL now */
			mavlink_log_critical(mavlink_fd, "#audio: Not switching off HIL (safety)");
			ret = TRANSITION_DENIED;
			break;

		case HIL_STATE_ON:
			if (current_status->arming_state == ARMING_STATE_INIT
			    || current_status->arming_state == ARMING_STATE_STANDBY
			    || current_status->arming_state == ARMING_STATE_STANDBY_ERROR) {

				/* Disable publication of all attached sensors */
				/* list directory */
				DIR		*d;
				d = opendir("/dev");

				if (d) {
					struct dirent	*direntry;
					char devname[24];

					while ((direntry = readdir(d)) != NULL) {

						/* skip serial ports */
						if (!strncmp("tty", direntry->d_name, 3)) {
							continue;
						}

						/* skip mtd devices */
						if (!strncmp("mtd", direntry->d_name, 3)) {
							continue;
						}

						/* skip ram devices */
						if (!strncmp("ram", direntry->d_name, 3)) {
							continue;
						}

						/* skip MMC devices */
						if (!strncmp("mmc", direntry->d_name, 3)) {
							continue;
						}

						/* skip mavlink */
						if (!strcmp("mavlink", direntry->d_name)) {
							continue;
						}

						/* skip console */
						if (!strcmp("console", direntry->d_name)) {
							continue;
						}

						/* skip null */
						if (!strcmp("null", direntry->d_name)) {
							continue;
						}

						snprintf(devname, sizeof(devname), "/dev/%s", direntry->d_name);

						int sensfd = ::open(devname, 0);

						if (sensfd < 0) {
							warn("failed opening device %s", devname);
							continue;
						}

						int block_ret = ::ioctl(sensfd, DEVIOCSPUBBLOCK, 1);
						close(sensfd);

						printf("Disabling %s: %s\n", devname, (block_ret == OK) ? "OK" : "ERROR");
					}
					closedir(d);
					ret = TRANSITION_CHANGED;
					mavlink_log_critical(mavlink_fd, "Switched to ON hil state");


				} else {
					/* failed opening dir */
					mavlink_log_info(mavlink_fd, "FAILED LISTING DEVICE ROOT DIRECTORY");
					ret = TRANSITION_DENIED;
				}
			} else {
				mavlink_log_critical(mavlink_fd, "Not switching to HIL when armed");
				ret = TRANSITION_DENIED;
			}
			break;

		default:
			warnx("Unknown HIL state");
			break;
		}
	}

	if (ret == TRANSITION_CHANGED) {
		current_status->hil_state = new_state;
		current_status->timestamp = hrt_absolute_time();
		// XXX also set lockdown here
		orb_publish(ORB_ID(vehicle_status), status_pub, current_status);
	}
	return ret;
}

/**
 * Check failsafe and main status and set navigation status for navigator accordingly
 */
bool set_nav_state(struct vehicle_status_s *status, const bool data_link_loss_enabled, const bool mission_finished)
{
	navigation_state_t nav_state_old = status->nav_state;

	bool armed = (status->arming_state == ARMING_STATE_ARMED || status->arming_state == ARMING_STATE_ARMED_ERROR);
	status->failsafe = false;

	/* evaluate main state to decide in normal (non-failsafe) mode */
	switch (status->main_state) {
	case MAIN_STATE_ACRO:
	case MAIN_STATE_MANUAL:
	case MAIN_STATE_ALTCTL:
	case MAIN_STATE_POSCTL:
		/* require RC for all manual modes */
		if (status->rc_signal_lost && armed) {
			status->failsafe = true;

			if (status->condition_global_position_valid && status->condition_home_position_valid) {
				status->nav_state = NAVIGATION_STATE_AUTO_RTL;
			} else if (status->condition_local_position_valid) {
				status->nav_state = NAVIGATION_STATE_LAND;
			} else if (status->condition_local_altitude_valid) {
				status->nav_state = NAVIGATION_STATE_DESCEND;
			} else {
				status->nav_state = NAVIGATION_STATE_TERMINATION;
			}

		} else {
			switch (status->main_state) {
			case MAIN_STATE_ACRO:
				status->nav_state = NAVIGATION_STATE_ACRO;
				break;

			case MAIN_STATE_MANUAL:
				status->nav_state = NAVIGATION_STATE_MANUAL;
				break;

			case MAIN_STATE_ALTCTL:
				status->nav_state = NAVIGATION_STATE_ALTCTL;
				break;

			case MAIN_STATE_POSCTL:
				status->nav_state = NAVIGATION_STATE_POSCTL;
				break;

			default:
				status->nav_state = NAVIGATION_STATE_MANUAL;
				break;
			}
		}
		break;

	case MAIN_STATE_AUTO_MISSION:
		/* go into failsafe
		 * - if either the datalink is enabled and lost as well as RC is lost
		 * - if there is no datalink and the mission is finished */
		if (((status->data_link_lost && data_link_loss_enabled) && status->rc_signal_lost) ||
		    (!data_link_loss_enabled && status->rc_signal_lost && mission_finished)) {
			status->failsafe = true;

			if (status->condition_global_position_valid && status->condition_home_position_valid) {
				status->nav_state = NAVIGATION_STATE_AUTO_RTL;
			} else if (status->condition_local_position_valid) {
				status->nav_state = NAVIGATION_STATE_LAND;
			} else if (status->condition_local_altitude_valid) {
				status->nav_state = NAVIGATION_STATE_DESCEND;
			} else {
				status->nav_state = NAVIGATION_STATE_TERMINATION;
			}

		/* also go into failsafe if just datalink is lost */
		} else if (status->data_link_lost && data_link_loss_enabled) {
			status->failsafe = true;

			if (status->condition_global_position_valid && status->condition_home_position_valid) {
				status->nav_state = NAVIGATION_STATE_AUTO_RTGS;
			} else if (status->condition_local_position_valid) {
				status->nav_state = NAVIGATION_STATE_LAND;
			} else if (status->condition_local_altitude_valid) {
				status->nav_state = NAVIGATION_STATE_DESCEND;
			} else {
				status->nav_state = NAVIGATION_STATE_TERMINATION;
			}

		/* don't bother if RC is lost and mission is not yet finished */
		} else if (status->rc_signal_lost) {

			/* this mode is ok, we don't need RC for missions */
			status->nav_state = NAVIGATION_STATE_AUTO_MISSION;
		} else {
			/* everything is perfect */
			status->nav_state = NAVIGATION_STATE_AUTO_MISSION;
		}
		break;

	case MAIN_STATE_AUTO_LOITER:
		/* go into failsafe if datalink and RC is lost */
		if ((status->data_link_lost && data_link_loss_enabled) && status->rc_signal_lost) {
			status->failsafe = true;

			if (status->condition_global_position_valid && status->condition_home_position_valid) {
				status->nav_state = NAVIGATION_STATE_AUTO_RTL;
			} else if (status->condition_local_position_valid) {
				status->nav_state = NAVIGATION_STATE_LAND;
			} else if (status->condition_local_altitude_valid) {
				status->nav_state = NAVIGATION_STATE_DESCEND;
			} else {
				status->nav_state = NAVIGATION_STATE_TERMINATION;
			}

		/* also go into failsafe if just datalink is lost */
		} else if (status->data_link_lost && data_link_loss_enabled) {
			status->failsafe = true;

			if (status->condition_global_position_valid && status->condition_home_position_valid) {
				status->nav_state = NAVIGATION_STATE_AUTO_RTGS;
			} else if (status->condition_local_position_valid) {
				status->nav_state = NAVIGATION_STATE_LAND;
			} else if (status->condition_local_altitude_valid) {
				status->nav_state = NAVIGATION_STATE_DESCEND;
			} else {
				status->nav_state = NAVIGATION_STATE_TERMINATION;
			}

		/* go into failsafe if RC is lost and datalink loss is not set up */
		} else if (status->rc_signal_lost && !data_link_loss_enabled) {
			status->failsafe = true;

			if (status->condition_global_position_valid && status->condition_home_position_valid) {
				status->nav_state = NAVIGATION_STATE_AUTO_RTGS;
			} else if (status->condition_local_position_valid) {
				status->nav_state = NAVIGATION_STATE_LAND;
			} else if (status->condition_local_altitude_valid) {
				status->nav_state = NAVIGATION_STATE_DESCEND;
			} else {
				status->nav_state = NAVIGATION_STATE_TERMINATION;
			}

		/* don't bother if RC is lost if datalink is connected */
		} else if (status->rc_signal_lost) {

			/* this mode is ok, we don't need RC for loitering */
			status->nav_state = NAVIGATION_STATE_AUTO_LOITER;
		} else {
			/* everything is perfect */
			status->nav_state = NAVIGATION_STATE_AUTO_LOITER;
		}
		break;

	case MAIN_STATE_AUTO_RTL:
		/* require global position and home */
		if ((!status->condition_global_position_valid || !status->condition_home_position_valid)) {
			status->failsafe = true;

			if (status->condition_local_position_valid) {
				status->nav_state = NAVIGATION_STATE_LAND;
			} else if (status->condition_local_altitude_valid) {
				status->nav_state = NAVIGATION_STATE_DESCEND;
			} else {
				status->nav_state = NAVIGATION_STATE_TERMINATION;
			}
		} else {
			status->nav_state = NAVIGATION_STATE_AUTO_RTL;
		}
		break;

	default:
		break;
	}

	return status->nav_state != nav_state_old;
}

int prearm_check(const struct vehicle_status_s *status, const int mavlink_fd)
{
	int ret;

	int fd = open(ACCEL_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		mavlink_log_critical(mavlink_fd, "#audio: FAIL: ACCEL SENSOR MISSING");
		ret = fd;
		goto system_eval;
	}

	ret = ioctl(fd, ACCELIOCSELFTEST, 0);

	if (ret != OK) {
		mavlink_log_critical(mavlink_fd, "#audio: FAIL: ACCEL CALIBRATION");
		goto system_eval;
	}

	/* check measurement result range */
	struct accel_report acc;
	ret = read(fd, &acc, sizeof(acc));

	if (ret == sizeof(acc)) {
		/* evaluate values */
		float accel_scale = sqrtf(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);

		if (accel_scale < 9.78f || accel_scale > 9.83f) {
			mavlink_log_info(mavlink_fd, "#audio: Accelerometer calibration recommended.");
		}

		if (accel_scale > 30.0f /* m/s^2 */) {
			mavlink_log_critical(mavlink_fd, "#audio: FAIL: ACCEL RANGE");
			/* this is frickin' fatal */
			ret = ERROR;
			goto system_eval;
		} else {
			ret = OK;
		}
	} else {
		mavlink_log_critical(mavlink_fd, "#audio: FAIL: ACCEL READ");
		/* this is frickin' fatal */
		ret = ERROR;
		goto system_eval;
	}

	if (!status->is_rotary_wing) {
		int fd = open(AIRSPEED_DEVICE_PATH, O_RDONLY);

		if (fd < 0) {
			mavlink_log_critical(mavlink_fd, "#audio: FAIL: AIRSPEED SENSOR MISSING");
			ret = fd;
			goto system_eval;
		}

		struct differential_pressure_s diff_pres;

		ret = read(fd, &diff_pres, sizeof(diff_pres));

		if (ret == sizeof(diff_pres)) {
			if (fabsf(diff_pres.differential_pressure_filtered_pa > 5.0f)) {
				mavlink_log_critical(mavlink_fd, "#audio: WARNING AIRSPEED CALIBRATION MISSING");
				// XXX do not make this fatal yet
				ret = OK;
			}
		} else {
			mavlink_log_critical(mavlink_fd, "#audio: FAIL: AIRSPEED READ");
			/* this is frickin' fatal */
			ret = ERROR;
			goto system_eval;
		}
	}

system_eval:
	close(fd);
	return ret;
}
