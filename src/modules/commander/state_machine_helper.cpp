/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @author Thomas Gubler	<thomas@px4.io>
 * @author Julian Oes		<julian@oes.ch>
 * @author Sander Smeets	<sander@droneslab.com>
 */
#include <px4_config.h>

#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <dirent.h>
#include <fcntl.h>
#include <string.h>
#include <math.h>

#include <px4_posix.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/differential_pressure.h>
#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_device.h>

#include "state_machine_helper.h"
#include "commander_helper.h"
#include "PreflightCheck.h"
#ifndef __PX4_NUTTX
#include "DevMgr.hpp"
using namespace DriverFramework;
#endif

#ifdef CONFIG_ARCH_BOARD_MINDPX_V2
#define AVIONICS_ERROR_VOLTAGE	3.75f
#define AVIONICS_WARN_VOLTAGE	3.9f
#else
#define AVIONICS_ERROR_VOLTAGE	4.5f
#define AVIONICS_WARN_VOLTAGE	4.9f
#endif

// This array defines the arming state transitions. The rows are the new state, and the columns
// are the current state. Using new state and current state you can index into the array which
// will be true for a valid transition or false for a invalid transition. In some cases even
// though the transition is marked as true additional checks must be made. See arming_state_transition
// code for those checks.
static const bool arming_transitions[vehicle_status_s::ARMING_STATE_MAX][vehicle_status_s::ARMING_STATE_MAX] = {
	//                                                    INIT,  STANDBY, ARMED, ARMED_ERROR, STANDBY_ERROR, REBOOT, IN_AIR_RESTORE
	{ /* vehicle_status_s::ARMING_STATE_INIT */           true,  true,    false, false,       true,          false,  false },
	{ /* vehicle_status_s::ARMING_STATE_STANDBY */        true,  true,    true,  true,        false,         false,  false },
	{ /* vehicle_status_s::ARMING_STATE_ARMED */          false, true,    true,  false,       false,         false,  true },
	{ /* vehicle_status_s::ARMING_STATE_ARMED_ERROR */    false, false,   true,  true,        false,         false,  false },
	{ /* vehicle_status_s::ARMING_STATE_STANDBY_ERROR */  true,  true,    true,  true,        true,          false,  false },
	{ /* vehicle_status_s::ARMING_STATE_REBOOT */         true,  true,    false, false,       true,          true,   true },
	{ /* vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE */ false, false,   false, false,       false,         false,  false }, // NYI
};

// You can index into the array with an arming_state_t in order to get its textual representation
static const char *const state_names[vehicle_status_s::ARMING_STATE_MAX] = {
	"ARMING_STATE_INIT",
	"ARMING_STATE_STANDBY",
	"ARMING_STATE_ARMED",
	"ARMING_STATE_ARMED_ERROR",
	"ARMING_STATE_STANDBY_ERROR",
	"ARMING_STATE_REBOOT",
	"ARMING_STATE_IN_AIR_RESTORE",
};

static hrt_abstime last_preflight_check = 0;	///< initialize so it gets checked immediately
static int last_prearm_ret = 1;			///< initialize to fail

transition_result_t arming_state_transition(struct vehicle_status_s *status,
		struct battery_status_s *battery,
		const struct safety_s *safety,
		arming_state_t new_arming_state,
		struct actuator_armed_s *armed,
		bool fRunPreArmChecks,
		orb_advert_t *mavlink_log_pub,	///< uORB handle for mavlink log
		status_flags_s *status_flags,
		float avionics_power_rail_voltage,
		bool can_arm_without_gps)
{
	// Double check that our static arrays are still valid
	// 对静态数组的有效性进行两次检查
	ASSERT(vehicle_status_s::ARMING_STATE_INIT == 0);
	ASSERT(vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE == vehicle_status_s::ARMING_STATE_MAX - 1); // 6 = 7-1

	transition_result_t ret = TRANSITION_DENIED;
	arming_state_t current_arming_state = status->arming_state;
	bool feedback_provided = false;

	/* only check transition if the new state is actually different from the current one */
	// 如果新状态与当前的不同，则仅检查转换
	if (new_arming_state == current_arming_state) {
		ret = TRANSITION_NOT_CHANGED;

	} else {

		/*
		 * Get sensing state if necessary
		 * 如果需要的话，则获取感应状态
		 */
		int prearm_ret = OK;

		/* only perform the pre-arm check if we have to */
		// 如果需要的话，仅执行解锁前的检查
		if (fRunPreArmChecks && new_arming_state == vehicle_status_s::ARMING_STATE_ARMED
		    && status->hil_state == vehicle_status_s::HIL_STATE_OFF) {

			prearm_ret = preflight_check(status, mavlink_log_pub, true /* pre-arm */, false /* force_report */,
						     status_flags, battery, can_arm_without_gps); // 判断是否可以解锁
		}

		/* re-run the pre-flight check as long as sensors are failing */
		// 只要传感器崩溃，则重新运行解锁前的检查
		if (!status_flags->condition_system_sensors_initialized
		    && (new_arming_state == vehicle_status_s::ARMING_STATE_ARMED
			|| new_arming_state == vehicle_status_s::ARMING_STATE_STANDBY)
		    && status->hil_state == vehicle_status_s::HIL_STATE_OFF) { // 传感器初始化&&新解锁状态就绪&&非HIL状态

			if (last_preflight_check == 0 || hrt_absolute_time() - last_preflight_check > 1000 * 1000) { // 间隔一秒进行检查
				prearm_ret = preflight_check(status, mavlink_log_pub, false /* pre-flight */, false /* force_report */,
							     status_flags, battery, can_arm_without_gps);
				status_flags->condition_system_sensors_initialized = !prearm_ret;    // ？？？？？？？？
				last_preflight_check = hrt_absolute_time();
				last_prearm_ret = prearm_ret;

			} else {
				prearm_ret = last_prearm_ret;
			}
		}

		/*
		 * Perform an atomic state update
		 */
#ifdef __PX4_NUTTX
		irqstate_t flags = px4_enter_critical_section();
#endif

		/* enforce lockdown in HIL */
		if (status->hil_state == vehicle_status_s::HIL_STATE_ON) {
			armed->lockdown = true;
			prearm_ret = OK;
			status_flags->condition_system_sensors_initialized = true;

			/* recover from a prearm fail */
			if (status->arming_state == vehicle_status_s::ARMING_STATE_STANDBY_ERROR) {
				status->arming_state = vehicle_status_s::ARMING_STATE_STANDBY;
			}

		} else {
			armed->lockdown = false;
		}

		// Check that we have a valid state transition
		bool valid_transition = arming_transitions[new_arming_state][status->arming_state];

		if (valid_transition) {
			// We have a good transition. Now perform any secondary validation.
			if (new_arming_state == vehicle_status_s::ARMING_STATE_ARMED) {

				//      Do not perform pre-arm checks if coming from in air restore
				//      Allow if vehicle_status_s::HIL_STATE_ON
				if (status->arming_state != vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE &&
				    status->hil_state == vehicle_status_s::HIL_STATE_OFF) {

					// Fail transition if pre-arm check fails
					if (prearm_ret) {
						/* the prearm check already prints the reject reason */
						feedback_provided = true;
						valid_transition = false;

						// Fail transition if we need safety switch press

					} else if (safety->safety_switch_available && !safety->safety_off) {

						mavlink_and_console_log_critical(mavlink_log_pub, "NOT ARMING: Press safety switch first!");
						feedback_provided = true;
						valid_transition = false;
					}

					// Perform power checks only if circuit breaker is not
					// engaged for these checks
					if (!status_flags->circuit_breaker_engaged_power_check) {
						// Fail transition if power is not good
						if (!status_flags->condition_power_input_valid) {

							mavlink_and_console_log_critical(mavlink_log_pub, "NOT ARMING: Connect power module.");
							feedback_provided = true;
							valid_transition = false;
						}

						// Fail transition if power levels on the avionics rail
						// are measured but are insufficient
						if (status_flags->condition_power_input_valid && (avionics_power_rail_voltage > 0.0f)) {
							// Check avionics rail voltages
							if (avionics_power_rail_voltage < 4.5f) {
								mavlink_and_console_log_critical(mavlink_log_pub, "NOT ARMING: Avionics power low: %6.2f Volt",
												 (double)avionics_power_rail_voltage);
								feedback_provided = true;
								valid_transition = false;

							} else if (avionics_power_rail_voltage < 4.9f) {
								mavlink_and_console_log_critical(mavlink_log_pub, "CAUTION: Avionics power low: %6.2f Volt",
												 (double)avionics_power_rail_voltage);
								feedback_provided = true;

							} else if (avionics_power_rail_voltage > 5.4f) {
								mavlink_and_console_log_critical(mavlink_log_pub, "CAUTION: Avionics power high: %6.2f Volt",
												 (double)avionics_power_rail_voltage);
								feedback_provided = true;
							}
						}
					}

				}

			} else if (new_arming_state == vehicle_status_s::ARMING_STATE_STANDBY
				   && status->arming_state == vehicle_status_s::ARMING_STATE_ARMED_ERROR) {
				new_arming_state = vehicle_status_s::ARMING_STATE_STANDBY_ERROR;
			}
		}

		// HIL can always go to standby
		if (status->hil_state == vehicle_status_s::HIL_STATE_ON && new_arming_state == vehicle_status_s::ARMING_STATE_STANDBY) {
			valid_transition = true;
		}

		// Check if we are trying to arm, checks look good but we are in STANDBY_ERROR
		if (status->arming_state == vehicle_status_s::ARMING_STATE_STANDBY_ERROR) {

			if (new_arming_state == vehicle_status_s::ARMING_STATE_ARMED) {

				if (status_flags->condition_system_sensors_initialized) {
					mavlink_and_console_log_critical(mavlink_log_pub, "Preflight check resolved, reboot before arming");

				} else {
					mavlink_and_console_log_critical(mavlink_log_pub, "Preflight check failed, refusing to arm");
				}

				feedback_provided = true;

			} else if ((new_arming_state == vehicle_status_s::ARMING_STATE_STANDBY) &&
				   status_flags->condition_system_sensors_initialized) {
				mavlink_and_console_log_critical(mavlink_log_pub, "Preflight check resolved, reboot to complete");
				feedback_provided = true;

			} else {
				// Silent ignore
				feedback_provided = true;
			}

			// Sensors need to be initialized for STANDBY state, except for HIL

		} else if ((status->hil_state != vehicle_status_s::HIL_STATE_ON) &&
			   (new_arming_state == vehicle_status_s::ARMING_STATE_STANDBY) &&
			   (status->arming_state != vehicle_status_s::ARMING_STATE_STANDBY_ERROR)) {

			if (!status_flags->condition_system_sensors_initialized) {

				if (status_flags->condition_system_hotplug_timeout) {
					if (!status_flags->condition_system_prearm_error_reported) {
						mavlink_and_console_log_critical(mavlink_log_pub,
										 "Not ready to fly: Sensors not set up correctly");
						status_flags->condition_system_prearm_error_reported = true;
					}
				}

				feedback_provided = true;
				valid_transition = false;
			}
		}

		// Finish up the state transition
		if (valid_transition) {
			armed->armed = new_arming_state == vehicle_status_s::ARMING_STATE_ARMED
				       || new_arming_state == vehicle_status_s::ARMING_STATE_ARMED_ERROR;
			armed->ready_to_arm = new_arming_state == vehicle_status_s::ARMING_STATE_ARMED
					      || new_arming_state == vehicle_status_s::ARMING_STATE_STANDBY;
			ret = TRANSITION_CHANGED;
			status->arming_state = new_arming_state;
		}

		/* reset feedback state */
		if (status->arming_state != vehicle_status_s::ARMING_STATE_STANDBY_ERROR &&
		    status->arming_state != vehicle_status_s::ARMING_STATE_INIT &&
		    valid_transition) {
			status_flags->condition_system_prearm_error_reported = false;
		}

		/* end of atomic state update */
#ifdef __PX4_NUTTX
		px4_leave_critical_section(flags);
#endif
	}

	if (ret == TRANSITION_DENIED) {
		/* print to MAVLink and console if we didn't provide any feedback yet */
		if (!feedback_provided) {
			mavlink_and_console_log_critical(mavlink_log_pub, "TRANSITION_DENIED: %s - %s", state_names[status->arming_state],
							 state_names[new_arming_state]);
		}
	}

	return ret;
}

bool is_safe(const struct vehicle_status_s *status, const struct safety_s *safety, const struct actuator_armed_s *armed)
{
	// System is safe if:
	// 1) Not armed
	// 2) Armed, but in software lockdown (HIL)
	// 3) Safety switch is present AND engaged -> actuators locked
	// 如果满足以下条件，则系统是安全的：
	// 1) 未解锁
	// 2) 已解锁，三十处于软件锁定状态(HIL)
	// 3) 有安全开关并且未按下 -> 执行器锁定
	if (!armed->armed || (armed->armed && armed->lockdown) || (safety->safety_switch_available && !safety->safety_off)) {
		return true;

	} else {
		return false;
	}
}


//////////////////////  飞行模式切换条件  //////////////////////
transition_result_t
main_state_transition(struct vehicle_status_s *status, main_state_t new_main_state, uint8_t &main_state_prev,
		      status_flags_s *status_flags, struct commander_state_s *internal_state)
{
	transition_result_t ret = TRANSITION_DENIED;

	/* transition may be denied even if the same state is requested because conditions may have changed */
	// 因为情况可能已经改变，即使是相同的状态也可能会拒绝状态切换
	switch (new_main_state) { // 想要切换到的状态
	case commander_state_s::MAIN_STATE_MANUAL:  // 0 
	case commander_state_s::MAIN_STATE_STAB:    // 8
		ret = TRANSITION_CHANGED;				// 自稳  手动可以直接切换
		break;

	case commander_state_s::MAIN_STATE_ACRO:    // 6
	case commander_state_s::MAIN_STATE_RATTITUDE: // 9
		if (status->is_rotary_wing) {     // 仅用于旋翼机
			ret = TRANSITION_CHANGED;     
		}

		break;

	case commander_state_s::MAIN_STATE_ALTCTL:     // 1

		/* need at minimum altitude estimate */
		// 至少需要存在高度估计
		if (status_flags->condition_local_altitude_valid ||   
		    status_flags->condition_global_position_valid) {
			ret = TRANSITION_CHANGED;     	// 只要Local或者Global其中一个高度有效，即可切换定高模式
		}

		break;

	case commander_state_s::MAIN_STATE_POSCTL:      // 2

		/* need at minimum local position estimate */
		// 至少需要本地位置估计
		if (status_flags->condition_local_position_valid ||
		    status_flags->condition_global_position_valid) {
			ret = TRANSITION_CHANGED;		// 也就是只要Local 或 Global 其中一个定位有效，即可切换 POSCTL
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_LOITER:    //  4

		/* need global position estimate */
		// 需要全球位置估计
		if (status_flags->condition_global_position_valid) {
			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_FOLLOW_TARGET:    // 12
	case commander_state_s::MAIN_STATE_AUTO_MISSION:		  // 3	
	case commander_state_s::MAIN_STATE_AUTO_RTL:		      // 5
	case commander_state_s::MAIN_STATE_AUTO_TAKEOFF:		  // 10
	case commander_state_s::MAIN_STATE_AUTO_LAND:			  // 11

		/* need global position and home position */
		// 需要全球位置和起飞点位置
		if (status_flags->condition_global_position_valid && status_flags->condition_home_position_valid) {
			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_OFFBOARD:

		/* need offboard signal */
		// 需要外部信号
		if (!status_flags->offboard_control_signal_lost) {
			ret = TRANSITION_CHANGED;
		}

		break;

	case commander_state_s::MAIN_STATE_MAX:   // 13
	default:
		break;
	}

	if (ret == TRANSITION_CHANGED) {
		if (internal_state->main_state != new_main_state) {
			main_state_prev = internal_state->main_state;
			internal_state->main_state = new_main_state;  // 更新状态
			internal_state->timestamp = hrt_absolute_time();

		} else {
			ret = TRANSITION_NOT_CHANGED;
		}
	}

	return ret;
}

/**
 * Transition from one hil state to another
 * 从一个HIL状态转换到另一个状态
 */
transition_result_t hil_state_transition(hil_state_t new_state, orb_advert_t status_pub,
		struct vehicle_status_s *current_status, orb_advert_t *mavlink_log_pub)
{
	transition_result_t ret = TRANSITION_DENIED;

	if (current_status->hil_state == new_state) {
		ret = TRANSITION_NOT_CHANGED;

	} else {
		switch (new_state) {
		case vehicle_status_s::HIL_STATE_OFF:
			/* we're in HIL and unexpected things can happen if we disable HIL now */
			mavlink_and_console_log_critical(mavlink_log_pub, "Not switching off HIL (safety)");
			ret = TRANSITION_DENIED;
			break;

		case vehicle_status_s::HIL_STATE_ON:
			if (current_status->arming_state == vehicle_status_s::ARMING_STATE_INIT
			    || current_status->arming_state == vehicle_status_s::ARMING_STATE_STANDBY
			    || current_status->arming_state == vehicle_status_s::ARMING_STATE_STANDBY_ERROR) {

#ifdef __PX4_NUTTX
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
					mavlink_and_console_log_critical(mavlink_log_pub, "Switched to ON hil state");

				} else {
					/* failed opening dir */
					mavlink_log_info(mavlink_log_pub, "FAILED LISTING DEVICE ROOT DIRECTORY");
					ret = TRANSITION_DENIED;
				}

#else

				// Handle VDev devices
				const char *devname;
				unsigned int handle = 0;

				for (;;) {
					devname = px4_get_device_names(&handle);

					if (devname == NULL) {
						break;
					}

					/* skip mavlink */
					if (!strcmp("/dev/mavlink", devname)) {
						continue;
					}


					int sensfd = px4_open(devname, 0);

					if (sensfd < 0) {
						warn("failed opening device %s", devname);
						continue;
					}

					int block_ret = px4_ioctl(sensfd, DEVIOCSPUBBLOCK, 1);
					px4_close(sensfd);

					printf("Disabling %s: %s\n", devname, (block_ret == OK) ? "OK" : "ERROR");
				}


				// Handle DF devices
				const char *df_dev_path;
				unsigned int index = 0;

				for (;;) {
					if (DevMgr::getNextDeviceName(index, &df_dev_path) < 0) {
						break;
					}

					DevHandle h;
					DevMgr::getHandle(df_dev_path, h);

					if (!h.isValid()) {
						warn("failed opening device %s", df_dev_path);
						continue;
					}

					int block_ret = h.ioctl(DEVIOCSPUBBLOCK, 1);
					DevMgr::releaseHandle(h);

					printf("Disabling %s: %s\n", df_dev_path, (block_ret == OK) ? "OK" : "ERROR");
				}

				ret = TRANSITION_CHANGED;
				mavlink_and_console_log_critical(mavlink_log_pub, "Switched to ON hil state");
#endif

			} else {
				mavlink_and_console_log_critical(mavlink_log_pub, "Not switching to HIL when armed");
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
 * 检查失效保护和主状态，然后根据具体情况为navigator设置导航状态
 */
bool set_nav_state(struct vehicle_status_s *status, struct commander_state_s *internal_state,
		   const bool data_link_loss_enabled, const bool mission_finished,
		   const bool stay_in_failsafe, status_flags_s *status_flags, bool landed,
		   const bool rc_loss_enabled, const int offb_loss_act, const int offb_loss_rc_act)
{
	navigation_state_t nav_state_old = status->nav_state;

	bool armed = (status->arming_state == vehicle_status_s::ARMING_STATE_ARMED
		      || status->arming_state == vehicle_status_s::ARMING_STATE_ARMED_ERROR);
	status->failsafe = false;

	/* evaluate main state to decide in normal (non-failsafe) mode */
	// 评估主状态以决定正常（非失效保护）模式
	switch (internal_state->main_state) {
	case commander_state_s::MAIN_STATE_ACRO:
	case commander_state_s::MAIN_STATE_MANUAL:
	case commander_state_s::MAIN_STATE_RATTITUDE:
	case commander_state_s::MAIN_STATE_STAB:
	case commander_state_s::MAIN_STATE_ALTCTL:

		/* require RC for all manual modes */
		// 所有模式都需要遥控操作
		if (rc_loss_enabled && (status->rc_signal_lost || status_flags->rc_signal_lost_cmd) && armed) {
			status->failsafe = true;
			// 遥控信号丢失，进入失控保护模式

			if (status_flags->condition_global_position_valid && status_flags->condition_home_position_valid) {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RCRECOVER; 
				// GPS位置有效，飞行器进入遥控恢复模式?

			} else if (status_flags->condition_local_position_valid) {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;
				// 导航坐标系中的位置有效，飞行器进入自动降落模式

			} else if (status_flags->condition_local_altitude_valid) {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;
				// 导航坐标系高度有效，飞行器进入下降模式（无位置控制）

			} else {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION; // 否则，终止飞行
			}

		} else { // 遥控器正常
			switch (internal_state->main_state) {
			case commander_state_s::MAIN_STATE_ACRO:
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_ACRO;
				break;

			case commander_state_s::MAIN_STATE_MANUAL:
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;
				break;

			case commander_state_s::MAIN_STATE_RATTITUDE:
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_RATTITUDE;
				break;

			case commander_state_s::MAIN_STATE_STAB:
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_STAB;
				break;

			case commander_state_s::MAIN_STATE_ALTCTL:
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_ALTCTL;
				break;

			default:
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL; // 默认进入手动控制模式
				break;
			}
		}

		break;

	case commander_state_s::MAIN_STATE_POSCTL: {
			const bool rc_lost = rc_loss_enabled && (status->rc_signal_lost || status_flags->rc_signal_lost_cmd);

			if (rc_lost && armed) { // 解锁情况下遥控器信号丢失
				status->failsafe = true; // 进入失控保护模式

				if (status_flags->condition_global_position_valid &&
				    status_flags->condition_home_position_valid &&
				    !status_flags->gps_failure) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RCRECOVER;
				// GPS位置有效，飞行器进入遥控恢复模式?

				} else if (status_flags->condition_local_position_valid &&
					   !status_flags->gps_failure) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;
				// 导航坐标系位置有效并且GPS有效，飞行器进入自动降落模式

				} else if (status_flags->condition_local_altitude_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;
				// 导航坐标系高度有效，飞行器进入下降模式（无位置控制）

				} else {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION; //否则终止飞行
				}

				/* As long as there is RC, we can fallback to ALTCTL, or STAB. */
				/* A local position estimate is enough for POSCTL for multirotors,
				 * this enables POSCTL using e.g. flow.
				 * For fixedwing, a global position is needed. */
				// 只要有遥控信号，就可以切换到定高或者自稳模式
				// 对于多旋翼来说，有导航位置估计就足以进行定点控制，使用光流可以进入定点模式

			} else if (((status->is_rotary_wing && !status_flags->condition_local_position_valid) ||
				    (!status->is_rotary_wing && !status_flags->condition_global_position_valid))
				   && armed) { // 旋翼机，导航位置无效；或者非旋翼机，GPS位置无效
				status->failsafe = true; // 进入失控保护

				if (status_flags->condition_local_altitude_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_ALTCTL; // 定高

				} else {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_STAB; // 自稳
				}

			} else { // 非失效保护状态
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_POSCTL; // 定点
			}
		}
		break;

	case commander_state_s::MAIN_STATE_AUTO_MISSION:

		/* go into failsafe
		 * - if commanded to do so
		 * - if we have an engine failure
		 * - if we have vtol transition failure
		 * - depending on datalink, RC and if the mission is finished */
		/* 进入失效保护模式
		 * - 如果命令要求
		 * - 如果存在电机故障
		 * - 如果垂直起降机型变形失败
		 * - 取决于数据链、遥控信号以及任务是否完成
		 */

		/* first look at the commands */
		// 首先检查命令
		if (status->engine_failure_cmd) { // 电机故障
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL;

		} else if (status_flags->data_link_lost_cmd) { // 数据链丢失
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTGS; // 自动返回地面站

		} else if (status_flags->gps_failure_cmd) { // GPS故障
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND; 
			status->failsafe = true;

		} else if (status_flags->rc_signal_lost_cmd) { // 遥控信号丢失
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RCRECOVER;

		} else if (status_flags->vtol_transition_failure_cmd) { // 变形失败
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;// 返航

			/* finished handling commands which have priority, now handle failures */
			// 结束对高优先级的命令的处理，进行失效保护

		} else if (status_flags->gps_failure) { // GPS失效
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND; // 降落
			status->failsafe = true;

		} else if (status->engine_failure) { // 电机故障
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL; // 降落

		} else if (status_flags->vtol_transition_failure) { // 变形失败
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;  // 返航

		} else if (status->mission_failure) { // 任务失败
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;  // 返航

			/* datalink loss enabled:
			 * check for datalink lost: this should always trigger RTGS */
			/* 使能数据链丢失模式：
			 * 检查数据链是否丢失：这将触发飞行器自动返回地面站
			 */

		} else if (data_link_loss_enabled && status->data_link_lost) {
			status->failsafe = true;

			if (status_flags->condition_global_position_valid && status_flags->condition_home_position_valid) {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTGS;

			} else if (status_flags->condition_local_position_valid) {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;

			} else if (status_flags->condition_local_altitude_valid) {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;

			} else {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
			}

			/* datalink loss disabled:
			 * check if both, RC and datalink are lost during the mission
			 * or all links are lost after the mission finishes in air: this should always trigger RCRECOVER */
			/* 使能数据链丢失模式：
			 * 检查任务过程中数据链和遥控信号是否丢失
			 * 或者在任务结束后飞行器还在空中时所有链路都丢失了：这应该触发RCRECOVER模式
			 */

		} else if (!data_link_loss_enabled && status->rc_signal_lost && status->data_link_lost && !landed && mission_finished) {
			status->failsafe = true;

			if (status_flags->condition_global_position_valid && status_flags->condition_home_position_valid) {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RCRECOVER;

			} else if (status_flags->condition_local_position_valid) {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;

			} else if (status_flags->condition_local_altitude_valid) {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;

			} else {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
			}

			/* stay where you are if you should stay in failsafe, otherwise everything is perfect */

		} else if (!stay_in_failsafe) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_LOITER:

		/* go into failsafe on a engine failure */
		if (status->engine_failure) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL;

		} else if (status_flags->gps_failure) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;
			status->failsafe = true;

			/* also go into failsafe if just datalink is lost */
			// 如果仅有数据链丢失也进入失效保护

		} else if (status->data_link_lost && data_link_loss_enabled) {
			status->failsafe = true;

			if (status_flags->condition_global_position_valid && status_flags->condition_home_position_valid) {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTGS;

			} else if (status_flags->condition_local_position_valid) {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;

			} else if (status_flags->condition_local_altitude_valid) {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;

			} else {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
			}

			/* go into failsafe if RC is lost and datalink loss is not set up */
			// 如果遥控信号丢失并且数据链丢失未设置，也进入失效保护

		} else if (status->rc_signal_lost && !data_link_loss_enabled) {
			status->failsafe = true;

			if (status_flags->condition_global_position_valid && status_flags->condition_home_position_valid) {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTGS;

			} else if (status_flags->condition_local_position_valid) {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;

			} else if (status_flags->condition_local_altitude_valid) {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;

			} else {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
			}

			/* don't bother if RC is lost if datalink is connected */
			// 在数据链连上的情况下遥控信号丢失不做任何操作

		} else if (status->rc_signal_lost) {

			/* this mode is ok, we don't need RC for loitering */
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;

		} else {
			/* everything is perfect */
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_RTL:

		/* require global position and home, also go into failsafe on an engine failure */

		if (status->engine_failure) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL;

		} else if (status_flags->gps_failure) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;
			status->failsafe = true;

		} else if ((!status_flags->condition_global_position_valid ||
			    !status_flags->condition_home_position_valid)) {
			status->failsafe = true;

			if (status_flags->condition_local_position_valid) {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;

			} else if (status_flags->condition_local_altitude_valid) {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;

			} else {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
			}

		} else {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_FOLLOW_TARGET:

		/* require global position and home */
		// 需要GPS位置以及起飞点位置

		if (status->engine_failure) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL;

		} else if (!status_flags->condition_global_position_valid) {
			status->failsafe = true;

			if (status_flags->condition_local_position_valid) {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;

			} else if (status_flags->condition_local_altitude_valid) {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;

			} else {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
			}

		} else {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_TAKEOFF:

		/* require global position and home */
		// 需要GPS位置以及起飞点位置

		if (status->engine_failure) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL;

		} else if (status_flags->gps_failure || (!status_flags->condition_global_position_valid ||
				!status_flags->condition_home_position_valid)) {
			status->failsafe = true;

			if (status_flags->condition_local_position_valid) {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;

			} else if (status_flags->condition_local_altitude_valid) {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;

			} else {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
			}

		} else {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF;
		}

		break;

	case commander_state_s::MAIN_STATE_AUTO_LAND:

		/* require global position and home */
		// 需要GPS位置以及起飞点位置

		if (status->engine_failure) {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL;

		} else if (status_flags->gps_failure || (!status_flags->condition_global_position_valid ||
				!status_flags->condition_home_position_valid)) {
			status->failsafe = true;

			if (status_flags->condition_local_altitude_valid) {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;

			} else {
				status->nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
			}

		} else {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;
		}

		break;

	case commander_state_s::MAIN_STATE_OFFBOARD:

		/* require offboard control, otherwise stay where you are */
		if (status_flags->offboard_control_signal_lost && !status->rc_signal_lost) {
			status->failsafe = true;

			if (status_flags->offboard_control_loss_timeout && offb_loss_rc_act < 5 && offb_loss_rc_act >= 0) {
				if (offb_loss_rc_act == 3 && status_flags->condition_global_position_valid
				    && status_flags->condition_home_position_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;

				} else if (offb_loss_rc_act == 0 && status_flags->condition_global_position_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_POSCTL;

				} else if (offb_loss_rc_act == 1 && status_flags->condition_local_altitude_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_ALTCTL;

				} else if (offb_loss_rc_act == 2) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;

				} else if (offb_loss_rc_act == 4 && status_flags->condition_global_position_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;

				} else if (status_flags->condition_local_altitude_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;

				} else {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
				}

			} else {
				if (status_flags->condition_global_position_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_POSCTL;

				} else if (status_flags->condition_local_altitude_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_ALTCTL;

				} else {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
				}
			}

		} else if (status_flags->offboard_control_signal_lost && status->rc_signal_lost) {
			status->failsafe = true;

			if (status_flags->offboard_control_loss_timeout && offb_loss_act < 3 && offb_loss_act >= 0) {
				if (offb_loss_act == 2 && status_flags->condition_global_position_valid
				    && status_flags->condition_home_position_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;

				} else if (offb_loss_act == 1 && status_flags->condition_global_position_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;

				} else if (offb_loss_act == 0 && status_flags->condition_global_position_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;

				} else if (status_flags->condition_local_altitude_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;

				} else {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
				}

			} else {
				if (status_flags->condition_global_position_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;

				} else if (status_flags->condition_local_altitude_valid) {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_DESCEND;

				} else {
					status->nav_state = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
				}
			}

		} else {
			status->nav_state = vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
		}

	default:
		break;
	}

	return status->nav_state != nav_state_old;
}

int preflight_check(struct vehicle_status_s *status, orb_advert_t *mavlink_log_pub, bool prearm, bool force_report,
		    status_flags_s *status_flags, battery_status_s *battery, bool can_arm_without_gps)
{
	/*
	 */
	bool reportFailures = force_report || (!status_flags->condition_system_prearm_error_reported &&
					       status_flags->condition_system_hotplug_timeout);

	bool checkAirspeed = false;

	/* Perform airspeed check only if circuit breaker is not
	 * engaged and it's not a rotary wing */
	// 只要断路器空闲并且机型不是旋翼机，就执行空速检查
	if (!status_flags->circuit_breaker_engaged_airspd_check && (!status->is_rotary_wing || status->is_vtol)) {
		checkAirspeed = true;
	}

	bool preflight_ok = Commander::preflightCheck(mavlink_log_pub, true, true, true, true,
			    checkAirspeed, (status->rc_input_mode == vehicle_status_s::RC_IN_MODE_DEFAULT),
			    !can_arm_without_gps, true, reportFailures);

	if (!status_flags->circuit_breaker_engaged_usb_check && status_flags->usb_connected && prearm) {
		preflight_ok = false;

		if (reportFailures) {
			mavlink_and_console_log_critical(mavlink_log_pub, "ARMING DENIED: Flying with USB is not safe");
		}
	}

	if (battery->warning == battery_status_s::BATTERY_WARNING_CRITICAL) {
		preflight_ok = false;

		if (reportFailures) {
			mavlink_and_console_log_critical(mavlink_log_pub, "ARMING DENIED: VERY LOW BATTERY");
		}
	}

	/* report once, then set the flag */
	// 报告一次，然后设置标志位
	if (reportFailures && !preflight_ok) {
		status_flags->condition_system_prearm_error_reported = true;
	}

	return !preflight_ok;   // 不给飞
}
