/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file companion_process_status.cpp
 * Monitor health status of companion processes
 */

#include <drivers/drv_hrt.h>
#include "companion_process_status.h"

static orb_advert_t *mavlink_log_pub;


CompanionProcessStatus::CompanionProcessStatus(){

	//initialize time
	_time_zero = hrt_absolute_time();
	_time_message = hrt_absolute_time();
}

void CompanionProcessStatus::poll_subscriptions(){

    if (_companion_process_status_sub.update()) {
		_new_status_received = true;
	}
}

void CompanionProcessStatus::determine_required_processes(){

	//determine required processes from parameters
	param_t _param_mpc_obs_avoid = param_find("MPC_OBS_AVOID");
	param_get(_param_mpc_obs_avoid, &_avoidance_required);

}

void CompanionProcessStatus::determine_action(){

	bool reported_before = false;
	companion_process_status_s last_status;

	if(_new_status_received){
		//data for obstacle avoidance is at array location 0, data for VIO at location 1
		if(_companion_process_status_sub.get().component == (int)MAV_COMPONENT::MAV_COMP_ID_OBSTACLE_AVOIDANCE){
			_process_type = ProcessType::Avoidance;
		}else if(_companion_process_status_sub.get().component == (int)MAV_COMPONENT::MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY){
			_process_type = ProcessType::VIO;
		}else{
			return;
		}

		//find last status of same process
		if(_companion_process_status_history[(int)_process_type].timestamp == 0){
			_companion_process_status_history[(int)_process_type] = _companion_process_status_sub.get();
			reported_before = false;
		}else{
			last_status = _companion_process_status_history[(int)_process_type];
			_companion_process_status_history[(int)_process_type] = _companion_process_status_sub.get();
			reported_before = true;
		}


		if(reported_before){
			if(last_status.state != _companion_process_status_sub.get().state){
				if (_companion_process_status_sub.get().state == companion_process_status_s::MAV_STATE_BOOT){
					_first_registration_time[(int)_process_type] = _companion_process_status_sub.get().timestamp;
				}else if(_companion_process_status_sub.get().state == companion_process_status_s::MAV_STATE_ACTIVE){
					mavlink_log_info(mavlink_log_pub, "companion process %s is ready", toString(_process_type));
				}else{
					mavlink_log_critical(mavlink_log_pub, "companion process %s %s", toString(_process_type), toString((MAV_STATE)_companion_process_status_sub.get().state));
				}
			}
		}

		//check for timeouts in starting up
		if(_companion_process_status_sub.get().state == companion_process_status_s::MAV_STATE_BOOT &&
				_companion_process_status_sub.get().timestamp - _first_registration_time[(int)_process_type] > STARTUP_TIMEOUT){
			mavlink_log_critical(mavlink_log_pub,"companion process %s taking too long to start", toString(_process_type));
		}
		_new_status_received = false;
	}

	//check if required processes are missing
	if(_time_message + THROTTLE_MESSAGES < hrt_absolute_time()){
		bool message_sent = false;

		if(_avoidance_required && _companion_process_status_history[(int)ProcessType::Avoidance].component != (int)MAV_COMPONENT::MAV_COMP_ID_OBSTACLE_AVOIDANCE && hrt_absolute_time() - _time_zero > NO_SIGNAL_TIMEOUT){
			mavlink_log_critical(mavlink_log_pub, "Obstacle avoidance did not start");
			message_sent = true;
		}


		if(_avoidance_required && _companion_process_status_history[(int)ProcessType::Avoidance].component == (int)MAV_COMPONENT::MAV_COMP_ID_OBSTACLE_AVOIDANCE && _companion_process_status_history[(int)ProcessType::Avoidance].timestamp + NO_SIGNAL_TIMEOUT < hrt_absolute_time()){
			mavlink_log_critical(mavlink_log_pub, "Obstacle avoidance process died");
			message_sent = true;
		}

		if(_vio_required && _companion_process_status_history[(int)ProcessType::VIO].component != (int)MAV_COMPONENT::MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY && hrt_absolute_time() - _time_zero > NO_SIGNAL_TIMEOUT){
			mavlink_log_critical(mavlink_log_pub, "VIO process did not start");
			message_sent = true;
		}


		if(_vio_required && _companion_process_status_history[(int)ProcessType::VIO].component == (int)MAV_COMPONENT::MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY && _companion_process_status_history[(int)ProcessType::VIO].timestamp + NO_SIGNAL_TIMEOUT < hrt_absolute_time()){
			mavlink_log_critical(mavlink_log_pub, "VIO process died");
			message_sent = true;
		}

		if(message_sent){
			_time_message = hrt_absolute_time();
		}
	}
}

void CompanionProcessStatus::check_companion_process_status(orb_advert_t *mav_log_pub){
	mavlink_log_pub = mav_log_pub;
	determine_required_processes();
	poll_subscriptions();
	determine_action();
}

const char* CompanionProcessStatus::toString(ProcessType type){
	switch (type)
	{
		case ProcessType::Avoidance: return "AVOIDANCE";
		case ProcessType::VIO: return "VIO";
		default: return "[Unknown process type]";
	}
}

const char* CompanionProcessStatus::toString(MAV_STATE state){
	switch (state)
	{
		case MAV_STATE::UNINITIALIZED: return "UNINITIALIZED";
		case MAV_STATE::STARTING: return "STARTING";
		case MAV_STATE::CALIBRATING: return "CALIBRATING";
		case MAV_STATE::STANDBY: return "STANDBY";
		case MAV_STATE::ACTIVE: return "ACTIVE";
		case MAV_STATE::TIMEOUT: return "TIMEOUT";
		case MAV_STATE::EMERGENCY: return "EMERGENCY";
		case MAV_STATE::POWEROFF: return "POWEROFF";
		case MAV_STATE::ABORTING: return "ABORTING";
		default: return "[Unknown process state]";
	}
}
