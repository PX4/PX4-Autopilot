/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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


Companion_Process_Status::Companion_Process_Status(){

	//initialize time
	_time_zero = hrt_absolute_time();
	_time_message = hrt_absolute_time();
}

void Companion_Process_Status::poll_subscriptions(){

    if (_companion_process_status_sub.update()) {
		_companion_process_status = _companion_process_status_sub.get();
		_new_status_received = true;
	}
}

void Companion_Process_Status::determine_required_processes(int32_t use_obs_avoid){

	//reset required processes
	for(int i = 0; i<3; i++){
		_required_processes[i] = 0;
	}

	//determine required processes from parameters
	if(use_obs_avoid){
		_required_processes[1] = 1;
	}

}

void Companion_Process_Status::determine_action(){

	bool reported_before = false;
	companion_process_status_s last_status;
	int process_number = 11;

	if(_new_status_received){
		const char* message_type = _companion_process_types[_companion_process_status.type];
		const char* message_state = _companion_process_states[_companion_process_status.state];

		//find last status of same process
		for(int i = 0; i<10; i++){
			if(_companion_process_status_history[i].pid == _companion_process_status.pid){
				last_status = _companion_process_status_history[i];
				_companion_process_status_history[i] = _companion_process_status;
				process_number = i;
				reported_before = true;
			}
		}

		if(!reported_before){
			for(int i = 0; i<10; i++){
				if(_companion_process_status_history[i].pid == 0  && !reported_before){
					_companion_process_status_history[i] = _companion_process_status;
					reported_before = true;
					process_number = i;
				}
			}
		}else{
			if((last_status.state != _companion_process_status.state)){
				if (_companion_process_status.state == companion_process_status_s::COMPANION_PROCESS_STATE_STARTING){
					_companion_process_status_first_registration[process_number] = _companion_process_status;
				}else if(_companion_process_status.state == companion_process_status_s::COMPANION_PROCESS_STATE_HEALTHY){
					mavlink_log_info(mavlink_log_pub, "companion process %s is ready\n", message_type);
				}else{
					mavlink_log_critical(mavlink_log_pub, "companion process %s %s\n", message_type, message_state);
				}
			}
		}


		//check for timeouts in starting up
		if(_companion_process_status.state == companion_process_status_s::COMPANION_PROCESS_STATE_STARTING &&
				_companion_process_status.timestamp - _companion_process_status_first_registration[process_number].timestamp > STARTUP_TIMEOUT){
			mavlink_log_critical(mavlink_log_pub,"companion process %s taking too long to start\n", message_type);
		}
		_new_status_received = false;
	}

	//check if required processes are missing
	if(_time_message + THROTTLE_MESSAGES < hrt_absolute_time()){
		for(int j = 0; j < 3; j++){
			int processes_found = 0;
			for(int i = 0; i<10; i++){
				if(_companion_process_status_history[i].type == j && _companion_process_status_history[i].pid != 0){
					processes_found ++;
				}
			}
			if(processes_found < _required_processes[j] && hrt_absolute_time() - _time_zero > NO_SIGNAL_TIMEOUT){
				mavlink_log_critical(mavlink_log_pub, "%s did not start\n", _companion_process_types[j]);
				_time_message = hrt_absolute_time();
			}
		}
		for(int i = 0; i<10; i++){
			if(_companion_process_status_history[i].pid != 0 && _companion_process_status_history[i].timestamp + NO_SIGNAL_TIMEOUT < hrt_absolute_time()){
				mavlink_log_critical(mavlink_log_pub, "%s process died\n", _companion_process_types[_companion_process_status_history[i].type]);
				_time_message = hrt_absolute_time();
			}
		}
	}
}

void Companion_Process_Status::check_companion_process_status(orb_advert_t *mav_log_pub, int32_t use_obs_avoid){
	mavlink_log_pub = mav_log_pub;
	determine_required_processes(use_obs_avoid);
	poll_subscriptions();
	determine_action();
}

