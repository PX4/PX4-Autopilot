/****************************************************************************
 *
 *   Copyright (c) 2020-2023 PX4 Development Team. All rights reserved.
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

#include "ff_escs.h"

ModuleBase::Descriptor FFescOutput::desc{FFescOutput::task_spawn, FFescOutput::custom_command, FFescOutput::print_usage};

#include <math.h>
#include <float.h>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/sem.hpp>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/actuator_armed.h>

// hardware specific includes
#include <board_config.h>
#include <nuttx/can/can.h>

#ifdef CONFIG_ARCH_CHIP_STM32
#include "stm32.h"
#endif
#include "stm32_can.h"

// #undef PX4_DEBUG
// #define PX4_DEBUG PX4_INFO

/*
 * RELEVANT INFOs:
 *
 * The driver has been developed following the design principles:
 * - during arming/disarming sequences no telemetry is request. This has been done to avoid potential message collisions on the CAN bus.
 * - when an ESC is not ACKing back an arm request the driver will abort arming sequence and command disarm to ALL the ESCs.
 * - during the disarming sequence, if an ESC is not ACKing back we'll just keep on disarming all the other ESCs
 * - PWM signal going AND the CAN command need to match value. For this reason the driver needs to also do the PWM spoolup (enabled when instanciating _mixing_output)
 * - At power up we need to wait 4 seconds before publishing esc_status. This is required as the ESCs might still be booting up and thus fire some fault flags (due to the fact that they are not yet ready)
 *
 * Motors timing:
 * Within the first 500ms after arming the system will spoolup the PWM values from DISARMED to IDLE. during this time the driver will send arm commands to the motors.
 * In order to guarantee that we can catch the case where an ESC is not ACKing back an Arm request and thus trigger a disarm, there is a timeout threshold of 25ms.
 *  Given that the PWM spoolup time lasts 500ms by setting a timeout to 25ms we can guarantee that in the worst case scenario we don't exceed 500ms.
 *
 * Worst case scenario (Quadcopter): All 4 motors don't ack back to an ARM request
 * In enable_esc_outputs:max_count = 10 the loop will wait up to 10*2ms = 20ms for each motor -> 4*20ms = 80ms
 *
 * Worst case scenario (Octacoper): 8*20ms = 160ms
 *
 */

FFescOutput::FFescOutput() : OutputModuleInterface(MODULE_NAME, px4::wq_configurations::ff_escs)
{
	PX4_INFO("FF CAN ESC driver started");
}

FFescOutput::~FFescOutput()
{
	perf_free(_comms_errors);
	perf_free(_cycle_perf);
	::close(_file_descriptor);
	PX4_INFO("driver stopped");
}

void
FFescOutput::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		exit_and_cleanup(desc);
		return;
	}

	if (initialize_driver() != PX4_OK) {
		PX4_ERR("Failed to initialize. Aborting");
		return;
	}

	perf_begin(_cycle_perf);

	_mixing_output.update();

	// Reset answer timeout when disarmed
	if (!_armed_motors) {
		_esc_answer_timeout = false;
	}

	// Request ESCs to go into desired arming state
	const bool arm_motors = _armed_motors && !_esc_answer_timeout;

	for (int index = 0; index < MOTOR_COUNT; index++) {
		uint8_t index_mask = 1 << index;
		const bool need_to_arm = arm_motors && ((index_mask & _armed_esc_mask) == 0);
		const bool need_to_disarm = !arm_motors && ((index_mask & ~_armed_esc_mask) == 0);
		const bool ready_for_next_request = _arming_state_requested_time == 0;

		// Send next request to change arming state
		if ((need_to_arm || need_to_disarm) && ready_for_next_request) {
			if (need_to_arm) {
				enable_esc_output(index);
				PX4_DEBUG("Arm command ESC %d", index);
				_armed_esc_mask |= index_mask;
			}

			if (need_to_disarm) {
				disable_esc_output(index);
				PX4_DEBUG("Disarm command ESC %d", index);
				_armed_esc_mask &= ~index_mask;
			}

			_arming_state_acknowledged_esc_mask &= ~index_mask;
			_arming_state_request_index = index;
			_arming_state_requested_time = hrt_absolute_time();
		}
	}

	if (!_esc_answer_timeout && _arming_state_requested_time > 0) {

		_esc_answer_timeout = hrt_elapsed_time(&_arming_state_requested_time) > 25_ms;

		if (_esc_answer_timeout) {
			_arming_state_requested_time = 0;
			PX4_ERR("ESC %d answer timed out", _arming_state_request_index);
		}

	} else {
		request_esc_telemetry(_telemetry_index);
		_telemetry_index++;

		if (_telemetry_index > MOTOR_COUNT) {
			_telemetry_index = 1;
		}

		_led_control.updateLeds(arm_motors, _esc_status_pub.get().esc_online_flags);
	}

	read_messages();

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}

	// publish when motor index wraps (which is robust against motor timeouts)
	if (_publish_esc_status) {
		// skip telemetry just after boot to avoid false positive failures being published
		if (hrt_elapsed_time(&_time_at_boot) > 4_s) {
			esc_status_s &esc_status = _esc_status_pub.get();
			esc_status.esc_online_flags = check_online_escs(esc_status);
			esc_status.esc_armed_flags = _armed_esc_mask;
			++esc_status.counter;
			esc_status.timestamp = hrt_absolute_time();
			_esc_status_pub.update();
		}

		_publish_esc_status = false;
	}

	const can_msg_s *cmd = _new_command.load();

	if (cmd) {
		if (!_armed_motors) {
			send_can_msg(*cmd);

		} else {
			PX4_INFO("Commands are only allowed while vehicle is disarmed.");
		}

		_new_command.store(nullptr);
	}

	_mixing_output.updateSubscriptions(false);

	perf_end(_cycle_perf);
}

int FFescOutput::task_spawn(int argc, char *argv[])
{
	FFescOutput *instance = new FFescOutput();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int FFescOutput::init()
{
	ScheduleNow();
	return PX4_OK;
}

int FFescOutput::custom_command(int argc, char *argv[])
{
	const char *input = argv[0];

	// Module must be running to run these commands
	if (!is_running(desc)) {
		PX4_ERR("not running");
		return PX4_ERROR;
	}

	if (argc < 3) {
		print_usage("Missing motor index parameter.");
		return 1;
	}

	// If command is reverse

	int motor_index = -1; // select motor index, default: -1=all
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "m:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'm':
			motor_index = atoi(myoptarg);
			break;

		default:
			return print_usage("unrecognized flag");
		}
	}

	if (motor_index < 1 || motor_index > 16) {
		print_usage("Motor Index out of boundaries.");
		return PX4_ERROR;
	}

	if (!strcmp(input, "reverse")) {

		PX4_INFO("Reversing motor %d", motor_index);
		get_instance<FFescOutput>(desc)->send_command_thread_safe(motor_index);

		return PX4_OK;
	}

	// If we got here, we didn't recognize the command
	return print_usage("unknown command");
}

int FFescOutput::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ff_escs", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("reverse", "Permanently reverses spin direction of specified motor.");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 1, 16, "Motor index to reverse (1-16). Must be disarmed. ", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int FFescOutput::print_status()
{
	PX4_INFO("Running");
	perf_print_counter(_cycle_perf);
	perf_print_counter(_comms_errors);
	_mixing_output.printStatus();
	return 0;
}

bool FFescOutput::updateOutputs(float outputs[MOTOR_COUNT], unsigned num_outputs, unsigned num_control_groups_updated)
{
	_armed_motors = false;

	// Infer if ESCs should be armed
	// On Astro 900 is disarmed, 1075 is armed
	// If one of the motors needs to spin they need to be armed
	for (int i = 0; i < MOTOR_COUNT; i++) {
		if (outputs[i] > 1000.f) {
			_armed_motors = true;
		}
	}

	for (int i = 0; i < MOTOR_COUNT; i++) {
		localPWMList[i] = convert_pwm_to_can(outputs[i]);
	}

	// Setup message
	can_msg_s msg{};
	msg.cm_hdr.ch_id = CAN_ADDR_CONTROL_1;		// Set channel id as 0b101011.
	msg.cm_hdr.ch_rtr = 0;				// Do not turn on Remote Transmission Request.
	msg.cm_hdr.ch_dlc = 8; 				// Data Length Code: = 8 Bytes.

	// set msg payload to command
	msg.cm_data[0] = localPWMList[0] & 0xFF;
	msg.cm_data[1] = (localPWMList[0] >> 8) & 0xFF;

	msg.cm_data[2] = localPWMList[1] & 0xFF;
	msg.cm_data[3] = (localPWMList[1] >> 8) & 0xFF;

	msg.cm_data[4] = localPWMList[2] & 0xFF;
	msg.cm_data[5] = (localPWMList[2] >> 8) & 0xFF;

	msg.cm_data[6] = localPWMList[3] & 0xFF;
	msg.cm_data[7] = (localPWMList[3] >> 8) & 0xFF;

	// send the message
	send_can_msg(msg);

	// set msg payload to command
	if (MOTOR_COUNT >= 6) {

		/* TODO: This section can be optimized */
		msg = {};

		msg.cm_hdr.ch_id = CAN_ADDR_CONTROL_2;
		msg.cm_hdr.ch_rtr = 0;				// Do not turn on Remote Transmission Request.
		msg.cm_hdr.ch_dlc = 8; 				// Data Length Code: = 8 Bytes.

		msg.cm_data[0] = localPWMList[4] & 0xFF;
		msg.cm_data[1] = (localPWMList[4] >> 8) & 0xFF;

		msg.cm_data[2] = localPWMList[5] & 0xFF;
		msg.cm_data[3] = (localPWMList[5] >> 8) & 0xFF;

		if (MOTOR_COUNT >= 8) {
			msg.cm_data[4] = localPWMList[6] & 0xFF;
			msg.cm_data[5] = (localPWMList[6] >> 8) & 0xFF;

			msg.cm_data[6] = localPWMList[7] & 0xFF;
			msg.cm_data[7] = (localPWMList[7] >> 8) & 0xFF;
		}

		// send the message
		send_can_msg(msg);
	}

	return true;
}

int FFescOutput::initialize_driver()
{
	// File descriptor initialized?
	if (_file_descriptor >= 0) {
		return PX4_OK;
	}

	// Configure port flags for read/write, non-controlling, non-blocking
	int flags = (O_RDWR | O_NONBLOCK);

	// Open the CAN port
	_file_descriptor = ::open("/dev/can1", flags);

	if (_file_descriptor < 0) {
		PX4_ERR("Failed to open CAN device. ERROR (%i)", errno);
		return PX4_ERROR;
	}

	if (::ioctl(_file_descriptor, CANIOC_SET_ABOM, 1) != 0) {
		PX4_ERR("failed to set ABOM");
		return -1;
	}

	if (::ioctl(_file_descriptor, CANIOC_SET_NART, 1) != 0) {
		PX4_ERR("failed to set NART");
		return -1;
	}

	_led_control.init(_file_descriptor);

	_time_at_boot = hrt_absolute_time();

	// Publish an initial esc_status
	// This guarantees that if no ESCs are physically connected Commander will deny arming
	esc_status_s &esc_status = _esc_status_pub.get();
	esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_CAN;
	esc_status.esc_count = MOTOR_COUNT;
	esc_status.timestamp = _time_at_boot;
	_esc_status_pub.update();

	return PX4_OK;

}

int FFescOutput::read_can_data(uint8_t *buf, int len)
{
	ssize_t bytes_read = ::read(_file_descriptor, buf, len);

	// Check if read is successful.
	if ((size_t)bytes_read < CAN_MSGLEN(0) || bytes_read < 0) {
		// Do not parse the data
		return PX4_ERROR;

	} else {
		// Message is good to be parsed!
		return bytes_read;
	}

	// Shouldn't reach here.
	return -2;
}

void FFescOutput::parse_data(struct can_msg_s &msg_p)
{
	int index = 0;
	uint32_t uid = 0;

	switch (msg_p.cm_hdr.ch_id) {

	case CAN_UID_RESPONSE:

		memcpy(&uid, &msg_p.cm_data[0], sizeof(uint32_t));

		// check/build the ID list
		for (int i = 0; i < MOTOR_COUNT; i++) {
			// break out if we have already added it
			if (_bootloader_ids[i] == uid) {
				break;

				// add it and break out if we have an empty space in the list

			} else if (_bootloader_ids[i] == 0) {
				_bootloader_ids[i] = uid;
				break;
			}
		}


		break;

	case CAN_ADDR_ARM_ACK:

		index = msg_p.cm_data[0] - 1;

		if ((index < MOTOR_COUNT) && (index >= 0)) {
			if (_arming_state_request_index == index) {
				PX4_DEBUG("Arm ACK from ESC %d",index);
				_arming_state_acknowledged_esc_mask |= (1 << index);
				_publish_esc_status = true;
				_arming_state_requested_time = 0;
			} else {
				PX4_ERR("Arm ACK from different ID!");
			}
		}

		break;

	case CAN_ADDR_DISARM_ACK:

		index = msg_p.cm_data[0] - 1;

		if ((index < MOTOR_COUNT) && (index >= 0)) {
			if (_arming_state_request_index == index) {
				PX4_DEBUG("Disarm ACK from ESC %d",index);
				_arming_state_acknowledged_esc_mask |= (1 << index);
				_publish_esc_status = true;
				_arming_state_requested_time = 0;
			} else {
				PX4_ERR("Disarm ACK from different ID!");
			}

		}

		break;

	case CAN_ADDR_TELEMETRY_RESPONSE:
	{
		EscTelemetry *esc_telemetry = (EscTelemetry *)msg_p.cm_data;

		if (esc_telemetry->esc_id < 1 || MOTOR_COUNT < esc_telemetry->esc_id) {
			break;
		}

		esc_status_s &esc_status = _esc_status_pub.get();
		esc_report_s &esc_report = esc_status.esc[esc_telemetry->esc_id - 1];

		esc_report.timestamp = hrt_absolute_time();
		esc_report.esc_voltage = ((float)esc_telemetry->voltage / 0.064f) / 1000.0f;
		esc_report.esc_current = ((float)esc_telemetry->current - 32768.0f) * 0.1f; // Phase current, Convert to Amps
		esc_report.esc_state = esc_telemetry->fault_state;
		esc_report.motor_temperature = esc_telemetry->motor_temperature - 55; // temperature field is unsigned but +55 degree
		esc_report.esc_rpm = esc_telemetry->rpm;

		uint8_t failures_flags_low = eval_telem1_failures(esc_telemetry->fault_state);
			failures_flags_low |= check_motor_temperature((float) esc_report.motor_temperature, esc_telemetry->esc_id - 1);

		esc_report.failures = (esc_report.failures & (0xff << 8)) | failures_flags_low;
		esc_report.actuator_function = (uint8_t)_mixing_output.outputFunction(esc_telemetry->esc_id - 1);

		break;
	}

	case CAN_ADDR_TELEMETRY_RESPONSE_2:
	{
		EscTelemetry2 *esc_telemetry = (EscTelemetry2 *)msg_p.cm_data;

		if (esc_telemetry->esc_id < 1 || MOTOR_COUNT < esc_telemetry->esc_id) {
			break;
		}

		esc_status_s &esc_status = _esc_status_pub.get();
		esc_report_s &esc_report = esc_status.esc[esc_telemetry->esc_id - 1];

		esc_report.timestamp 		= hrt_absolute_time();
		esc_report.esc_control_state 	= esc_telemetry->esc_control_state;
		esc_report.esc_temperature 	= (float)esc_telemetry->esc_temperature - 55.0f;	// temperature report is actually temp+55. Subtract 55 to get actual temp.
		esc_report.motor_flux 		= esc_telemetry->motor_flux;		// motor flux estimate (wb*100000)
		esc_report.esc_errorcount 	= esc_telemetry->reserved;

		uint16_t failures_flags_high  = eval_telem2_failures(esc_telemetry->reserved);
			 failures_flags_high |= check_esc_temperature((float) esc_report.esc_temperature , esc_telemetry->esc_id - 1);

		esc_report.failures = failures_flags_high | (esc_report.failures & 0xff);

		// the index has wrapped, flag it to allow publishing esc_status
		if (esc_telemetry->esc_id <= _last_telemetry_esc_id) {
			_publish_esc_status = true;
		}

		_last_telemetry_esc_id = esc_telemetry->esc_id;

		break;
	}

	case CAN_SET_ADDR_JUMP_ACK:
	case CAN_EXECUTE_JUMP_ACK:
	default:
		break;

	}
}

uint16_t FFescOutput::convert_pwm_to_can(float pwmIn)
{
	// The CAN PWM value is in units of microseconds * 16
	long pwmOut = lroundf(pwmIn * 16.f);

	// absolute maximum is 2000us * 16 and we can have 0 zero pwmouts.
	return math::constrain(pwmOut, 0L, 2000L * 16L);
}

int FFescOutput::send_can_msg(const struct can_msg_s &msg_p)
{
	// Variables for controlling CAN transmission
	size_t msgsize;
	ssize_t bytes_written;

	// CAN_MSGLEN is a convenience macro.
	// It gets the data length of the message and computes the actual size of the CAN message.
	msgsize = CAN_MSGLEN(msg_p.cm_hdr.ch_dlc);

	// Do the write operation
	bytes_written = ::write(_file_descriptor,(uint8_t *)&msg_p, msgsize);

	// Check if message send is success
	if ((size_t)bytes_written != msgsize || bytes_written < 0) {
		perf_count(_comms_errors);
		return PX4_ERROR;

	} else {
		// Write success
		return PX4_OK;
	}

	// Shouldn't reach here.
	return -2;
}

int FFescOutput::send_command_thread_safe(uint8_t motor_index){
	struct can_msg_s msg {};
	PX4_INFO("sending reverse command 0x%x to motor %d", CAN_ADDR_REVERSE, motor_index);

	// Setup message
	msg.cm_hdr.ch_id  = CAN_ADDR_REVERSE;	// 0x3F is the reverse flag
	msg.cm_hdr.ch_rtr = 0;
	msg.cm_hdr.ch_dlc = 8;

	// first byte is motor ID
	msg.cm_data[0] = motor_index;

	// Magic numbers for the packet
	msg.cm_data[1] = (CAN_VERIFICATION_REVERSE_LR >> 8)  & 0xFF;
	msg.cm_data[2] = (CAN_VERIFICATION_REVERSE_LR >> 16) & 0xFF;
	msg.cm_data[3] = (CAN_VERIFICATION_REVERSE_LR >> 24) & 0xFF;
	msg.cm_data[4] = (CAN_VERIFICATION_REVERSE_HR >> 0)  & 0xFF;
	msg.cm_data[5] = (CAN_VERIFICATION_REVERSE_HR >> 8)  & 0xFF;
	msg.cm_data[6] = (CAN_VERIFICATION_REVERSE_HR >> 16) & 0xFF;
	msg.cm_data[7] = (CAN_VERIFICATION_REVERSE_HR >> 24) & 0xFF;

	PX4_INFO("arm magic: %x %x %x %x %x %x %x %x", msg.cm_data[0], msg.cm_data[1], msg.cm_data[2], msg.cm_data[3], msg.cm_data[4],
		msg.cm_data[5], msg.cm_data[6], msg.cm_data[7]);

	_new_command.store(&msg);

	// wait until main thread processed it
	while (_new_command.load()) {
		px4_usleep(1000);
	}
	PX4_INFO("Reverse command sent. Reboot and check, command can be missed.");
	return PX4_OK;
}

void FFescOutput::request_esc_telemetry(uint8_t current_esc)
{
	struct can_msg_s msg {};

	msg.cm_hdr.ch_id = 0x2A;	// 0x2A or 42 or 0b101010 for telemetry request packet
	msg.cm_hdr.ch_rtr = 0;		// Do not turn on Remote Transmission Request.
	msg.cm_hdr.ch_dlc = 1; 		// Data Length Code: = 1 Byte (motor ID)

	// cycle through motor IDs to get telem round robin
	msg.cm_data[0] = current_esc;

	send_can_msg(msg);
}

void FFescOutput::enable_esc_output(const uint8_t index)
{
	struct can_msg_s msg {};
	msg.cm_hdr.ch_id = CAN_ADDR_ARM_SET;
	msg.cm_hdr.ch_rtr = 0;
	msg.cm_hdr.ch_dlc = 8;
	msg.cm_data[0] = index + 1;
	msg.cm_data[1] = 0x53;
	msg.cm_data[2] = 0x8d;
	msg.cm_data[3] = 0x02;
	msg.cm_data[4] = 0x44;
	msg.cm_data[5] = 0x8e;
	msg.cm_data[6] = 0x25;
	msg.cm_data[7] = 0x91;
	send_can_msg(msg);

}

void FFescOutput::disable_esc_output(const uint8_t index)
{
	struct can_msg_s msg {};
	msg.cm_hdr.ch_id = CAN_ADDR_DISARM_SET;
	msg.cm_hdr.ch_rtr = 0;
	msg.cm_hdr.ch_dlc = 8;
	msg.cm_data[0] = index + 1;
	msg.cm_data[1] = 0x5c;
	msg.cm_data[2] = 0x52;
	msg.cm_data[3] = 0x2b;
	msg.cm_data[4] = 0x88;
	msg.cm_data[5] = 0xd6;
	msg.cm_data[6] = 0xd1;
	msg.cm_data[7] = 0xf1;
	send_can_msg(msg);
}

uint8_t FFescOutput::eval_telem1_failures(const uint8_t fault_state)
{
	/*
	 * Bit 0 = over current
	 * Bit 1 = PWM dead
	 * Bit 2 = Hardware fault
	 * Bit 3 = over voltage
	 * Bit 4 = PWM doesn't match CAN
	 * Bit 5 = boot time self-check failed
	 * Bit 6 = motor stopped: Set when RPM = 0, Reset when RPM > 1000RPM. Note: 1000RPM is above idle speed, so when the ESCs are initially reaching idle speed this flag will be set.
	 * Bit 7 = CAN failed (switched over to PWM at somepoint, and latched)
	 */

	uint8_t fault_flags = 0;

	if (fault_state & (1 << EscFault::OverCurrent)) {
		fault_flags |= (1 << esc_report_s::FAILURE_OVER_CURRENT);
	}

	if (fault_state & ((1 << EscFault::DeadPwm) | (1 << EscFault::CommandInconsistency))) {
		fault_flags |= (1 << esc_report_s::FAILURE_INCONSISTENT_CMD);
	}

	if (fault_state & (1 << EscFault::CANFailed)) {
		fault_flags |= (1 << esc_report_s::FAILURE_INCONSISTENT_CMD);
	}

	if (fault_state & (1 << EscFault::HardwareFault)) {
		fault_flags |= (1 << esc_report_s::FAILURE_GENERIC);
	}

	if (fault_state & (1 << EscFault::OverVoltage)) {
		fault_flags |= (1 << esc_report_s::FAILURE_OVER_VOLTAGE);
	}

	if (fault_state & (1 << EscFault::BootTimeCheckFail)) {
		fault_flags |= (1 << esc_report_s::FAILURE_GENERIC);
	}

	return fault_flags;
}

uint16_t FFescOutput::eval_telem2_failures(const uint32_t fault_state){

	/*
	 * Bit 0 = DRV Over temp error
	 * Bit 1 = DRV 135C oh no
	 * Bit 2 = DRV 125C critical
	 * Bit 3 = DRV 105C warning
	 */

	uint16_t fault_flags = 0;

	if (fault_state & (1 << EscFault2::DRV_105C)) {
		fault_flags |= (1 << esc_report_s::FAILURE_WARN_ESC_TEMPERATURE) ;
	}

	if (fault_state & (1 << EscFault2::DRV_125C)) {
		fault_flags |= (1 << esc_report_s::FAILURE_OVER_ESC_TEMPERATURE) ;
	}

	if (fault_state & (1 << EscFault2::DRV_135C)) {
		fault_flags |= (1 << esc_report_s::FAILURE_OVER_ESC_TEMPERATURE) ;
	}

	if (fault_state & (1 << EscFault2::DRV_OTW)) {
		fault_flags |= (1 << esc_report_s::FAILURE_OVER_ESC_TEMPERATURE) ;
	}

	return fault_flags;
}

uint8_t FFescOutput::check_online_escs(const esc_status_s &esc_status){
	int esc_status_flags = 0;

	for (int i = 0; i < esc_status_s::CONNECTED_ESC_MAX; i++) {
		if ((esc_status.esc[i].timestamp > 0)
		    && (hrt_elapsed_time(&esc_status.esc[i].timestamp) < MOTOR_ERROR_TIMEOUT)) {
			esc_status_flags |= (1 << i);
		}
	}

	return esc_status_flags;
}

uint8_t FFescOutput::check_motor_temperature(const float motor_temperature, const uint8_t index){

	uint8_t motor_temp_fault = 0;

	if (motor_temperature > MOTOR_MAX_TEMP) {
		motor_temp_fault |= (1 << esc_report_s::FAILURE_MOTOR_OVER_TEMPERATURE);

		_last_motor_overtemp_fault[index] = (1 << esc_report_s::FAILURE_MOTOR_OVER_TEMPERATURE);
		_last_motor_overtemp_time[index] =  hrt_absolute_time();

	} else if (motor_temperature > MOTOR_WARN_TEMP && (_last_motor_overtemp_fault[index] != (1 << esc_report_s::FAILURE_MOTOR_OVER_TEMPERATURE))) {
		motor_temp_fault |= (1 << esc_report_s::FAILURE_MOTOR_WARN_TEMPERATURE);
		_last_motor_overtemp_fault[index] = (1 << esc_report_s::FAILURE_MOTOR_WARN_TEMPERATURE);
		_last_motor_overtemp_time[index] =  hrt_absolute_time();
	} else {
		// A temperature fault has been already fired, we latch it for 5 seconds
		if (_last_motor_overtemp_time[index] > 0 && hrt_elapsed_time(&_last_motor_overtemp_time[index]) < 5_s) {
			motor_temp_fault |= _last_motor_overtemp_fault[index];
		} else {
			_last_motor_overtemp_time[index] = 0;
			_last_motor_overtemp_fault[index] = 0;
		}
	}

	return motor_temp_fault;
}

uint16_t FFescOutput::check_esc_temperature(const float esc_temperature, const uint8_t index){

	uint16_t esc_temp_fault = 0;

	if (esc_temperature > ESC_MAX_TEMP) {
		esc_temp_fault |= (1 << esc_report_s::FAILURE_OVER_ESC_TEMPERATURE);
		_last_esc_overtemp_fault[index] = (1 << esc_report_s::FAILURE_OVER_ESC_TEMPERATURE);
		_last_esc_overtemp_time[index] =  hrt_absolute_time();

	} else if (esc_temperature > ESC_WARN_TEMP && (_last_esc_overtemp_fault[index] != (1 << esc_report_s::FAILURE_OVER_ESC_TEMPERATURE))) {
		esc_temp_fault |= (1 << esc_report_s::FAILURE_WARN_ESC_TEMPERATURE);
		_last_esc_overtemp_fault[index] = (1 << esc_report_s::FAILURE_WARN_ESC_TEMPERATURE);
		_last_esc_overtemp_time[index]  =  hrt_absolute_time();
	} else {
		// A temperature fault has been already fired, we latch it for X seconds
		if (_last_esc_overtemp_time[index]  > 0 && hrt_elapsed_time(&_last_esc_overtemp_time[index] ) < 5_s) {
			esc_temp_fault |= _last_esc_overtemp_fault[index];
		} else {
			_last_esc_overtemp_time[index]  = 0;
			_last_esc_overtemp_fault[index] = 0;
		}
	}

	return esc_temp_fault;
}

void FFescOutput::read_messages(){
	uint8_t rxmsg[sizeof(struct can_msg_s)];

	for (int i = 0; i < MAX_READ_MSG_CNT_PER_LOOP; i++) {

		int num_read = read_can_data((uint8_t*)&rxmsg, sizeof(rxmsg));
		if (num_read <= 0) {
			break;
		}

		int msglen;
		for (int j = 0; j <= (int) (num_read - CAN_MSGLEN(0)); j += msglen) {
			struct can_msg_s *msg = (struct can_msg_s*) &rxmsg[j];
			msglen = CAN_MSGLEN(msg->cm_hdr.ch_dlc);
			parse_data(*msg);
		}
	}
}

int ff_escs_main(int argc, char *argv[])
{
	return ModuleBase::main(FFescOutput::desc, argc, argv);
}
