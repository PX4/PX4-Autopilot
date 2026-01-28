/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file TattuCan.cpp
 * @author Jacob Dahl <dahl.jakejacob@gmail.com>
 *
 * Driver for the Tattu 12S 1600mAh Smart Battery connected over CAN.
 *
 * This driver simply decodes the CAN frames based on the specification
 * as provided in the Tattu datasheet DOC 001 REV D, which is highly
 * specific to the 12S 1600mAh battery. Other models of Tattu batteries
 * will NOT work with this driver in its current form.
 *
 */

#include "TattuCan.hpp"

extern orb_advert_t mavlink_log_pub;

TattuCan::TattuCan() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::uavcan)
{
}

TattuCan::~TattuCan()
{
}

void TattuCan::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	if (!_initialized) {

		_fd = ::open("/dev/can0", O_RDWR);

		if (_fd < 0) {
			PX4_INFO("FAILED TO OPEN /dev/can0");
			return;
		}

		_initialized = true;
	}

	uint8_t data[64] {};
	CanFrame received_frame{};
	received_frame.payload = &data;

	Tattu12SBatteryMessage tattu_message = {};

	while (receive(&received_frame) > 0) {

		// Find the start of a transferr
		if ((received_frame.payload_size == 8) && ((uint8_t *)received_frame.payload)[7] == TAIL_BYTE_START_OF_TRANSFER) {
		} else {
			continue;
		}

		// We have the start of a transfer
		size_t offset = 5;
		memcpy(&tattu_message, &(((uint8_t *)received_frame.payload)[2]), offset);

		while (receive(&received_frame) > 0) {

			size_t payload_size = received_frame.payload_size - 1;
			// TODO: add check to prevent buffer overflow from a corrupt 'payload_size' value
			// TODO: AND look for TAIL_BYTE_START_OF_TRANSFER to indicate end of transfer. Untested...
			memcpy(((char *)&tattu_message) + offset, received_frame.payload, payload_size);
			offset += payload_size;
		}

		battery_status_s battery_status = {};
		battery_status.timestamp = hrt_absolute_time();
		battery_status.connected = true;
		battery_status.cell_count = 12;

		sprintf(battery_status.serial_number, "%d", tattu_message.manufacturer);
		battery_status.id = static_cast<uint8_t>(tattu_message.sku);

		battery_status.cycle_count = tattu_message.cycle_life;
		battery_status.state_of_health = static_cast<uint16_t>(tattu_message.health_status);

		battery_status.voltage_v = static_cast<float>(tattu_message.voltage) / 1000.0f;
		battery_status.current_a = static_cast<float>(tattu_message.current) / 1000.0f;
		battery_status.remaining = static_cast<float>(tattu_message.remaining_percent) / 100.0f;
		battery_status.temperature = static_cast<float>(tattu_message.temperature);
		battery_status.capacity = tattu_message.standard_capacity;
		battery_status.voltage_cell_v[0] = static_cast<float>(tattu_message.cell_1_voltage) / 1000.0f;
		battery_status.voltage_cell_v[1] = static_cast<float>(tattu_message.cell_2_voltage) / 1000.0f;
		battery_status.voltage_cell_v[2] = static_cast<float>(tattu_message.cell_3_voltage) / 1000.0f;
		battery_status.voltage_cell_v[3] = static_cast<float>(tattu_message.cell_4_voltage) / 1000.0f;
		battery_status.voltage_cell_v[4] = static_cast<float>(tattu_message.cell_5_voltage) / 1000.0f;
		battery_status.voltage_cell_v[5] = static_cast<float>(tattu_message.cell_6_voltage) / 1000.0f;
		battery_status.voltage_cell_v[6] = static_cast<float>(tattu_message.cell_7_voltage) / 1000.0f;
		battery_status.voltage_cell_v[7] = static_cast<float>(tattu_message.cell_8_voltage) / 1000.0f;
		battery_status.voltage_cell_v[8] = static_cast<float>(tattu_message.cell_9_voltage) / 1000.0f;
		battery_status.voltage_cell_v[9] = static_cast<float>(tattu_message.cell_10_voltage / 1000.0f);
		battery_status.voltage_cell_v[10] = static_cast<float>(tattu_message.cell_11_voltage) / 1000.0f;
		battery_status.voltage_cell_v[11] = static_cast<float>(tattu_message.cell_12_voltage) / 1000.0f;

		_battery_status_pub.publish(battery_status);
	}
}

int16_t TattuCan::receive(CanFrame *received_frame)
{
	if ((_fd < 0) || (received_frame == nullptr)) {
		PX4_INFO("fd < 0");
		return -1;
	}

	struct pollfd fds {};

	fds.fd = _fd;

	fds.events = POLLIN;

	// Jake: this doesn't block even in blocking mode, dunno if we need it
	::poll(&fds, 1, 0);

	// Only execute this part if can0 is changed.
	if (fds.revents & POLLIN) {

		// Try to read.
		struct can_msg_s receive_msg;
		const ssize_t nbytes = ::read(fds.fd, &receive_msg, sizeof(receive_msg));

		if (nbytes < 0 || (size_t)nbytes < CAN_MSGLEN(0) || (size_t)nbytes > sizeof(receive_msg)) {
			PX4_INFO("error");
			return -1;

		} else {
			received_frame->extended_can_id = receive_msg.cm_hdr.ch_id;
			received_frame->payload_size = receive_msg.cm_hdr.ch_dlc;
			memcpy((void *)received_frame->payload, receive_msg.cm_data, receive_msg.cm_hdr.ch_dlc);
			return nbytes;
		}
	}

	return 0;
}

int TattuCan::start()
{
	// There is a race condition at boot that sometimes causes opening of
	// /dev/can0 to fail. We will delay 0.5s to be safe.
	uint32_t delay_us = 500000;
	ScheduleOnInterval(1000000 / SAMPLE_RATE, delay_us);
	return PX4_OK;
}

int TattuCan::task_spawn(int argc, char *argv[])
{
	TattuCan *instance = new TattuCan();

	if (!instance) {
		PX4_ERR("driver allocation failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	instance->start();
	return 0;
}

int TattuCan::print_usage(const char *reason)
{
	if (reason) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for reading data from the Tattu 12S 16000mAh smart battery.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("tattu_can", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int TattuCan::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		PX4_INFO("not running");
		return PX4_ERROR;
	}

	return print_usage("Unrecognized command.");
}

extern "C" __EXPORT int tattu_can_main(int argc, char *argv[])
{
	return TattuCan::main(argc, argv);
}
