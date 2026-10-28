/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
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
 * @file Roboclaw.cpp
 *
 * Roboclaw Motor Driver
 *
 * references:
 * http://downloads.orionrobotics.com/downloads/datasheets/motor_controller_robo_claw_R0401.pdf
 *
 */

#include "Roboclaw.hpp"

// MixingOutput updates and encoder publication need more stack than a UART-only worker.
static constexpr px4::wq_config_t roboclaw_work_queue{"wq:roboclaw", 4096, px4::wq_configurations::ttyUnknown.relative_priority};

ModuleBase::Descriptor Roboclaw::desc{task_spawn, custom_command, print_usage};

Roboclaw::Roboclaw(const char *device_name) :
	OutputModuleInterface(MODULE_NAME, roboclaw_work_queue),
	_uart(device_name)
{
	_mixing_output.setMaxTopicUpdateRate(20_ms);
	_mixing_output.setAllDisarmedValues(127);
}

Roboclaw::~Roboclaw()
{
	_uart.close();
}

bool Roboclaw::init()
{
	updateParams();
	_controller_count = _param_rbclw_count.get();

	if (_controller_count < 1 || _controller_count > MAX_CONTROLLERS
	    || _param_rbclw_address.get() < 128 || _param_rbclw_address.get() + _controller_count > 136
	    || _param_rbclw_counts_rev.get() < 1 || _param_rbclw_baud.get() <= 0) {
		PX4_ERR("Invalid controller count, address range or encoder resolution");
		return false;
	}

	PX4_INFO("Opening %s at %ld baud", _uart.getPort(), (long)_param_rbclw_baud.get());

	if (!_uart.setBaudrate(_param_rbclw_baud.get()) || !_uart.open()) {
		PX4_ERR("Could not open %s at %ld baud", _uart.getPort(), (long)_param_rbclw_baud.get());
		return false;
	}

	_base_address = _param_rbclw_address.get();

	for (unsigned i = 0; i < _controller_count; ++i) {
		_address = _base_address + i;
		uint8_t response[6] {};
		PX4_INFO("Probing address 0x%02x", _address);

		if (receiveTransaction(Command::ReadStatus, response, sizeof(response)) != sizeof(response)) {
			PX4_ERR("No valid response from address 0x%02x", _address);
			stopMotors();
			return false;
		}
	}

	stopMotors();
	PX4_INFO("Connected to %u controller(s) on %s at %lu baud", _controller_count, _uart.getPort(),
		 (unsigned long)_uart.getBaudrate());
	_mixing_output.setMaxNumOutputs(_controller_count * CHANNELS_PER_CONTROLLER);
	return true;
}

void Roboclaw::reversibleMaskChanged(uint32_t reversible_mask)
{
	for (unsigned i = 0; i < _controller_count * CHANNELS_PER_CONTROLLER; ++i) {
		int32_t minimum = 0;
		param_get(_mixing_output.minParamHandle(i), &minimum);
		const OutputFunction function = _mixing_output.outputFunction(i);

		// Non-reversible motors use the forward half of the packet serial range.
		if (function >= OutputFunction::Motor1 && function <= OutputFunction::MotorMax
		    && !(reversible_mask & (1u << i))) {
			minimum = 127;
		}

		_mixing_output.minValue(i) = math::constrain(minimum, int32_t{0}, int32_t{127});
	}
}

void Roboclaw::mixerChanged()
{
	reversibleMaskChanged(_mixing_output.reversibleOutputs());
}

bool Roboclaw::updateOutputs(float outputs[MAX_ACTUATORS], unsigned num_outputs, unsigned num_control_groups_updated)
{
	bool success = true;

	for (unsigned i = 0; i < _controller_count * CHANNELS_PER_CONTROLLER; ++i) {
		_address = _base_address + i / CHANNELS_PER_CONTROLLER;
		float value = 0.f;

		if (i < num_outputs && _mixing_output.isFunctionSet(i) && PX4_ISFINITE(outputs[i])) {
			value = math::constrain((outputs[i] - 127.f) / 127.f, -1.f, 1.f);
		}

		if (setMotorSpeed(i % CHANNELS_PER_CONTROLLER, value) != OK) {
			++_communication_errors;
			success = false;
		}
	}

	return success;
}

void Roboclaw::stopMotors()
{
	if (!_uart.isOpen()) { return; }

	for (unsigned i = 0; i < _controller_count; ++i) {
		_address = _base_address + i;

		if (setMotorSpeed(0, 0.f) != OK) { ++_communication_errors; }

		if (setMotorSpeed(1, 0.f) != OK) { ++_communication_errors; }
	}
}

void Roboclaw::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();
		stopMotors();
		exit_and_cleanup(desc);
		return;
	}

	if (!_initialized) {
		if (!init()) {
			ScheduleClear();
			_mixing_output.unregister();
			exit_and_cleanup(desc);
			return;
		}

		_initialized = true;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s parameter_update;
		_parameter_update_sub.copy(&parameter_update);
		updateParams();
		reversibleMaskChanged(_mixing_output.reversibleOutputs());
	}

	_mixing_output.update();

	// Poll one controller at a time, even when no actuator functions are assigned.
	if (hrt_elapsed_time(&_last_encoder_read) >= 100_ms) {
		_last_encoder_read = hrt_absolute_time();
		_address = _base_address + _encoder_controller;

		if (readEncoder(_encoder_controller) != OK) {
			++_communication_errors;
		}

		_encoder_controller = (_encoder_controller + 1) % _controller_count;
	}

	_mixing_output.updateSubscriptions(false);
	ScheduleDelayed(20_ms);
}

int Roboclaw::readEncoder(unsigned controller)
{
	static constexpr int ENCODER_MESSAGE_SIZE = 10; // response size for ReadEncoderCounters
	static constexpr int ENCODER_SPEED_MESSAGE_SIZE = 7; // response size for CMD_READ_SPEED_{1,2}

	uint8_t buffer_position[ENCODER_MESSAGE_SIZE];
	uint8_t buffer_speed_right[ENCODER_SPEED_MESSAGE_SIZE];
	uint8_t buffer_speed_left[ENCODER_SPEED_MESSAGE_SIZE];

	if (receiveTransaction(Command::ReadSpeedMotor1, buffer_speed_right,
			       ENCODER_SPEED_MESSAGE_SIZE) < ENCODER_SPEED_MESSAGE_SIZE) {
		return ERROR;
	}

	if (receiveTransaction(Command::ReadSpeedMotor2, buffer_speed_left,
			       ENCODER_SPEED_MESSAGE_SIZE) < ENCODER_SPEED_MESSAGE_SIZE) {
		return ERROR;
	}

	if (receiveTransaction(Command::ReadEncoderCounters, buffer_position, ENCODER_MESSAGE_SIZE) < ENCODER_MESSAGE_SIZE) {
		return ERROR;
	}

	int64_t speed_right = swapBytesInt32(buffer_speed_right);
	int64_t speed_left = swapBytesInt32(buffer_speed_left);

	int32_t position_right = swapBytesInt32(&buffer_position[0]);
	int32_t position_left = swapBytesInt32(&buffer_position[4]);

	wheel_encoders_s wheel_encoders{};
	wheel_encoders.wheel_speed[0] = static_cast<float>(speed_right) / math::max(_param_rbclw_counts_rev.get(), int32_t{1}) * M_TWOPI_F;
	wheel_encoders.wheel_speed[1] = static_cast<float>(speed_left) / math::max(_param_rbclw_counts_rev.get(), int32_t{1}) * M_TWOPI_F;
	wheel_encoders.wheel_angle[0] = static_cast<float>(position_right) / math::max(_param_rbclw_counts_rev.get(), int32_t{1}) * M_TWOPI_F;
	wheel_encoders.wheel_angle[1] = static_cast<float>(position_left) / math::max(_param_rbclw_counts_rev.get(), int32_t{1}) * M_TWOPI_F;
	wheel_encoders.timestamp = hrt_absolute_time();

	if (!_wheel_encoders_pub[controller].publish(wheel_encoders)) { return ERROR; }

	++_encoder_updates[controller];

	return OK;
}

int Roboclaw::setMotorSpeed(unsigned motor, float value)
{
	const Command command = motor == 0
				? (value >= 0.f ? Command::DriveForwardMotor1 : Command::DriveBackwardsMotor1)
				: (value >= 0.f ? Command::DriveForwardMotor2 : Command::DriveBackwardsMotor2);
	uint8_t data = static_cast<uint8_t>(lroundf(fabsf(math::constrain(value, -1.f, 1.f)) * INT8_MAX));
	return sendTransaction(command, &data, sizeof(data));
}

int Roboclaw::sendTransaction(Command cmd, uint8_t *write_buffer, size_t bytes_to_write)
{
	if (writeCommandWithPayload(cmd, write_buffer, bytes_to_write) != OK) {
		return ERROR;
	}

	return readAcknowledgement();
}

int Roboclaw::writeCommandWithPayload(Command command, uint8_t *wbuff, size_t bytes_to_write)
{
	if (bytes_to_write > 2) { return ERROR; }

	const size_t packet_size = 2 + bytes_to_write + 2;
	uint8_t buffer[6] {};

	// Add address + command ID
	buffer[0] = _address;
	buffer[1] = static_cast<uint8_t>(command);

	// Add payload
	if (bytes_to_write > 0 && wbuff) {
		memcpy(&buffer[2], wbuff, bytes_to_write);
	}

	// Add checksum
	uint16_t sum = _calcCRC(buffer, packet_size - 2);
	buffer[packet_size - 2] = (sum >> 8) & 0xFF;
	buffer[packet_size - 1] = sum & 0xFFu;

	// Write to device
	_uart.flush();
	ssize_t bytes_written = _uart.writeBlocking(buffer, packet_size, TRANSACTION_TIMEOUT_MS);

	// Not all bytes sent
	if (bytes_written != static_cast<ssize_t>(packet_size)) {
		PX4_ERR("Only wrote %zd out of %zu bytes", bytes_written, packet_size);
		return ERROR;
	}

	return OK;
}

int Roboclaw::readAcknowledgement()
{
	uint8_t acknowledgement{0};
	int bytes_read = _uart.readAtLeast(&acknowledgement, 1, 1, TRANSACTION_TIMEOUT_MS);

	if ((bytes_read != 1) || (acknowledgement != 0xFF)) {
		return ERROR;
	}

	return OK;
}

int Roboclaw::receiveTransaction(Command command, uint8_t *read_buffer, size_t bytes_to_read)
{
	if (writeCommand(command) != OK) {
		return ERROR;
	}

	return readResponse(command, read_buffer, bytes_to_read);
}

int Roboclaw::writeCommand(Command command)
{
	uint8_t buffer[2];

	// Just address + command ID
	buffer[0] = _address;
	buffer[1] = static_cast<uint8_t>(command);

	_uart.flush();
	ssize_t bytes_written = _uart.writeBlocking(buffer, sizeof(buffer), TRANSACTION_TIMEOUT_MS);

	if (bytes_written != 2) {
		PX4_ERR("Only wrote %zd out of %d bytes", bytes_written, 2);
		return ERROR;
	}

	return OK;
}

int Roboclaw::readResponse(Command command, uint8_t *read_buffer, size_t bytes_to_read)
{
	const int total_bytes_read = _uart.readAtLeast(read_buffer, bytes_to_read, bytes_to_read, TRANSACTION_TIMEOUT_MS);

	if (total_bytes_read != static_cast<int>(bytes_to_read) || total_bytes_read < 2) {
		return ERROR;
	}

	// Verify response checksum
	uint8_t address = _address;
	uint8_t command_byte = static_cast<uint8_t>(command);
	uint16_t crc_calculated = _calcCRC(&address, 1); // address
	crc_calculated = _calcCRC(&command_byte, 1, crc_calculated); // command
	crc_calculated = _calcCRC(read_buffer, total_bytes_read - 2, crc_calculated); // received payload
	uint16_t crc_received = (read_buffer[total_bytes_read - 2] << 8) + read_buffer[total_bytes_read - 1];

	if (crc_calculated != crc_received) {
		return ERROR;
	}

	return total_bytes_read;
}

uint16_t Roboclaw::_calcCRC(const uint8_t *buffer, size_t bytes, uint16_t init)
{
	uint16_t crc = init;

	for (size_t byte = 0; byte < bytes; byte++) {
		crc = crc ^ (((uint16_t) buffer[byte]) << 8);

		for (uint8_t bit = 0; bit < 8; bit++) {
			if (crc & 0x8000) {
				crc = (crc << 1) ^ 0x1021;

			} else {
				crc = crc << 1;
			}
		}
	}

	return crc;
}

int32_t Roboclaw::swapBytesInt32(uint8_t *buffer)
{
	return (static_cast<uint32_t>(buffer[0]) << 24)
	       | (buffer[1] << 16)
	       | (buffer[2] << 8)
	       | buffer[3];
}

int Roboclaw::task_spawn(int argc, char *argv[])
{
	if (argc != 2 || !device::Serial::validatePort(argv[1])) {
		print_usage("Usage: roboclaw start <UART device>");
		return ERROR;
	}

	Roboclaw *instance = new Roboclaw(argv[1]);

	if (!instance) {
		PX4_ERR("alloc failed");
		return ERROR;
	}

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;

	instance->ScheduleNow();

	return OK;
}

int Roboclaw::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Roboclaw::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
### Description

This driver communicates over UART with the [Roboclaw motor driver](https://www.basicmicro.com/motor-controller).
It performs two tasks:

 - Control the motors based on the OutputModuleInterface.
 - Read the wheel encoders and publish the raw data in the `wheel_encoders` uORB topic

In order to use this driver, the Roboclaw should be put into Packet Serial mode (see the linked documentation), and
your flight controller's UART port should be connected to the Roboclaw as shown in the documentation. Enable the
driver with `RBCLW_SER_CFG` and set `RBCLW_BAUD` to match the controllers. Set `RBCLW_COUNT` and configure consecutive
unique addresses starting at `RBCLW_ADDRESS`. Each controller uses two output channels (M1, M2) and publishes a
separate wheel_encoders instance. Bus parameters require a driver restart.

The command to start this driver is: `$ roboclaw start <UART device>`
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("roboclaw", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

int Roboclaw::print_status()
{
	PX4_INFO("Port: %s, baud: %lu", _uart.getPort(), (unsigned long)_uart.getBaudrate());
	PX4_INFO("Communication errors: %lu", (unsigned long)_communication_errors);

	for (unsigned i = 0; i < _controller_count; ++i) {
		PX4_INFO("Address 0x%02x, channels %u-%u, encoder updates: %lu",
			 _base_address + i, i * 2 + 1, i * 2 + 2, (unsigned long)_encoder_updates[i]);
	}

	_mixing_output.printStatus();
	return 0;
}

extern "C" __EXPORT int roboclaw_main(int argc, char *argv[])
{
	return ModuleBase::main(Roboclaw::desc, argc, argv);
}
