/****************************************************************************
 *
 *   Copyright (c) 2021 Auterion. All rights reserved.
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
 * @file ff_smartbattery.cpp
 * Driver for Freefly Smartbatteries based on Lazarus CAN protocol.
 *
 * @author Claudio Micheli <claudio@auterion.com>
 */


#include "ff_smartbattery.hpp"

ModuleBase::Descriptor FF_SmartBattery::desc{FF_SmartBattery::task_spawn, FF_SmartBattery::custom_command, FF_SmartBattery::print_usage};

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_armed.h>

// hardware specific includes
#include <board_config.h>
#include <nuttx/can/can.h>

#include "stm32_can.h"
#ifdef CONFIG_ARCH_CHIP_STM32
#include "stm32.h"
#endif

#define SMART_BATTERY_CAN_PATH "/dev/can0"

using namespace time_literals;

FF_SmartBattery::FF_SmartBattery() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	ModuleParams(nullptr),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms_errors"))

{
	// Wait 300ms after arming to check for batteries to rebort being in "airmode"
	_battery_arm_hysteresis.set_hysteresis_time_from(false, 300_ms);
	// Wait 3s after disarming before commanding batteries to go back to "hotswap" mode
	_battery_arm_hysteresis.set_hysteresis_time_from(true, 3_s);

	const float expected_filter_dt = static_cast<float>(SAMPLE_INTERVAL_US) / 1_s;

	for (uint8_t index = 0; index < BATTERIES_COUNT; index++) {
		_current_filter_a[index].setParameters(expected_filter_dt, .5f);
		_current_average_filter_a[index].setParameters(expected_filter_dt, 100.f);

		//Initialize strongly filtered current to an estimated average consumption, per battery
		_current_average_filter_a[index].reset(14.f);
	}
}

FF_SmartBattery::~FF_SmartBattery()
{
	ScheduleClear();
	perf_free(_cycle_perf);
	perf_free(_comms_errors);
	::close(_fd);
}

int FF_SmartBattery::task_spawn(int argc, char *argv[])
{
	FF_SmartBattery *instance = new FF_SmartBattery();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
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

bool FF_SmartBattery::init()
{
	for (uint8_t index = 0; index < BATTERIES_COUNT; index++) {
		_battery_status[index].timestamp = hrt_absolute_time();
		_battery_status[index].connected = false;
		_battery_status[index].is_required = true;
		_battery_status_pub[index].publish(_battery_status[index]);
	}

	// ScheduleOnInterval needs ALWAYS to be at the end of the init(), you might end up in race conditions otherwise!
	ScheduleOnInterval(SAMPLE_INTERVAL_US);

	return true;
}

void FF_SmartBattery::Run()
{
	if (should_exit()) {
		exit_and_cleanup(desc);
		return;
	}

	// File descriptor initialized?
	if (_fd < 0) {
		if (open_can_port() != PX4_OK) {
			PX4_ERR("Failed to open CAN device. Aborting");
			ScheduleClear();
			return;
		}
	}

	perf_begin(_cycle_perf);

	// if we are here means that the CAN port has been successfully opened
	request_battery_info();

	update_params();

	read_messages();

	check_vehicle_arm_state();

	publish_messages();

	perf_end(_cycle_perf);
}

int FF_SmartBattery::open_can_port()
{
	// Configure port flags for read/write, non-controlling, non-blocking.
	int flags = (O_RDWR | O_NONBLOCK);

	// Open the serial port.
	_fd = ::open(SMART_BATTERY_CAN_PATH, flags);

	if (_fd < 0) {
		PX4_ERR("Failed to open CAN device. ERROR (%i)", errno);
		return PX4_ERROR;
	}

	if (::ioctl(_fd, CANIOC_SET_ABOM, 1) != 0) {
		PX4_ERR("failed to set ABOM");
		return PX4_ERROR;
	}

	return PX4_OK;

}

void FF_SmartBattery::read_messages()
{
	uint8_t rxmsg[sizeof(struct can_msg_s)];

	for (int i = 0; i < MAX_READ_MSG_CNT_PER_LOOP; i++) {

		int num_read = read_can_data((uint8_t *)&rxmsg, sizeof(rxmsg));

		if (num_read <= 0) {
			break;
		}

		int msglen;

		for (int j = 0; j <= (int)(num_read - CAN_MSGLEN(0)); j += msglen) {
			struct can_msg_s *msg = (struct can_msg_s *) &rxmsg[j];
			msglen = CAN_MSGLEN(msg->cm_hdr.ch_dlc);
			parse_data(*msg);
		}
	}
}

int FF_SmartBattery::read_can_data(uint8_t *buf, int len)
{
	ssize_t bytes_read = read(_fd, buf, len);

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

void FF_SmartBattery::parse_data(struct can_msg_s &msg_p)
{
	PX4_DEBUG("MSD ID:  0x%02X", msg_p.cm_hdr.ch_id);
	int battery_index = determineIndex(msg_p.cm_hdr.ch_id);

	// 0x3FF is an ID that the battery publishes if it's not yet enumerated.
	// 0x400 is an ID that the battery publishes if it's not yet enumerated.
	// This needs to be fixed on the smartbattery firmware, for now we just ignore it.
	if (battery_index < 0) {
		if (msg_p.cm_hdr.ch_id != 0x3FF && msg_p.cm_hdr.ch_id != 0x400) {
			PX4_INFO("ID 0x%02X not supported", msg_p.cm_hdr.ch_id);
		}

		return;
	}

	switch (static_cast<MsgCode>(msg_p.cm_data[0])) {

	case MsgCode::Status: {
			const StatusMsg *statusMessage = (const StatusMsg *) &msg_p.cm_data[1];
			update_battery_status(battery_index, statusMessage);
			break;
		}

	case MsgCode::Values: {
			const ValuesMsg *valuesMessage = (const ValuesMsg *) &msg_p.cm_data[1];
			update_battery_values(battery_index, valuesMessage);
			break;
		}

	case MsgCode::Cell1to3_V:
		update_battery_cells(battery_index, 0, msg_p.cm_data);
		break;

	case MsgCode::Cell4to6_V:
		update_battery_cells(battery_index, 3, msg_p.cm_data);
		break;

	case MsgCode::Heartbeat:
		break;

	case MsgCode::FW_Version: {
			uint8_t major_version = msg_p.cm_data[1];
			uint8_t minor_version = msg_p.cm_data[2];
			_battery_freefly_pub[battery_index].get().firmware_version[0] = major_version;
			_battery_freefly_pub[battery_index].get().firmware_version[1] = minor_version;
			_battery_status[battery_index].faults = checkFirmwareVersion(battery_index, major_version, minor_version);
			break;
		}

	case MsgCode::UUID: {
			uint32_t uuid = msg_p.cm_data[4] << 24 | msg_p.cm_data[3] << 16 | msg_p.cm_data[2] << 8 | msg_p.cm_data[1];
			snprintf(_battery_info_pub[battery_index].get().serial_number, 10, "%08lX", uuid);
			break;
		}

	case MsgCode::Health: {
			const HealthDataMsg *HealthData = (const HealthDataMsg *) &msg_p.cm_data[1];
			_battery_status[battery_index].state_of_health = HealthData->StateOfHealth;
			_battery_status[battery_index].cycle_count = HealthData->CycleCount;
			_battery_status[battery_index].capacity = HealthData->Capacity * 10;	// in mAh
			break;
		}

	case MsgCode::MfgData: {
			// Day + Month×32 + (Year–1980)×512
			uint8_t model = msg_p.cm_data[5];
			_battery_freefly_pub[battery_index].get().model = model;
			_battery_status[battery_index].manufacture_date = msg_p.cm_data[3] + msg_p.cm_data[2] * 32 +
					(msg_p.cm_data[1] - 1980) * 512;
			_battery_status[battery_index].faults = checkIncompatibleModel(battery_index, model);
			break;
		}

	default:
		PX4_DEBUG("Unknown payload id: 0x%02X \n",  msg_p.cm_data[0]);
		break;
	}
}

int FF_SmartBattery::determineIndex(uint16_t batteryID)
{
	for (uint8_t index = 0; index < BATTERIES_COUNT; index++) {
		if (batteryID == _battery_info[index].status_id) {
			return index;
		}
	}

	return -1;
}

void FF_SmartBattery::update_battery_status(uint8_t index, const StatusMsg *battery)
{
	_update_battery_status[index] |= 1 << static_cast<int>(AckMessage::Status);

	_battery_status[index].timestamp = hrt_absolute_time();
	_battery_status[index].remaining = battery->StateOfCharge / 100.0f;
	_battery_status[index].id = _battery_info[index].battery_id;
	_battery_info_pub[index].get().id = _battery_info[index].battery_id;
	_battery_freefly_pub[index].get().id = _battery_info[index].battery_id;
	_battery_status[index].source = battery_status_s::SOURCE_EXTERNAL;
	_battery_status[index].cell_count = CELL_COUNT;
	uint32_t custom_faults = (battery->FaultFlags[2] << 16) | (battery->FaultFlags[1] << 8) | battery->FaultFlags[0];
	_battery_freefly_pub[index].get().custom_faults = custom_faults;
	_battery_status[index].faults = determinePX4FaultFlags(index, battery->FaultFlags, battery->StatusFlags, custom_faults);
	_battery_status[index].warning = determineWarning(_battery_status[index].remaining);

	const uint8_t mode = battery->Mode;
	_battery_freefly_pub[index].get().mode = mode;

	// We only expect to receive these modes
	if (mode != SmartBatteryState::Hotswap
	    && mode != SmartBatteryState::Airmode) {
		PX4_INFO("unsupported battery mode %d", mode);
	}

	// Battery needs to report being in airmode shortly after arming
	if (_battery_arm_hysteresis.get_state() && mode != SmartBatteryState::Airmode) {
		_battery_status[index].faults |= (1 << battery_status_s::FAULT_FAILED_TO_ARM);
	}

	// Work around battery management firmware bug where state of charge report suddenly jumps to 0%
	const bool state_of_charge_exactly_zero = (battery->StateOfCharge == 0);
	const bool no_faults_reported = (_battery_status[index].faults == 0);
	// 3.35v per cell = 20.1v battery, usually the first spike crosses that threshold just shy of 30% SOC
	const bool acceptable_voltage = 3.35f < (_battery_status[index].voltage_v / static_cast<float>(CELL_COUNT));

	if (state_of_charge_exactly_zero && no_faults_reported && acceptable_voltage) {
		// Flag a hardware fault
		_battery_status[index].faults = (1 << battery_status_s::FAULT_HARDWARE_FAILURE);
		// Resort to Critical warning instead of emergency to allow for RTL while no other things fail
		_battery_status[index].warning = battery_status_s::WARNING_CRITICAL;
	}
}

void FF_SmartBattery::update_battery_values(const uint8_t index, const ValuesMsg *battery)
{
	_update_battery_status[index] |= 1 << static_cast<int>(AckMessage::Values);

	_battery_status[index].timestamp = hrt_absolute_time();
	_battery_status[index].voltage_v = (float)battery->Voltage / 1000.f;
	_battery_status[index].current_a = (float)battery->Current / 100.f;	// mA*10 from datasheet

	_current_filter_a[index].update(_battery_status[index].current_a);
	_battery_status[index].current_average_a = _current_average_filter_a[index].getState();

	_battery_status[index].time_remaining_s = computeRemainingTime(_battery_status[index].current_a, index);

	_battery_status[index].discharged_mah = (float) battery->Energy_consumed / (CELL_COUNT * 3.7f);
	_battery_status[index].temperature = (float)battery->Temperature;
}

// each cell message provides the info about only 3 cells per time.
void FF_SmartBattery::update_battery_cells(const uint8_t index, uint8_t startIndex, const uint8_t *payload_data)
{

	switch (startIndex) {
	case 0:
		_update_battery_status[index] |= 1 << static_cast<int>(AckMessage::Cells1);
		break;

	case 3:
		_update_battery_status[index] |= 1 << static_cast<int>(AckMessage::Cells2);
		break;
	}

	_battery_status[index].voltage_cell_v[startIndex] = ((payload_data[2] << 8) |  payload_data[1]) / 1000.f;
	_battery_status[index].voltage_cell_v[startIndex + 1] = ((payload_data[4] << 8) |  payload_data[3]) / 1000.f;
	_battery_status[index].voltage_cell_v[startIndex + 2] = ((payload_data[6] << 8) |  payload_data[5]) / 1000.f;
}

void FF_SmartBattery::publish_messages()
{
	// We only publish once all the battery fields have been updated,
	// we check this by verifying that all the bitfields are set (for each battery)

	const hrt_abstime now = hrt_absolute_time();

	for (uint8_t index = 0; index < BATTERIES_COUNT; index++) {

		bool all_messages_received = _update_battery_status[index] == ((1 << (static_cast<int>(AckMessage::Count))) - 1);

		_battery_status[index].is_required = true;

		if (all_messages_received) {
			_update_battery_status[index] = 0;
			// when we've received all the required messages from the battery we flag it as connected.
			_battery_status[index].connected = true;
		}

		if (_battery_status[index].connected) {
			// Batteries are publishing at 25Hz
			_battery_status[index].connected = (now - _battery_status[index].timestamp < 500_ms);

		} else {
			// reset all the informations that we request to the battery on connection.
			_battery_info_pub[index].get().serial_number[0] = '\0';
			_battery_status[index].state_of_health = 0;
			_battery_status[index].cycle_count = 0;
			_battery_status[index].capacity = 0;
			_battery_freefly_pub[index].get().firmware_version[0] = 0;
			_battery_freefly_pub[index].get().firmware_version[1] = 0;
		}

		if (all_messages_received || !_battery_status[index].connected) {
			_battery_status_pub[index].publish(_battery_status[index]);
			_battery_info_pub[index].get().timestamp = now;
			_battery_info_pub[index].update();
			_battery_freefly_pub[index].get().timestamp = now;
			_battery_freefly_pub[index].update();
		}
	}
}

void FF_SmartBattery::update_params()
{
	parameter_update_s pupdate;

	if (_parameter_update_sub.update(&pupdate)) {
		updateParams();
	}
}

void FF_SmartBattery::check_vehicle_arm_state()
{
	if (_armed_sub.updated()) {
		actuator_armed_s armed{};
		_armed_sub.copy(&armed);

		// update the armed status and check that we're not locked down
		const bool arm_motors = (armed.armed && !armed.lockdown);
		_battery_arm_hysteresis.set_state_and_update(arm_motors, hrt_absolute_time());

		// For disarming an hysteresis is used such that the battery stays in airmode
		// for a small extra duration while propellers are spinning down and
		// current might flow back to the battery.
		if (arm_motors && !_was_armed) {
			change_mode(SmartBatteryState::Airmode);
			_was_armed = true;

		} else if (!arm_motors && _was_armed && !_battery_arm_hysteresis.get_state()) {
			change_mode(SmartBatteryState::Hotswap);
			_was_armed = false;
		}
	}
}

void FF_SmartBattery::change_mode(const uint8_t mode)
{
	struct can_msg_s msg {};

	for (uint8_t index = 0; index < BATTERIES_COUNT; index++) {
		if (mode == SmartBatteryState::Airmode) {

			msg.cm_hdr.ch_id = _battery_info[index].cmd_id;
			msg.cm_hdr.ch_rtr = 0;
			msg.cm_hdr.ch_dlc = 2;		// Data Length Code

			msg.cm_data[0] = 0x05;
			msg.cm_data[1] = 0x05;

		} else if (mode == SmartBatteryState::Hotswap) {

			msg.cm_hdr.ch_id = _battery_info[index].cmd_id;
			msg.cm_hdr.ch_rtr = 0;
			msg.cm_hdr.ch_dlc = 2;		// Data Length Code

			msg.cm_data[0] = 0x05;
			msg.cm_data[1] = 0x04;		// Hotswap

		}

		send_can_msg(msg);
	}
}

void FF_SmartBattery::request_battery_info()
{
	if (hrt_elapsed_time(&_last_battery_request) < 1_s) {
		return;
	}

	can_msg_s msg {};

	for (int i = 0; i < BATTERIES_COUNT; i++) {
		if ((_battery_freefly_pub[i].get().firmware_version[0] + _battery_freefly_pub[i].get().firmware_version[1]) == 0) {

			msg.cm_hdr.ch_id = _battery_info[i].cmd_id;
			msg.cm_hdr.ch_rtr = 0;
			msg.cm_hdr.ch_dlc = 5;		// Data Length Code

			msg.cm_data[0] = static_cast<int>(MsgCode::RequestStatCodes);
			msg.cm_data[1] = static_cast<int>(MsgCode::UUID);
			msg.cm_data[2] = static_cast<int>(MsgCode::FW_Version);
			msg.cm_data[3] = static_cast<int>(MsgCode::Health);
			msg.cm_data[4] = static_cast<int>(MsgCode::MfgData);

			send_can_msg(msg);
		}
	}

	_last_battery_request = hrt_absolute_time();
}

float FF_SmartBattery::computeRemainingTime(float current_a, const uint8_t index)
{
	float time_remaining_s{NAN};

	if (_was_armed) {
		// only update with positive numbers
		// Filter current very strong, we basically want the average consumption
		_current_average_filter_a[index].update(fmaxf(current_a, 0.f));
	}

	// Remaining time estimation only possible with capacity
	float sum_current_ma = 0;
	float sum_remaining_capacity_mah = 0;
	const float min_current_ma = 6500.f;

	for (uint8_t j = 0 ; j < BATTERIES_COUNT ; j++) {
		sum_current_ma +=  fmaxf(_current_average_filter_a[j].getState() * 1e3f, min_current_ma);
		sum_remaining_capacity_mah += _battery_status[j].remaining * _battery_status[j].capacity;
	}

	if (_battery_status[index].capacity > 0.f) {
		time_remaining_s = math::constrain((sum_remaining_capacity_mah / sum_current_ma) * 3600.f, 0.f, 2400.f);
	}

	return time_remaining_s;
}

int FF_SmartBattery::send_can_msg(const struct can_msg_s &msg_p)
{
	// Variables for controlling CAN transmission
	size_t msgsize;
	ssize_t bytes_written;

	msgsize = CAN_MSGLEN(msg_p.cm_hdr.ch_dlc);

	bytes_written = write(_fd, (uint8_t *)&msg_p, msgsize);

	if ((size_t)bytes_written != msgsize || bytes_written < 0) {
		perf_count(_comms_errors);
		return PX4_ERROR;

	} else {
		return PX4_OK;
	}

	// Shouldn't reach here.
	return -2;
}

uint8_t FF_SmartBattery::determineWarning(const float remaining_v)
{
	// propagate warning state only if the state is higher, otherwise remain in current warning state
	if (remaining_v < _emergency_thr.get()) {
		return battery_status_s::WARNING_EMERGENCY;

	} else if (remaining_v < _crit_thr.get()) {
		return battery_status_s::WARNING_CRITICAL;

	} else if (remaining_v < _low_thr.get()) {
		return battery_status_s::WARNING_LOW;

	} else {
		return 0;
	}
}

uint16_t FF_SmartBattery::determinePX4FaultFlags(const uint8_t index, const uint8_t FaultFlags[3],
		const uint8_t statusFlags, const uint32_t custom_faults)
{
	// TODO - Fill this method with flags recognised by AEPX4
	uint32_t faults = (FaultFlags[2] << 16) | (FaultFlags[1] << 8) | FaultFlags[0];

	// We initialize px4_faults to reset all the bitflags that are checked in this function and white list the ones that were set previously
	uint16_t px4_faults = (_battery_status[index].faults & (1 << battery_status_s::FAULT_INCOMPATIBLE_MODEL | 1 <<
			       battery_status_s::FAULT_INCOMPATIBLE_FIRMWARE));

	for (uint8_t fault_index = 0; fault_index < static_cast<uint8_t>(FaultFlagMask::Count); fault_index++) {
		if (faults & (1 << fault_index)) {

			// fire the warning only one time, the first byte is reserved for px4 recognised faults
			if (!(custom_faults & (1 << fault_index))) {
				PX4_WARN("%s", FAILURES_MSG[fault_index]);
			}

			switch (static_cast<FaultFlagMask>(fault_index)) {

			case FaultFlagMask::Fault_Cell_OV_HW:
				px4_faults |= (1 << battery_status_s::FAULT_SPIKES);
				break;

			case FaultFlagMask::Fault_OT_Cells_Chg:
			case FaultFlagMask::Fault_OT_Cells_Dsg:
			case FaultFlagMask::Fault_OT_PCB_Chg:
			case FaultFlagMask::Fault_OT_PCB_Dsg:
			case FaultFlagMask::Fault_OT_FET:
				px4_faults |= (1 << battery_status_s::FAULT_OVER_TEMPERATURE);
				break;

			case FaultFlagMask::Fault_UT_Cells_Chg:
			case FaultFlagMask::Fault_UT_Cells_Dsg:
			case FaultFlagMask::Fault_UT_PCB_Chg:
			case FaultFlagMask::Fault_UT_PCB_Dsg:
				px4_faults |= (1 << battery_status_s::FAULT_UNDER_TEMPERATURE);
				break;

			case FaultFlagMask::Fault_OCD:
			case FaultFlagMask::Fault_OCD_HW:
			case FaultFlagMask::Fault_SCD_HW:
			case FaultFlagMask::Fault_OCC:
				px4_faults |= (1 << battery_status_s::FAULT_OVER_CURRENT);
				break;

			case FaultFlagMask::Fault_Cell_UV_Extreme:
			case FaultFlagMask::Fault_Cell_UV_HW:
				px4_faults |= (1 << battery_status_s::FAULT_DEEP_DISCHARGE);
				break;

			case FaultFlagMask::Fault_Discharge_Cutoff:
			case FaultFlagMask::Fault_CellBalance:
			case FaultFlagMask::Fault_PackMeasure:
			case FaultFlagMask::Fault_AFE_Comms:
			case FaultFlagMask::Fault_FuelGauge_Comms:
			case FaultFlagMask::Fault_FET_Short:
				px4_faults |= (1 << battery_status_s::FAULT_HARDWARE_FAILURE);
				break;


			default:
				break;
			}
		}
	}

	// Check for the battery warnings. These are stored in the General Status flags byte, not the fault flags.
	// These will trigger the same battery status fault as an error for now
	// In future, warnings and errors will be distinguished separately similar to how bat voltage
	for (uint8_t fault_index = 0; fault_index < static_cast<uint8_t>(StatusFlagMask::Count); fault_index++) {
		if (statusFlags & (1 << fault_index)) {

			switch (static_cast<StatusFlagMask>(fault_index)) {

			case StatusFlagMask::Status_OT_Cells_Warning:
			case StatusFlagMask::Status_OT_PCB_Warning:
				px4_faults |= (1 << battery_status_s::FAULT_OVER_TEMPERATURE);
				break;

			default:
				break;
			}
		}
	}


	px4_faults |= checkIncompatibleVoltage(index);

	return px4_faults;
}

uint16_t FF_SmartBattery::checkFirmwareVersion(const uint8_t index, const uint8_t major, const uint8_t minor)
{
	if (((major << 8) | minor) < ((FW_VERSION_MAJOR << 8) | FW_VERSION_MINOR)) {

		if (!(_battery_status[index].faults & (1 << battery_status_s::FAULT_INCOMPATIBLE_FIRMWARE))) {
			PX4_WARN("Battery FW incompatible, current %d.%d - required %d.%d ",
				 major, minor, FW_VERSION_MAJOR, FW_VERSION_MINOR);
		}

		return (_battery_status[index].faults | (1 << battery_status_s::FAULT_INCOMPATIBLE_FIRMWARE));
	}

	return (_battery_status[index].faults & ~(1 << battery_status_s::FAULT_INCOMPATIBLE_FIRMWARE)) ;
}

uint16_t FF_SmartBattery::checkIncompatibleVoltage(const uint8_t index)
{
	// Only check for incompatible voltage when the drone is not armed
	if (_was_armed) {
		return 0;
	}

	int highest_battery_index = 0;
	uint8_t connected_batteries = 0;

	for (int i = 0; i < BATTERIES_COUNT; i++) {
		if (_battery_status[i].connected) {

			if ((_battery_status[i].voltage_v > _battery_status[highest_battery_index].voltage_v)
			    || !_battery_status[highest_battery_index].connected) {

				highest_battery_index = i;
			}

			connected_batteries++;
		}
	}

	if ((connected_batteries > 1)) {

		if ((_battery_status[highest_battery_index].voltage_v - _battery_status[index].voltage_v) > INCONSISTENCY_THRESHOLD) {
			return (1 << battery_status_s::FAULT_INCOMPATIBLE_VOLTAGE);
		}
	}

	return 0;
}

uint16_t FF_SmartBattery::checkIncompatibleModel(const uint8_t index, const uint8_t model)
{
	if (model != EXPECTED_BATTERY_MODEL) {

		if (!(_battery_status[index].faults & (1 << battery_status_s::FAULT_INCOMPATIBLE_MODEL))) {
			PX4_WARN("Battery Model incompatible, current %d - required %d", model, EXPECTED_BATTERY_MODEL);
		}

		return (_battery_status[index].faults | (1 << battery_status_s::FAULT_INCOMPATIBLE_MODEL));
	}

	return (_battery_status[index].faults & ~(1 << battery_status_s::FAULT_INCOMPATIBLE_MODEL));
}

int FF_SmartBattery::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FF_SmartBattery::print_usage(const char *reason)
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

	PRINT_MODULE_USAGE_NAME("ff_smartbattery", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int FF_SmartBattery::print_status()
{
	PX4_INFO("Running");
	perf_print_counter(_cycle_perf);

	return 0;
}

extern "C" __EXPORT int ff_smartbattery_main(int argc, char *argv[])
{
	return ModuleBase::main(FF_SmartBattery::desc, argc, argv);
}
