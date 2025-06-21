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
 * @file Battery.hpp
 *
 * Defines basic functionality of Cyphal Battery subscription
 *
 * @author Jacob Crabill <jacob@flyvoly.com>
 */

#pragma once

#include <uORB/topics/battery_info.h>
#include <uORB/topics/battery_status.h>
#include <uORB/PublicationMulti.hpp>

// UDRAL Specification Messages
#include <reg/udral/physics/electricity/SourceTs_0_1.h>
#include <reg/udral/service/battery/Parameters_0_3.h>
#include <reg/udral/service/battery/Status_0_2.h>

#include "../DynamicPortSubscriber.hpp"

#define KELVIN_OFFSET 273.15f
#define WH_TO_JOULE   3600

class UavcanBmsSubscriber : public UavcanDynamicPortSubscriber
{
public:
	UavcanBmsSubscriber(CanardHandle &handle, UavcanParamManager &pmgr, uint8_t instance = 0) :
		UavcanDynamicPortSubscriber(handle, pmgr, "udral.", "energy_source", instance)
	{
		_subj_sub.next = &_status_sub;

		_status_sub._subject_name = _status_name;
		_status_sub._canard_sub.user_reference = this;
		_status_sub.next = &_parameters_sub;

		_parameters_sub._subject_name = _parameters_name;
		_parameters_sub._canard_sub.user_reference = this;
		_parameters_sub.next = nullptr;
	}

	void subscribe() override
	{
		// Subscribe to messages reg.drone.physics.electricity.SourceTs.0.1
		_canard_handle.RxSubscribe(CanardTransferKindMessage,
					   _subj_sub._canard_sub.port_id,
					   reg_udral_physics_electricity_SourceTs_0_1_EXTENT_BYTES_,
					   CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
					   &_subj_sub._canard_sub);

		// Subscribe to messages reg.drone.service.battery.Status.0.2
		_canard_handle.RxSubscribe(CanardTransferKindMessage,
					   _status_sub._canard_sub.port_id,
					   reg_udral_service_battery_Status_0_2_EXTENT_BYTES_,
					   CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
					   &_status_sub._canard_sub);

		// Subscribe to messages reg.drone.service.battery.Parameters.0.3
		_canard_handle.RxSubscribe(CanardTransferKindMessage,
					   _parameters_sub._canard_sub.port_id,
					   reg_udral_service_battery_Parameters_0_3_EXTENT_BYTES_,
					   CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
					   &_parameters_sub._canard_sub);
	};

	void callback(const CanardRxTransfer &receive) override
	{
		if (receive.metadata.port_id == _subj_sub._canard_sub.port_id) {
			reg_udral_physics_electricity_SourceTs_0_1 source_ts {};
			size_t source_ts_size_in_bytes = receive.payload_size;
			reg_udral_physics_electricity_SourceTs_0_1_deserialize_(&source_ts,
					(const uint8_t *)receive.payload,
					&source_ts_size_in_bytes);
			bat_status.timestamp = hrt_absolute_time(); //FIXME timesyncronization source_ts.timestamp.microsecond;
			bat_status.voltage_v = source_ts.value.power.voltage.volt;
			bat_status.current_a = source_ts.value.power.current.ampere;

			bat_status.connected = true; // Based on some threshold or an error??

			// Estimate discharged mah from Joule
			if (PX4_ISFINITE(_nominal_voltage)) {
				bat_status.discharged_mah = (source_ts.value.full_energy.joule - source_ts.value.energy.joule)
							    / (_nominal_voltage * WH_TO_JOULE);
			}

			bat_status.remaining = source_ts.value.energy.joule / source_ts.value.full_energy.joule;

			// TODO uORB publication rate limiting
			_battery_status_pub.publish(bat_status);

			_battery_info.timestamp = bat_status.timestamp;
			_battery_info.id = bat_status.id;
			_battery_info_pub.publish(_battery_info);

		} else if (receive.metadata.port_id == _status_sub._canard_sub.port_id) {
			reg_udral_service_battery_Status_0_2 bat {};
			size_t bat_size_in_bytes = receive.payload_size;
			reg_udral_service_battery_Status_0_2_deserialize_(&bat, (const uint8_t *)receive.payload, &bat_size_in_bytes);

			bat_status.scale = -1; // What does the mean?

			bat_status.temperature = bat.temperature_min_max[1].kelvin - KELVIN_OFFSET; // PX4 uses degC we assume

			bat_status.cell_count = bat.cell_voltages.count;

			uint32_t cell_count = bat_status.cell_count;

			if (cell_count > (sizeof(bat_status.voltage_cell_v) / sizeof(bat_status.voltage_cell_v[0]))) {
				cell_count = sizeof(bat_status.voltage_cell_v) / sizeof(bat_status.voltage_cell_v[0]);
			}

			float voltage_cell_min = FLT_MAX_EXP;
			float voltage_cell_max = FLT_MIN_EXP;

			for (uint32_t i = 0; i < cell_count; i++) {
				bat_status.voltage_cell_v[i] = bat.cell_voltages.elements[i];

				if (bat_status.voltage_cell_v[i] > voltage_cell_max) {
					voltage_cell_max = bat_status.voltage_cell_v[i];
				}

				if (bat_status.voltage_cell_v[i] < voltage_cell_min) {
					voltage_cell_min = bat_status.voltage_cell_v[i];
				}
			}

			bat_status.max_cell_voltage_delta = voltage_cell_max - voltage_cell_min; // Current delta or max delta over time?

		} else if (receive.metadata.port_id == _parameters_sub._canard_sub.port_id) {
			reg_udral_service_battery_Parameters_0_3 parameters {};
			size_t parameters_size_in_bytes = receive.payload_size;
			reg_udral_service_battery_Parameters_0_3_deserialize_(&parameters,
					(const uint8_t *)receive.payload,
					&parameters_size_in_bytes);

			bat_status.capacity = parameters.design_capacity.coulomb / 3.6f; // Coulomb -> mAh
			bat_status.cycle_count = parameters.cycle_count;
			snprintf(_battery_info.serial_number, sizeof(_battery_info.serial_number), "%" PRIu64, parameters.unique_id);
			bat_status.state_of_health = parameters.state_of_health_pct;
			bat_status.max_error = 1; // UAVCAN didn't spec'ed this, but we're optimistic
			bat_status.id = 0; //TODO instancing
			_nominal_voltage = parameters.nominal_voltage.volt;
		}
	}

private:
	float _nominal_voltage = NAN;

	uORB::PublicationMulti<battery_info_s> _battery_info_pub{ORB_ID(battery_info)};
	uORB::PublicationMulti<battery_status_s> _battery_status_pub{ORB_ID(battery_status)};

	SubjectSubscription _status_sub;
	SubjectSubscription _parameters_sub;

	const char *_status_name = "battery_status";
	const char *_parameters_name = "battery_parameters";

	battery_info_s _battery_info{};
	battery_status_s bat_status {0};
};
