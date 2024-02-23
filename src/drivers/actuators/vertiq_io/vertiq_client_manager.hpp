/****************************************************************************
 *
 *   Copyright (c) 2012-2024 PX4 Development Team. All rights reserved.
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
#ifndef VERTIQ_CLLIENT_MANAGER_HPP
#define VERTIQ_CLLIENT_MANAGER_HPP

#include <drivers/device/device.h>
#include <lib/led/led.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/perf/perf_counter.h>

#include <parameters/param.h>

#include <px4_log.h>
#include <px4_platform_common/module.h>

#include "vertiq_serial_interface.hpp"

#include "entry_wrapper.hpp"

#include "iq-module-communication-cpp/inc/propeller_motor_control_client.hpp"
#include "iq-module-communication-cpp/inc/brushless_drive_client.hpp"
#include "iq-module-communication-cpp/inc/arming_handler_client.hpp"

#include "iq-module-communication-cpp/inc/esc_propeller_input_parser_client.hpp"

#ifdef CONFIG_USE_IFCI_CONFIGURATION
#include "iq-module-communication-cpp/inc/iquart_flight_controller_interface_client.hpp"
#endif //CONFIG_USE_IFCI_CONFIGURATION

#ifdef CONFIG_USE_PULSING_CONFIGURATION
#include "iq-module-communication-cpp/inc/voltage_superposition_client.hpp"
#include "iq-module-communication-cpp/inc/pulsing_rectangular_input_parser_client.hpp"
#endif //CONFIG_USE_PULSING_CONFIGURATION

static const uint8_t _kBroadcastID = 63;

class VertiqClientManager
{
public:
	/**
	* @brief Construct a new VertiqClientManager object
	*
	* @param serial_interface A pointer to a VertiqSerialInterface object
	*/
	VertiqClientManager(VertiqSerialInterface *serial_interface);

	/**
	* @brief Initialize all of our clients with the object ID given by the PX4 parameter TARGET_MODULE_ID
	*/
	void Init(uint8_t object_id);

	/**
	* @brief Handle the IQUART interface. Make sure that we update TX and RX buffers
	*/
	void HandleClientCommunication();

	/**
	* @brief Add a set to the output buffer that will force the connected motor to arm
	*/
	void SendSetForceArm();

	/**
	* @brief Add a set to the output buffer that will force the connected motor to disarm
	*/
	void SendSetForceDisarm();

	/**
	* @brief Add a set to the output buffer that will force the connected motor to coast
	*/
	void SendSetCoast();

	/**
	* @brief Add a set to the output buffer that will force the connected motor to spin at a given setpoint
	*
	* @param velocity_setpoint the raw 16-bit velocity command going to the motor
	*/
	void SendSetVelocitySetpoint(uint16_t velocity_setpoint);

	/**
	* @brief Set all of the IQUART configuration init flags to true
	*/
	void MarkIquartConfigsForRefresh();

	/**
	* @brief Send a Get command to all of the parameters involved in IQUART configuration, and make sure the PX4 parameters and module values agree
	*/
	void UpdateIquartConfigParams();

	/**
	* @brief Until the timeout is reached, keep trying to update the PX4 parameters to match, as appropraite, the value on the module. This can
	*	mean either setting the PX4 parameter to match the motor or vice versa
	*/
	void CoordinateIquartWithPx4Params(hrt_abstime timeout = 2_s);

	/**
	* @brief Gives access to the object ID currently being used
	*
	* @return The value stored in _object_id_now
	*/
	uint8_t GetObjectIdNow();

	/**
	* @brief When the target module ID parameter changes, we need to delete and remake all of our clients in order to make sure that we're talking to the
	*        correct motor.
	*
	* @param new_object_id The new target module ID that we should use to instantiate our new clients
	*/
	void UpdateClientsToNewObjId(uint8_t new_object_id);

	/**
	* @brief Adds a new client to our array of Configuration Clients. Configurations clients are those meant to interface a configurable module parameter
	* with a PX4 parameter. These are clients whose module ID will change over time, and will be dynamically updated
	*
	* @param client a pointer to the new client
	*/
	void AddNewConfigurationClient(ClientAbstract * client);

	/**
	* @brief Adds a new client to our array of Operational Clients. Operational clients are those meant to hold an operational client such as those used
	* for direct motor control. Operational clients should have a constant module ID, and should be made only once
	*/
	void AddNewOperationalClient(ClientAbstract * client);

	/**
	* @brief Returns the number of clients added to our Configuration Clients array
	*
	* @return The value _configuration_clients_in_use
	*/
	uint8_t GetNumberOfConfigurationClients();

	/**
	* @brief Returns the number of clients added to our Operational Clients array
	*
	* @return The value _operational_clients_in_use
	*/
	uint8_t GetNumberOfOperationalClients();

	/**
	* @brief Returns a pointer to our IFCI client used for telemetry
	*
	* @return The _telem_ifci pointer
	*/
	IQUartFlightControllerInterfaceClient *GetTelemIFCI();

	/**
	* @brief Creates and adds a new entry wrapper object to our array of entry wrappers
	*
	* @param px4_param A parameter stored in PX4. This can be found with the param_find function
	* @param entry A pointer to a Vertiq client entry
	*/
	template <typename iquart_data_type , typename px4_data_type>
	void AddNewClientEntry(param_t px4_param, ClientEntryAbstract *entry){
		if(_added_entry_wrappers < MAX_CLIENT_ENTRIES){
			_entry_wrappers[_added_entry_wrappers] = new EntryWrapper<iquart_data_type, px4_data_type>;
			_entry_wrappers[_added_entry_wrappers]->ConfigureStruct(px4_param, entry);
			_added_entry_wrappers++;
		}else{
			PX4_INFO("Could not add this entry. Maximum number exceeded");
		}
	}

private:
	/**
	* @brief Initialize all of the Vertiq Clients that we want to use
	*
	* @param object_id The object ID with which to initialize our clients
	*/
	void InitVertiqClients(uint8_t object_id);

	/**
	* @brief Initialize all of the Entry Wrappers
	*/
	void InitEntryWrappers();

	uint8_t _object_id_now;

	//We need a serial handler in order to talk over the serial port
	VertiqSerialInterface *_serial_interface;

	//These exist whenever we're using IQUART
	EntryWrapper<float, float> _velocity_max_entry;
	EntryWrapper<float, float> _voltage_max_entry;
	EntryWrapper<uint8_t, int32_t> _control_mode_entry;
	EntryWrapper<uint8_t, int32_t> _motor_direction_entry;
	EntryWrapper<uint8_t, int32_t> _fc_direction_entry;

#ifdef CONFIG_USE_IFCI_CONFIGURATION
	EntryWrapper<uint8_t, int32_t> _throttle_cvi_entry;
#endif //CONFIG_USE_IFCI_CONFIGURATION

#ifdef CONFIG_USE_PULSING_CONFIGURATION
	EntryWrapper<uint8_t, int32_t> _pulsing_voltage_mode_entry;
	EntryWrapper<uint8_t, int32_t> _x_cvi_entry;
	EntryWrapper<uint8_t, int32_t> _y_cvi_entry;
	EntryWrapper<float, float> _pulse_zero_angle_entry;
	EntryWrapper<float, float> _pulse_velo_cutoff_entry;
	EntryWrapper<float, float> _pulse_torque_offset_entry;
	EntryWrapper<float, float> _pulse_volt_limit_entry;
#endif //CONFIG_USE_PULSING_CONFIGURATION

////////////////////////////////////////////////////////////////////////
//Vertiq client information
	//Some constants to help us out
	static const uint8_t MAXIMUM_CONFIGURATION_CLIENTS = 20; //These are clients whose module ID will change when Target Module ID changes
	static const uint8_t MAXIMUM_OPERATIONAL_CLIENTS = 20; //These are clients that are used for module control/telemetry. They have a static Module ID

	//Client Arrays
	ClientAbstract *_configuration_client_array[MAXIMUM_CONFIGURATION_CLIENTS];
	uint8_t _configuration_clients_in_use = 0;

	ClientAbstract *_operational_client_array[MAXIMUM_OPERATIONAL_CLIENTS];
	uint8_t _operational_clients_in_use = 0;

	//Clients
	PropellerMotorControlClient _broadcast_prop_motor_control;
	ArmingHandlerClient _broadcast_arming_handler;
	IQUartFlightControllerInterfaceClient *_telem_ifci;
	EscPropellerInputParserClient *_prop_input_parser_client;

#ifdef CONFIG_USE_IFCI_CONFIGURATION
	//Make all of the clients that we need to talk to the IFCI config params
	IQUartFlightControllerInterfaceClient *_ifci_client;
#endif //CONFIG_USE_IFCI_CONFIGURATION

#ifdef CONFIG_USE_PULSING_CONFIGURATION
	VoltageSuperPositionClient *_voltage_superposition_client;
	PulsingRectangularInputParserClient *_pulsing_rectangular_input_parser_client;
#endif //CONFIG_USE_PULSING_CONFIGURATION
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
//Vertiq Client Entry information
	static const uint8_t MAX_CLIENT_ENTRIES = 40;

#ifdef CONFIG_USE_PULSING_CONFIGURATION
	uint8_t _added_entry_wrappers = 13;
	AbstractEntryWrapper *_entry_wrappers[MAX_CLIENT_ENTRIES] = {&_velocity_max_entry, &_voltage_max_entry, &_pulse_zero_angle_entry, &_pulse_velo_cutoff_entry, &_pulse_torque_offset_entry, &_pulse_volt_limit_entry, &_control_mode_entry, &_motor_direction_entry, &_fc_direction_entry, &_throttle_cvi_entry, &_pulsing_voltage_mode_entry, &_x_cvi_entry, &_y_cvi_entry};
#elif defined(CONFIG_USE_IFCI_CONFIGURATION)
	uint8_t _added_entry_wrappers = 6;
	AbstractEntryWrapper *_entry_wrappers[MAX_CLIENT_ENTRIES] = {&_velocity_max_entry, &_voltage_max_entry, &_control_mode_entry, &_motor_direction_entry, &_fc_direction_entry, &_throttle_cvi_entry};
#else
	uint8_t _added_entry_wrappers = 5;
	AbstractEntryWrapper *_entry_wrappers[MAX_CLIENT_ENTRIES] = {&_velocity_max_entry, &_voltage_max_entry, &_control_mode_entry, &_motor_direction_entry, &_fc_direction_entry};
#endif
////////////////////////////////////////////////////////////////////////
};

#endif
