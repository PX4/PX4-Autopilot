/****************************************************************************
*
*   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
#ifndef VERTIQ_CONFIGURATION_CLIENT_HANDLER_HPP
#define VERTIQ_CONFIGURATION_CLIENT_HANDLER_HPP

/**
	Vertiq modules communicate through a system of clients and entries. Clients contain entries, and entries
	specifiy some sort of Vertiq module parameter or control. Entries can be gotten (its value returned to the requester), set (the
	module's internal value set to the value requested by the user), and saved (writes the currently stored module value to its persistent memory).

	This class exists to coordinate PX4 parameters with Vertiq entries
*/

#include <lib/mixer_module/mixer_module.hpp>

#include "entry_wrapper.hpp"
#include "vertiq_client_manager.hpp"
#include "iq-module-communication-cpp/inc/esc_propeller_input_parser_client.hpp"

#include "iq-module-communication-cpp/inc/arming_handler_client.hpp"
#include "iq-module-communication-cpp/inc/iquart_flight_controller_interface_client.hpp"

#ifdef CONFIG_USE_PULSING_CONFIGURATION
#include "iq-module-communication-cpp/inc/voltage_superposition_client.hpp"
#include "iq-module-communication-cpp/inc/pulsing_rectangular_input_parser_client.hpp"
#endif //CONFIG_USE_PULSING_CONFIGURATION

class VertiqConfigurationHandler
{
public:

	/**
	* @brief Construct a new Vertiq Configuration Handler object
	*
	* @param ser A pointer to our serial interface
	* @param client_manager A pointer to our client manager
	*/
	VertiqConfigurationHandler(VertiqSerialInterface *ser, VertiqClientManager *client_manager);

	/**
	 * @brief Initialize all of our client entry wrappers
	 *
	 */
	void InitClientEntryWrappers();

	/**
	* @brief Set all of the IQUART configuration init flags to true
	*/
	void MarkConfigurationEntriesForRefresh();

	/**
	* @brief Send a Get command to all of the parameters involved in IQUART configuration, and make sure the PX4 parameters and module values agree
	*/
	void UpdateIquartConfigParams();

	/**
	* @brief Until the timeout is reached, keep trying to update the PX4 parameters to match, as appropraite, the value on the module. This can
	*	mean either setting the PX4 parameter to match the motor or vice versa
	 *
	 * @param timeout The maximum amount of time we can keep trying to get responses
	 */
	void CoordinateIquartWithPx4Params(hrt_abstime timeout = 100_ms);

	/**
	* @brief Gives access to the object ID currently being used
	*
	* @return The value stored in _object_id_now
	*/
	uint8_t GetObjectIdNow();

	/**
	* @brief When the target module ID parameter changes, we need to delete and remake all of our configuration clients in order to make sure that they're talking to the
	*        correct motor.
	*
	* @param new_object_id The new target module ID that we should use to instantiate our new clients
	*/
	void UpdateClientsToNewObjId(uint8_t new_object_id);

	/**
	* @brief Initialize all of the Vertiq Clients that we want to use
	*
	* @param object_id The object ID with which to initialize our clients
	*/
	void InitConfigurationClients(uint8_t object_id);

	/**
	 * @brief Delete and recreate a client with a new object ID
	 *
	 * @tparam client_type The actual type of Client you want to delete and remake
	 * @param client A pointer ref to the client being deleted
	 * @param object_id The object id we're making the new client with
	 */
	template <typename client_type>
	void DestroyAndRecreateClient(client_type *&client, uint8_t object_id)
	{
		if (client) {
			delete client;
			client = new client_type(object_id);
		}
	}

	/**
	* @brief Creates and adds a new entry wrapper object to our array of entry wrappers
	*
	* @param px4_param A parameter stored in PX4. This can be found with the param_find function
	* @param entry A pointer to a Vertiq client entry
	*/
	template <typename iquart_data_type, typename px4_data_type>
	void AddNewClientEntry(param_t px4_param, ClientEntryAbstract *entry)
	{
		if (_added_configuration_entry_wrappers < MAX_CLIENT_ENTRIES) {
			_configuration_entry_wrappers[_added_configuration_entry_wrappers] = new EntryWrapper<iquart_data_type, px4_data_type>;
			_configuration_entry_wrappers[_added_configuration_entry_wrappers]->ConfigureStruct(px4_param, entry);
			_added_configuration_entry_wrappers++;

		} else {
			PX4_INFO("Could not add this entry. Maximum number exceeded");
		}
	}

private:

	uint8_t _object_id_now; //The object ID we're targeting right now
	VertiqSerialInterface *_serial_interface; //We need a serial handler in order to talk over the serial port
	VertiqClientManager *_client_manager;

////////////////////////////////////////////////////////////////////////
//Vertiq Client information
	//Known Configuration Clients can be created as pointers to certain types of clients
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
	AbstractEntryWrapper *_configuration_entry_wrappers[MAX_CLIENT_ENTRIES];
	uint8_t _added_configuration_entry_wrappers = 0;
////////////////////////////////////////////////////////////////////////

};

#endif //VERTIQ_CONFIGURATION_CLIENT_HANDLER_HPP
