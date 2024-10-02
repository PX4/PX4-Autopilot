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

#ifndef ENTRY_WRAPPER_HPP
#define ENTRY_WRAPPER_HPP

#include <parameters/param.h>

#include <px4_log.h>
#include <px4_platform_common/module.h>

#include "vertiq_serial_interface.hpp"
#include "iq-module-communication-cpp/inc/client_communication.hpp"

class AbstractEntryWrapper
{
public:
	AbstractEntryWrapper() {}
	virtual void ConfigureStruct(param_t parameter, ClientEntryAbstract *entry) = 0;
	virtual void SetNeedsInit() = 0;
	virtual ClientEntryAbstract *GetClientEntry() = 0;
	virtual void SendGet(VertiqSerialInterface *ser) = 0;
	virtual void Update(VertiqSerialInterface *serial_interface) = 0;
};

template <typename module_data_type, typename px4_data_type>
class EntryWrapper : public AbstractEntryWrapper
{
public:

	param_t _param;
	ClientEntry<module_data_type> *_entry;
	bool _needs_init;

	px4_data_type _px4_value;

	EntryWrapper() {}

	/**
	* @brief Initializes our EntryWrapper with a PX4 parameter and a Vetiq entry
	*
	* @param parameter A parameter stored in PX4. This can be found with the param_find function
	* @param entry A pointer to a Vertiq client entry
	*/
	void ConfigureStruct(param_t parameter, ClientEntryAbstract *entry)
	{
		_param = parameter;
		_entry = (ClientEntry<module_data_type> *)entry;
		_needs_init = true;
	}

	/**
	* @brief Sets our _needs_init flag high
	*/
	void SetNeedsInit()
	{
		_needs_init = true;
	}

	/**
	* @brief Returns a pointer to our Vertiq entry
	*
	* @return _entry
	*/
	ClientEntryAbstract *GetClientEntry()
	{
		return _entry;
	}

	/**
	* @brief Sends a get message to our Vertiq entry
	*
	* @param ser A pointer to our serial interface
	*/
	void SendGet(VertiqSerialInterface *ser)
	{
		_entry->get(*ser->GetIquartInterface());
		ser->ProcessSerialTx();
	}

	/**
	* @brief Sends both a set and save message to our Vertiq entry with a new value
	*
	* @param ser A pointer to our serial interface
	* @param value The new value we are setting and saving on the connected motor
	*/
	void SendSetAndSave(VertiqSerialInterface *ser, module_data_type value)
	{
		_entry->set(*ser->GetIquartInterface(), value);
		_entry->save(*ser->GetIquartInterface());
		ser->ProcessSerialTx();
	}

	/**
	 * @brief Checks to see if two values are the same. We will treat all parameters as floats regardless of their given type
	 *
	 * @param val1 The first value to compare
	 * @param val2 The second value to compare
	 * @param tolerance A tolerance to use in order to tell if the values are the same
	 * @return true If the values are the same
	 * @return false If the values are different
	 */
	bool ValuesAreTheSame(float val1, float val2, float tolerance = 0.001f)
	{
		float diff = val1 - val2;

		if (diff < 0) {
			return -diff < tolerance;
		}

		return diff < tolerance;
	}

	/**
	* @brief Updates the relationship between the Vertiq and PX4 parameters. First, check to see if the connected motor sent data for us to read. Then, depending on the _needs_init
	* flag, we either set the gotten value into the PX4 parameter, or we set the new PX4 value to the motor.
	*
	* @param serial_interface A pointer to a serial interface used to send data
	*/
	void Update(VertiqSerialInterface *serial_interface)
	{
		//Make sure we have a way to store the data from PX4 and from the module
		module_data_type module_value = 0;
		px4_data_type px4_value = 0;

		//If there's new data in the entry
		if (_entry->IsFresh()) {
			//Grab the data
			module_value = _entry->get_reply();

			//If we need to set the PX4 data to the module's value, otherwise set the module value
			if (_needs_init) {
				px4_value = (px4_data_type)module_value;
				param_set(_param, &px4_value);
				_needs_init = false;

			} else {
				//Grab the PX4 value
				param_get(_param, &px4_value);

				//If the value here is different from the module's value, set and save it
				//Treat everything as a float to avoid type issues in comparison
				if (!ValuesAreTheSame((float)px4_value, (float)module_value)) {
					SendSetAndSave(serial_interface, px4_value);
				}
			}
		}
	}
};

#endif //ENTRY_WRAPPER_HPP
