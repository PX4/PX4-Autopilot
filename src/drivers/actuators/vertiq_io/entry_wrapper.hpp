/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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

	void ConfigureStruct(param_t parameter, ClientEntryAbstract *entry)
	{
		_param = parameter;
		_entry = (ClientEntry<module_data_type> *)entry;
		_needs_init = true;
	}

	void SetNeedsInit()
	{
		_needs_init = true;
	}

	ClientEntryAbstract *GetClientEntry()
	{
		return _entry;
	}

	void SendGet(VertiqSerialInterface *ser)
	{
		_entry->get(*ser->get_iquart_interface());
	}

	void SendSetAndSave(VertiqSerialInterface *ser, module_data_type value)
	{
		_entry->set(*ser->get_iquart_interface(), value);
		_entry->save(*ser->get_iquart_interface());
		ser->process_serial_tx();
	}

	bool ValuesAreTheSame(int32_t val1, int32_t val2, int32_t tolerance)
	{
		return (val1 == val2);
	}

	bool ValuesAreTheSame(float val1, float val2, float tolerance)
	{
		float diff = val1 - val2;
		return (abs(diff) < tolerance);
	}

	void Update(VertiqSerialInterface *serial_interface)
	{
		module_data_type module_value = 0;
		px4_data_type px4_value = 0;

		if (_entry->IsFresh()) {
			module_value = _entry->get_reply();

			if (_needs_init) {
				// PX4_INFO("combo param needed init");
				px4_value = (px4_data_type)module_value;
				param_set(_param, &px4_value);
				_needs_init = false;
				// InitParameter(_param, &(combined_entry->_needs_init), &px4_value);

			} else {
				// PX4_INFO("combo param did not need init");
				param_get(_param, &px4_value);

				if (!ValuesAreTheSame(px4_value, (px4_data_type)module_value, (px4_data_type)(0.01))) {
					SendSetAndSave(serial_interface, px4_value);
				}
			}
		}
	}
};
