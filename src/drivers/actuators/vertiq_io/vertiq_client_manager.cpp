
#include "vertiq_client_manager.hpp"

VertiqClientManager::VertiqClientManager(VertiqSerialInterface * serial_interface) :
	_serial_interface(serial_interface),
	_needs_iquart_init(true),
	_broadcast_prop_motor_control(_kBroadcastID), //Initialize with a module ID of 63 for broadcasting
	_broadcast_arming_handler(_kBroadcastID)

{
	_client_array[0] = &_broadcast_prop_motor_control;
	_client_array[1] = &_broadcast_arming_handler;
}

void VertiqClientManager::Init(uint8_t object_id){
	#ifdef CONFIG_USE_SYSTEM_CONTROL_CLIENT
		static SystemControlClient sys_control = SystemControlClient(object_id);
		_system_control = &sys_control;
		_client_array[_clients_in_use] = _system_control;
		_clients_in_use++;
	#endif

	#ifdef CONFIG_USE_IFCI_CONFIGURATION
		static IQUartFlightControllerInterfaceClient ifci = IQUartFlightControllerInterfaceClient(object_id);
		_ifci_client = &ifci;
		_client_array[_clients_in_use] = _ifci_client;
		_clients_in_use++;

		static EscPropellerInputParserClient prop_input_parser = EscPropellerInputParserClient(object_id);
		_prop_input_parser_client = &prop_input_parser;
		_client_array[_clients_in_use] = _prop_input_parser_client;
		_clients_in_use++;
	#endif


	_serial_interface->SetNumberOfClients(_clients_in_use);
}

void VertiqClientManager::HandleClientCommunication(){
	//Update our serial tx before we take in the RX
	_serial_interface->process_serial_tx();

	//Update our serial rx
	_serial_interface->process_serial_rx(&_motor_interface, _client_array);
}

IFCI * VertiqClientManager::GetMotorInterface(){
	return &_motor_interface;
}

uint8_t VertiqClientManager::GetNumberOfClients(){
	return _clients_in_use;
}

void VertiqClientManager::SendSetForceArm(){
	_broadcast_arming_handler.motor_armed_.set(*_serial_interface->get_iquart_interface(), 1);
}

void VertiqClientManager::SendSetForceDisarm(){
	_broadcast_arming_handler.motor_armed_.set(*_serial_interface->get_iquart_interface(), 0);
}

void VertiqClientManager::SendSetCoast(){
	_broadcast_prop_motor_control.ctrl_coast_.set(*_serial_interface->get_iquart_interface());
}

void VertiqClientManager::SendSetVelocitySetpoint(uint16_t velocity_setpoint){
	_broadcast_prop_motor_control.ctrl_velocity_.set(*_serial_interface->get_iquart_interface(), velocity_setpoint);
}

void VertiqClientManager::InitParameter(param_t parameter, bool * init_bool, char descriptor, EntryData value){
	float float_value = value.float_data;
	uint32_t uint_value = value.uint_data;

	switch(descriptor){
		case 'f':
			param_set(parameter, &(float_value));
			*init_bool = false;
		break;

		case 'b':
			param_set(parameter, &(uint_value));
			*init_bool = false;
		break;
	}
}


void VertiqClientManager::SendSetAndSave(ClientEntryAbstract * entry, char descriptor, EntryData value){
	//Note that we have to use the brackets to make sure that the ClientEntry objects have a scope
	switch(descriptor){
		case 'f': {
			ClientEntry<float> * float_entry = (ClientEntry<float> *)(entry);
			float_entry->set(*_serial_interface->get_iquart_interface(), value.float_data);
			float_entry->save(*_serial_interface->get_iquart_interface());
		break;
		}
		case 'b':{
			ClientEntry<uint8_t> * byte_entry = (ClientEntry<uint8_t> *)(entry);
			byte_entry->set(*_serial_interface->get_iquart_interface(), value.uint_data);
			byte_entry->save(*_serial_interface->get_iquart_interface());
		break;
		}
	}

	_serial_interface->process_serial_tx();
}

bool VertiqClientManager::FloatsAreClose(float val1, float val2, float tolerance){
	float diff = val1 - val2;
	return(abs(diff) < tolerance);
}


#ifdef CONFIG_USE_SYSTEM_CONTROL_CLIENT
void VertiqClientManager::GetAllSystemControlEntries(){
	// _system_control->dev_id_.get(*_serial_interface->get_iquart_interface());
	// _system_control->rev_id_.get(*_serial_interface->get_iquart_interface());
	// _system_control->uid1_.get(*_serial_interface->get_iquart_interface());
	// _system_control->uid2_.get(*_serial_interface->get_iquart_interface());
	// _system_control->uid3_.get(*_serial_interface->get_iquart_interface());
	// _system_control->mem_size_.get(*_serial_interface->get_iquart_interface());
	// _system_control->build_year_.get(*_serial_interface->get_iquart_interface());
	// _system_control->build_month_.get(*_serial_interface->get_iquart_interface());
	// _system_control->build_day_.get(*_serial_interface->get_iquart_interface());
	// _system_control->build_hour_.get(*_serial_interface->get_iquart_interface());
	// _system_control->build_minute_.get(*_serial_interface->get_iquart_interface());
	// _system_control->build_second_.get(*_serial_interface->get_iquart_interface());
	// _system_control->module_id_.get(*_serial_interface->get_iquart_interface());
	// _system_control->time_.get(*_serial_interface->get_iquart_interface());
	// _system_control->firmware_version_.get(*_serial_interface->get_iquart_interface());
	// _system_control->hardware_version_.get(*_serial_interface->get_iquart_interface());
	// _system_control->electronics_version_.get(*_serial_interface->get_iquart_interface());
	// _system_control->firmware_valid_.get(*_serial_interface->get_iquart_interface());
	// _system_control->applications_present_.get(*_serial_interface->get_iquart_interface());
	// _system_control->bootloader_version_.get(*_serial_interface->get_iquart_interface());
	// _system_control->upgrade_version_.get(*_serial_interface->get_iquart_interface());
	// _system_control->system_clock_.get(*_serial_interface->get_iquart_interface());
	// _system_control->control_flags_.get(*_serial_interface->get_iquart_interface());
	// _system_control->pcb_version_.get(*_serial_interface->get_iquart_interface());

	WaitForSystemControlResponses(100_ms);
}


void VertiqClientManager::WaitForSystemControlResponses(hrt_abstime timeout){
	//Get the start time and the end time
	hrt_abstime time_now = hrt_absolute_time();
	hrt_abstime end_time = time_now + timeout;

	uint32_t read_value = 0;

	while(time_now < end_time){
		// if(_system_control->dev_id_.IsFresh()){
		// 	read_value = _system_control->dev_id_.get_reply() & 0x0000FFFF;
		// 	param_set(param_find("DEVICE_ID"), &read_value);
		// }

		// if(_system_control->rev_id_.IsFresh()){
		// 	read_value = _system_control->rev_id_.get_reply() & 0x0000FFFF;
		// 	param_set(param_find("REV_ID"), &read_value);
		// }

		if(_system_control->uid1_.IsFresh()){
			read_value = _system_control->uid1_.get_reply();
			param_set(param_find("UID1"), &read_value);
		}

		if(_system_control->uid2_.IsFresh()){
			read_value = _system_control->uid2_.get_reply();
			param_set(param_find("UID2"), &read_value);
		}

		if(_system_control->uid3_.IsFresh()){
			read_value = _system_control->uid3_.get_reply();
			param_set(param_find("UID3"), &read_value);
		}

		// if(_system_control->mem_size_.IsFresh()){
		// 	read_value = _system_control->mem_size_.get_reply() & 0x0000FFFF;
		// 	param_set(param_find("MEM_SIZE"), &read_value);
		// }

		// if(_system_control->build_year_.IsFresh()){
		// 	read_value = _system_control->build_year_.get_reply() & 0x0000FFFF;
		// 	param_set(param_find("BUILD_YEAR"), &read_value);
		// }

		// if(_system_control->build_month_.IsFresh()){
		// 	read_value = _system_control->build_month_.get_reply() & 0x000000FF;
		// 	param_set(param_find("BUILD_MONTH"), &read_value);
		// }

		// if(_system_control->build_day_.IsFresh()){
		// 	read_value = _system_control->build_day_.get_reply() & 0x000000FF;
		// 	param_set(param_find("BUILD_DAY"), &read_value);
		// }

		// if(_system_control->build_hour_.IsFresh()){
		// 	read_value = _system_control->build_hour_.get_reply() & 0x000000FF;
		// 	param_set(param_find("BUILD_HOUR"), &read_value);
		// }

		// if(_system_control->build_minute_.IsFresh()){
		// 	read_value = _system_control->build_minute_.get_reply() & 0x000000FF;
		// 	param_set(param_find("BUILD_MIN"), &read_value);
		// }

		// if(_system_control->build_second_.IsFresh()){
		// 	read_value = _system_control->build_second_.get_reply() & 0x000000FF;
		// 	param_set(param_find("BUILD_SEC"), &read_value);
		// }

		// if(_system_control->module_id_.IsFresh()){
		// 	read_value = _system_control->module_id_.get_reply() & 0x000000FF;
		// 	param_set(param_find("MODULE_ID"), &read_value);
		// }

		// if(_system_control->time_.IsFresh()){
		// 	float time = _system_control->time_.get_reply();
		// 	param_set(param_find("MODULE_TIME"), &time);
		// }

		// if(_system_control->firmware_version_.IsFresh()){
		// 	read_value = _system_control->firmware_version_.get_reply();
		// 	param_set(param_find("FIRMWARE_VERSION"), &read_value);
		// }

		// if(_system_control->hardware_version_.IsFresh()){
		// 	read_value = _system_control->hardware_version_.get_reply();
		// 	param_set(param_find("HARDWARE_VERSION"), &read_value);
		// }

		// if(_system_control->electronics_version_.IsFresh()){
		// 	read_value = _system_control->electronics_version_.get_reply();
		// 	param_set(param_find("ELEC_VERSION"), &read_value);
		// }

		// if(_system_control->firmware_valid_.IsFresh()){
		// 	read_value = _system_control->firmware_valid_.get_reply() & 0x000000FF;
		// 	param_set(param_find("FIRMWARE_VALID"), &read_value);
		// }

		if(_system_control->applications_present_.IsFresh()){
			read_value = _system_control->applications_present_.get_reply() & 0x000000FF;
			param_set(param_find("APPS_PRESENT"), &read_value);
		}

		if(_system_control->bootloader_version_.IsFresh()){
			read_value = _system_control->bootloader_version_.get_reply();
			param_set(param_find("BOOT_VERSION"), &read_value);
		}

		// if(_system_control->upgrade_version_.IsFresh()){
		// 	read_value = _system_control->upgrade_version_.get_reply();
		// 	param_set(param_find("UPGRADE_VERSION"), &read_value);
		// }

		// if(_system_control->system_clock_.IsFresh()){
		// 	read_value = _system_control->system_clock_.get_reply();
		// 	param_set(param_find("SYS_CLOCK"), &read_value);
		// }

		// if(_system_control->control_flags_.IsFresh()){
		// 	read_value = _system_control->control_flags_.get_reply();
		// 	param_set(param_find("CTRL_FLAGS"), &read_value);
		// }

		// if(_system_control->pcb_version_.IsFresh()){
		// 	read_value = _system_control->pcb_version_.get_reply();
		// 	param_set(param_find("PCB_VERSION"), &read_value);
		// }

		time_now = hrt_absolute_time();

	}

	PX4_INFO("Done getting responses");
}

#endif //CONFIG_USE_SYSTEM_CONTROL_CLIENT

#ifdef CONFIG_USE_IFCI_CONFIGURATION

void VertiqClientManager::UpdateIfciConfigParams(){
	_prop_input_parser_client->velocity_max_.get(*_serial_interface->get_iquart_interface());
	_prop_input_parser_client->volts_max_.get(*_serial_interface->get_iquart_interface());
	_prop_input_parser_client->mode_.get(*_serial_interface->get_iquart_interface());
	_prop_input_parser_client->sign_.get(*_serial_interface->get_iquart_interface());
	_ifci_client->throttle_cvi_.get(*_serial_interface->get_iquart_interface());

	//Update our serial tx before we take in the RX
	_serial_interface->process_serial_tx();

	CoordinateIquartWithPx4Params(100_ms);
}

void VertiqClientManager::CoordinateIquartWithPx4Params(hrt_abstime timeout){
	//Get the start time and the end time
	hrt_abstime time_now = hrt_absolute_time();
	hrt_abstime end_time = time_now + timeout;

	EntryData entry_values;

	uint32_t module_read_value = 0;
	float module_float_value = 0;

	int32_t px4_read_value = 0;
	float px4_float_value = 0;

	while(time_now < end_time){
		if(_prop_input_parser_client->velocity_max_.IsFresh()){
			module_float_value = _prop_input_parser_client->velocity_max_.get_reply();

			if(_init_velocity_max){
				entry_values.float_data = module_float_value;
				InitParameter(param_find("MAX_VELOCITY"), &_init_velocity_max, 'f', entry_values);
			}else{
				param_get(param_find("MAX_VELOCITY"), &px4_float_value);

				if(!FloatsAreClose(px4_float_value, module_float_value)){
					entry_values.float_data = px4_float_value;
					SendSetAndSave(&(_prop_input_parser_client->velocity_max_), 'f', entry_values);
				}
			}
		}

		if(_prop_input_parser_client->volts_max_.IsFresh()){
			module_float_value = _prop_input_parser_client->volts_max_.get_reply();
			if(_init_volts_max){
				entry_values.float_data = module_float_value;
				InitParameter(param_find("MAX_VOLTS"), &_init_volts_max, 'f', entry_values);
			}else{
				param_get(param_find("MAX_VOLTS"), &px4_float_value);

				if(!FloatsAreClose(px4_float_value, module_float_value)){
					entry_values.float_data = px4_float_value;
					SendSetAndSave(&(_prop_input_parser_client->volts_max_), 'f', entry_values);
				}
			}
		}

		if(_prop_input_parser_client->mode_.IsFresh()){
			module_read_value = _prop_input_parser_client->mode_.get_reply();
			if(_init_mode){
				entry_values.uint_data = module_read_value;
				InitParameter(param_find("CONTROL_MODE"), &_init_mode, 'b', entry_values);
			}else{
				param_get(param_find("CONTROL_MODE"), &px4_read_value);

				if((uint32_t)px4_read_value != module_read_value){
					entry_values.uint_data = (uint32_t)px4_read_value;
					SendSetAndSave(&(_prop_input_parser_client->mode_), 'b', entry_values);
					PX4_INFO("control mode changed");
				}
			}
		}

		if(_prop_input_parser_client->sign_.IsFresh()){
			module_read_value = _prop_input_parser_client->sign_.get_reply();
			if(_init_motor_dir){
				entry_values.uint_data = module_read_value;
				InitParameter(param_find("VERTIQ_MOTOR_DIR"), &_init_motor_dir, 'b', entry_values);
			}else{
				param_get(param_find("VERTIQ_MOTOR_DIR"), &px4_read_value);

				if((uint32_t)px4_read_value != module_read_value){
					entry_values.uint_data = (uint32_t)px4_read_value;
					SendSetAndSave(&(_prop_input_parser_client->sign_), 'b', entry_values);
					PX4_INFO("motor dir changed");
				}
			}
		}

		if(_ifci_client->throttle_cvi_.IsFresh()){
			module_read_value = _ifci_client->throttle_cvi_.get_reply();
			if(_init_throttle_cvi){
				entry_values.uint_data = module_read_value;
				InitParameter(param_find("THROTTLE_CVI"), &_init_throttle_cvi, 'b', entry_values);
			}else{
				param_get(param_find("THROTTLE_CVI"), &px4_read_value);

				if((uint32_t)px4_read_value != module_read_value){
					entry_values.uint_data = (uint32_t)px4_read_value;
					SendSetAndSave(&(_ifci_client->throttle_cvi_), 'b', entry_values);
					PX4_INFO("throttle cvi changed");
				}
			}
		}

		//Update
		time_now = hrt_absolute_time();
		//Update our serial rx
		_serial_interface->process_serial_rx(&_motor_interface, _client_array);
		// HandleClientCommunication();
	}
}

#endif
