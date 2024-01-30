
#include "vertiq_client_manager.hpp"

VertiqClientManager::VertiqClientManager(VertiqSerialInterface * serial_interface) :
	_serial_interface(serial_interface),
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

#ifdef CONFIG_USE_SYSTEM_CONTROL_CLIENT
void VertiqClientManager::UpdateSystemControlEntries(){
	_system_control->hardware_version_.get(*_serial_interface->get_iquart_interface());
	_system_control->firmware_version_.get(*_serial_interface->get_iquart_interface());
	WaitForSystemControlResponses();
}

void VertiqClientManager::WaitForSystemControlResponses(hrt_abstime timeout){
	//Get the start time and the end time
	hrt_abstime time_now = hrt_absolute_time();
	hrt_abstime end_time = time_now + timeout;

	//While there's still more time to keep trying
	while(time_now < end_time){
		if(_system_control->hardware_version_.IsFresh()){
			uint32_t hardware_version = _system_control->hardware_version_.get_reply();
			param_t hardware_param = param_find("HARDWARE_VERSION");
			param_set(hardware_param, &hardware_version);
		}

		if(_system_control->firmware_version_.IsFresh()){
			uint32_t firmware_version = _system_control->firmware_version_.get_reply();
			param_t firmware_param = param_find("FIRMWARE_VERSION");
			param_set(firmware_param, &firmware_version);
		}

		time_now = hrt_absolute_time();
	}

	PX4_INFO("Done getting responses");
}
#endif
