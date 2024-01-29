
#include "vertiq_client_manager.hpp"

VertiqClientManager::VertiqClientManager(VertiqSerialInterface * serial_interface) :
	_serial_interface(serial_interface),
	_broadcast_prop_motor_control(_kBroadcastID), //Initialize with a module ID of 63 for broadcasting
	_arming_handler(_kBroadcastID)
{
	_client_array[0] = &_broadcast_prop_motor_control;
	_client_array[1] = &_arming_handler;
}

void VertiqClientManager::Init(uint8_t object_id){
	// #ifdef CONFIG_USE_SYSTEM_CONTROL_CLIENT
	// static SystemControlClient sys_control = SystemControlClient(object_id);
	// _system_control = &sys_control;
	// #endif
}

void VertiqClientManager::HandleClientCommunication(){
	//Update our serial rx
	_serial_interface->process_serial_rx(&_motor_interface, _client_array);

	//Update our serial tx
	_serial_interface->process_serial_tx();
}

IFCI * VertiqClientManager::GetMotorInterface(){
	return &_motor_interface;
}

uint8_t VertiqClientManager::GetNumberOfClients(){
	return NUM_CLIENTS;
}

void  VertiqClientManager::SendSetForceArm(){
	_arming_handler.motor_armed_.set(*_serial_interface->get_iquart_interface(), 1);
}

void  VertiqClientManager::SendSetForceDisarm(){
	_arming_handler.motor_armed_.set(*_serial_interface->get_iquart_interface(), 0);
}

void  VertiqClientManager::SendSetCoast(){
	_broadcast_prop_motor_control.ctrl_coast_.set(*_serial_interface->get_iquart_interface());
}

void  VertiqClientManager::SendSetVelocitySetpoint(uint16_t velocity_setpoint){
	_broadcast_prop_motor_control.ctrl_velocity_.set(*_serial_interface->get_iquart_interface(), velocity_setpoint);
}

