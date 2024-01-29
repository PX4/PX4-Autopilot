#ifndef VERTIQ_CLLIENT_MANAGER_HPP
#define VERTIQ_CLLIENT_MANAGER_HPP

#include <drivers/device/device.h>
#include <lib/led/led.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/perf/perf_counter.h>

#include <px4_log.h>
#include <px4_platform_common/module.h>

#include "vertiq_serial_interface.hpp"
#include "ifci.hpp"

#include "iq-module-communication-cpp/inc/propeller_motor_control_client.hpp"
#include "iq-module-communication-cpp/inc/brushless_drive_client.hpp"
#include "iq-module-communication-cpp/inc/arming_handler_client.hpp"

#ifdef CONFIG_USE_SYSTEM_CONTROL_CLIENT
#include "iq-module-communication-cpp/inc/system_control_client.hpp"
#endif

class VertiqClientManager{
	public:
	VertiqClientManager(VertiqSerialInterface * serial_interface);

	void Init(uint8_t object_id);

	/**
	* @brief Handle the IQUART interface. Make sure that we update TX and RX buffers
	*/
	void HandleClientCommunication();

	IFCI * GetMotorInterface();

	void SendSetForceArm();
	void SendSetForceDisarm();
	void SendSetCoast();
	void SendSetVelocitySetpoint(uint16_t velocity_setpoint);

	uint8_t GetNumberOfClients();

	private:
	//We need a serial handler in order to talk over the serial port
	VertiqSerialInterface * _serial_interface;

	// #ifdef CONFIG_USE_SYSTEM_CONTROL_CLIENT
	// SystemControlClient * _system_control;
	// #endif

	//IQUART Client configuration
	IFCI _motor_interface;

	//Vertiq client information
	static const uint8_t _kBroadcastID = 63;
	static const uint8_t NUM_CLIENTS = 2;
	PropellerMotorControlClient _broadcast_prop_motor_control;
	ArmingHandlerClient _arming_handler;
	ClientAbstract * _client_array[NUM_CLIENTS];
};

#endif
