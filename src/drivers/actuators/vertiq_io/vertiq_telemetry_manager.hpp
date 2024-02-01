
#ifndef VERTIQ_TELEMETRY_MANAGER_HPP
#define VERTIQ_TELEMETRY_MANAGER_HPP

#include <drivers/device/device.h>
#include <lib/led/led.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/perf/perf_counter.h>

#include <px4_log.h>
#include <px4_platform_common/module.h>

#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/led_control.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/actuator_test.h>

#include "ifci.hpp"

class VertiqTelemetryManager{
	public:

	/**
	* @brief Construct a new VertiqTelemetryManager object with a pointer to an IFCI handler
	* @param motor_interface A pointer to and IFCI interface
	*/
	VertiqTelemetryManager(IFCI * motor_interface);

	/**
	* @brief Initialize the telemetry manager with the bitmask set in the PX4 parameters
	* @param telem_bitmask The bitmask set in the PX4 parameters as VERTIQ_TEL_MSK
	*/
	void Init(uint16_t telem_bitmask);

	/**
	* @brief Start publishing the ESC statuses to the uORB esc_status topic
	*/
	void StartPublishing(uORB::Publication<esc_status_s> * esc_status_pub);

	/**
	* @brief Part of initialization. Find the first and last positions that indicate modules to grab telemetry from
	*/
	void FindFirstAndLastTelemetryPositions();

	/**
	* @brief Attempt to grab the telemetry response from the currently targeted module. Handles
	* updating when to start requesting telemetry from the next module.
	*/
	uint16_t UpdateTelemetry();

	/**
	* @brief Determing the next module to request telemetry from
	*/
	uint16_t FindNextMotorForTelemetry();

	/**
	* @brief return access to our esc_status_s handler
	* @return _esc_status: our instance of an esc_status_s
	*/
	esc_status_s GetEscStatus();

	private:

	IFCI * _motor_interface;

	//We want to publish our ESC Status to anyone who will listen
	esc_status_s		_esc_status;

	static const uint8_t MAX_SUPPORTABLE_IFCI_CVS = 16;

	//Store the telemetry bitmask for who we want to get telemetry from
	uint16_t _telem_bitmask = 0;

	//The number of modules that we're going to request telemetry from
	uint8_t _number_of_modules_for_telem = 0;

	//The bit position of the first module whose telemetry we should get
	uint16_t _first_module_for_telem = 0;

	//The bit position of the last module whose telemetry we should get
	uint16_t _last_module_for_telem = 0;

	//Current target for telemetry
	uint16_t _current_telemetry_target_module_id = 0;

	//This is the variable we're actually going to use in the brodcast packed control message
	//We set and use it to _current_telemetry_target_module_id until we send the first
	//broadcast message with this as the tail byte. after that first transmission, set it
	//to an impossible module ID
	uint16_t _telemetry_request_id = 0;

	//The amount of time (in ms) that we'll wait for a telemetry response
	static const hrt_abstime _telem_timeout = 50_ms;

	//The system time the last time that we got telemetry
	hrt_abstime _time_of_last_telem_request = 0;

	static const uint8_t _impossible_module_id = 255;
};

#endif
