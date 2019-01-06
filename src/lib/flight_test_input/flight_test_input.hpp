
#pragma once

#include <mathlib/mathlib.h>
#include <stdint.h>

#include <controllib/block/BlockParam.hpp>
#include <drivers/drv_hrt.h>
#include <systemlib/mavlink_log.h>
#include <uORB/topics/flight_test_input.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_status.h>

class FlightTestInput : public control::SuperBlock
{
public:
	FlightTestInput(const char *name);
	~FlightTestInput() override = default;

	// Inject current test input
	void inject(float &input);

private:

	// Update test input computation
	void update();

	const char *_name;

	// parameters
	control::BlockParamInt _type;
	control::BlockParamInt _enabled;
	control::BlockParamFloat _dwell_time;
	control::BlockParamFloat _bias;
	control::BlockParamFloat _amplitude;
	control::BlockParamFloat _standard_deviation;

	enum FlightTestInputState {
		TEST_INPUT_OFF = 0,
		TEST_INPUT_INIT,
		TEST_INPUT_RUNNING,
		TEST_INPUT_COMPLETE
	} _state{TEST_INPUT_OFF};

        orb_advert_t	_mavlink_log_pub{nullptr};

	float _time_running{0.0f};

	hrt_abstime	_last_update{0};

	orb_advert_t _flight_test_input_pub{nullptr};

	// system navigation state captured during test input init
	uint8_t _nav_state{0};

	uORB::Subscription<manual_control_setpoint_s>	_manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription<vehicle_status_s>			_vehicle_status_sub{ORB_ID(vehicle_status)};


	float AWGN_generate();
};
