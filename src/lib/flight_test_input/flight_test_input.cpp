#include "flight_test_input.hpp"

#include <controllib/block/BlockParam.hpp>
#include <systemlib/mavlink_log.h>

FlightTestInput::FlightTestInput(const char *name) :
	SuperBlock(NULL, name),
	_name(name),
	_type(this, "TYPE"),
	_enabled(this, "EN"),
	_dwell_time(this, "TIME"),
	_bias(this, "BIAS"),
	_amplitude(this, "AMP"),
	_standard_deviation(this, "SD")
{
}

void
FlightTestInput::update()
{
	float dt = 0.0f;

	if (_last_update > 0) {
		dt = hrt_elapsed_time(&_last_update) / 1e6f;
	}

	_last_update = hrt_absolute_time();

	// check vehicle status for changes to publication state
	_vehicle_status_sub.update();

#ifdef CONFIG_ARCH_BOARD_SITL
	const bool switch_was_on = true;
	const bool switch_on = true;
#else
	const bool switch_was_on = (_manual_control_setpoint_sub.get().fti_switch == manual_control_setpoint_s::SWITCH_POS_ON);
	_manual_control_setpoint_sub.update();
	const bool switch_on = (_manual_control_setpoint_sub.get().fti_switch == manual_control_setpoint_s::SWITCH_POS_ON);
#endif

	const bool was_enabled = _enabled.get();
	_enabled.update();
	const bool is_enabled = _enabled.get();


	switch (_state) {
	case TEST_INPUT_OFF:

		// manual switch: OFF -> ON
		if (!switch_was_on && switch_on) {
			_state = TEST_INPUT_INIT;
		}

		// param: OFF -> ON
		if (!was_enabled && is_enabled) {
			_state = TEST_INPUT_INIT;
		}

		break;

	case TEST_INPUT_INIT:
		// Initialize sweep variables and store current autopilot mode
		mavlink_and_console_log_info(&_mavlink_log_pub, "Flight test input %s injection starting", _name);

		// only use FTI (flight test input) param values as they were set during init
		updateParams();

		// abort sweep if any system mode (main_state or nav_state) change
		_nav_state = _vehicle_status_sub.get().nav_state;

		_time_running = 0.0f;

		_state = TEST_INPUT_RUNNING;

		break;

	case TEST_INPUT_RUNNING:

		if ((switch_was_on && switch_on) // manual switch still on
		    && _enabled.get() // enabled
		   ) {

			// increment time
			_time_running += dt;

			if (_time_running >= _dwell_time.get()) {
				_state = TEST_INPUT_COMPLETE;
			}

		} else {
			mavlink_log_critical(&_mavlink_log_pub, "Flight test input %s aborted", _name);
			_state = TEST_INPUT_OFF;
		}

		break;

	case TEST_INPUT_COMPLETE:

		// only return to off state once param is reset to 0
		mavlink_log_info(&_mavlink_log_pub, "#Flight test input %s completed", _name);

		_state = TEST_INPUT_OFF;

		break;
	}
}

// Generates additive white Gaussian Noise samples with zero mean and a standard deviation of 1.
float FlightTestInput::AWGN_generate()
{
	float temp2 = 0.0f;
	int p = 1;

	while (p > 0) {
		temp2 = (rand() / ((float)RAND_MAX));//need to add _standard_deviation.get()

		if (temp2 == 0.0f) {
			// temp2 is >= (RAND_MAX / 2)
			p = 1;

		} else {
			// temp2 is < (RAND_MAX / 2)
			p = -1;
		}
	}

	float temp1 = cosf((2.0f * M_PI_F) * rand() / ((float)RAND_MAX));

        float result = (sqrtf(-2.0f * logf(temp2)) * temp1) * _standard_deviation.get();

	return result;
}

void
FlightTestInput::inject(float &input)
{
	update();

	const float inject_input = input;
	float inject_output = inject_input;

	if (_state == TEST_INPUT_RUNNING) {

		float noise = 0.0f;

		switch (_type.get()) {
		case flight_test_input_s::TYPE_NONE:
			inject_output = inject_input + _bias.get();
			break;

		case flight_test_input_s::TYPE_FAILURE:
			inject_output = 0.0f;
			break;

		case flight_test_input_s::TYPE_GAUSSIAN:
			noise = AWGN_generate() * _amplitude.get();
			inject_output = inject_input + noise + _bias.get();
			break;

		case flight_test_input_s::TYPE_NOISE_WHITE:
			//noise = white_noise() * _amplitude.get();
			inject_output = inject_input + noise + _bias.get();
			break;

		case flight_test_input_s::TYPE_NOISE_PINK:
			//noise = pink_noise() * _amplitude.get();
			inject_output = inject_input + noise + _bias.get();
			break;

		case flight_test_input_s::TYPE_NOISE_BROWN:
			//noise = brown_noise() * _amplitude.get();
			inject_output = inject_input + noise + _bias.get();
			break;

		case flight_test_input_s::TYPE_NOISE_BLUE:
			//noise = blue_noise() * _amplitude.get();
			inject_output = inject_input + noise + _bias.get();
			break;

		case flight_test_input_s::TYPE_NOISE_VIOLET:
			//noise = violet_noise() * _amplitude.get();
			inject_output = inject_input + noise + _bias.get();
			break;

		case flight_test_input_s::TYPE_NOISE_GREY:
			//noise = grey_noise() * _amplitude.get();
			inject_output = inject_input + noise + _bias.get();
			break;
		}

		// update logging
		flight_test_input_s fti = {};
		fti.type = _type.get();

		strncpy(&fti.name[0], _name, sizeof(fti.name) - 1);
		fti.name[sizeof(fti.name) - 1] = 0;

		fti.input = inject_input;
		fti.added_noise = noise;
		fti.output = inject_output;

		fti.timestamp = hrt_absolute_time();

		int instance;
		orb_publish_auto(ORB_ID(flight_test_input), &_flight_test_input_pub, &fti, &instance, ORB_PRIO_DEFAULT);
	}

	input = inject_output;
}
