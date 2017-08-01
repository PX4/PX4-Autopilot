#include "linux_ina219.hpp"

//---------------------------------------------------------------------------------------------//
void linux_ina219::start() {
	if (linux_ina219::__is_running)
		return;
	/**
	 * BAT_N_CELLS
BAT_V_CHARGED
BAT_V_EMPTY
BAT_V_DIV
BAT_A_PER_V
	 */
	// gets the parameters for the esc's pwm
	//param_get(param_find("PWM_DISARMED"), &rpi_pca9685_pwm_out::_pwm_disarmed);
	//param_get(param_find("PWM_MIN"), &rpi_pca9685_pwm_out::_pwm_min);
	//param_get(param_find("PWM_MAX"), &rpi_pca9685_pwm_out::_pwm_max);

	linux_ina219::ina219 = new linux_ina219::INA219(linux_ina219::__ina219_bus,INA219_ADDRESS);
	linux_ina219::ina219->calibration32v2a();
	__battery_status_pub = orb_advertise(ORB_ID(battery_status),
			&__battery_status_data);

	if (nullptr == __battery_status_pub) {
		PX4_WARN("error: battery status advertise failed");
		return;
	}
	linux_ina219::__should_exit = false;
	/* start the task */
	int result = work_queue(HPWORK, &_work, (worker_t) & linux_ina219::running,
			nullptr, 0);
	if (result < 0) {
		PX4_ERR("linux_ina219 start failed");
		delete (linux_ina219::ina219);
		linux_ina219::ina219 = nullptr;
		return;
	}

	linux_ina219::__is_running = true;
}
//---------------------------------------------------------------------------------------------//
void linux_ina219::stop() {
	if (linux_ina219::__is_running) {
		linux_ina219::__should_exit = true;
		linux_ina219::__is_running = false;
		PX4_INFO("Stop linux_ina219");
		return;
	}
	PX4_INFO("linux_ina219 is not running");
}
//----------------------------------------------------------------------------------------------//
void linux_ina219::status() {
	PX4_INFO("linux_ina219 is %s", __is_running ? "running" : "not running");
}
//---------------------------------------------------------------------------------------------//
void linux_ina219::usage() {
	PX4_INFO("linux_ina219 start|stop|status -b <bus>");
}
//---------------------------------------------------------------------------------------------//
void linux_ina219::running(int argc,char**argv) {
	PX4_INFO("持续测量");
	float v;
	float sv;
	//float a;
	//a = linux_ina219::ina219->getCurrentMa();
	//a = 1.0;
	v = linux_ina219::ina219->getBusVoltage();
	sv = linux_ina219::ina219->getShuntVoltage();
	PX4_INFO("V: %f, SV: %sv",v,sv);
	linux_ina219::__battery_status_data.timestamp = hrt_absolute_time(); // required for logger
	linux_ina219::__battery_status_data.voltage_v = v;
	linux_ina219::__battery_status_data.voltage_filtered_v = v;
	//linux_ina219::__battery_status_data.current_a = a;
	//linux_ina219::__battery_status_data.current_filtered_a=a;
	//linux_ina219::__battery_status_data.discharged_mah;
	//linux_ina219::__battery_status_data.remaining;
	//linux_ina219::__battery_status_data.scale;
	linux_ina219::__battery_status_data.cell_count = 3;
	linux_ina219::__battery_status_data.connected = true;
	linux_ina219::__battery_status_data.warning = 0;
	//linux_ina219::__battery_status_data._padding0[6] = {0}; // required for logger
	memset(linux_ina219::__battery_status_data._padding0,0,sizeof(linux_ina219::__battery_status_data._padding0));
	orb_publish(ORB_ID(battery_status), __battery_status_pub,
			&__battery_status_data);
	if (false==linux_ina219::__should_exit) {
			PX4_INFO("next turn");
			work_queue(HPWORK, &_work, (worker_t) &linux_ina219::running,nullptr,
					USEC2TICK(INA219_INTERVAL_US));
	}
}
//---------------------------------------------------------------------------------------------//
int linux_ina219_main(int argc, char**argv) {
	int command = -1;
	int start = 0;
	for (start = 0; start < argc; ++start) {
		if (0 == strcmp("start", argv[start])) {
			command = 0;
			continue;
		}
		if (0 == strcmp("stop", argv[start])) {
			command = 1;
			continue;
		}
		if (0 == strcmp("status", argv[start])) {
			command = 2;
			continue;
		}

		if (0 == strcmp("-b", argv[start])) {
			linux_ina219::__ina219_bus = atoi(argv[start + 1]);
		}

	}
	if (-1 == command) {
		linux_ina219::usage();
		return -1;
	}
	switch (command) {
	case 0:
		linux_ina219::start();
		break;
	case 1:
		linux_ina219::stop();
		break;
	case 2:
		linux_ina219::status();
	}
	return 0;
}

