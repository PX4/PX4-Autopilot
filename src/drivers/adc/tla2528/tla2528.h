#pragma once

#include <stdint.h>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/drivers/device/i2c.h>
#include <uORB/topics/adc_report.h>
#include <uORB/PublicationMulti.hpp>

#include <px4_platform_common/module_params.h>
//#include <uORB/SubscriptionCallback.hpp>
//#include <uORB/topics/gpio_config.h>
//#include <uORB/topics/gpio_in.h>
//#include <uORB/topics/gpio_out.h>
//#include <uORB/topics/gpio_request.h>





using namespace time_literals;


class TLA2528 : public device::I2C, public I2CSPIDriver<TLA2528>, public ModuleParams
{
public:
	TLA2528(const I2CSPIDriverConfig &config);
	~TLA2528() override;
	//virtual ~TLA2528() = default;
	static void print_usage();

	int init() override;
	void RunImpl();
	int probe() override;

	static void set_state(uint8_t state);
	static void set_interval(uint16_t interval);

private:

	perf_counter_t _cycle_perf;
	perf_counter_t _comms_errors;

	static constexpr int NUM_CHANNELS = 16;
	uORB::PublicationMulti<adc_report_s> _adc_report_pub{ORB_ID(adc_report)};

	struct init_config_t {
		uint16_t interval;
		uint8_t state;
		uint8_t num_pins;
	};

	static init_config_t config_data;
	adc_report_s _adc_report{};

	void exit_and_cleanup() override;
	int conf();
	int reset();
	int calibrate();
	void adc_get();
};
