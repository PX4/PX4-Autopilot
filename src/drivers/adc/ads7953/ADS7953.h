#pragma once

#include <drivers/device/spi.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/drivers/device/spi.h>
#include <lib/parameters/param.h>
#include <uORB/topics/adc_report.h>
#include <uORB/PublicationMulti.hpp>
#include <lib/mixer_module/mixer_module.hpp>
#include <px4_platform_common/module_params.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/Subscription.hpp>

using namespace time_literals;


class ADS7953 : public device::SPI, public I2CSPIDriver<ADS7953>, public ModuleParams
{
public:
	ADS7953(const I2CSPIDriverConfig &config);
	virtual ~ADS7953() = default;
	static void print_usage();

	int init() override;
	void RunImpl();
	int probe() override;


private:
	static constexpr int NUM_CHANNELS = 16;
	uORB::PublicationMulti<adc_report_s> _adc_report_pub{ORB_ID(adc_report)};

	static const hrt_abstime SAMPLE_INTERVAL{50_ms};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::ADC_ADS7953_REFV>) _adc_ads7953_refv
	)
	adc_report_s _adc_report{};

	int get_measurements();
	int rw_msg(uint8_t *recv_data, uint8_t ch, bool change_channel);
};
