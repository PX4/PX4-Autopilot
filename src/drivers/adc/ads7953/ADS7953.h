#pragma once

#include <drivers/device/spi.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/drivers/device/spi.h>
#include <lib/parameters/param.h>
#include <uORB/topics/adc_report.h>
#include <uORB/PublicationMulti.hpp>
using namespace time_literals;


class ADS7953 : public device::SPI, public I2CSPIDriver<ADS7953>
{
public:
	ADS7953(const I2CSPIDriverConfig &config);
	virtual ~ADS7953() = default;
	static void print_usage();

	int init() override;
	void RunImpl();
	int probe() override;


private:
	uORB::PublicationMulti<adc_report_s> _adc_report_pub{ORB_ID(adc_report)};

	static const hrt_abstime SAMPLE_INTERVAL{50_ms};

	adc_report_s _adc_report{};

	int get_measurements();
	int rw_msg(uint8_t *recv_data, uint8_t ch, bool change_channel);
};
