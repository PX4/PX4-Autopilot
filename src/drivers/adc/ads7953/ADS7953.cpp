
#include "ADS7953.h"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <drivers/drv_adc.h>
#include <parameters/param.h>

ADS7953::ADS7953(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	ModuleParams(nullptr)
{
	static_assert(arraySize(adc_report_s::channel_id) >= NUM_CHANNELS, "ADS7953 reports 16 channels");
}

int ADS7953::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		PX4_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	_adc_report.device_id = this->get_device_id();
	_adc_report.v_ref = _adc_ads7953_refv.get();
	_adc_report.resolution = 4096;

	for (unsigned i = 0; i < PX4_MAX_ADC_CHANNELS; ++i) {
		_adc_report.channel_id[i] = -1;
	}

	ScheduleOnInterval(10_ms);
	return PX4_OK;
}

int ADS7953::probe()
{
	// The ADS7953 has no ID register which we can check, so we verify the device via the returned channel ID.
	// We set the mode to "manual mode" and the channel to measure to 1.
	// If the returned channel ID on the third message is 1, we assume the ADS7953 is connected.
	uint8_t recv_data[2];

	int ret = rw_msg(&recv_data[0], 1, true);

	if (ret != PX4_OK) {
		PX4_DEBUG("ADS7953 probing failed (%i)", ret);
		return ret;
	}

	ret |= rw_msg(&recv_data[0], 0, false);
	ret |= rw_msg(&recv_data[0], 0, true);

	if (ret != PX4_OK || (recv_data[0] >> 4) != 1U) {
		PX4_DEBUG("ADS7953 probing failed (%i)", ret);
		return PX4_ERROR;
	}

	PX4_INFO("ADS7953 was found");
	return PX4_OK;
}

int ADS7953::rw_msg(uint8_t *recv_data, uint8_t ch, bool change_channel)
{
	uint8_t send_data[2];

	if (change_channel) {
		send_data[0] = 0x10 | (ch >> 1);
		send_data[1] = 0x00 | (ch << 7);

	} else {
		send_data[0] = 0x00;
		send_data[1] = 0x00;
	}

	return transfer(&send_data[0], &recv_data[0], 2);
}

int ADS7953::get_measurements()
{
	uint8_t recv_data[2];
	int count = 0;
	uint16_t mask = 0x00;
	uint8_t idx = 0;

	while (count < NUM_CHANNELS) {
		if (rw_msg(&recv_data[0], idx, true) == PX4_OK) {
			uint8_t ch_id = (recv_data[0] >> 4);

			//check if we already have a measurement for the returned channel
			if (!(mask & (1U << ch_id))) {
				mask |= (1U << ch_id);
				count++;
				_adc_report.channel_id[ch_id] = ch_id;
				_adc_report.raw_data[ch_id] = ((((uint16_t) recv_data[0]) & 0x0F) << 8) | recv_data[1];
			}
		}

		// Find index to measure next
		for (int i = 1; i <= NUM_CHANNELS; i++) {
			uint8_t candidate_id = (idx + i) % NUM_CHANNELS;

			if (!(mask & (1U << candidate_id))) {
				idx = candidate_id;
				break;
			}
		}
	}

	return 0;
}

void ADS7953::RunImpl()
{
	get_measurements();
	_adc_report.timestamp = hrt_absolute_time();
	_adc_report_pub.publish(_adc_report);

	for (unsigned i = 0; i < PX4_MAX_ADC_CHANNELS; ++i) {
		_adc_report.channel_id[i] = -1;
	}
}
