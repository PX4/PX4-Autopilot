/****************************************************************************

 * Copyright (c) 2026, PX4 Development Team.
 * All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ****************************************************************************/

#pragma once
#include <drivers/device/i2c.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <drivers/drv_sensor.h>
#include "af9838_registers.hpp"
#include <uORB/uORB.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/Publication.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/events.h>


#ifndef MODULE_NAME
#  define MODULE_NAME "af9838"
#endif

namespace AF9838
{
extern float g_cli_rate_hz;
static constexpr float LSB_TO_uT = 0.1f;    // micro-Tesla / LSB
static constexpr float uT_TO_G   = 0.01f;   // µT → Gauss
static constexpr float ONE_G = 9.80665f;
}
static constexpr uint8_t  I2C_ADDRESS_DEFAULT = AF9838::I2C_ADDR; // 0x0C
static constexpr uint32_t I2C_SPEED           = 100000;           // 100 kHz
// -----------------------------------------------------------------

class AF9838_Driver : public device::I2C, public I2CSPIDriver<AF9838_Driver>
{
public:
	AF9838_Driver(const I2CSPIDriverConfig &config);

	static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, int runtime_instance);

	int  init() override;
	int  probe() override;
	void RunImpl();
	void start();
	void stop();
	void print_status();
	static void print_usage();

private:
	orb_advert_t _mavlink_log_pub{nullptr};
	int8_t _last_accuracy{-1};

	using device::I2C::transfer;
	uint8_t read_reg(uint8_t reg);
	int     read_block(uint8_t reg, uint8_t *buf, size_t len);
	int     write_reg(uint8_t reg, uint8_t val);

	int     soft_reset();
	int     configure();
	int     read_measurement();

	hrt_abstime _last_run{0};
	uint32_t _measure_interval_us{10000};

	PX4Magnetometer _px4_mag;
	bool            _running{false};
	bool _waiting_data{false};
	hrt_abstime _last_trigger{0};
	bool            _algo_inited{false};

	static constexpr uint16_t AF9838_MED_FILTER_MAX_NUM = 15;
	static constexpr uint16_t AF9838_MED_FILTER_NUMBER  = 5;

	uint16_t _mdf_num      = AF9838_MED_FILTER_NUMBER;
	uint16_t _mdf_init_idx = 0;
	uint16_t _mdf_data_idx = 0;
	float    _mdf_data[3][AF9838_MED_FILTER_MAX_NUM] {};

	void af9838_median_filter(float mag[3]);

	uORB::Subscription _accel_sub {ORB_ID(vehicle_acceleration)};
	hrt_abstime _last_accel_ts{0};

};
